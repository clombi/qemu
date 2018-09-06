/*
 * QEMU sPAPR Coherent Accelerator Processor Interface (CAPI)
 *
 * Copyright 2018 Greg Kurz, IBM Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/ppc/spapr.h"
#include "hw/ppc/spapr_capi.h"
#include "cpu.h"
#include "mmu-radix64.h"

/* from kernel header <misc/ocxl-config.h> */
#define OCXL_EXT_CAP_ID_PASID                 0x1B
#define OCXL_EXT_CAP_ID_DVSEC                 0x23

#define OCXL_PASID_MAX_WIDTH                  0x4

#define OCXL_DVSEC_VENDOR_OFFSET              0x4
#define OCXL_DVSEC_ID_OFFSET                  0x8
#define OCXL_DVSEC_TL_ID                      0xF000
#define   OCXL_DVSEC_TL_BACKOFF_TIMERS          0x10
#define   OCXL_DVSEC_TL_RECV_CAP                0x18
#define   OCXL_DVSEC_TL_SEND_CAP                0x20
#define   OCXL_DVSEC_TL_RECV_RATE               0x30
#define   OCXL_DVSEC_TL_SEND_RATE               0x50
#define OCXL_DVSEC_FUNC_ID                    0xF001
#define   OCXL_DVSEC_FUNC_OFF_INDEX             0x08
#define   OCXL_DVSEC_FUNC_OFF_ACTAG             0x0C
#define OCXL_DVSEC_AFU_INFO_ID                0xF003
#define   OCXL_DVSEC_AFU_INFO_AFU_IDX           0x0A
#define   OCXL_DVSEC_AFU_INFO_OFF               0x0C
#define   OCXL_DVSEC_AFU_INFO_DATA              0x10
#define OCXL_DVSEC_AFU_CTRL_ID                0xF004
#define   OCXL_DVSEC_AFU_CTRL_AFU_IDX           0x0A
#define   OCXL_DVSEC_AFU_CTRL_TERM_PASID        0x0C
#define   OCXL_DVSEC_AFU_CTRL_ENABLE            0x0F
#define   OCXL_DVSEC_AFU_CTRL_PASID_SUP         0x10
#define   OCXL_DVSEC_AFU_CTRL_PASID_EN          0x11
#define   OCXL_DVSEC_AFU_CTRL_PASID_BASE        0x14
#define   OCXL_DVSEC_AFU_CTRL_ACTAG_SUP         0x18
#define   OCXL_DVSEC_AFU_CTRL_ACTAG_EN          0x1A
#define   OCXL_DVSEC_AFU_CTRL_ACTAG_BASE        0x1C
#define OCXL_DVSEC_VENDOR_ID                  0xF0F0
#define   OCXL_DVSEC_VENDOR_CFG_VERS            0x0C
#define   OCXL_DVSEC_VENDOR_TLX_VERS            0x10
#define   OCXL_DVSEC_VENDOR_DLX_VERS            0x20

#define OCXL_DVSEC_TEMPL_VERSION         0x0
#define OCXL_DVSEC_TEMPL_NAME            0x4
#define OCXL_DVSEC_TEMPL_AFU_VERSION     0x1C
#define OCXL_DVSEC_TEMPL_MMIO_GLOBAL     0x20
#define OCXL_DVSEC_TEMPL_MMIO_GLOBAL_SZ  0x28
#define OCXL_DVSEC_TEMPL_MMIO_PP         0x30
#define OCXL_DVSEC_TEMPL_MMIO_PP_SZ      0x38
#define OCXL_DVSEC_TEMPL_MEM_SZ          0x3C
#define OCXL_DVSEC_TEMPL_WWID            0x40

/*BAR0 + x200_0000 : BAR0 + x3FF_FFFF
 *AFU per Process PSA (64kB per Process, max 512 processes)
 */
#define OCXL_AFU_PER_PPROCESS_PSA_ADDR_START  0x2000000
#define OCXL_AFU_PER_PPROCESS_PSA_ADDR_END    0x3FFFFFF
#define OCXL_AFU_PER_PPROCESS_PSA_LENGTH      0x10000

#define MEMCPY_WE_CMD_VALID	(0x1 << 0)

struct memcpy_work_element {
    volatile uint8_t cmd; /* valid, wrap, cmd */
    volatile uint8_t status;
    uint16_t length;
    uint8_t cmd_extra;
    uint8_t reserved[3];
    uint64_t atomic_op;
    uint64_t src;  /* also irq EA or atomic_op2 */
    uint64_t dst;
} __packed;

static target_ulong h_spa_setup(PowerPCCPU *cpu,
                                sPAPRMachineState *spapr,
                                target_ulong opcode,
                                target_ulong *args)
{
    target_ulong p_spa_mem = args[0];

    fprintf(stderr, "%s: %#lx\n", __func__, p_spa_mem);
    return H_SUCCESS;
}

static void *memcopy_status_t(void *arg)
{
    sPAPRCAPIDeviceState *s = (sPAPRCAPIDeviceState *)arg;
    struct timeval test_timeout, temp;
    struct memcpy_work_element first_we;
    uint32_t pid = pci_default_read_config(PCI_DEVICE(s), 0xfe0, 4);
    hwaddr wed = ppc_radix64_get_phys_page_virtual(s->wed, pid);

    temp.tv_sec = 5;
    temp.tv_usec = 0;

    gettimeofday(&test_timeout, NULL);
    timeradd(&test_timeout, &temp, &test_timeout);

    fprintf(stderr, "%s - (pasid: %d) WED EA: %#lx GPA: %#lx pidr: %d\n",
            __func__, s->pasid, s->wed, wed, pid);

    for (;; gettimeofday(&temp, NULL)) {
        if (timercmp(&temp, &test_timeout, >)) {
            fprintf(stderr, "%s - timeout polling for completion\n",
                    __func__);
            break;
        }

        /* wait the validation of the command */
        cpu_physical_memory_read(wed, &first_we, sizeof(first_we));

        if (first_we.cmd == MEMCPY_WE_CMD_VALID) {
            hwaddr src = ppc_radix64_get_phys_page_virtual(first_we.src, pid);
            hwaddr dst = ppc_radix64_get_phys_page_virtual(first_we.dst, pid);
            char *buf;

            /* copy data from src to dst */
            fprintf(stderr, "%s - copy %d bytes from "
                    "%#lx (EA %#lx) to %#lx (EA %#lx)\n",
                    __func__, first_we.length, first_we.src, src,
                    first_we.dst, dst);
            buf = g_malloc(first_we.length);
            cpu_physical_memory_read(src, buf, first_we.length);
            cpu_physical_memory_write(dst, buf, first_we.length);
            g_free(buf);

            stb_phys(&address_space_memory, wed + 1, 0x1); /* write status */
            s->status = 0; /* The call succeeded */
            break;
        }
    }
    return NULL;
}

static uint64_t capi_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    sPAPRCAPIDeviceState *s = opaque;

    fprintf(stderr, "%s - addr: 0x%lx,  size: 0x%x\n",
            __func__, addr, size);

    switch (addr & 0xFF) {
    case 0x10: /* Process Status Register 64bits */
        return s->status;
    break;
    case 0x14: 
    break;
    }
    return 0;
}

static void capi_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned width)
{
    sPAPRCAPIDeviceState *s = opaque;
    pthread_t thr;

    fprintf(stderr, "%s - addr: 0x%lx, val: 0x%lx\n",
            __func__, addr, val);

    if ((addr >= OCXL_AFU_PER_PPROCESS_PSA_ADDR_START) &&
        (addr <= OCXL_AFU_PER_PPROCESS_PSA_ADDR_END))
    {    
         /* WED Register (x0000)
          *     [63:12] Base EA of the start of the work element queue.
          */
         switch (addr & 0xFF) {
         case 0x0:
             s->wed = val & ~0xfff;
             s->pasid = (addr - OCXL_AFU_PER_PPROCESS_PSA_ADDR_START)/OCXL_AFU_PER_PPROCESS_PSA_LENGTH;
         break;
         case 0x4:
             s->wed += (val << 32);
             s->status = 0x3; /* Process Element has been terminated/Removed */

             /* start a thread to update the memcopy status */
             pthread_create(&thr, NULL, memcopy_status_t, s);
         break;
         case 0x28:
         case 0x2C:
         break;
         }
    }
}

static void spapr_capi_device_realize(PCIDevice *pdev, Error **errp)
{
    sPAPRCAPIDeviceState *s = SPAPR_CAPI_DEVICE(pdev);
    uint16_t offset, next_offset, cap_size;

    static const MemoryRegionOps capi_mmio_ops = {
        .read = capi_mmio_read,
        .write = capi_mmio_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
        .impl = {
            .min_access_size = 4,
            .max_access_size = 4,
        }
    };

    /* Configuration Space Header */
    pci_set_word(pdev->config + PCI_COMMAND, PCI_COMMAND_MEMORY);
    pci_set_word(pdev->config + PCI_STATUS, PCI_STATUS_CAP_LIST);
    pci_set_byte(pdev->config + PCI_REVISION_ID, 0x00);
    pci_set_byte(pdev->config + PCI_HEADER_TYPE, 0x80);

    /* mmio BAR 0 - 64MB */
    memory_region_init_io(&s->mmio, OBJECT(s), &capi_mmio_ops,
                          s, "capi-mmio", 0x4000000);

    pci_register_bar(pdev, 0,
            PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64,
            &s->mmio);

    /* Capabilities ? */
    /*pcie_endpoint_cap_init(pdev, 0x40);*/

    /* Extended Capabilities in Configuration Space
     * always begin at offset 100h
     */
    offset = PCI_CONFIG_SPACE_SIZE;
    cap_size = 0x100;

    /* Process Address Space ID Extended Capability */
    next_offset = offset + PCI_EXT_CAP_PASID_SIZEOF;
    pcie_add_capability(pdev, OCXL_EXT_CAP_ID_PASID, 0x1, offset, PCI_EXT_CAP_PASID_SIZEOF);
    pci_set_long(pdev->config + offset, PCI_EXT_CAP(OCXL_EXT_CAP_ID_PASID, 0x1, next_offset));
    pci_set_long(pdev->config + offset + OCXL_PASID_MAX_WIDTH, 0x00000900);

    /* Designated Vendor Specific Extended Capabilities - Transport Layer */
    offset = next_offset;
    next_offset = offset + cap_size;
    pcie_add_capability(pdev, OCXL_EXT_CAP_ID_DVSEC, 0x1, offset, cap_size);
    pci_set_long(pdev->config + offset, PCI_EXT_CAP(OCXL_EXT_CAP_ID_DVSEC, 0x1, next_offset));
    pci_set_long(pdev->config + offset + OCXL_DVSEC_VENDOR_OFFSET, PCI_VENDOR_ID_IBM);
    pci_set_long(pdev->config + offset + OCXL_DVSEC_ID_OFFSET, OCXL_DVSEC_TL_ID);

    /* Designated Vendor Specific Extended Capabilities - Function Configuration */
    offset = next_offset;
    next_offset = offset + cap_size;
    pcie_add_capability(pdev, OCXL_EXT_CAP_ID_DVSEC, 0x1, offset, cap_size);
    pci_set_long(pdev->config + offset + OCXL_DVSEC_VENDOR_OFFSET, PCI_VENDOR_ID_IBM);
    pci_set_long(pdev->config + offset + OCXL_DVSEC_ID_OFFSET, (0x1 << 31) | OCXL_DVSEC_FUNC_ID);

    /* Designated Vendor Specific Extended Capabilities - AFU Information */
    offset = next_offset;
    next_offset = offset + cap_size;
    pcie_add_capability(pdev, OCXL_EXT_CAP_ID_DVSEC, 0x1, offset, cap_size);
    pci_set_long(pdev->config + offset + OCXL_DVSEC_VENDOR_OFFSET, PCI_VENDOR_ID_IBM);
    pci_set_long(pdev->config + offset + OCXL_DVSEC_ID_OFFSET, OCXL_DVSEC_AFU_INFO_ID);
    pci_set_byte(pdev->wmask + offset + OCXL_DVSEC_AFU_INFO_AFU_IDX, 0x3f);
    pci_set_long(pdev->wmask + offset + OCXL_DVSEC_AFU_INFO_OFF, 0xffffffff);
    s->dvsec_afu_info_id = offset;

    /* Designated Vendor Specific Extended Capabilities - AFU Control */
    offset = next_offset;
    next_offset = 0;
    pcie_add_capability(pdev, OCXL_EXT_CAP_ID_DVSEC, 0x1, offset, cap_size);
    pci_set_long(pdev->config + offset + OCXL_DVSEC_VENDOR_OFFSET, PCI_VENDOR_ID_IBM);
    pci_set_long(pdev->config + offset + OCXL_DVSEC_ID_OFFSET, OCXL_DVSEC_AFU_CTRL_ID);
    pci_set_byte(pdev->config + offset + OCXL_DVSEC_AFU_CTRL_ENABLE, 0x01);
    pci_set_byte(pdev->config + offset + OCXL_DVSEC_AFU_CTRL_PASID_SUP, 0x9);
    pci_set_byte(pdev->config + offset + OCXL_DVSEC_AFU_CTRL_ACTAG_SUP, 0x20);

    /* FIXME: temporary hack to access PIDR */
    pci_set_long(pdev->wmask + 0xfe0, 0xffffffff);
}

static void spapr_capi_device_exit(PCIDevice *pdev)
{
}

static uint32_t spapr_capi_read_config(PCIDevice *pdev, uint32_t addr, int l)
{
    sPAPRCAPIDeviceState *s = SPAPR_CAPI_DEVICE(pdev);
    uint32_t val;

    if (addr == s->dvsec_afu_info_id + OCXL_DVSEC_AFU_INFO_DATA) {
        uint32_t dvsec_afu_info_off =
            s->dvsec_afu_info_id + OCXL_DVSEC_AFU_INFO_OFF;
        int32_t offset =
            pci_default_read_config(pdev, dvsec_afu_info_off, 4) & 0x7ffffff;

        if (offset >= OCXL_DVSEC_TEMPL_WWID) {
            fprintf(stderr, "%s: OCXL_DVSEC_AFU_INFO_DATA 0x%x\n", __func__,
                    offset);
            return 0;
        } else if (offset >= OCXL_DVSEC_TEMPL_MEM_SZ) {
            uint8_t templ_mem_sz = 0x1a;

            val =
                templ_mem_sz & 0xff;
#if 0
            fprintf(stderr, "%s: OCXL_DVSEC_TEMPL_MEM_SZ 0x%x\n", __func__,
                    val);
#endif
        } else if (offset >= OCXL_DVSEC_TEMPL_MMIO_PP_SZ) {
            uint32_t templ_mmio_pp_stride = 0x1;

            val =
                templ_mmio_pp_stride << 16;
#if 0
            fprintf(stderr, "%s: OCXL_DVSEC_TEMPL_MMIO_PP_SZ 0x%x\n",
                    __func__, val);
#endif
        } else if (offset >= OCXL_DVSEC_TEMPL_MMIO_PP) {
            uint8_t templ_mmio_pp = 0x0;
            uint16_t templ_mmio_pp_offset_lo = 0x200;
            uint32_t templ_mmio_pp_offset_hi = 0x0;

            if (offset == OCXL_DVSEC_TEMPL_MMIO_PP) {
                val =
                    templ_mmio_pp +
                    (templ_mmio_pp_offset_lo << 16);
            } else {
                val = templ_mmio_pp_offset_hi;
            }
#if 0
            fprintf(stderr, "%s: OCXL_DVSEC_TEMPL_MMIO_PP 0x%x\n", __func__,
                    val);
#endif
        } else if (offset >= OCXL_DVSEC_TEMPL_MMIO_GLOBAL_SZ) {
            uint32_t templ_mmio_global_size = 0x2000000;

            val = templ_mmio_global_size;
#if 0
            fprintf(stderr, "%s: OCXL_DVSEC_TEMPL_MMIO_GLOBAL_SZ 0x%x\n",
                    __func__, val);
#endif
        } else if (offset >= OCXL_DVSEC_TEMPL_MMIO_GLOBAL) {
            uint8_t templ_mmio_global = 0x0;
            uint16_t templ_mmio_global_offset_lo = 0x0;
            uint32_t templ_mmio_global_offset_hi = 0x0;

            if (offset == OCXL_DVSEC_TEMPL_MMIO_GLOBAL) {
                val =
                    templ_mmio_global +
                    (templ_mmio_global_offset_lo << 16);
            } else {
                val = templ_mmio_global_offset_hi;
            }
#if 0
            fprintf(stderr, "%s: OCXL_DVSEC_TEMPL_MMIO_GLOBAL 0x%x\n", __func__,
                    val);
#endif
        } else if (offset >= OCXL_DVSEC_TEMPL_AFU_VERSION) {
            uint8_t templ_afu_major_version = 0x1;
            uint8_t templ_afu_minor_version = 0x0;

            val =
                (templ_afu_minor_version << 16) +
                (templ_afu_major_version << 24);
#if 0
            fprintf(stderr, "%s: OCXL_DVSEC_TEMPL_VERSION 0x%x\n", __func__,
                    val);
#endif
        } else if (offset >= OCXL_DVSEC_TEMPL_NAME) {
            const char *templ_name = "IBM,MEMCPY3";
            int i;

            offset -= OCXL_DVSEC_TEMPL_NAME;
            val = 0;

            for (i = offset; i < offset + l; i++) {
                int j = 2 * offset + l - i - 1;

                val <<= 8;
                if (j < strlen(templ_name)) {
                    val |= templ_name[j];
                }
            }
#if 0
            fprintf(stderr, "%s: OCXL_DVSEC_TEMPL_NAME[%d] 0x%x\n", __func__,
                    offset, val);
#endif
        } else if (offset >= OCXL_DVSEC_TEMPL_VERSION) {
            uint8_t templ_major_version = 0x0;
            uint8_t templ_minor_version = 0x5;
            uint16_t templ_size = 0x58;

            val =
                templ_minor_version +
                (templ_major_version << 8) +
                (templ_size << 16);
#if 0
            fprintf(stderr, "%s: OCXL_DVSEC_TEMPL_VERSION 0x%x\n", __func__,
                    val);
#endif
        }
    } else {
        val = pci_default_read_config(pdev, addr, l);
#if 0
        fprintf(stderr, "%s: 0x%x 0x%x (%d)\n", __func__, addr, val, l);
#endif
    }
    return val;
}

static void spapr_capi_write_config(PCIDevice *pdev, uint32_t addr,
                                    uint32_t val, int l)
{
    sPAPRCAPIDeviceState *s = SPAPR_CAPI_DEVICE(pdev);

    if (addr == s->dvsec_afu_info_id + OCXL_DVSEC_AFU_INFO_OFF) {
        val |= 1<<31; /* Valid bit */
    }
#if 0
    fprintf(stderr, "%s: 0x%x 0x%x (%d)\n", __func__, addr, val, l);
#endif
    pci_default_write_config(pdev, addr, val, l);
}

static void spapr_capi_device_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(oc);

    pc->realize = spapr_capi_device_realize;
    pc->exit = spapr_capi_device_exit;
    pc->vendor_id = PCI_VENDOR_ID_IBM;
    pc->device_id = 0x062b; /* from ocxl linux driver */
    pc->class_id = 0x1200; /* from CAPI adapter config space on zaiuslp14 */
    pc->subsystem_vendor_id = PCI_VENDOR_ID_IBM;
    pc->subsystem_id = 0x060f; /* from CAPI adapter config space on zaiuslp14 */
    pc->config_read = spapr_capi_read_config;
    pc->config_write = spapr_capi_write_config;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "sPAPR CAPI device";
    dc->user_creatable = true;

    /* hcall-spa */
    spapr_register_hypercall(KVMPPC_H_SPA_SETUP, h_spa_setup);
}

static const TypeInfo spapr_capi_device_info = {
    .name          = TYPE_SPAPR_CAPI_DEVICE,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(sPAPRCAPIDeviceState),
    .class_init    = spapr_capi_device_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { },
    },
};

static void spapr_capi_register_type(void)
{
    type_register_static(&spapr_capi_device_info);
}
type_init(spapr_capi_register_type)
