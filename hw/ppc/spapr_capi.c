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
#include "hw/pci-host/spapr.h"
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

/*BAR0 + x200_0000 : BAR0 + x3FF_FFFF
 *AFU per Process PSA (64kB per Process, max 512 processes)
 */
#define OCXL_AFU_PER_PPROCESS_PSA_ADDR_START  0x2000000
#define OCXL_AFU_PER_PPROCESS_PSA_ADDR_END    0x3FFFFFF
#define OCXL_AFU_PER_PPROCESS_PSA_LENGTH      0x10000

#define OCXL_DVSEC_TEMPL_VERSION         0x0
#define OCXL_DVSEC_TEMPL_NAME            0x4
#define OCXL_DVSEC_TEMPL_AFU_VERSION     0x1C
#define OCXL_DVSEC_TEMPL_MMIO_GLOBAL     0x20
#define OCXL_DVSEC_TEMPL_MMIO_GLOBAL_SZ  0x28
#define OCXL_DVSEC_TEMPL_MMIO_PP         0x30
#define OCXL_DVSEC_TEMPL_MMIO_PP_SZ      0x38
#define OCXL_DVSEC_TEMPL_MEM_SZ          0x3C
#define OCXL_DVSEC_TEMPL_WWID            0x40

#define TEMPL_LEN               0x58
uint8_t *afu_info_id;

#define MEMCPY_WE_CMD_VALID	(0x1 << 0)
#define MEMCPY_WE_CMD_WRAP	(0x1 << 1)
#define MEMCPY_WE_CMD_COPY		0
#define MEMCPY_WE_CMD_IRQ		1
#define MEMCPY_WE_CMD_STOP		2
#define MEMCPY_WE_CMD_WAKE_HOST_THREAD	3
#define MEMCPY_WE_CMD_INCREMENT		4
#define MEMCPY_WE_CMD_ATOMIC		5
#define MEMCPY_WE_CMD_TRANSLATE_TOUCH	6

#define SPA_XSL_TF          (1ull << (63-3))  /* Translation fault */
#define SPA_XSL_S           (1ull << (63-38)) /* Store operation */

#define DEBUG
#ifdef DEBUG
    #define pr_debug(...) fprintf(stderr, __VA_ARGS__)
#else
    #define pr_debug(...) do { } while (0)
#endif

#define pr_log(...) fprintf(stderr, __VA_ARGS__)

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

struct data_t {
    sPAPRCAPIDeviceState *s;
    int pasid;
};

/* temporarly solution */
int irq_index = 0;

static inline int get_pasid(hwaddr addr) {
    return ((addr - OCXL_AFU_PER_PPROCESS_PSA_ADDR_START)/OCXL_AFU_PER_PPROCESS_PSA_LENGTH);
}

static target_ulong h_irq_info(PowerPCCPU *cpu,
                               sPAPRMachineState *spapr,
                               target_ulong opcode,
                               target_ulong *args)
{
    sPAPRCAPIDeviceState *s;
    PCIDevice *pdev;
    target_ulong buid = args[0];
    target_ulong config_addr = args[1];
    target_ulong afu_hwirq = args[2];

    pr_debug("%s: buid: %#lx, config_addr: %#lx (afu_hwirq: "
             "%#lx - index: %d)\n",
             __func__, buid, config_addr, afu_hwirq, irq_index);

    pdev = spapr_pci_find_dev(spapr, buid, config_addr);
    if (!pdev)
        return H_PARAMETER;

    s = SPAPR_CAPI_DEVICE(pdev);

    s->afu_hwirq[irq_index++] = afu_hwirq;
    if (irq_index == MAX_AFU_IRQ)
        irq_index = 0;

    return H_SUCCESS;
}

static target_ulong h_attach_pe(PowerPCCPU *cpu,
                                sPAPRMachineState *spapr,
                                target_ulong opcode,
                                target_ulong *args)
{
    sPAPRCAPIDeviceState *s;
    PCIDevice *pdev;
    target_ulong buid = args[0];
    target_ulong config_addr = args[1];
    target_ulong pasid = args[2];
    target_ulong pidr = args[3];
    target_ulong xsl_hwirq = args[4];

    pr_debug("%s: buid: %#lx, config_addr: %#lx (pasid: %#lx, "
             "pidr: %#lx), xsl_hwirq: %#lx\n",
             __func__, buid, config_addr, pasid, pidr, xsl_hwirq);

    pdev = spapr_pci_find_dev(spapr, buid, config_addr);
    if (!pdev)
        return H_PARAMETER;

    s = SPAPR_CAPI_DEVICE(pdev);

    if (pasid > PASID_MAX)
        return H_PARAMETER;

    s->mcmd[pasid].pidr = pidr;
    s->xsl_hwirq = xsl_hwirq;

    return H_SUCCESS;
}

static target_ulong h_read_xsl_regs(PowerPCCPU *cpu,
                                    sPAPRMachineState *spapr,
                                    target_ulong opcode,
                                    target_ulong *args)
{
    sPAPRCAPIDeviceState *s;
    PCIDevice *pdev;
    target_ulong buid = args[0];
    target_ulong config_addr = args[1];

    pdev = spapr_pci_find_dev(spapr, buid, config_addr);
    if (!pdev)
        return H_PARAMETER;

    s = SPAPR_CAPI_DEVICE(pdev);
    args[0] = s->dsisr;
    args[1] = s->dar;
    args[2] = s->pe_handle;

    pr_debug("%s: dsisr: %#lx, dar: %#lx, pe_handle: %#lx\n",
             __func__, args[0], args[1], args[2]);

    return H_SUCCESS;
}

static void *memcopy_status_t(void *arg)
{
    sPAPRMachineState *spapr = SPAPR_MACHINE(qdev_get_machine());
    struct data_t *data = (struct data_t *)arg;
    struct sPAPRCAPIDeviceState *s = data->s;
    struct timeval test_timeout, temp;
    struct memcpy_work_element mwe;
    struct memcopy_cmd *mcmd;
    uint8_t cmd;
    hwaddr wed, src, dst;
    char *buf;
    int pasid;
    uintptr_t rc = 0;

    pasid = data->pasid;
    if (pasid > PASID_MAX) {
        rc = -EINVAL;
        goto out;
    }

    /* start timeout */
    temp.tv_sec = 5;
    temp.tv_usec = 0;
    gettimeofday(&test_timeout, NULL);
    timeradd(&test_timeout, &temp, &test_timeout);

    /* get physical address */
    mcmd = &s->mcmd[pasid];
    wed = ppc_radix64_eaddr_to_hwaddr(mcmd->wed, mcmd->pidr);

    pr_debug("%s - (pasid: %d) WED EA: %#lx GPA: %#lx, context.id: %#x\n",
             __func__, pasid, mcmd->wed, wed, mcmd->pidr);

    for (;; gettimeofday(&temp, NULL)) {
        if (timercmp(&temp, &test_timeout, >)) {
            if (!mcmd->status)
                pr_log("%s - (pasid: %d) timeout polling for completion\n",
                       __func__, pasid);
            break;
        }

        /* retrieve the command */
        cpu_physical_memory_read(wed, &mwe, sizeof(mwe));

        /* wait for the validation of the command
         * [0] - Valid
         * [1] - Wrap
         * [7:2] - Cmd[5:0]
         */
        if ((mwe.cmd & 0x01) == MEMCPY_WE_CMD_VALID) {
            cmd = (mwe.cmd & 0xFC) >> 2;

            switch (cmd) {
            case MEMCPY_WE_CMD_COPY:
                src = ppc_radix64_eaddr_to_hwaddr(mwe.src, mcmd->pidr);
                dst = ppc_radix64_eaddr_to_hwaddr(mwe.dst, mcmd->pidr);

                /* TODO: we should have read/write functions that automatically
                 *       trigger page faults.
                 */
                /* only one page fault handled. Other threads have to wait */
                if (dst == -1) {
                    if (!s->dsisr) {
                        /* First solution: Guest, through a hcall requests
                         * fault information
                         */
                        s->dsisr = SPA_XSL_TF | SPA_XSL_S;
                        s->dar = mwe.dst;
                        s->pe_handle = pasid;

                        /* Second solution: As we have for native environment, we could use
                         *                  allocated buffer
                                stb_phys(&address_space_memory, disr, (uint32_t)(SPA_XSL_TF | SPA_XSL_S));
                                stb_phys(&address_space_memory, dar, mwe.dst);
                                stb_phys(&address_space_memory, pe_handle, pasid);
                         */
                
                        qemu_irq_pulse(spapr_qirq(spapr, s->xsl_hwirq));
                        pr_debug("%s - MEMCPY_WE_CMD_COPY - fault at %#lx (xsl_hwirq: %d)\n",
                                 __func__, mwe.dst, s->xsl_hwirq);
                    }
                    /* wait for the page fault handled */
                    continue;
                }

                /* the page fault has been handled. Let's the other
                 * threads playing this that
                 */
                s->dsisr = 0;
                s->dar = 0;
                s->pe_handle = 0;

                /* copy data from src to dst */
                pr_debug("%s - MEMCPY_WE_CMD_COPY - copy %d bytes from "
                         "%#lx (PA %#lx) to %#lx (PA %#lx)\n",
                        __func__, mwe.length, mwe.src, src, mwe.dst, dst);
                buf = g_malloc(mwe.length);
                cpu_physical_memory_read(src, buf, mwe.length);
                cpu_physical_memory_write(dst, buf, mwe.length);
                g_free(buf);

                stb_phys(&address_space_memory, wed + 1, 0x1); /* write status */
                mcmd->status = 0x3; /* Process Element has been terminated/Removed */
                irq_index = 0;
                break;

            case MEMCPY_WE_CMD_IRQ:
                /* raise an interrupt */
                /* afu_irq_handle = le64toh(mwe.src);*/

                pr_debug("%s - MEMCPY_WE_CMD_IRQ - irq hwirq: %d\n",
                         __func__, s->afu_hwirq[0]);

                qemu_irq_pulse(spapr_qirq(spapr, s->afu_hwirq[0]));

                stb_phys(&address_space_memory, wed + 1, 0x1); /* write status */
                mcmd->status = 0x3; /* Process Element has been terminated/Removed */
                break;
            default:
                /* no more command */
                goto out;
            }

            /* next command */
            mcmd->wed += sizeof(struct memcpy_work_element);
            wed = ppc_radix64_eaddr_to_hwaddr(mcmd->wed, mcmd->pidr);
        } else {
            if (mcmd->status) {
                /* no more command */
                goto out;
            }
        }
    }
out:
    g_free(data);
    return ((void *)rc);
}

static uint64_t capi_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    sPAPRCAPIDeviceState *s = opaque;
    struct memcopy_cmd *mcmd;
    int pasid;

    pr_debug("%s - addr: 0x%lx,  size: 0x%x\n", __func__, addr, size);

    if ((addr >= OCXL_AFU_PER_PPROCESS_PSA_ADDR_START) &&
        (addr <= OCXL_AFU_PER_PPROCESS_PSA_ADDR_END))
    {
        pasid = get_pasid(addr);
        mcmd = &s->mcmd[pasid];

        switch (addr & 0xFF) {
        case 0x10: /* Process Status Register 64bits */
            return mcmd->status;
        break;
        case 0x14: 
        break;
        }
    }
    return 0;
}

static void capi_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned width)
{
    sPAPRCAPIDeviceState *s = opaque;
    struct data_t *data;
    pthread_t thr;
    int pasid;

    pr_debug("%s - addr: 0x%lx, val: 0x%lx\n", __func__, addr, val);

    if ((addr >= OCXL_AFU_PER_PPROCESS_PSA_ADDR_START) &&
        (addr <= OCXL_AFU_PER_PPROCESS_PSA_ADDR_END))
    {
        pasid = get_pasid(addr);
        data = g_malloc(sizeof (struct data_t));

        /* WED Register (x0000)
         *     [63:12] Base EA of the start of the work element queue.
         */
        switch (addr & 0xFF) {
        case 0x0:
            s->mcmd[pasid].wed = val & ~0xfff;
        break;
        case 0x4:
            s->mcmd[pasid].wed += (val << 32);
            s->mcmd[pasid].status = 0x0; /* Process Valid - Process has been added
                                          * to AFU and not Terminated/Removed */

            /* start a thread to update the memcopy status */
            data->s = s;
            data->pasid = pasid;
            pthread_create(&thr, NULL, memcopy_status_t, data);
        break;
        case 0x28:
            s->mcmd[pasid].err_irq_ea = val & ~0xfff;
        break;
        case 0x2C:
            s->mcmd[pasid].err_irq_ea += (val << 32);
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
            .min_access_size = 8,
            .max_access_size = 8,
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
    if (afu_info_id)
        g_free(afu_info_id);
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

        if (offset >= OCXL_DVSEC_TEMPL_WWID)
            return 0;

        if (!afu_info_id) {
                afu_info_id = g_malloc(TEMPL_LEN);
                *(uint32_t *)&afu_info_id[0x0] = 0x00580005;
                *(uint32_t *)&afu_info_id[0x4] = 0x2c4d4249;
                *(uint32_t *)&afu_info_id[0x8] = 0x434d454d;
                *(uint32_t *)&afu_info_id[0xC] = 0x00335950;
                *(uint32_t *)&afu_info_id[0x10] = 0x00000000;
                *(uint32_t *)&afu_info_id[0x14] = 0x00000000;
                *(uint32_t *)&afu_info_id[0x18] = 0x00000000;
                *(uint32_t *)&afu_info_id[0x1C] = 0x01002401;
                *(uint32_t *)&afu_info_id[0x20] = 0x00000000;
                *(uint32_t *)&afu_info_id[0x24] = 0x00000000;
                *(uint32_t *)&afu_info_id[0x28] = 0x02000000;
                *(uint32_t *)&afu_info_id[0x2C] = 0x00000000;
                *(uint32_t *)&afu_info_id[0x30] = 0x02000000;
                *(uint32_t *)&afu_info_id[0x34] = 0x00000000;
                *(uint32_t *)&afu_info_id[0x38] = 0x00010000;
                *(uint32_t *)&afu_info_id[0x3C] = 0x0000001a;
                *(uint32_t *)&afu_info_id[0x40] = 0x00000000;
                *(uint32_t *)&afu_info_id[0x44] = 0x00000000;
                *(uint32_t *)&afu_info_id[0x48] = 0x00000000;
                *(uint32_t *)&afu_info_id[0x4C] = 0x00000000;
                *(uint32_t *)&afu_info_id[0x50] = 0x00000000;
                *(uint32_t *)&afu_info_id[0x54] = 0x00000000;
        }
        val = (*(uint32_t *)(afu_info_id + offset));

    } else
        val = pci_default_read_config(pdev, addr, l);

    return val;
}

static void spapr_capi_write_config(PCIDevice *pdev, uint32_t addr,
                                    uint32_t val, int l)
{
    sPAPRCAPIDeviceState *s = SPAPR_CAPI_DEVICE(pdev);

    if (addr == s->dvsec_afu_info_id + OCXL_DVSEC_AFU_INFO_OFF) {
        val |= 1<<31; /* Valid bit */
    }

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

    /* hcall */
    spapr_register_hypercall(KVMPPC_H_IRQ_INFO, h_irq_info);
    spapr_register_hypercall(KVMPPC_H_ATTACH_PE, h_attach_pe);
    spapr_register_hypercall(KVMPPC_H_READ_XSL_REGS, h_read_xsl_regs);
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
