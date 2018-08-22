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
#include "hw/ppc/spapr_capi.h"

/* from kernel header <misc/ocxl-config.h> */
#define OCXL_EXT_CAP_ID_PASID                 0x1B
#define OCXL_EXT_CAP_ID_DVSEC                 0x23

#define OCXL_PASID_MAX_WIDTH           	      0x4

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

static uint64_t capi_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    fprintf(stderr, "capi_mmio_read 0x%x: 0x%x\n", (unsigned) addr, (unsigned) size);
    return 0;
}

static void capi_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned width)
{
    fprintf(stderr, "capi_mmio_write\n");
}

static void spapr_capi_device_realize(PCIDevice *pdev, Error **errp)
{
    sPAPRCAPIDeviceState *s = SPAPR_CAPI_DEVICE(pdev);
    uint16_t offset, next_offset, cap_size;

    static const MemoryRegionOps capi_mmio_ops = {
        .read = capi_mmio_read,
        .write = capi_mmio_write,
        .endianness = DEVICE_BIG_ENDIAN,
        .impl = {
            .min_access_size = 1,
            .max_access_size = 1,
        }
    };

    /* Configuration Space Header */
    pci_set_byte(pdev->config + PCI_COMMAND, PCI_COMMAND_MEMORY);
    pci_set_word(pdev->config + PCI_STATUS, 0x1000);
    pci_set_byte(pdev->config + PCI_REVISION_ID, 0x00);
    pci_set_byte(pdev->config + PCI_HEADER_TYPE, 0x80);

    /* mmio BAR 0 - 64MB*/
    memory_region_init_io(&s->mmio, OBJECT(s), &capi_mmio_ops,
                          s, "capi-mmio", 0x100000uLL);

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

    /* Designated Vendor Specific Extended Capabilities - AFU Control */
    offset = next_offset;
    next_offset = 0;
    pcie_add_capability(pdev, OCXL_EXT_CAP_ID_DVSEC, 0x1, offset, cap_size);
    pci_set_long(pdev->config + offset + OCXL_DVSEC_VENDOR_OFFSET, PCI_VENDOR_ID_IBM);
    pci_set_long(pdev->config + offset + OCXL_DVSEC_ID_OFFSET, OCXL_DVSEC_AFU_CTRL_ID);
    pci_set_byte(pdev->config + offset + OCXL_DVSEC_AFU_CTRL_ENABLE, 0x01);
    pci_set_byte(pdev->config + offset + OCXL_DVSEC_AFU_CTRL_PASID_SUP, 0x9);
    pci_set_byte(pdev->config + offset + OCXL_DVSEC_AFU_CTRL_ACTAG_SUP, 0x20);
}

static void spapr_capi_device_exit(PCIDevice *pdev)
{
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
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "sPAPR CAPI device";
    dc->user_creatable = true;
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
