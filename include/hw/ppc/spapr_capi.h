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

#ifndef HW_SPAPR_CAPI_H
#define HW_SPAPR_CAPI_H

#include "hw/pci/pci.h"

#define TYPE_SPAPR_CAPI_DEVICE "spapr-capi-device"
#define SPAPR_CAPI_DEVICE(obj) \
    OBJECT_CHECK(sPAPRCAPIDeviceState, (obj), TYPE_SPAPR_CAPI_DEVICE)

typedef struct sPAPRCAPIDeviceState {
    PCIDevice parent;

    MemoryRegion mmio;
    uint64_t wed;
    uint64_t status;
    int pasid;

    uint16_t dvsec_afu_info_id;
} sPAPRCAPIDeviceState;
#endif
