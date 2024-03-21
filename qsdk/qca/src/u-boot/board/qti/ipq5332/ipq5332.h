// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _IPQ5332_H_
#define _IPQ5332_H_

#include <configs/ipq5332.h>
#include <asm/u-boot.h>

typedef enum {
	SMEM_SPINLOCK_ARRAY = 7,
	SMEM_AARM_PARTITION_TABLE = 9,
	SMEM_HW_SW_BUILD_ID = 137,
	SMEM_USABLE_RAM_PARTITION_TABLE = 402,
	SMEM_POWER_ON_STATUS_INFO = 403,
	SMEM_MACHID_INFO_LOCATION = 425,
	SMEM_IMAGE_VERSION_TABLE = 469,
	SMEM_BOOT_FLASH_TYPE = 498,
	SMEM_BOOT_FLASH_INDEX = 499,
	SMEM_BOOT_FLASH_CHIP_SELECT = 500,
	SMEM_BOOT_FLASH_BLOCK_SIZE = 501,
	SMEM_BOOT_FLASH_DENSITY = 502,
	SMEM_BOOT_DUALPARTINFO = 503,
	SMEM_PARTITION_TABLE_OFFSET = 504,
	SMEM_SPI_FLASH_ADDR_LEN = 505,
	SMEM_FIRST_VALID_TYPE = SMEM_SPINLOCK_ARRAY,
	SMEM_LAST_VALID_TYPE = SMEM_SPI_FLASH_ADDR_LEN,
	SMEM_MAX_SIZE = SMEM_SPI_FLASH_ADDR_LEN + 1,
} smem_mem_type_t;

/* MACH IDs for various RDPs */
#define MACH_TYPE_IPQ5332_RDP468	0x8060000
#define MACH_TYPE_IPQ5332_RDP441	0x8060001
#define MACH_TYPE_IPQ5332_RDP442	0x8060002
#define MACH_TYPE_IPQ5332_RDP446	0x8060004
#define MACH_TYPE_IPQ5332_RDP474	0x8060006
#define MACH_TYPE_IPQ5332_RDP472	0x8060101
#define MACH_TYPE_IPQ5332_DB_MI01_1	0x1060001
#define MACH_TYPE_IPQ5332_DB_MI02_1	0x1060003
#define MACH_TYPE_IPQ5332_DB_MI03_1	0x1060002

/* Crashdump Magic registers & values */
#define TCSR_BOOT_MISC_REG			((u32*)0x193D100)

#define DLOAD_MAGIC_COOKIE			0x10
#define DLOAD_DISABLED				0x40
#define DLOAD_ENABLE				BIT(4)
#define DLOAD_DISABLE				(~BIT(4))
#define CRASHDUMP_RESET				BIT(11)

/* DT Fixup nodes */
#define LINUX_5_4_NAND_DTS_NODE		"/soc/nand@79b0000/"
#define LINUX_5_4_MMC_DTS_NODE		"/soc/sdhci@7804000/"
#define LINUX_5_4_USB_DTS_NODE		"/soc/usb3@8A00000/dwc3@8A00000/"
#define LINUX_5_4_USB_DR_MODE_FIXUP	"/soc/usb3@8A00000/dwc3@8A00000%dr_mode%?peripheral"
#define LINUX_5_4_USB_MAX_SPEED_FIXUP	"/soc/usb3@8A00000/dwc3@8A00000%maximum-speed%?high-speed"
#define LINUX_5_4_DLOAD_DTS_NODE	"/qti,scm_restart_reason/"

#define LINUX_6_1_NAND_DTS_NODE		"/soc@0/nand@79b0000/"
#define LINUX_6_1_MMC_DTS_NODE		"/soc@0/mmc@7804000/"
#define LINUX_6_1_USB_DTS_NODE		"/soc@0/usb3@8a00000/dwc3@8a00000/"
#define LINUX_6_1_USB_DR_MODE_FIXUP	"/soc@0/usb3@8a00000/dwc3@8a00000%dr_mode%?peripheral"
#define LINUX_6_1_USB_MAX_SPEED_FIXUP	"/soc@0/usb3@8a00000/dwc3@8a00000%maximum-speed%?high-speed"
#define LINUX_6_1_DLOAD_DTS_NODE	"/firmware/scm/"

#define LINUX_RSVD_MEM_DTS_NODE		"/reserved-memory/"
#define STATUS_OK			"status%?okay"
#define STATUS_DISABLED			"status%?disabled"

#endif /* _IPQ5332_H_ */
