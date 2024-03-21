// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/sizes.h>
#include <asm/arch/sysmap-ipq5332.h>

#ifndef __ASSEMBLY__
#include <compiler.h>
extern uint32_t g_board_machid;
#endif

#define CONFIG_HAS_CUSTOM_SYS_INIT_SP_ADDR
#define CONFIG_CUSTOM_SYS_INIT_SP_ADDR         	(CONFIG_TEXT_BASE -	\
						CONFIG_SYS_MALLOC_LEN - \
						CONFIG_ENV_SIZE -	\
						GENERATED_GBL_DATA_SIZE)

#define CONFIG_SYS_BAUDRATE_TABLE		{ 115200, 230400, 	\
							460800, 921600 }

#define CFG_SYS_HZ_CLOCK			24000000
#define CFG_SYS_SDRAM_BASE			0x40000000
#define KERNEL_START_ADDR                   	CFG_SYS_SDRAM_BASE
#define BOOT_PARAMS_ADDR                    	(KERNEL_START_ADDR + 0x100)

#define CONFIG_SYS_NONCACHED_MEMORY		(1 << 20)

#define CONFIG_MACH_TYPE			(g_board_machid)

#define PHY_ANEG_TIMEOUT			100
#define FDT_HIGH				0x48500000

#define IPQ5332_UBOOT_END_ADDRESS		CONFIG_TEXT_BASE + \
							CONFIG_TEXT_SIZE
#define IPQ5332_DDR_SIZE			(0x3UL * SZ_1G)
#define IPQ5332_DDR_UPPER_SIZE_MAX		(IPQ5332_DDR_SIZE - \
						(CFG_SYS_SDRAM_BASE - \
						IPQ5332_UBOOT_END_ADDRESS))
#define IPQ5332_DDR_LOWER_SIZE			(CONFIG_TEXT_BASE - \
							CFG_SYS_SDRAM_BASE)
#define ROOT_FS_PART_NAME			"rootfs"
