// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <common.h>
#include <cpu_func.h>
#include <asm/cache.h>
#include <configs/ipq9574.h>
#include <asm/global_data.h>
#include <jffs2/load_kernel.h>
#include <mtd_node.h>
#include <sysreset.h>
#include <linux/psci.h>
#ifdef CONFIG_ARM64
#include <asm/armv8/mmu.h>
#endif

#include "../common/ipq_board.h"

#include <asm/io.h>
#include <linux/delay.h>

#define CMN_BLK_ADDR			0x0009B780
#define FREQUENCY_MASK			0xfffffdf0
#define INTERNAL_48MHZ_CLOCK		0x7
#define PORT_WRAPPER_MAX		0xFF
#define CONFIG_NAME_MAX_LEN		128

DECLARE_GLOBAL_DATA_PTR;

#if CONFIG_FDT_FIXUP_PARTITIONS
struct node_info ipq_fnodes[] = {
	{ "n25q128a11", MTD_DEV_TYPE_NOR},
	{ "micron,n25q128a11", MTD_DEV_TYPE_NOR},
	{ "qcom,ipq9574-nand", MTD_DEV_TYPE_NAND},
};

int ipq_fnode_entires = ARRAY_SIZE(ipq_fnodes);

struct node_info * fnodes = ipq_fnodes ;
int * fnode_entires = &ipq_fnode_entires;
#endif

#ifdef CONFIG_DTB_RESELECT
struct machid_dts_map machid_dts[] = {
	{ MACH_TYPE_IPQ9574_RDP417, "ipq9574-rdp417"},
	{ MACH_TYPE_IPQ9574_RDP418, "ipq9574-rdp418"},
	{ MACH_TYPE_IPQ9574_RDP418_EMMC, "ipq9574-rdp418"},
	{ MACH_TYPE_IPQ9574_RDP437, "ipq9574-rdp437"},
	{ MACH_TYPE_IPQ9574_RDP433, "ipq9574-rdp433"},
	{ MACH_TYPE_IPQ9574_RDP449, "ipq9574-rdp418" },
	{ MACH_TYPE_IPQ9574_RDP433_MHT_PHY, "ipq9574-rdp433-mht-phy"},
	{ MACH_TYPE_IPQ9574_RDP453, "ipq9574-rdp453"},
	{ MACH_TYPE_IPQ9574_RDP454, "ipq9574-rdp454"},
	{ MACH_TYPE_IPQ9574_RDP433_MHT_SWT, "ipq9574-rdp433-mht-switch"},
	{ MACH_TYPE_IPQ9574_RDP467, "ipq9574-rdp459" },
	{ MACH_TYPE_IPQ9574_RDP455, "ipq9574-rdp433" },
	{ MACH_TYPE_IPQ9574_RDP459, "ipq9574-rdp459"},
	{ MACH_TYPE_IPQ9574_RDP457, "ipq9574-rdp418" },
	{ MACH_TYPE_IPQ9574_RDP456, "ipq9574-rdp459" },
	{ MACH_TYPE_IPQ9574_RDP469, "ipq9574-rdp469"},
	{ MACH_TYPE_IPQ9574_RDP461, "ipq9574-rdp461"},
	{ MACH_TYPE_IPQ9574_DB_AL01_C1, "ipq9574-db-al01-c1"},
	{ MACH_TYPE_IPQ9574_DB_AL01_C2, "ipq9574-db-al01-c2"},
	{ MACH_TYPE_IPQ9574_DB_AL01_C3, "ipq9574-db-al01-c3"},
	{ MACH_TYPE_IPQ9574_DB_AL02_C1, "ipq9574-db-al02-c1"},
	{ MACH_TYPE_IPQ9574_DB_AL02_C2, "ipq9574-db-al02-c2"},
	{ MACH_TYPE_IPQ9574_DB_AL02_C3, "ipq9574-db-al02-c3"},
};

int machid_dts_nos = ARRAY_SIZE(machid_dts);

struct machid_dts_map * machid_dts_info = machid_dts;
int * machid_dts_entries = &machid_dts_nos;
#endif /* CONFIG_DTB_RESELECT */

struct dumpinfo_t dumpinfo_n[] = {
	/* TZ stores the DDR physical address at which it stores the
	 * APSS regs, UTCM copy dump. We will have the TZ IMEM
	 * IMEM Addr at which the DDR physical address is stored as
	 * the start
	 *     --------------------
         *     |  DDR phy (start) | ----> ------------------------
         *     --------------------       | APSS regsave (8k)    |
         *                                ------------------------
         *                                |                      |
	 *                                | 	 UTCM copy	 |
         *                                |        (192k)        |
	 *                                |                      |
         *                                ------------------------
	 */

	{ "EBICS0.BIN", 0x40000000, 0x40000000, 0 },
	{ "CODERAM.BIN", 0x00200000, 0x00028000, 0 },
	{ "DATARAM.BIN", 0x00290000, 0x00014000, 0 },
	{ "MSGRAM.BIN", 0x00060000, 0x00006000, 1 },
	{ "IMEM.BIN", 0x08600000, 0x00001000, 0 },
};
int dump_entries_n = ARRAY_SIZE(dumpinfo_n);

struct dumpinfo_t * dumpinfo = dumpinfo_n;
int * dump_entries = &dump_entries_n;

void reset_cpu(void)
{
	reset_crashdump();
	psci_sys_reset(SYSRESET_COLD);
	return;
}

int print_cpuinfo(void)
{
        return 0;
}

void lowlevel_init(void)
{
	return;
}

void ipq_uboot_fdt_fixup(uint32_t machid)
{
	int ret, len = 0, config_nos = 0;
	char config[CONFIG_NAME_MAX_LEN];
	char *config_list[6] = { NULL };

	switch (machid)
	{
		case MACH_TYPE_IPQ9574_RDP449:
			config_list[config_nos++] = "config@al02-c6";
			config_list[config_nos++] = "config@rdp449";
			config_list[config_nos++] = "config-rdp449";
			break;
		case MACH_TYPE_IPQ9574_RDP467:
			config_list[config_nos++] = "config@al02-c11";
			config_list[config_nos++] = "config@rdp467";
			config_list[config_nos++] = "config-rdp467";
			break;
		case MACH_TYPE_IPQ9574_RDP455:
			config_list[config_nos++] = "config@al02-c12";
			config_list[config_nos++] = "config@rdp455";
			config_list[config_nos++] = "config-rdp455";
			break;
		case MACH_TYPE_IPQ9574_RDP457:
			config_list[config_nos++] = "config@al02-c15";
			config_list[config_nos++] = "config@rdp457";
			config_list[config_nos++] = "config-rdp457";
			break;
		case MACH_TYPE_IPQ9574_RDP456:
			config_list[config_nos++] = "config@al02-c16";
			config_list[config_nos++] = "config@rdp456";
			config_list[config_nos++] = "config-rdp456";
			break;
	}

	if (config_nos)
	{
		while (config_nos--) {
			strlcpy(&config[len], config_list[config_nos],
					CONFIG_NAME_MAX_LEN - len);
			len += strnlen(config_list[config_nos],
					CONFIG_NAME_MAX_LEN) + 1;
			if (len > CONFIG_NAME_MAX_LEN) {
				printf("skipping uboot fdt fixup err: "
						"config name len overflow\n");
				return;
			}
		}

		/*
		 * Open in place with a new length.
		*/
		ret = fdt_open_into(gd->fdt_blob, (void *)gd->fdt_blob,
				fdt_totalsize(gd->fdt_blob) + len);
		if (ret)
			 printf("uboot-fdt-fixup: Cannot expand FDT: %s\n",
					 fdt_strerror(ret));

		ret = fdt_setprop((void *)gd->fdt_blob, 0, "config_name",
				config, len);
		if (ret)
			printf("uboot-fdt-fixup: unable to set "
					"config_name(%d)\n", ret);
	}
	return;
}

void ipq_config_cmn_clock(void)
{
	unsigned int reg_val;
	/*
	 * Init CMN clock for ethernet
	 */
	reg_val = readl(CMN_BLK_ADDR + 4);
	reg_val = (reg_val & FREQUENCY_MASK) | INTERNAL_48MHZ_CLOCK;
	writel(reg_val, CMN_BLK_ADDR + 0x4);
	reg_val = readl(CMN_BLK_ADDR);
	reg_val = reg_val | 0x40;
	writel(reg_val, CMN_BLK_ADDR);
	mdelay(1);
	reg_val = reg_val & (~0x40);
	writel(reg_val, CMN_BLK_ADDR);
	mdelay(1);
	writel(0xbf, CMN_BLK_ADDR);
	mdelay(1);
	writel(0xff, CMN_BLK_ADDR);
	mdelay(1);
}

#ifdef CONFIG_ARM64
/*
 * Set XN (PTE_BLOCK_PXN | PTE_BLOCK_UXN)bit for all dram regions
 * and Peripheral block except uboot code region
 */
static struct mm_region ipq9574_mem_map[] = {
	{
		/* Peripheral block */
		.virt = 0x0UL,
		.phys = 0x0UL,
		.size = CFG_SYS_SDRAM_BASE,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN

	}, {
		/* DDR region upto u-boot CONFIG_TEXT_BASE */
		.virt = CFG_SYS_SDRAM_BASE,
		.phys = CFG_SYS_SDRAM_BASE,
		.size = CONFIG_TEXT_BASE - CFG_SYS_SDRAM_BASE,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	}, {
		/* DDR region U-boot text base */
		.virt = CONFIG_TEXT_BASE,
		.phys = CONFIG_TEXT_BASE,
		.size = CONFIG_TEXT_SIZE,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		/*
		 * DDR region after u-boot text base
		 * added dummy 0xBAD0FF5EUL,
		 * will update the actual DDR limit
		 */
		.virt = CONFIG_TEXT_BASE + CONFIG_TEXT_SIZE,
		.phys = CONFIG_TEXT_BASE + CONFIG_TEXT_SIZE,
		.size = 0xBAD0FF5EUL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	}, {
		/* List terminator */
		0,
	}
};

struct mm_region *mem_map = ipq9574_mem_map;
#endif
