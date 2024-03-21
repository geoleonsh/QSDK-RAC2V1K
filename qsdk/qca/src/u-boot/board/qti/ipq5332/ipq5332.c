// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <common.h>
#include <cpu_func.h>
#include <asm/cache.h>
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

#define PLL_POWER_ON_AND_RESET			0x9B780
#define PLL_REFERENCE_CLOCK			0x9B784
#define FREQUENCY_MASK				0xfffffdf0
#define INTERNAL_48MHZ_CLOCK			0x7

DECLARE_GLOBAL_DATA_PTR;

#if CONFIG_FDT_FIXUP_PARTITIONS
struct node_info ipq_fnodes[] = {
	{ "n25q128a11", MTD_DEV_TYPE_NOR},
	{ "micron,n25q128a11", MTD_DEV_TYPE_NOR},
	{ "qcom,ipq5332-nand", MTD_DEV_TYPE_NAND},
};

int ipq_fnode_entires = ARRAY_SIZE(ipq_fnodes);

struct node_info * fnodes = ipq_fnodes ;
int * fnode_entires = &ipq_fnode_entires;
#endif

#ifdef CONFIG_DTB_RESELECT
struct machid_dts_map machid_dts[] = {
	{ MACH_TYPE_IPQ5332_RDP468, "ipq5332-rdp468"},
	{ MACH_TYPE_IPQ5332_RDP441, "ipq5332-rdp441"},
	{ MACH_TYPE_IPQ5332_RDP442, "ipq5332-rdp442"},
	{ MACH_TYPE_IPQ5332_RDP446, "ipq5332-rdp446"},
	{ MACH_TYPE_IPQ5332_RDP474, "ipq5332-rdp474"},
	{ MACH_TYPE_IPQ5332_RDP472, "ipq5332-rdp472"},
	{ MACH_TYPE_IPQ5332_DB_MI01_1, "ipq5332-db-mi01.1"},
	{ MACH_TYPE_IPQ5332_DB_MI02_1, "ipq5332-db-mi02.1"},
	{ MACH_TYPE_IPQ5332_DB_MI03_1, "ipq5332-db-mi03.1"},
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

void board_cache_init(void)
{
	ipq_smem_flash_info_t *sfi = get_ipq_smem_flash_info();
	icache_enable();
#if !CONFIG_IS_ENABLED(SYS_DCACHE_OFF)
	/* Disable L2 as TCM in recovery mode */
	if (!sfi->flash_type)
		writel(0x08000000, 0xB110010);

	dcache_enable();
#endif
}

void lowlevel_init(void)
{
	return;
}

void ipq_config_cmn_clock(void)
{
	unsigned int reg_val;
	/*
	 * Init CMN clock for ethernet
	 */
	reg_val = readl(PLL_REFERENCE_CLOCK);
	reg_val = (reg_val & FREQUENCY_MASK) | INTERNAL_48MHZ_CLOCK;
	/*Select clock source*/
	writel(reg_val, PLL_REFERENCE_CLOCK);

	/* Soft reset to calibration clocks */
	reg_val = readl(PLL_POWER_ON_AND_RESET);
	reg_val &= ~BIT(6);
	writel(reg_val, PLL_POWER_ON_AND_RESET);
	mdelay(1);
	reg_val |= BIT(6);
	writel(reg_val, PLL_POWER_ON_AND_RESET);
	mdelay(1);
}

#ifdef CONFIG_ARM64
/*
 * Set XN (PTE_BLOCK_PXN | PTE_BLOCK_UXN)bit for all dram regions
 * and Peripheral block except uboot code region
 */
static struct mm_region ipq5332_mem_map[] = {
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

struct mm_region *mem_map = ipq5332_mem_map;
#endif
