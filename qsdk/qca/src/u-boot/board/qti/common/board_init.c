// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2013, 2015-2017, 2020 The Linux Foundation. All rights reserved.
 *
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Based on smem.c from lk.
 *
 * Copyright (c) 2009, Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/byteorder.h>
#include <memalign.h>
#include <fdtdec.h>
#include <command.h>
#include <env.h>
#include <env_internal.h>
#include <linux/delay.h>
#include <part.h>
#include <dm.h>
#include <smem.h>
#include <common.h>
#include <lmb.h>
#ifdef CONFIG_PHY_AQUANTIA
#include <u-boot/crc.h>
#include <miiphy.h>
#endif
#ifdef CONFIG_MMC_SDHCI
#include <mmc.h>
#include <sdhci.h>
#endif
#ifdef CONFIG_CMD_NAND
#include <nand.h>
#endif
#ifdef CONFIG_CMD_UBI
#include <ubi_uboot.h>
#endif
#include <net.h>
#include <spi.h>
#include <spi_flash.h>
#ifdef CONFIG_ARM64
#include <asm/armv8/mmu.h>
#endif
#include <cpu_func.h>
#include <asm/cache.h>

#include "ipq_board.h"

DECLARE_GLOBAL_DATA_PTR;

uint32_t g_board_machid;
char g_board_dts[BOARD_DTS_MAX_NAMELEN] = { 0 };

struct udevice *smem;

ipq_smem_flash_info_t ipq_smem_flash_info;
struct smem_ptable *ptable;
socinfo_t ipq_socinfo;

extern int ipq_smem_get_socinfo(void);
extern int part_get_info_efi(struct blk_desc *dev_desc, int part,
		struct disk_partition *info);

void set_ethmac_addr(void);

__weak void ipq_uboot_fdt_fixup(uint32_t machid)
{
	return;
}

__weak void set_flash_secondary_type(uint32_t flash_type)
{
	return;
}

__weak void ipq_config_cmn_clock(void)
{
	return;
}

__weak void board_cache_init(void)
{
	icache_enable();
#if !CONFIG_IS_ENABLED(SYS_DCACHE_OFF)
	dcache_enable();
#endif
}

ipq_smem_flash_info_t * get_ipq_smem_flash_info(void)
{
	return &ipq_smem_flash_info;
}

struct smem_ptable * get_ipq_part_table_info(void)
{
	return ptable;
}

socinfo_t * get_socinfo(void)
{
	return &ipq_socinfo;
}

void *smem_get_item(unsigned int item) {

	int ret = 0;
	struct udevice *smem_tmp;
	const char *name = "smem";
	size_t size;
	unsigned long int reloc_flag = (gd->flags & GD_FLG_RELOC);

	if (reloc_flag == 0)
		ret = uclass_get_device_by_name(UCLASS_SMEM, name, &smem_tmp);
	else if(!smem)
		ret = uclass_get_device_by_name(UCLASS_SMEM, name, &smem);

	if (ret < 0) {
		printf("Failed to find SMEM node. Check device tree %d\n",ret);
		return 0;
	}

	return smem_get(reloc_flag ? smem : smem_tmp, -1, item, &size);
}

#ifdef CONFIG_CMD_NAND
uint32_t get_nand_block_size(uint8_t dev_id)
{
	struct mtd_info *mtd = get_nand_dev_by_index(0);
	return mtd->erasesize;
}
#endif

uint32_t get_part_block_size(struct smem_ptn *p,
					   ipq_smem_flash_info_t *sfi)
{
#ifdef CONFIG_CMD_NAND
        return (part_which_flash(p) == 1) ?
		get_nand_block_size(0)
		: sfi->flash_block_size;
#else
	return sfi->flash_block_size;
#endif
}

/*
 * smem_getpart - retreive partition start and size
 * @part_name: partition name
 * @start: location where the start offset is to be stored
 * @size: location where the size is to be stored
 *
 * Retreive the start offset in blocks and size in blocks, of the
 * specified partition.
 */
int smem_getpart(char *part_name, uint32_t *start, uint32_t *size)
{
	unsigned i;
	ipq_smem_flash_info_t *sfi = &ipq_smem_flash_info;
	struct smem_ptn *p;
	uint32_t bsize;
#ifdef CONFIG_CMD_NAND
	struct mtd_info *mtd = get_nand_dev_by_index(0);
#endif
	if (!ptable)
		return -ENODEV;

	for (i = 0; i < ptable->len; i++) {
		if (!strncmp(ptable->parts[i].name, part_name,
			     SMEM_PTN_NAME_MAX))
			break;
	}
	if (i == ptable->len)
		return -ENOENT;

	p = &ptable->parts[i];
	bsize = get_part_block_size(p, sfi);

	*start = p->start;

	if (p->size == (~0u)) {
		/*
		 * Partition size is 'till end of device', calculate
		 * appropriately
		 */
#ifdef CONFIG_CMD_NAND
		*size = (mtd->size / bsize) - p->start;
#else
		*size = 0;
		bsize = bsize;
#endif
	} else {
		*size = p->size;
	}

	return 0;
}

/*
 * This function should only be used when sfi->flash_type is
 * SMEM_BOOT_SPI_FLASH
 * retrieve the which_flash flag based on partition name.
 * flash_var is 1 if partition is in NAND.
 * flash_var is 0 if partition is in NOR.
 * flash_var is -1 if partition is in EMMC.
 */
unsigned int get_which_flash_param(char *part_name)
{
	int i;
	int flash_var = -1;

	for (i = 0; i < ptable->len; i++) {
		struct smem_ptn *p = &ptable->parts[i];
		if (strcmp(p->name, part_name) == 0)
			flash_var = part_which_flash(p);
	}

	return flash_var;
}

int get_current_board_flash_config(void)
{
	int ret;
	int board_type;

	ret = get_which_flash_param("rootfs");
	if (ret == -1) {
		board_type = SMEM_BOOT_NORPLUSEMMC;
	} else if (ret) {
		board_type = SMEM_BOOT_NORPLUSNAND;
	} else {
		board_type = SMEM_BOOT_SPI_FLASH;
	}

	return board_type;
}
/*
 * get flash block size based on partition name.
 */
static inline uint32_t get_flash_block_size(char *name,
					    ipq_smem_flash_info_t *smem)
{
#ifdef CONFIG_CMD_NAND
	return (get_which_flash_param(name) == 1) ?
		get_nand_block_size(0)
		: smem->flash_block_size;
#else
	return smem->flash_block_size;
#endif
}

void ipq_set_part_entry(char *name, ipq_smem_flash_info_t *smem,
		ipq_part_entry_t *part, uint32_t start, uint32_t size)
{
	uint32_t bsize = get_flash_block_size(name, smem);
	part->offset = ((loff_t)start) * bsize;
	part->size = ((loff_t)size) * bsize;
}

void get_kernel_fs_part_details(void)
{
	int ret, i;
	uint32_t start;         /* block number */
	uint32_t size;          /* no. of blocks */

	ipq_smem_flash_info_t *smem = &ipq_smem_flash_info;

	struct { char *name; ipq_part_entry_t *part; } entries[] = {
		{ "0:HLOS", &smem->hlos },
		{ "rootfs", &smem->rootfs },
	};

	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		ret = smem_getpart(entries[i].name, &start, &size);
		if (ret < 0) {
			ipq_part_entry_t *part = entries[i].part;

			debug("cdp: get part failed for %s\n",
				entries[i].name);
			part->offset = 0xBAD0FF5E;
			part->size = 0xBAD0FF5E;
		} else {
			ipq_set_part_entry(entries[i].name, smem,
					entries[i].part, start, size);
		}
	}

	return;
}

#ifdef CONFIG_CMD_UBI
int init_ubi_part(void)
{
	int ret;
	uint32_t offset;
	uint32_t part_size = 0;
	uint32_t size_block, start_block;
	ipq_smem_flash_info_t *sfi = get_ipq_smem_flash_info();
	struct ubi_device *ubi = ubi_get_device(0);
	char runcmd[128];

	if(ubi == NULL) {
		if (((sfi->flash_type == SMEM_BOOT_NAND_FLASH) ||
			(sfi->flash_type == SMEM_BOOT_QSPI_NAND_FLASH))) {
			ret = smem_getpart(ROOT_FS_PART_NAME,
					&start_block, &size_block);
			if (ret)
				return ret;

			offset = sfi->flash_block_size * start_block;
			part_size = sfi->flash_block_size * size_block;
		} else if (sfi->flash_type == SMEM_BOOT_SPI_FLASH &&
					get_which_flash_param(ROOT_FS_PART_NAME)) {
			ret = getpart_offset_size(ROOT_FS_PART_NAME, &offset,
									&part_size);
			if (ret)
				return ret;
		}

		if (!part_size)
			return -ENOENT;

		snprintf(runcmd, sizeof(runcmd),
			"setenv mtdids nand0=nand0 && "
			"setenv mtdparts mtdparts=nand0:0x%x@0x%x(fs) && "
			"ubi part fs", part_size, offset);

		if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
			return CMD_RET_FAILURE;
	} else
		ubi_put_device(ubi);

	return 0;
}
#endif

#if CONFIG_IS_ENABLED(NAND_QTI)
void board_nand_init(void)
{
	struct udevice *dev;
	int ret;

	ret = uclass_get_device_by_driver(UCLASS_MTD,
					  DM_DRIVER_GET(qti_nand), &dev);
	if (ret && ret != -ENODEV)
		pr_err("Failed to initialize %s. (error %d)\n",
				dev->name, ret);
}
#endif

int board_init(void)
{
	ipq_smem_bootconfig_info_t *ipq_smem_bootconfig_info;
	ipq_smem_flash_info_t *sfi = &ipq_smem_flash_info;
	uint32_t board_type;
	uint32_t *flash_type;
	uint32_t *flash_chip_select;
	uint32_t *primary_mibib;
	uint32_t *flash_index;
	uint32_t *flash_block_size;
	uint32_t *flash_density;

	gd->bd->bi_boot_params = BOOT_PARAMS_ADDR;

	flash_type = smem_get_item(SMEM_BOOT_FLASH_TYPE);
	if (IS_ERR_OR_NULL(flash_type)) {
		debug("Failed to get SMEM item: SMEM_BOOT_FLASH_TYPE\n");
		flash_type = NULL;
	}

	flash_index = smem_get_item(SMEM_BOOT_FLASH_INDEX);
	if (IS_ERR_OR_NULL(flash_index)) {
		debug("Failed to get SMEM item: SMEM_BOOT_FLASH_INDEX\n");
		flash_index = NULL;
	}

	flash_chip_select = smem_get_item(SMEM_BOOT_FLASH_CHIP_SELECT);
	if (IS_ERR_OR_NULL(flash_chip_select)) {
		debug("Failed to get SMEM item: SMEM_BOOT_FLASH_CHIP_SELECT\n");
		flash_chip_select = NULL;
	}

	flash_block_size = smem_get_item(SMEM_BOOT_FLASH_BLOCK_SIZE);
	if (IS_ERR_OR_NULL(flash_block_size)) {
		debug("Failed to get SMEM item: SMEM_BOOT_FLASH_BLOCK_SIZE\n");
		flash_block_size = NULL;
	}

	flash_density = smem_get_item(SMEM_BOOT_FLASH_DENSITY);
	if (IS_ERR_OR_NULL(flash_density)) {
		debug("Failed to get SMEM item: SMEM_BOOT_FLASH_DENSITY\n");
		flash_density = NULL;
	}

	primary_mibib = smem_get_item(SMEM_PARTITION_TABLE_OFFSET);
	if (IS_ERR_OR_NULL(primary_mibib)) {
		debug("Failed to get SMEM item: " \
				"SMEM_PARTITION_TABLE_OFFSET\n");
		primary_mibib = NULL;
	}

	ipq_smem_bootconfig_info = smem_get_item(SMEM_BOOT_DUALPARTINFO);
	if (IS_ERR_OR_NULL(ipq_smem_bootconfig_info) ||
		(ipq_smem_bootconfig_info->magic_start !=
			_SMEM_DUAL_BOOTINFO_MAGIC_START) ||
		(ipq_smem_bootconfig_info->magic_end !=
			_SMEM_DUAL_BOOTINFO_MAGIC_END)) {
		debug("Failed to get SMEM item: SMEM_BOOT_DUALPARTINFO\n");
		ipq_smem_bootconfig_info = NULL;
	}

	sfi->flash_type = (!flash_type ? SMEM_BOOT_NO_FLASH : *flash_type);
	sfi->flash_index = (!flash_index ? 0 : *flash_index);
	sfi->flash_chip_select = (!flash_chip_select ? 0 : *flash_chip_select);
	sfi->flash_block_size = (!flash_block_size ? 0: *flash_block_size);
	sfi->flash_density = (!flash_density ? 0 : *flash_density);
	sfi->primary_mibib = (!primary_mibib ? 0 : *primary_mibib);
	sfi->ipq_smem_bootconfig_info = ipq_smem_bootconfig_info;

	switch(sfi->flash_type) {
	case SMEM_BOOT_MMC_FLASH:
	case SMEM_BOOT_NO_FLASH:
		break;
	default:
		ptable = smem_get_item(SMEM_AARM_PARTITION_TABLE);
		if (IS_ERR_OR_NULL(ptable)) {
			debug("Failed to get SMEM item: " \
					"SMEM_AARM_PARTITION_TABLE\n");
			return -ENODEV;
		}

		if (ptable->magic[0] != _SMEM_PTABLE_MAGIC_1 ||
			ptable->magic[1] != _SMEM_PTABLE_MAGIC_2)
			return -ENOMSG;
	}

	board_type = (sfi->flash_type == SMEM_BOOT_SPI_FLASH) ?
			get_current_board_flash_config() :
			sfi->flash_type;

	switch(board_type) {
	case SMEM_BOOT_NORPLUSEMMC:
		sfi->flash_secondary_type = SMEM_BOOT_MMC_FLASH;
		break;
	case SMEM_BOOT_NORPLUSNAND:
		sfi->flash_secondary_type = SMEM_BOOT_QSPI_NAND_FLASH;
		break;
	default:
		break;
	}
	/*
	 * To set SoC specific secondary flash type to
	 * eMMC/NAND device based on the one that is enabled.
	 */
	set_flash_secondary_type(sfi->flash_secondary_type);

	/*
	 * get soc_version, cpu_type, machid
	 */

	ipq_smem_get_socinfo();

#ifdef CONFIG_BOARD_TYPES
	gd->board_type = board_type;
#endif
	return 0;
}

int ipq_smem_get_socinfo()
{
	union ipq_platform *platform_type;

	platform_type = smem_get_item(SMEM_HW_SW_BUILD_ID);
	if (IS_ERR_OR_NULL(platform_type)) {
		debug("Failed to get SMEM item: SMEM_HW_SW_BUILD_ID\n");
		return -ENODEV;
	}

	ipq_socinfo.cpu_type = platform_type->v1.id;
	ipq_socinfo.version = platform_type->v1.version;
	ipq_socinfo.soc_version_major =
				SOCINFO_VERSION_MAJOR(ipq_socinfo.version);
	ipq_socinfo.soc_version_minor =
				SOCINFO_VERSION_MINOR(ipq_socinfo.version);
	ipq_socinfo.machid = g_board_machid;

	return 0;

}

/**
 * mibib_ptable_init - initializes SMEM partition table
 *
 * Initialize partition table from MIBIB.
 */
int mibib_ptable_init(unsigned int* addr)
{
	struct smem_ptable* mib_ptable;
	ipq_smem_flash_info_t *sfi = &ipq_smem_flash_info;

	mib_ptable = (struct smem_ptable*) addr;
	if (mib_ptable->magic[0] != _SMEM_PTABLE_MAGIC_1 ||
		mib_ptable->magic[1] != _SMEM_PTABLE_MAGIC_2)
		return -ENOMSG;

	/* In recovery & mmc boot, ptable will not be initialized.
	 * So, allocate ptable memory in recovery mode.
	 */
	if ((sfi->flash_type == SMEM_BOOT_NO_FLASH) ||
			(sfi->flash_type == SMEM_BOOT_MMC_FLASH)) {
		if (!ptable)
			ptable = malloc(sizeof(struct smem_ptable));
	} else
		debug("smem ptable found: ver: %d len: %d\n",
				ptable->version, ptable->len);

	memcpy(ptable, addr, sizeof(struct smem_ptable));
	return 0;
}

/*
 * This function is called in the very beginning.
 * Retreive the machtype info from SMEM and map the board specific
 * parameters. Shared memory region at Dram address
 * contains the machine id/ board type data polulated by SBL.
 */
int board_early_init_f(void)
{
#ifdef CONFIG_SMEM_VERSION_C
	union ipq_platform *platform_type;

	platform_type = smem_get_item(SMEM_HW_SW_BUILD_ID);
	if (IS_ERR_OR_NULL(platform_type)) {
		debug("Failed to get SMEM item: SMEM_HW_SW_BUILD_ID\n");
		return -ENODEV;
	}

	g_board_machid = ((platform_type->v1.hw_platform << 24) |
			  ((SOCINFO_VERSION_MAJOR(
				platform_type->v1.platform_version)) << 16) |
			  ((SOCINFO_VERSION_MINOR(
				platform_type->v1.platform_version)) << 8) |
			  (platform_type->v1.hw_platform_subtype));
#else
	struct smem_machid_info *machid_info;
	machid_info = smem_get_item(SMEM_MACHID_INFO_LOCATION);
	if (IS_ERR_OR_NULL(machid_info)) {
		debug("Failed to get SMEM item: SMEM_MACHID_INFO_LOCATION\n");
		return -ENODEV;
	}

	g_board_machid = machid_info->machid;
#endif

	return 0;
}

int board_fix_fdt(void *rw_fdt_blob)
{
	ipq_uboot_fdt_fixup(g_board_machid);
	return 0;
}

#ifdef CONFIG_MULTI_DTB_FIT
int board_fit_config_name_match(const char *name)
{
	if (!strcmp(name, g_board_dts)) {
		printf("Booting %s\n", name);
		return 0;
	}

	return -1;
}
#endif /* CONFIG_MULTI_DTB_FIT */

#ifdef CONFIG_DTB_RESELECT
int embedded_dtb_select(void)
{
	int rescan;
	unsigned int i;
	for (i=0; i<*machid_dts_entries; i++)
		if (machid_dts_info[i].machid == g_board_machid)
			strlcpy(g_board_dts, machid_dts_info[i].dts,
					BOARD_DTS_MAX_NAMELEN);

	fdtdec_resetup(&rescan);

	return 0;
}
#endif /* CONFIG_DTB_RESELECT */

int board_late_init(void)
{
	ipq_smem_flash_info_t *sfi = &ipq_smem_flash_info;
	if (sfi->flash_type != SMEM_BOOT_MMC_FLASH) {
		get_kernel_fs_part_details();
	}
#ifdef CONFIG_QTI_NSS_SWITCH
	/*
	 * configure CMN clock for ethernet
	 */
	ipq_config_cmn_clock();
#endif
	/*
	 * setup mac address
	 */
	set_ethmac_addr();
	/*
	 * setup machid
	 */
	env_set_hex("machid", gd->bd->bi_arch_number);

#ifdef CONFIG_PREBOOT
	/*
	 * forceset preboot env to avoid SDI/crashdump path system bootup
	 */
	env_set("preboot", CONFIG_PREBOOT);
#endif

	return 0;
}

int dram_init(void)
{
	int i, ret = CMD_RET_SUCCESS;
	int count = 0;
	struct smem_ram_ptable *ram_ptable;
	struct smem_ram_ptn *p;

	ram_ptable = smem_get_item(SMEM_USABLE_RAM_PARTITION_TABLE);
	if (IS_ERR_OR_NULL(ram_ptable)) {
		debug("Failed to get SMEM item: " \
				"SMEM_USABLE_RAM_PARTITION_TABLE\n");
		ret = -ENODEV;
	}

	gd->ram_size = 0;
	/* Check validy of RAM */
	for (i = 0; i < CONFIG_RAM_NUM_PART_ENTRIES; i++) {
		p = &ram_ptable->parts[i];
		if (p->category == RAM_PARTITION_SDRAM &&
					p->type == RAM_PARTITION_SYS_MEMORY) {
			gd->ram_size += p->size;
			debug("Detected memory bank %u: "
				"start: 0x%llx size: 0x%llx\n",
					count, p->start, p->size);
			count++;
		}
        }

	if (!count) {
		printf("Failed to detect any memory bank\n");
		ret = CMD_RET_FAILURE;
	}

	return ret;
}

void *env_sf_get_env_addr(void)
{
        return NULL;
}

#ifdef CONFIG_MMC_SDHCI
int part_get_info_efi_by_name(const char *name, struct disk_partition *info)
{
	struct blk_desc *mmc_dev;
	int ret = -1;
	int i;

	mmc_dev = blk_get_devnum_by_uclass_id(UCLASS_MMC, 0);

	if (mmc_dev->type == DEV_TYPE_UNKNOWN)
		goto done;

	for (i = 1; i < GPT_ENTRY_NUMBERS; i++) {
		ret = part_get_info_efi(mmc_dev, i, info);
		if (ret != 0) {
			/* no more entries in table */
			goto done;
		}
		if (strcmp(name, (const char *)info->name) == 0) {
			/* matched */
			ret = 0;
			goto done;
		}
	}
done:
	return ret;
}
#endif

enum env_location env_get_location(enum env_operation op, int prio)
{
	int ret = ENVL_NOWHERE;
	uint32_t *flash_type;

	if (prio)
		return ENVL_UNKNOWN;

	flash_type = smem_get_item(SMEM_BOOT_FLASH_TYPE);
	if (IS_ERR_OR_NULL(flash_type))
		return ret;

	if (*flash_type == SMEM_BOOT_SPI_FLASH) {
		ret = ENVL_SPI_FLASH;
	} else if (*flash_type == SMEM_BOOT_MMC_FLASH) {
		ret = ENVL_MMC;
	} else if ((*flash_type == SMEM_BOOT_QSPI_NAND_FLASH) ||
		(*flash_type == SMEM_BOOT_NAND_FLASH)) {
		ret = ENVL_NAND;
	} else { }

	return ret;
}

#ifdef CONFIG_MMC_SDHCI
int mmc_get_env_addr(struct mmc *mmc, int copy, u32 *env_addr)
{
	int ret;
	struct disk_partition disk_info;

	ret = part_get_info_efi_by_name("0:APPSBLENV", &disk_info);

	if (!ret) {
		*env_addr = (u32)disk_info.start * disk_info.blksz;
	}

	return ret;
}
#endif

void board_lmb_reserve(struct lmb *lmb)
{
	if (lmb)
		lmb->reserved.region[0].size = (CONFIG_TEXT_BASE -
				lmb->reserved.region[0].base);
}

#ifdef CONFIG_PHY_AQUANTIA
static int ipq_aquantia_load_memory(struct phy_device *phydev, u32 addr,
				const u8 *data, size_t len)
{
	size_t pos;
	u16 crc = 0, up_crc;

	phy_write(phydev, MDIO_MMD_VEND1, 0x200, BIT(12));
	phy_write(phydev, MDIO_MMD_VEND1, 0x202, addr >> 16);
	phy_write(phydev, MDIO_MMD_VEND1, 0x203, addr & 0xfffc);

	for (pos = 0; pos < len; pos += min(sizeof(u32), len - pos)) {
		u32 word = 0;

		memcpy(&word, &data[pos], min(sizeof(u32), len - pos));

		phy_write(phydev, MDIO_MMD_VEND1, 0x204,
			  (word >> 16));
		phy_write(phydev, MDIO_MMD_VEND1, 0x205,
			  word & 0xffff);

		phy_write(phydev, MDIO_MMD_VEND1, 0x200,
			  BIT(15) | BIT(14));

		/* keep a big endian CRC to match the phy processor */
		word = cpu_to_be32(word);
		crc = crc16_ccitt(crc, (u8 *)&word, sizeof(word));
	}

	up_crc = phy_read(phydev, MDIO_MMD_VEND1, 0x201);
	if (crc != up_crc) {
		printf("%s CRC Mismatch: Calculated 0x%04hx PHY 0x%04hx\n",
		       phydev->dev->name, crc, up_crc);
		return -EINVAL;
	}
	return 0;
}

static int ipq_aquantia_upload_firmware(struct phy_device *phydev,
		uint8_t *addr,	uint32_t file_size)
{
	int ret;
	uint8_t *buf = addr;
	uint32_t primary_header_ptr = 0x00000000;
	uint32_t primary_iram_ptr = 0x00000000;
	uint32_t primary_dram_ptr = 0x00000000;
	uint32_t primary_iram_sz = 0x00000000;
	uint32_t primary_dram_sz = 0x00000000;
	uint32_t phy_img_hdr_off = 0x300;
	uint16_t recorded_ggp8_val, daisy_chain_dis;
	u16 computed_crc, file_crc;

	phy_write(phydev, MDIO_MMD_VEND1, 0x300, 0xdead);
	phy_write(phydev, MDIO_MMD_VEND1, 0x301, 0xbeaf);
	if ((phy_read(phydev, MDIO_MMD_VEND1, 0x300) != 0xdead) &&
			(phy_read(phydev, MDIO_MMD_VEND1, 0x301) != 0xbeaf)) {
		printf("PHY::Scratchpad Read/Write test fail\n");
		ret = -EIO;
		goto exit;
	}

	file_crc = buf[file_size - 2] << 8 | buf[file_size - 1];
	computed_crc = crc16_ccitt(0, buf, file_size -2);
	if (file_crc != computed_crc) {
		printf("CRC check failed on phy fw file\n");
		ret = -EIO;
		goto exit;
	}

	printf("CRC check good on PHY FW (0x%04X)\n", computed_crc);
	daisy_chain_dis = phy_read(phydev, MDIO_MMD_VEND1, 0xc452);
	if (!(daisy_chain_dis & 0x1))
		phy_write(phydev, MDIO_MMD_VEND1, 0xc452, 0x1);

	phy_write(phydev, MDIO_MMD_VEND1, 0xc471, 0x40);
	recorded_ggp8_val = phy_read(phydev, MDIO_MMD_VEND1, 0xc447);
	if ((recorded_ggp8_val & 0x1f) != phydev->addr)
		phy_write(phydev, MDIO_MMD_VEND1, 0xc447, phydev->addr);

	phy_write(phydev, MDIO_MMD_VEND1, 0xc441, 0x4000);
	phy_write(phydev, MDIO_MMD_VEND1, 0xc001, 0x41);

	primary_header_ptr = (((buf[0x9] & 0x0F) << 8) | buf[0x8]) << 12;

	primary_iram_ptr = (buf[primary_header_ptr +
		phy_img_hdr_off + 0x4 + 2] << 16) |
		(buf[primary_header_ptr + phy_img_hdr_off + 0x4 + 1] << 8) |
		buf[primary_header_ptr + phy_img_hdr_off + 0x4];
	primary_iram_sz = (buf[primary_header_ptr +
		phy_img_hdr_off + 0x7 + 2] << 16) |
		(buf[primary_header_ptr + phy_img_hdr_off + 0x7 + 1] << 8) |
		buf[primary_header_ptr + phy_img_hdr_off + 0x7];
        primary_dram_ptr = (buf[primary_header_ptr +
		phy_img_hdr_off + 0xA + 2] << 16) |
		(buf[primary_header_ptr + phy_img_hdr_off + 0xA + 1] << 8) |
		buf[primary_header_ptr + phy_img_hdr_off + 0xA];
        primary_dram_sz = (buf[primary_header_ptr +
		phy_img_hdr_off + 0xD + 2] << 16) |
		(buf[primary_header_ptr + phy_img_hdr_off + 0xD + 1] << 8) |
		buf[primary_header_ptr + phy_img_hdr_off + 0xD];
	primary_iram_ptr += primary_header_ptr;
	primary_dram_ptr += primary_header_ptr;

	phy_write(phydev, MDIO_MMD_VEND1, 0x200, 0x1000);
	phy_write(phydev, MDIO_MMD_VEND1, 0x200, 0x0);
	computed_crc = 0;

	printf("PHYFW:Loading IRAM...........");
	ret = ipq_aquantia_load_memory(phydev, 0x40000000,
			&buf[primary_iram_ptr], primary_iram_sz);
	if (ret < 0)
		goto exit;
	printf("done.\n");

	printf("PHYFW:Loading DRAM..............");
	ret = ipq_aquantia_load_memory(phydev, 0x3ffe0000,
			&buf[primary_dram_ptr], primary_dram_sz);
	if (ret < 0)
		goto exit;
	printf("done.\n");

	phy_write(phydev, MDIO_MMD_VEND1, 0x0, 0x0);
	phy_write(phydev, MDIO_MMD_VEND1, 0xc001, 0x41);
	phy_write(phydev, MDIO_MMD_VEND1, 0xc001, 0x8041);
	mdelay(100);

	phy_write(phydev, MDIO_MMD_VEND1, 0xc001, 0x40);
	mdelay(100);
	printf("PHYFW loading done.\n");
exit:
	return ret;
}

int ipq_aquantia_load_fw(struct phy_device *phydev)
{
	ipq_smem_flash_info_t *sfi = get_ipq_smem_flash_info();
	u8 *fw_load_addr = NULL;
	int ret = 0;
	char runcmd[256];
	mbn_header_t *fwimg_header;
	char * eth_fw_part_name = IPQ_ETH_FW_PART_NAME;
	size_t part_size = 0;

	uint32_t start_blk;		/* starting block */
	uint32_t blk_cnt;		/* no. of blocks */

	ipq_part_entry_t ethphyfw;
	struct blk_desc *desc;
	struct disk_partition disk_info;

	/* check the smem info to see which flash used for booting */
	if ((sfi->flash_type == SMEM_BOOT_NAND_FLASH) ||
	    (sfi->flash_type == SMEM_BOOT_QSPI_NAND_FLASH) ||
	    (sfi->flash_type == SMEM_BOOT_SPI_FLASH)) {

		ret = smem_getpart(eth_fw_part_name, &start_blk, &blk_cnt);
		if (ret < 0) {
			debug("cdp: get part failed for %s\n",
					eth_fw_part_name);
			ret = -ENXIO;
			goto exit;
		} else {
			ipq_set_part_entry(eth_fw_part_name,
					sfi, &ethphyfw, start_blk, blk_cnt);
		}

		part_size = IPQ_ETH_FW_PART_SIZE;
	} else if (sfi->flash_type == SMEM_BOOT_MMC_FLASH) {
		blk_get_device_by_str("mmc", "0", &desc);
		part_get_info_by_name(desc, eth_fw_part_name,
				&disk_info);

		part_size = (((uint)disk_info.size) *
					((uint)disk_info.blksz));
	} else {
		printf("Unsupported BOOT flash type\n");
		ret = -ENXIO;
		goto exit;
	}

	fw_load_addr = (u8 *)malloc_cache_aligned(part_size);

	/* We only need memory equivalent to max size ETHPHYFW
	 * which is currently assumed as 512 KB.
	 */
	if (fw_load_addr == NULL) {
		printf("ETHPHYFW Loading failed, size = %zu\n", part_size);
		ret = -ENOMEM;
		goto exit;
	}

	memset(fw_load_addr, 0, part_size);

	if ((sfi->flash_type == SMEM_BOOT_NAND_FLASH) ||
	    (sfi->flash_type == SMEM_BOOT_QSPI_NAND_FLASH)) {
		snprintf(runcmd, sizeof(runcmd),
			 "nand read 0x%p 0x%llx 0x%llx && ",
			 fw_load_addr, ethphyfw.offset,
			 (long long unsigned int) part_size);

	} else if (sfi->flash_type == SMEM_BOOT_SPI_FLASH) {
		snprintf(runcmd, sizeof(runcmd),
			 "sf probe && " "sf read 0x%p 0x%llx 0x%llx && ",
			 fw_load_addr, ethphyfw.offset,
			 (long long unsigned int) part_size);

	} else if (sfi->flash_type == SMEM_BOOT_MMC_FLASH ) {
		snprintf(runcmd, sizeof(runcmd), "mmc read 0x%p 0x%X 0x%X",
			fw_load_addr, (uint)disk_info.start,
			(uint)disk_info.size);
	}

	debug("%s \n", runcmd);
	if (run_command(runcmd, 0) != 0) {
		ret = -1;
		goto free_nd_exit;
	}

	fwimg_header = (mbn_header_t *)(fw_load_addr);

	if (fwimg_header->image_type == 0x13 &&
			fwimg_header->header_vsn_num == 0x3) {
		ret = ipq_aquantia_upload_firmware(phydev,
				(uint8_t*)((uint32_t)sizeof(mbn_header_t)
				+ fw_load_addr),
				(uint32_t)(fwimg_header->image_size));
		if (ret != 0)
			goto free_nd_exit;
	} else {
		printf("bad magic on ETHPHYFW partition\n");
		ret = -1;
		goto free_nd_exit;
	}

free_nd_exit:
	free(fw_load_addr);
exit:
	return ret;
}
#endif /* CONFIG_PHY_AQUANTIA */

int get_eth_mac_address(uchar *enetaddr, int no_of_macs)
{
	ipq_smem_flash_info_t *sfi = get_ipq_smem_flash_info();
	size_t length = (6 * no_of_macs);
	int ret = 0;
	char *part_name = "0:ART";
#ifdef CONFIG_IPQ_SPI_NOR
	struct spi_flash *flash = NULL;
#endif
	uint32_t start_blk;
	uint32_t blk_cnt;
	ipq_part_entry_t art;
#if defined(CONFIG_MMC_SDHCI) && defined(CONFIG_SYS_MMC_ENV_PART)
	struct mmc *mmc;
	struct blk_desc *desc;
	struct disk_partition disk_info;
	unsigned char *mmc_blks = (unsigned char*) malloc_cache_aligned(512);
	if(mmc_blks == NULL)
		return -ENOMEM;
#endif

	/* check the smem info to see which flash used for booting */
	if ((sfi->flash_type == SMEM_BOOT_NAND_FLASH) ||
	    (sfi->flash_type == SMEM_BOOT_QSPI_NAND_FLASH) ||
	    (sfi->flash_type == SMEM_BOOT_SPI_FLASH)) {
		ret = smem_getpart(part_name, &start_blk, &blk_cnt);
		if (ret < 0) {
			debug("cdp: get part failed for %s\n",
					part_name);
			ret = -ENXIO;
			goto exit;
		} else {
			ipq_set_part_entry(part_name, sfi, &art, start_blk,
						blk_cnt);
		}
	} else if (sfi->flash_type == SMEM_BOOT_MMC_FLASH) {
#if defined(CONFIG_MMC_SDHCI) && defined(CONFIG_SYS_MMC_ENV_PART)
		mmc = find_mmc_device(CONFIG_SYS_MMC_ENV_DEV);
		if (!mmc) {
			printf("Failed to find MMC device \n");
			ret = -ENXIO;
			goto exit;
		}
		desc = mmc_get_blk_desc(mmc);
		if (!desc) {
			printf("Failed to find the desc \n");
			ret = -1;
			goto exit;
		}
		ret = part_get_info_by_name(desc, part_name, &disk_info);
		if (ret == -ENOENT) {
			printf("Failed to find the partition info \n");
			goto exit;
		}
#ifdef CONFIG_BLK
		ret = blk_dread(desc, (uint)disk_info.start, 1, mmc_blks);
#else
		ret = mmc->block_dev.block_read(&mmc->block_dev,
						(uint)disk_info.start,
						1,
						mmc_blks);
#endif
		if(ret < 0) {
			printf("MMC: 0:ART read failed %d\n", ret);
			goto exit;
		}
		memcpy(enetaddr, mmc_blks, length);
		if(mmc_blks)
			free(mmc_blks);
#endif
	} else {
		printf("Unsupported BOOT flash type\n");
		ret = -ENXIO;
		goto exit;
	}
#ifdef CONFIG_IPQ_SPI_NOR
	if (sfi->flash_type == SMEM_BOOT_SPI_FLASH) {
		flash = spi_flash_probe(CONFIG_SF_DEFAULT_BUS,
					CONFIG_SF_DEFAULT_CS,
					CONFIG_SF_DEFAULT_SPEED,
					CONFIG_SF_DEFAULT_MODE);
		if (flash == NULL){
			printf("No SPI flash device found\n");
			ret = -1;
		} else {
			spi_flash_read(flash, art.offset, length, enetaddr);
		}
	}
#endif
#ifdef CONFIG_CMD_NAND
	if ((sfi->flash_type == SMEM_BOOT_NAND_FLASH) ||
		(sfi->flash_type == SMEM_BOOT_QSPI_NAND_FLASH)) {
		nand_read(get_nand_dev_by_index(0),art.offset,
			&length, enetaddr);
	}
#endif
exit:
	return ret;
}

void set_ethmac_addr(void)
{
	int i, ret;
	uchar enetaddr[CONFIG_ETH_MAX_MAC * 6] = { 0 };
	uchar *mac_addr;
	char ethaddr[16] = "ethaddr";
	char mac[64];
	/* Get the MAC address from ART partition */
	ret = get_eth_mac_address(enetaddr, CONFIG_ETH_MAX_MAC);
	for (i = 0; (ret >= 0) && (i < CONFIG_ETH_MAX_MAC); i++) {
		mac_addr = &enetaddr[i * 6];
		if (!is_valid_ethaddr(mac_addr)) {
			printf("MAC%d Address from ART is not valid\n", i);
		} else {
			/*
			 * U-Boot uses these to patch the 'local-mac-address'
			 * dts entry for the ethernet entries, which in turn
			 * will be picked up by the HLOS driver
			 */
			snprintf(mac, sizeof(mac), "%x:%x:%x:%x:%x:%x",
					mac_addr[0], mac_addr[1],
					mac_addr[2], mac_addr[3],
					mac_addr[4], mac_addr[5]);
			env_set(ethaddr, mac);
		}
		snprintf(ethaddr, sizeof(ethaddr), "eth%daddr", (i + 1));
	}
}

void enable_caches(void)
{
#ifdef CONFIG_ARM64
	int i;

	/* Now Update the real DDR size based on Board configuration */
	for (i = 0; mem_map[i].size || mem_map[i].attrs; i++) {
		if (mem_map[i].size == 0xBAD0FF5EUL) {
			mem_map[i].size = gd->ram_top - mem_map[i].virt;
		}
	}
#endif
	board_cache_init();
}

static int do_aqloadfw(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	/*
	 * Firmware load by default
	 * so return success
	 */
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	aq_load_fw, 2, 0, do_aqloadfw,
	"Load firmware to AQ port",
	"phy_addr --> phy address of AQ port\n"
	);
