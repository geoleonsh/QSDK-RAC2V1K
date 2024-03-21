/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2018, 2020 The Linux Foundation. All rights reserved

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * FlashWrite command support
 */
#include <common.h>
#include <command.h>
#include <part.h>
#include <linux/mtd/mtd.h>
#include <nand.h>
#include <mmc.h>
#include <sdhci.h>
#include <ubi_uboot.h>
#include <fdtdec.h>
#include <nand.h>
#include <elf.h>

#include "ipq_board.h"

DECLARE_GLOBAL_DATA_PTR;
#ifdef CONFIG_SDHCI_SUPPORT
extern struct sdhci_host mmc_host;
#endif

#define GPT_PART_NAME "0:GPT"
#define GPT_BACKUP_PART_NAME "0:GPTBACKUP"
#define HEADER_MAGIC1 0xFE569FAC
#define HEADER_MAGIC2 0xCD7F127A
#define HEADER_VERSION 4

#define SHA1_SIG_LEN 41

enum {
	CMD_FLASH = 1,
	CMD_FLERASE,
	CMD_FLREAD,
};

struct fl_info {
	int flash_type;
	uint32_t offset;
	uint32_t address;
	uint32_t file_size;
	uint32_t part_size;
	char* ubi_vol_name;
	bool is_ubi;
} __attribute__ ((__packed__));

#define UPDATE_FL_INFO(_dest, _type, _offset, _address, _part_size,	\
			_file_size,_vol_name, _ubi)		\
			do {						\
				(_dest)->flash_type = _type;		\
				(_dest)->offset = _offset;		\
				(_dest)->address = _address;		\
				(_dest)->file_size = _file_size;	\
				(_dest)->part_size = _part_size;	\
				(_dest)->ubi_vol_name = _vol_name;	\
				(_dest)->is_ubi = _ubi;			\
			} while(0)					\

struct header {
	unsigned magic[2];
	unsigned version;
} __attribute__ ((__packed__));

static int g_flash = 0;

int isvalid_appsbl_image(uintptr_t load_addr)
{
	uint64_t e_type;
	int ret = 0;
	unsigned long board_type = 0;
	ipq_smem_flash_info_t *sfi = get_ipq_smem_flash_info();
	char *flash_name[12] = { "no", "nor", "nand", "onenand", "sdc", "mmc",
				"spi_nor", "norplusnand", "norplusemmc",
				"dummy", "dummy", "qspi_nand" };

	if (((Elf32_Ehdr *)load_addr)->e_ident[EI_CLASS] == ELFCLASS64)
		e_type = ((Elf64_Ehdr *)load_addr)->e_type;
	else
		e_type = ((Elf32_Ehdr *)load_addr)->e_type;

	board_type = (sfi->flash_type == SMEM_BOOT_SPI_FLASH) ?
			get_current_board_flash_config() :
			sfi->flash_type;

	if ((e_type & ET_LOOS) != ET_LOOS) {
		goto skip;
	} else {
		if ((e_type & 0xF) != board_type) {
			printf("### Invalid Image %s != %s\n",
					flash_name[board_type],
					flash_name[e_type & 0xF]);
			ret = CMD_RET_USAGE;
		}
	}
skip:
	return ret;
}

static int write_to_flash(struct fl_info *fl)
{
	int flash_type = fl->flash_type;
	uint32_t address = fl->address;
	uint32_t offset = fl->offset;
	uint32_t part_size = fl->part_size;
	uint32_t file_size = fl->file_size;
	char runcmd[256] = { 0 };

	if (((flash_type == SMEM_BOOT_NAND_FLASH) ||
		(flash_type == SMEM_BOOT_QSPI_NAND_FLASH))) {

		snprintf(runcmd, sizeof(runcmd),
			"nand erase 0x%x 0x%x && "
			"nand write 0x%x 0x%x 0x%x && ",
			offset, part_size,
			address, offset, file_size);

	} else if (flash_type == SMEM_BOOT_MMC_FLASH) {

		snprintf(runcmd, sizeof(runcmd),
			"mmc erase 0x%x 0x%x && "
			"mmc write 0x%x 0x%x 0x%x && ",
			offset, part_size,
			address, offset, file_size);

	} else if (flash_type == SMEM_BOOT_SPI_FLASH) {

		snprintf(runcmd, sizeof(runcmd),
			"sf probe && "
			"sf erase 0x%x 0x%x && "
			"sf write 0x%x 0x%x 0x%x && ",
			offset, part_size,
			address, offset, file_size);
	}

	if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
		return CMD_RET_FAILURE;

	return CMD_RET_SUCCESS;
}

static int fl_erase(struct fl_info *fl)
{
	int flash_type = fl->flash_type;
	uint32_t offset = fl->offset;
	uint32_t part_size = fl->part_size;
	char runcmd[128] = { 0 };

	if ((flash_type == SMEM_BOOT_NAND_FLASH) ||
		(flash_type == SMEM_BOOT_QSPI_NAND_FLASH)) {

		snprintf(runcmd, sizeof(runcmd), "nand erase 0x%x 0x%x ",
					 offset, part_size);

	} else if (flash_type == SMEM_BOOT_MMC_FLASH) {

		snprintf(runcmd, sizeof(runcmd), "mmc erase 0x%x 0x%x ",
				 offset, part_size);

	} else if (flash_type == SMEM_BOOT_SPI_FLASH) {

		snprintf(runcmd, sizeof(runcmd), "sf probe && "
				"sf erase 0x%x 0x%x ", offset, part_size);
	}

	if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
		return CMD_RET_FAILURE;

	return CMD_RET_SUCCESS;
}

static int fl_read(struct fl_info *fl)
{
	int flash_type = fl->flash_type;
	uint32_t address = fl->address;
	uint32_t offset = fl->offset;
	uint32_t part_size = fl->part_size;
	char* vol_name = fl->ubi_vol_name;
	bool ubi = fl->is_ubi;
	char runcmd[128] = { 0 };

	if (((flash_type == SMEM_BOOT_NAND_FLASH) ||
		(flash_type == SMEM_BOOT_QSPI_NAND_FLASH))) {

		if (ubi) {
			snprintf(runcmd, sizeof(runcmd),"ubi read 0x%x %s",
				address, vol_name);
		} else {

			snprintf(runcmd, sizeof(runcmd),
					"nand read 0x%x 0x%x 0x%x ",
					 address, offset, part_size);
		}
	} else if (flash_type == SMEM_BOOT_MMC_FLASH) {

		snprintf(runcmd, sizeof(runcmd),
				"mmc read 0x%x 0x%x 0x%x ",
				 address, offset, part_size);

	} else if (flash_type == SMEM_BOOT_SPI_FLASH) {

		snprintf(runcmd, sizeof(runcmd),
				"sf probe && "
				"sf read 0x%x 0x%x 0x%x ",
				 address, offset, part_size);
	}

	if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
		return CMD_RET_FAILURE;

	return CMD_RET_SUCCESS;
}

#ifdef CONFIG_CMD_UBI
int ubi_vol_present(char* ubi_vol_name)
{
	int i;
	int j=0;
	struct ubi_device *ubi = NULL;
	struct ubi_volume *vol;
	char runcmd[256];

	if (init_ubi_part())
		goto ubi_detach;

	ubi = ubi_get_device(0);
	for (i = 0; ubi && i < (ubi->vtbl_slots + 1); i++) {
		vol = ubi->volumes[i];
		if (!vol)
			continue;	/* Empty record */
		if (vol->name_len <= UBI_VOL_NAME_MAX &&
		    strnlen(vol->name, vol->name_len + 1) == vol->name_len) {
			j++;
			if (!strncmp(ubi_vol_name, vol->name,
						UBI_VOL_NAME_MAX)) {
				ubi_put_device(ubi);
				return 1;
			}
		}

		if (j == ubi->vol_count - UBI_INT_VOL_COUNT)
			break;
	}


	printf("volume or partition %s not found\n", ubi_vol_name);
ubi_detach:
	if (ubi)
		ubi_put_device(ubi);

	snprintf(runcmd, sizeof(runcmd), "ubi detach && ");
	run_command(runcmd, 0);
	return 0;
}

int write_ubi_vol(char* ubi_vol_name, uint32_t load_addr, uint32_t file_size)
{
	char runcmd[256];
	int ret = CMD_RET_SUCCESS;
	if (!strncmp(ubi_vol_name, "ubi_rootfs", UBI_VOL_NAME_MAX)) {
		snprintf(runcmd, sizeof(runcmd),
			"ubi remove rootfs_data &&"
			"ubi remove %s &&"
			"ubi create %s 0x%x &&"
			"ubi write 0x%x %s 0x%x &&"
			"ubi create rootfs_data",
			 ubi_vol_name, ubi_vol_name, file_size,
			 load_addr, ubi_vol_name, file_size);
	} else {
		snprintf(runcmd, sizeof(runcmd),
			"ubi write 0x%x %s 0x%x ",
			 load_addr, ubi_vol_name, file_size);
	}

	if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
		ret = CMD_RET_FAILURE;

	return ret;
}
#endif /* CONFIG_CMD_UBI */

#ifdef CONFIG_MMC
static int prepare_mmc_flash(char *part_name, uint32_t *offset,
				uint32_t *part_size, uint32_t* file_size)
{
	int ret;
	struct disk_partition disk_info = {0};
	struct blk_desc *blk_dev;

	blk_dev = blk_get_devnum_by_uclass_id(UCLASS_MMC, 0);
	if (blk_dev != NULL) {

		if (strncmp(GPT_PART_NAME,
				(const char *)part_name,
				sizeof(GPT_PART_NAME))  == 0) {
			*file_size = *file_size / blk_dev->blksz;
			*offset = 0;
			*part_size = (ulong) *file_size;
		}
		else if (strncmp(GPT_BACKUP_PART_NAME,
				(const char *)part_name,
				sizeof(GPT_BACKUP_PART_NAME)) == 0) {
			*file_size = *file_size / blk_dev->blksz;
			*offset = (ulong) blk_dev->lba - *file_size;
			*part_size = (ulong) *file_size;
		}
		else
		{
			ret = part_get_info_efi_by_name(
					part_name, &disk_info);
			if (ret)
				return ret;
			*offset = (ulong)disk_info.start;
			*part_size = (ulong)disk_info.size;
		}
	}
	return ret;
}
#endif

static int prepare_nand_flash(char *part_name, uint32_t *offset,
		uint32_t *part_size)
{

	uint32_t size_block, start_block;
	int ret;
	ipq_smem_flash_info_t *sfi = get_ipq_smem_flash_info();

	if ((sfi->flash_type == SMEM_BOOT_SPI_FLASH &&
		((sfi->flash_secondary_type == SMEM_BOOT_NAND_FLASH)||
		(sfi->flash_secondary_type == SMEM_BOOT_QSPI_NAND_FLASH)))
		&& (strncmp(part_name, "rootfs", 6) == 0)) {

		ret = getpart_offset_size(part_name, offset, part_size);
		if (ret)
			goto _exit;
	} else {
		ret = smem_getpart(part_name, &start_block, &size_block);
		if (ret)
			goto _exit;
		*offset = sfi->flash_block_size * start_block;
		*part_size = sfi->flash_block_size * size_block;
	}
_exit:
	return ret;
}

int do_flash(struct cmd_tbl *cmdtp, int flag, int argc,
		char * const argv[])
{
	int flash_cmd = 0;
	uint32_t offset, part_size, adj_size;
	uint32_t load_addr = 0;
	uint32_t file_size = 0;
	uint32_t size_block, start_block, file_size_cpy;
	char *part_name = NULL, *filesize, *loadaddr;
	int flash_type = -1;
	int ret = CMD_RET_FAILURE;
	offset = 0;
	part_size = 0;
	ipq_smem_flash_info_t *sfi = get_ipq_smem_flash_info();
	bool is_ubi = false;
	struct fl_info fl;
#ifdef CONFIG_MMC
	struct disk_partition disk_info = {0};
#endif
#ifdef CONFIG_CMD_NAND
	struct mtd_info *nand = get_nand_dev_by_index(0);
#endif
	if (strcmp(argv[0], "flash") == 0)
		flash_cmd = CMD_FLASH;
	else if (strcmp(argv[0], "flerase") == 0)
		flash_cmd = CMD_FLERASE;
	else
		flash_cmd = CMD_FLREAD;

	if (g_flash) {
		flash_type = g_flash;
	} else {
		flash_type = sfi->flash_type;
	}

	part_name = argv[1];

	if (flash_cmd == CMD_FLASH) {
		if ((argc < 2) || (argc > 5))
			goto usage_err;

		if (argc ==3 || argc == 5) {
			if(strncmp(argv[argc-1], "emmc", 4) == 0)
				flash_type = SMEM_BOOT_MMC_FLASH;
			else if (strncmp(argv[argc-1], "nand", 4) == 0)
				flash_type = SMEM_BOOT_QSPI_NAND_FLASH;
			else if (strncmp(argv[argc-1], "nor", 3) == 0)
				flash_type = SMEM_BOOT_SPI_FLASH;
			else
				goto usage_err;
		}

		if (argc == 2 || argc == 3) {
			loadaddr = env_get("fileaddr");
			if (loadaddr != NULL)
				load_addr = simple_strtoul(loadaddr, NULL, 16);
			else
				goto usage_err;

			filesize = env_get("filesize");
			if (filesize != NULL)
				file_size = simple_strtoul(filesize, NULL, 16);
			else
				goto usage_err;

		} else if (argc == 4 || argc ==5) {
			load_addr = simple_strtoul(argv[2], NULL, 16);
			file_size = simple_strtoul(argv[3], NULL, 16);

		} else
			goto usage_err;

		file_size_cpy = file_size;

		/*
		 * validate the APPSBL image
		 */
		if ((flash_type != SMEM_BOOT_NO_FLASH) &&
			(flash_type == sfi->flash_type) &&
			!strncmp(part_name, "0:APPSBL", strlen("0:APPSBL"))) {
			ret = isvalid_appsbl_image((uintptr_t)load_addr);
			if(ret)
				goto _exit;
		}
	} else {
		if (argc > 3 || argc < 2)
			goto usage_err;

		if (flash_cmd == CMD_FLREAD) {
			if (argc == 3)
				load_addr = simple_strtoul(argv[2], NULL, 16);
			else
				load_addr = CONFIG_SYS_LOAD_ADDR;
		} else {
			if (argc != 2)
				goto usage_err;
		}
	}

	if ((flash_type == SMEM_BOOT_NAND_FLASH) ||
		(flash_type == SMEM_BOOT_QSPI_NAND_FLASH)) {

		ret = prepare_nand_flash(part_name, &offset, &part_size);
		if(ret) {
#ifdef CONFIG_CMD_UBI
			is_ubi = ubi_vol_present(part_name);
#endif
		}
#ifdef CONFIG_MMC
	} else if ((flash_type == SMEM_BOOT_MMC_FLASH) ||
		(flash_type == SMEM_BOOT_NO_FLASH)) {

		ret = prepare_mmc_flash(part_name, &offset,
						&part_size, &file_size);
		if(ret)
			goto _exit;
#endif
	} else if (flash_type == SMEM_BOOT_SPI_FLASH) {
		if (get_which_flash_param(part_name) == 1) {
			/* NOR + NAND Parition */
			flash_type = SMEM_BOOT_NAND_FLASH;
			ret = getpart_offset_size(part_name, &offset,
					&part_size);
			if (ret)
				goto _exit;
#ifdef CONFIG_MMC
		} else if ((smem_getpart(part_name, &start_block, &size_block)
				== -ENOENT) &&
				(sfi->rootfs.offset == 0xBAD0FF5E)){
			/* NOR + EMMC Partition */
			ret = prepare_mmc_flash(part_name, &offset,
						&part_size, &file_size);
			if(ret)
				goto _exit;

			flash_type = SMEM_BOOT_MMC_FLASH;
#endif
		} else {
			/* NOR Partition / NAND volumes */
			ret = smem_getpart(part_name, &start_block,
							&size_block);
			if (ret) {
#ifdef CONFIG_CMD_UBI
				is_ubi = ubi_vol_present(part_name);
#endif
			} else {
				offset = sfi->flash_block_size * start_block;
				part_size = sfi->flash_block_size * size_block;
			}
		}
	} else
		goto usage_err;

#ifdef CONFIG_CMD_NAND
	if (((flash_type == SMEM_BOOT_NAND_FLASH) ||
		(flash_type == SMEM_BOOT_QSPI_NAND_FLASH))) {

		adj_size = file_size % nand->writesize;
		if (adj_size)
			file_size = file_size + (nand->writesize -
					adj_size);
	}
#endif
#ifdef CONFIG_MMC
	if (flash_type == SMEM_BOOT_MMC_FLASH) {
		if (!((strncmp(GPT_PART_NAME, (const char *)part_name,
			sizeof(GPT_PART_NAME))  == 0) ||
			(strncmp(GPT_BACKUP_PART_NAME,
			(const char *)part_name,
			sizeof(GPT_BACKUP_PART_NAME)) == 0))) {

			ret = part_get_info_efi_by_name(
				part_name, &disk_info);
			if (ret)
				goto _exit;

			if (disk_info.blksz) {
				file_size = file_size /
					disk_info.blksz;
				adj_size = file_size_cpy %
					disk_info.blksz;
				if (adj_size)
					file_size = file_size + 1;
			}
		}
	}
#endif
	if (!is_ubi && (file_size > part_size)) {
		printf("Image size is greater than "
				"partition memory\n");

		ret = CMD_RET_FAILURE;
		goto _exit;
	}

	if (ret && !is_ubi)
		goto _exit;

	UPDATE_FL_INFO(&fl, flash_type, offset, load_addr, part_size,
			file_size, part_name, is_ubi);

	switch(flash_cmd) {
	case CMD_FLERASE:
		if (is_ubi) {
			printf("ubi volume erase not supported \n");
			goto usage_err;
		}
		ret = fl_erase(&fl);
		break;
	case CMD_FLREAD:
		ret = fl_read(&fl);
		break;
	default:
#ifdef CONFIG_CMD_UBI
		if (is_ubi)
			ret = write_ubi_vol(part_name, load_addr, file_size);
		else
#endif
			ret = write_to_flash(&fl);
	}

_exit:
	return ret;
usage_err:
	return CMD_RET_USAGE;

}

static int do_mibib_reload(struct cmd_tbl *cmdtp, int flag, int argc,
char * const argv[])
{
	uint32_t load_addr, file_size;
	uint32_t page_size;
	uint8_t flash_type;
	struct header* mibib_hdr;
	ipq_smem_flash_info_t *sfi = get_ipq_smem_flash_info();
#ifdef CONFIG_CMD_UBI
	char runcmd[256];
	struct ubi_device *ubi = NULL;
#endif

	if (argc == 5) {
		flash_type = simple_strtoul(argv[1], NULL, 16);
		page_size = simple_strtoul(argv[2], NULL, 16);
		sfi->flash_block_size = simple_strtoul(argv[3], NULL, 16);
		sfi->flash_density = simple_strtoul(argv[4], NULL, 16);
		load_addr = env_get_ulong("fileaddr", 16, 0);
		file_size = env_get_ulong("filesize", 16, 0);
	} else
		return CMD_RET_USAGE;

	if (flash_type > 1) {
		printf("Invalid flash type \n");
		return CMD_RET_FAILURE;
	}

	if (file_size < 2 * page_size) {
		printf("Invalid filesize \n");
		return CMD_RET_FAILURE;
	}

	mibib_hdr = (struct header*)((uintptr_t) load_addr);
	if (mibib_hdr->magic[0] == HEADER_MAGIC1 &&
		mibib_hdr->magic[1] == HEADER_MAGIC2 &&
		mibib_hdr->version == HEADER_VERSION) {

		load_addr += page_size;
	}
	else {
		printf("Header magic/version is invalid\n");
		return CMD_RET_FAILURE;
	}

	if (mibib_ptable_init((unsigned int *)((uintptr_t) load_addr))) {
		printf("Table magic is invalid\n");
		return CMD_RET_FAILURE;
	}

	if (flash_type == 0) {
		/*NAND*/
		sfi->flash_type = SMEM_BOOT_QSPI_NAND_FLASH;
	} else {
		/* NOR*/
		sfi->flash_type = SMEM_BOOT_SPI_FLASH;
	}

#ifdef CONFIG_CMD_UBI
	ubi = ubi_get_device(0);
	if (ubi) {
		ubi_put_device(ubi);
		snprintf(runcmd, sizeof(runcmd), "ubi detach && ");
		run_command(runcmd, 0);
	}
#endif

	get_kernel_fs_part_details();

	return CMD_RET_SUCCESS;
}

void print_fl_msg(char *fname, bool started, int ret)
{
	printf("######################################## ");
	printf("Flashing %s %s\n", fname,
			started ? "Started" : ret ? "Failed" : "Done");
}

int do_xtract_n_flash(struct cmd_tbl *cmdtp, int flag, int argc,
char * const argv[])
{
	char runcmd[256], fname_stripped[256];
	char *file_name, *part_name;
	uint32_t load_addr, verbose;
	int ret = CMD_RET_SUCCESS;
	u16 flash_type = -1;

	if (argc < 4 || argc > 5)
		return CMD_RET_USAGE;

	verbose = env_get_ulong("verbose", 10, 0);
	load_addr = simple_strtoul(argv[1], NULL, 16);
	file_name = argv[2];
	part_name = argv[3];
	if (argc == 5) {
		if(strncmp(argv[4], "emmc", 4) == 0)
			flash_type = SMEM_BOOT_MMC_FLASH;
		else if (strncmp(argv[4], "nand", 4) == 0)
			flash_type = SMEM_BOOT_QSPI_NAND_FLASH;
		else if (strncmp(argv[4], "nor", 3) == 0)
			flash_type = SMEM_BOOT_SPI_FLASH;
		else
			return CMD_RET_USAGE;
	}

	snprintf(fname_stripped , sizeof(fname_stripped),
		"%.*s:",(int) (strlen(file_name) - SHA1_SIG_LEN), file_name);

	if (verbose)
		print_fl_msg(fname_stripped, 1, ret);
	else
		env_set("stdout", "nulldev");

	if(5 == argc) {

		snprintf(runcmd , sizeof(runcmd),
			"imxtract 0x%x %s && "
			"flash %s %s",
			load_addr, file_name,
			part_name, argv[4]);
	} else if(4 == argc) {
		snprintf(runcmd , sizeof(runcmd),
			"imxtract 0x%x %s && "
			"flash %s",
			load_addr, file_name,
			part_name);
	}
	if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
		ret = CMD_RET_FAILURE;

	if (verbose)
		print_fl_msg(fname_stripped, 0, ret);
	else {
		env_set("stdout", "serial");
		printf("Flashing %-30s %s\n", fname_stripped,
				ret ? "[ failed ]" : "[ done ]");
	}

	return ret;
}

static int do_flupdate(struct cmd_tbl *cmdtp, int flag, int argc,
			char * const argv[])
{
	int ret = CMD_RET_USAGE;

	if (argc < 2 || argc > 3)
		goto quit;

	if (!strncmp(argv[1], "set", 3)) {
		if(argc != 3)
			goto quit;

		if (!strncmp(argv[2], "mmc", 3))
			g_flash = SMEM_BOOT_MMC_FLASH;
		else if (!strncmp(argv[2], "nor", 3))
			g_flash = SMEM_BOOT_SPI_FLASH;
		else if (!strncmp(argv[2], "nand", 4))
			g_flash = SMEM_BOOT_QSPI_NAND_FLASH;
		else
			goto quit;

	} else if (!strncmp(argv[1], "clear", 5)) /* set flash type to default */
		g_flash = 0;
	else
		goto quit;

	ret = CMD_RET_SUCCESS;
quit:
	return ret;
}

static int do_flash_init(struct cmd_tbl *cmdtp, int flag, int argc,
		char * const argv[]) {
	return 0;
}

U_BOOT_CMD(
	flash,       5,      0,      do_flash,
	"flash part_name \n"
	"\tflash part_name load_addr file_size \n"
	"\tflash part_name flash_type{emmc/nand/nor} \n"
	"\tflash part_name load_addr file_size flash_type{emmc/nand/nor}\n",
	"flash the image at load_addr, given file_size in hex"
);

U_BOOT_CMD(
	flerase,       2,      0,      do_flash,
	"flerase part_name \n",
	"erases on flash the given partition \n"
);

U_BOOT_CMD(
	flread,       3,      0,      do_flash,
	"flread part_name location(optional)\n",
	"read from flash the given partition \n"
);

U_BOOT_CMD(
	mibib_reload,       5,      0,      do_mibib_reload,
	"mibib_reload fl_type pg_size blk_size chip_size\n",
	"reloads the smem partition info from mibib \n"
);

U_BOOT_CMD(
	xtract_n_flash,       5,      0,      do_xtract_n_flash,
	"xtract_n_flash addr filename partname \n",
	"xtract the image and flash \n"
);

U_BOOT_CMD(
	flashinit,       2,      0,      do_flash_init,
	"flashinit nand/mmc \n",
	"Init the flash \n"
);

U_BOOT_CMD(
	flupdate,       3,       0,       do_flupdate,
	"flupdate set mmc/nand/nor ; flupdate clear \n",
	"flash type update \n"
);
