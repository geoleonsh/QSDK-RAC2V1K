// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2015-2017, 2020 The Linux Foundation. All rights reserved.
 *
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <common.h>
#include <command.h>
#include <image.h>
#include <nand.h>
#include <errno.h>
#include <part.h>
#include <linux/mtd/ubi.h>
#include <mmc.h>
#include <part_efi.h>
#include <fdtdec.h>
#include <usb.h>
#include <elf.h>

#include "ipq_board.h"

#define XMK_STR(x)#x
#define MK_STR(x)XMK_STR(x)

#ifdef CONFIG_SYS_MAXARGS
#define MAX_BOOT_ARGS_SIZE	CONFIG_SYS_MAXARGS
#elif
#define MAX_BOOT_ARGS_SIZE	64
#endif

extern int part_get_info_efi_by_name(const char *name,
					struct disk_partition *info);
#ifdef CONFIG_IPQ_ELF_AUTH
typedef struct {
	unsigned int img_offset;
	unsigned int img_load_addr;
	unsigned int img_size;
} image_info;
#endif

unsigned int get_rootfs_active_partition(void)
{
	int i;
	ipq_smem_flash_info_t *sfi = get_ipq_smem_flash_info();

	if (!sfi->ipq_smem_bootconfig_info)
		return 0;

	for (i = 0; i < sfi->ipq_smem_bootconfig_info->numaltpart; i++) {
		if (strncmp("rootfs",
			sfi->ipq_smem_bootconfig_info->per_part_entry[i].name,
			     CONFIG_RAM_PART_NAME_LENGTH) == 0)
			return sfi->ipq_smem_bootconfig_info->		\
						per_part_entry[i].primaryboot;
	}

	return 0; /* alt partition not available */
}

#ifdef CONFIG_MTD
/*
 * Set the root device and bootargs for mounting root filesystem.
 */
int set_fs_bootargs(void)
{
	char *bootargs;
	char mtdids[256];
	ipq_smem_flash_info_t *sfi = get_ipq_smem_flash_info();

	if (((sfi->flash_type == SMEM_BOOT_NAND_FLASH) ||
			(sfi->flash_type == SMEM_BOOT_QSPI_NAND_FLASH))) {
		bootargs = "ubi.mtd=rootfs root=mtd:ubi_rootfs "
			"rootfstype=squashfs";
		if (env_get("fsbootargs") == NULL)
			env_set("fsbootargs", bootargs);

		snprintf(mtdids, sizeof(mtdids), "nand0=nand0");

	} else if (sfi->flash_type == SMEM_BOOT_SPI_FLASH) {
		if (get_which_flash_param("rootfs") ||
			((sfi->flash_secondary_type == SMEM_BOOT_NAND_FLASH) ||
			 (sfi->flash_secondary_type ==
			  SMEM_BOOT_QSPI_NAND_FLASH))) {
			bootargs = "ubi.mtd=rootfs root=mtd:ubi_rootfs "
				"rootfstype=squashfs";

			snprintf(mtdids, sizeof(mtdids),
				"nand0=nand0,nor0="
				CONFIG_IPQ_SPI_NOR_DEV_NAME);

			if (env_get("fsbootargs") == NULL)
				env_set("fsbootargs", bootargs);
		}
	}else {
		printf("bootipq: unsupported boot flash type\n");
		return -EINVAL;
	}

	return run_command("setenv bootargs ${bootargs} "
			"${fsbootargs} rootwait", 0);
}
#endif

#ifdef CONFIG_MMC
int set_uuid_bootargs(char *boot_args, char *part_name, int buflen,
			bool gpt_flag)
{
	int ret;
	struct disk_partition disk_info;

	if (buflen <= 0 || buflen > MAX_BOOT_ARGS_SIZE)
		return -EINVAL;

	ret = part_get_info_efi_by_name(part_name, &disk_info);
	if (ret) {
		printf("bootipq: unsupported partition name %s\n",part_name);
		return -EINVAL;
	}
#ifdef CONFIG_PARTITION_UUIDS
	snprintf(boot_args, MAX_BOOT_ARGS_SIZE, "root=PARTUUID=%s%s",
			disk_info.uuid, (gpt_flag == true)?" gpt" : "");
#else
	snprintf(boot_args, MAX_BOOT_ARGS_SIZE, "rootfsname==%s gpt",
			part_name, (gpt_flag == true)?" gpt" : "");
#endif
	env_set("fsbootargs", boot_args);

	return run_command("setenv bootargs ${bootargs} \
			${fsbootargs} rootwait", 0);
}

static int boot_mmc(int active_part, bool gpt_flag)
{
	struct disk_partition disk_info;
	char runcmd[MAX_BOOT_ARGS_SIZE];
	int ret;
	ipq_smem_flash_info_t *sfi = get_ipq_smem_flash_info();

	if (active_part)
		ret  = set_uuid_bootargs(runcmd, "rootfs_1",
				MAX_BOOT_ARGS_SIZE, gpt_flag);
	else
		ret  = set_uuid_bootargs(runcmd, "rootfs",
				MAX_BOOT_ARGS_SIZE, gpt_flag );

	if (sfi->ipq_smem_bootconfig_info != NULL) {
		if (active_part) {
			ret = part_get_info_efi_by_name("0:HLOS_1",
				&disk_info);
		} else {
			ret = part_get_info_efi_by_name("0:HLOS",
				&disk_info);
		}
	} else {
		ret = part_get_info_efi_by_name("0:HLOS", &disk_info);
	}

	if (ret == 0) {
		snprintf(runcmd, sizeof(runcmd), "mmc read 0x%x 0x%x 0x%x",
			 CONFIG_SYS_LOAD_ADDR,
			 (uint)disk_info.start, (uint)disk_info.size);
	}

	return run_command(runcmd, 0);
}
#endif

#ifdef CONFIG_NAND_QTI
static int boot_nand(void)
{
	ipq_smem_flash_info_t *sfi = get_ipq_smem_flash_info();
	char runcmd[256];
	int ret;

	ret = set_fs_bootargs();
	if (ret)
		return ret;
	/*
	 * The kernel is in seperate partition
	 */
	if (sfi->rootfs.offset == 0xBAD0FF5E) {
		printf(" bad offset of hlos");
		return -1;
	}

	if ((sfi->flash_type == SMEM_BOOT_NAND_FLASH) ||
			(sfi->flash_type == SMEM_BOOT_QSPI_NAND_FLASH)) {
		snprintf(runcmd, sizeof(runcmd),
			"setenv mtdids nand0=nand0 && "
			"setenv mtdparts "
			"mtdparts=nand0:0x%llx@0x%llx(fs),${msmparts} && "
			"ubi part fs && "
			"ubi read 0x%x kernel && ",
			sfi->rootfs.size, sfi->rootfs.offset,
			CONFIG_SYS_LOAD_ADDR);
	} else if ((sfi->flash_type == SMEM_BOOT_SPI_FLASH) &&
			(sfi->rootfs.offset != 0xBAD0FF5E) &&
			(sfi->flash_secondary_type ==
			 SMEM_BOOT_QSPI_NAND_FLASH)) {
		if (get_which_flash_param("rootfs")) {
			snprintf(runcmd, sizeof(runcmd),
				"nand device 0 && "
				"setenv mtdids nand0=nand0,nor0=spi0.0 && "
				"setenv mtdparts mtdparts=nand0:"
				"0x%llx@0x%llx(fs),${msmparts} && "
				"ubi part fs && "
				"ubi read 0x%x kernel && ",
				sfi->rootfs.size, sfi->rootfs.offset,
				CONFIG_SYS_LOAD_ADDR);
		} else {
			/*
			 * Kernel is in a separate partition
			 */
			snprintf(runcmd, sizeof(runcmd),
				"sf probe &&"
				"sf read 0x%x 0x%x 0x%x && ",
				CONFIG_SYS_LOAD_ADDR, (uint)sfi->hlos.offset,
				(uint)sfi->hlos.size);
		}
	}

	return run_command(runcmd, 0);
}
#endif

#ifdef CONFIG_IPQ_ELF_AUTH
static int parse_elf_image_phdr(image_info *img_info, unsigned int addr)
{
	Elf32_Ehdr *ehdr; /* Elf header structure pointer */
	Elf32_Phdr *phdr; /* Program header structure pointer */
	int i;

	ehdr = (Elf32_Ehdr *)addr;
	phdr = (Elf32_Phdr *)(addr + ehdr->e_phoff);

	if (!IS_ELF(*ehdr)) {
		printf("It is not a elf image \n");
		return -EINVAL;
	}

	if (ehdr->e_type != ET_EXEC) {
		printf("Not a valid elf image\n");
		return -EINVAL;
	}

	/* Load each program header */
	for (i = 0; i < NO_OF_PROGRAM_HDRS; ++i) {
		printf("Parsing phdr load addr 0x%x offset 0x%x"
			" size 0x%x type 0x%x\n",
			phdr->p_paddr, phdr->p_offset, phdr->p_filesz,
			phdr->p_type);
		if(phdr->p_type == PT_LOAD) {
			img_info->img_offset = phdr->p_offset;
			img_info->img_load_addr = phdr->p_paddr;
			img_info->img_size =  phdr->p_filesz;
			return 0;
		}
		++phdr;
	}

	return -EINVAL;
}
#endif


int config_select(ulong addr, char *rcmd, int rcmd_size)
{
	/* Selecting a config name from the list of available
	 * config names by passing them to the fit_conf_get_node()
	 * function which is used to get the node_offset with the
	 * config name passed. Based on the return value index based
	 * or board name based config is used.
	 */
	int len, i;
	const char *config = env_get("config_name");

	if (config) {
		printf("Manual device tree config selected!\n");
		if (fit_conf_get_node((void *)addr, config) >= 0) {
			snprintf(rcmd, rcmd_size, "bootm 0x%lx#%s\n",
				addr, config);
			return 0;
		}

	} else {
		for (i = 0;
			(config = fdt_stringlist_get(gd->fdt_blob, 0,
					"config_name", i,&len)); ++i) {
			if (config == NULL)
				break;
			if (fit_conf_get_node((void *)addr, config) >= 0) {
				snprintf(rcmd, rcmd_size, "bootm 0x%lx#%s\n",
					 addr, config);
				return 0;
			}
		}
	}

	printf("Config not available\n");
	return -1;
}

int do_bootipq(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	int ret;
	int flash_type;
	char runcmd[256];
	ulong load_address = CONFIG_SYS_LOAD_ADDR;
#ifdef CONFIG_IPQ_ELF_AUTH
	image_info img_info;
#endif
	bool secure_board = false;
#ifdef CONFIG_MMC
	int active_part = get_rootfs_active_partition();
#endif
#ifdef CONFIG_BOARD_TYPES
	flash_type = gd->board_type;
#else
	ipq_smem_flash_info_t *sfi = get_ipq_smem_flash_info();
	if (sfi->flash_secondary_type == SMEM_BOOT_MMC_FLASH)
		flash_type = SMEM_BOOT_NORPLUSEMMC;
	else if (sfi->flash_secondary_type == SMEM_BOOT_QSPI_NAND_FLASH)
		flash_type = SMEM_BOOT_NORPLUSNAND;
	else
		flash_type = sfi->flash_type;
#endif
	switch(flash_type) {
#ifdef CONFIG_MMC
	case SMEM_BOOT_MMC_FLASH:
		ret = boot_mmc(active_part, true);
		break;
	case SMEM_BOOT_NORPLUSEMMC:
		ret = boot_mmc(active_part, false);
		break;
#endif
#ifdef CONFIG_NAND_QTI
	case SMEM_BOOT_QSPI_NAND_FLASH:
	case SMEM_BOOT_NORPLUSNAND:
		ret = boot_nand();
		break;
#endif
	default:
		printf("Unsupported BOOT flash type\n");
		return -1;
	}
	/*
	 * set fdt_high parameter so that u-boot will not load
	 * dtb above FDT_HIGH region.
	 */

	ret = env_set("fdt_high", MK_STR(FDT_HIGH));
	if (ret)
		return CMD_RET_FAILURE;

#ifdef CONFIG_IPQ_ELF_AUTH
	ret = parse_elf_image_phdr(&img_info,CONFIG_SYS_LOAD_ADDR)
	if (ret != -EINVAL) {
		load_address += img_info.img_offset;
		ret = IMAGE_FORMAT_FIT;
	} else {
		printf("Unable to parse elf \n");
		return -1;
	}
#else
	if (secure_board)
		load_address += sizeof(mbn_header_t);
	ret = genimg_get_format((void *)load_address);
#endif
	if (ret == IMAGE_FORMAT_LEGACY) {
		snprintf(runcmd, sizeof(runcmd),
			 "bootm 0x%lx\n", load_address);
	} else {
		int noff = fit_image_get_node((void*)load_address, "kernel-1");
		if (noff < 0) {
			noff = fit_image_get_node((void*)load_address,
					"kernel@1");
			if (noff < 0)
				return CMD_RET_FAILURE;
		}

		if (!fit_image_check_arch((void*)load_address, noff,
					IH_ARCH_DEFAULT)) {
			printf("Cross Arch Kernel jump is not supported!!!\n");
			printf("Please use %d-bit kernel image.\n",
				((IH_ARCH_DEFAULT == IH_ARCH_ARM64)?64:32));
			return CMD_RET_FAILURE;
		}

		ret = config_select(load_address, runcmd, sizeof(runcmd));
	}

	if (ret)
		return ret;
	else
		return run_command(runcmd, 0);
}

U_BOOT_CMD(
	bootipq, 2, 0, do_bootipq,
	"bootipq from flash device",
	"bootipq [debug] - Load image(s) and boots the kernel\n"
	);
