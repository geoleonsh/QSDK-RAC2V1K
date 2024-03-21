// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2016-2019, The Linux Foundation. All rights reserved.
 *
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _IPQSMEM_H
#define _IPQSMEM_H

#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/byteorder.h>
#include <mtd_node.h>
#include <part.h>
#ifdef CONFIG_TARGET_IPQ9574
#include "../ipq9574/ipq9574.h"
#endif
#ifdef CONFIG_TARGET_IPQ5332
#include "../ipq5332/ipq5332.h"
#endif

#define __str_fmt(x)		"%-" #x "s"
#define _str_fmt(x)		__str_fmt(x)
#define smem_ptn_name_fmt	_str_fmt(SMEM_PTN_NAME_MAX)
#define IPQ_ETH_FW_PART_NAME	"0:ETHPHYFW"
#define IPQ_ETH_FW_PART_SIZE	0x80000
#define BOARD_DTS_MAX_NAMELEN	30

#ifdef CONFIG_SMEM_VERSION_C
#define part_which_flash(p)    (((p)->attr & 0xff000000) >> 24)

struct ram_partition_entry
{
	char name[CONFIG_RAM_PART_NAME_LENGTH];
				/**< Partition name, unused for now */
	u64 start_address;	/**< Partition start address in RAM */
	u64 length;		/**< Partition length in RAM in Bytes */
	u32 partition_attribute;/**< Partition attribute */
	u32 partition_category;	/**< Partition category */
	u32 partition_domain;	/**< Partition domain */
	u32 partition_type;	/**< Partition type */
	u32 num_partitions;	/**< Number of partitions on device */
	u32 hw_info;		/**< hw information such as type and freq */
	u8 highest_bank_bit;	/**< Highest bit corresponding to a bank */
	u8 reserve0;		/**< Reserved for future use */
	u8 reserve1;		/**< Reserved for future use */
	u8 reserve2;		/**< Reserved for future use */
	u32 reserved5;		/**< Reserved for future use */
	u64 available_length;	/**< Available Part length in RAM in Bytes */
};

struct usable_ram_partition_table
{
	u32 magic1;		/**< Magic number to identify valid RAM
					partition table */
	u32 magic2;		/**< Magic number to identify valid RAM
					partition table */
	u32 version;		/**< Version number to track structure
					definition changes and maintain
					backward compatibilities */
	u32 reserved1;		/**< Reserved for future use */

	u32 num_partitions;	/**< Number of RAM partition table entries */

	u32 reserved2;		/** < Added for 8 bytes alignment of header */

	/** RAM partition table entries */
	struct ram_partition_entry ram_part_entry[CONFIG_RAM_NUM_PART_ENTRIES];
};
#endif

struct smem_ram_ptn {
	char name[16];
	unsigned long long start;
	unsigned long long size;

	/* RAM Partition attribute: READ_ONLY, READWRITE etc.  */
	unsigned attr;

	/* RAM Partition category: EBI0, EBI1, IRAM, IMEM */
	unsigned category;

	/* RAM Partition domain: APPS, MODEM, APPS & MODEM (SHARED) etc. */
	unsigned domain;

	/* RAM Partition type: system, bootloader, appsboot, apps etc. */
	unsigned type;

	/* reserved for future expansion without changing version number */
	unsigned reserved2, reserved3, reserved4, reserved5;
} __attribute__ ((__packed__));

struct smem_ram_ptable {
#define _SMEM_RAM_PTABLE_MAGIC_1	0x9DA5E0A8
#define _SMEM_RAM_PTABLE_MAGIC_2	0xAF9EC4E2
	unsigned magic[2];
	unsigned version;
	unsigned reserved1;
	unsigned len;
	unsigned buf;
	struct smem_ram_ptn parts[32];
} __attribute__ ((__packed__));

/*
 * function declaration
 */
int smem_getpart(char *part_name, uint32_t *start, uint32_t *size);
int smem_ram_ptable_init(struct smem_ram_ptable *smem_ram_ptable);
int smem_ram_ptable_init_v2(
		struct usable_ram_partition_table *usable_ram_partition_table);


#define RAM_PARTITION_SDRAM 		14
#define RAM_PARTITION_SYS_MEMORY 	1
#define IPQ_NAND_ROOTFS_SIZE 		(64 << 20)

#define SOCINFO_VERSION_MAJOR(ver) 	((ver & 0xffff0000) >> 16)
#define SOCINFO_VERSION_MINOR(ver) 	(ver & 0x0000ffff)

#define INDEX_LENGTH			2
#define SEP1_LENGTH			1
#define VERSION_STRING_LENGTH		72
#define VARIANT_STRING_LENGTH		20
#define SEP2_LENGTH			1
#define OEM_VERSION_STRING_LENGTH	32
#define BUILD_ID_LEN			32

#define SMEM_PTN_NAME_MAX     		16
#define SMEM_PTABLE_PARTS_MAX 		32
#define SMEM_PTABLE_PARTS_DEFAULT 	16

enum {
	SMEM_BOOT_NO_FLASH        = 0,
	SMEM_BOOT_NOR_FLASH       = 1,
	SMEM_BOOT_NAND_FLASH      = 2,
	SMEM_BOOT_ONENAND_FLASH   = 3,
	SMEM_BOOT_SDC_FLASH       = 4,
	SMEM_BOOT_MMC_FLASH       = 5,
	SMEM_BOOT_SPI_FLASH       = 6,
	SMEM_BOOT_NORPLUSNAND     = 7,
	SMEM_BOOT_NORPLUSEMMC     = 8,
	SMEM_BOOT_QSPI_NAND_FLASH  = 11,
};

struct version_entry
{
	char index[INDEX_LENGTH];
	char colon_sep1[SEP1_LENGTH];
	char version_string[VERSION_STRING_LENGTH];
	char variant_string[VARIANT_STRING_LENGTH];
	char colon_sep2[SEP2_LENGTH];
	char oem_version_string[OEM_VERSION_STRING_LENGTH];
} __attribute__ ((__packed__));

typedef struct smem_pmic_type
{
	unsigned pmic_model;
	unsigned pmic_die_revision;
}pmic_type;

typedef struct ipq_platform_v1 {
	unsigned format;
	unsigned id;
	unsigned version;
	char     build_id[BUILD_ID_LEN];
	unsigned raw_id;
	unsigned raw_version;
	unsigned hw_platform;
	unsigned platform_version;
	unsigned accessory_chip;
	unsigned hw_platform_subtype;
}ipq_platform_v1;

typedef struct ipq_platform_v2 {
	ipq_platform_v1 v1;
	pmic_type pmic_info[3];
	unsigned foundry_id;
}ipq_platform_v2;

typedef struct ipq_platform_v3 {
	ipq_platform_v2 v2;
	unsigned chip_serial;
} ipq_platform_v3;

union ipq_platform {
	ipq_platform_v1 v1;
	ipq_platform_v2 v2;
	ipq_platform_v3 v3;
};

struct smem_machid_info {
	unsigned format;
	unsigned machid;
};

typedef struct soc_info {
	uint32_t cpu_type;
	uint32_t version;
	uint32_t soc_version_major;
	uint32_t soc_version_minor;
	unsigned int machid;
} socinfo_t;

typedef struct {
	loff_t offset;
	loff_t size;
} ipq_part_entry_t;

struct per_part_info
{
	char name[CONFIG_RAM_PART_NAME_LENGTH];
	uint32_t primaryboot;
};

typedef struct
{
#define _SMEM_DUAL_BOOTINFO_MAGIC_START		0xA3A2A1A0
#define _SMEM_DUAL_BOOTINFO_MAGIC_END		0xB3B2B1B0

	/* Magic number for identification when reading from flash */
	uint32_t magic_start;
	/* upgradeinprogress indicates to attempting the upgrade */
	uint32_t    age;
	/* numaltpart indicate number of alt partitions */
	uint32_t    numaltpart;

	struct per_part_info per_part_entry[CONFIG_NUM_ALT_PARTITION];

	uint32_t magic_end;

} ipq_smem_bootconfig_info_t;

typedef struct {
	uint32_t		flash_type;
	uint32_t		flash_index;
	uint32_t		flash_chip_select;
	uint32_t		flash_block_size;
	uint32_t		flash_density;
	uint32_t		flash_secondary_type;
	uint32_t		primary_mibib;
	ipq_part_entry_t	hlos;
	ipq_part_entry_t	rootfs;
	ipq_part_entry_t	dtb;
	ipq_smem_bootconfig_info_t *ipq_smem_bootconfig_info;
} ipq_smem_flash_info_t;

struct smem_ptn {
	char name[SMEM_PTN_NAME_MAX];
	unsigned start;
	unsigned size;
	unsigned attr;
} __attribute__ ((__packed__));

struct smem_ptable {
#define _SMEM_PTABLE_MAGIC_1 0x55ee73aa
#define _SMEM_PTABLE_MAGIC_2 0xe35ebddb
	unsigned magic[2];
	unsigned version;
	unsigned len;
	struct smem_ptn parts[SMEM_PTABLE_PARTS_MAX];
} __attribute__ ((__packed__));

typedef struct {
        unsigned int image_type;
        unsigned int header_vsn_num;
        unsigned int image_src;
        unsigned int image_dest_ptr;
        unsigned int image_size;
        unsigned int code_size;
        unsigned int signature_ptr;
        unsigned int signature_size;
        unsigned int cert_chain_ptr;
        unsigned int cert_chain_size;
} mbn_header_t;

/*
 * NAND Flash Configs
 */
#ifdef CONFIG_QSPI_LAYOUT_SWITCH
#define QTI_NAND_LAYOUT_SBL			0
#define QTI_NAND_LAYOUT_LINUX			1
#define QTI_NAND_LAYOUT_MAX			2

int qti_nand_get_curr_layout(void);
#endif

/*
 * Extern variables
 */
extern struct node_info * fnodes;
extern int * fnode_entires;

#ifdef CONFIG_DTB_RESELECT
struct machid_dts_map
{
    int machid;
    char* dts;
};

extern struct machid_dts_map * machid_dts_info;
extern int * machid_dts_entries;
#endif /* CONFIG_DTB_RESELECT */

/* Crashdump levels */
enum {
	FULL_DUMP,
	MINIMAL_DUMP
};

struct dumpinfo_t {
	char name[256]; /* use only file name in 8.3 format */
	uint32_t start;
	uint32_t size;
	int is_aligned_access; /* non zero represent 4 byte access */
	uint32_t is_redirected; /* If this flag is set, 'start' is considered
				 * a ptr to address to be dumped
				 */
	uint32_t offset; /* offset to be added to start address */
	uint32_t dump_level;
	uint32_t to_compress; /* non-zero represent for compressed dump*/
};

/*
 * Extern variables
 */
extern struct dumpinfo_t * dumpinfo;
extern int * dump_entries;

/*
 * Function declaration
 */
unsigned int get_which_flash_param(char *part_name);
int get_current_board_flash_config(void);
ipq_smem_flash_info_t * get_ipq_smem_flash_info(void);
socinfo_t * get_socinfo(void);
uint32_t get_part_block_size(struct smem_ptn *p, ipq_smem_flash_info_t *sfi);
struct smem_ptable * get_ipq_part_table_info(void);
int getpart_offset_size(char *part_name, uint32_t *offset, uint32_t *size);
unsigned int get_rootfs_active_partition(void);
int mibib_ptable_init(unsigned int* addr);
void get_kernel_fs_part_details(void);
#ifdef CONFIG_MMC
int part_get_info_efi_by_name(const char *name, struct disk_partition *info);
#endif
#ifdef CONFIG_CMD_UBI
int init_ubi_part(void);
#endif
void fdt_fixup_flash(void *blob);
void reset_crashdump(void);
#endif
