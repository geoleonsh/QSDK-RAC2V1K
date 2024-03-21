/*
 * Copyright (c) 2015-2018, 2020 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <stdio.h>
#include <memalign.h>
#include <command.h>
#include <common.h>
#include <time.h>
#include <cpu_func.h>
#include <mach/ipq_scm.h>
#include "ipq_board.h"

static int ipq_read_tcsr_boot_misc(void)
{
	u32 *dmagic = TCSR_BOOT_MISC_REG;
	return *dmagic;
}

static int ipq_iscrashed_crashdump_disabled(void)
{
	u32 dmagic = ipq_read_tcsr_boot_misc();
	return ((dmagic & DLOAD_DISABLED) ? 1 : 0);
}

static int ipq_iscrashed(void)
{
	u32 dmagic = ipq_read_tcsr_boot_misc();
	return ((dmagic & DLOAD_MAGIC_COOKIE) ? 1 : 0);
}

static int dump_to_dst(int is_aligned_access, uint32_t memaddr,
		uint32_t size, char *name)
{
	char *dumpdir;
	char runcmd[256];

	if (is_aligned_access) {
		if (CONFIG_SYS_LOAD_ADDR) {
			snprintf(runcmd, sizeof(runcmd),
					"cp.l 0x%x 0x%x 0x%x", memaddr,
					CONFIG_SYS_LOAD_ADDR, size / 4);
			if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
				return CMD_RET_FAILURE;

			memaddr = CONFIG_SYS_LOAD_ADDR;
		} else {
			printf("%s needs aligned access and temp address "
					"is not defined. Skipping...", name);
			return CMD_RET_FAILURE;
		}
	}

	dumpdir = env_get("dumpdir");
	if (dumpdir != NULL)
		printf("Using directory %s in TFTP server\n", dumpdir);
	else {
		dumpdir = "";
		printf("Env 'dumpdir' not set. Using / dir in TFTP server\n");
	}

	snprintf(runcmd, sizeof(runcmd), "tftpput 0x%x 0x%x %s/%s",
						memaddr, size, dumpdir, name);
	if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
		return CMD_RET_FAILURE;

	return CMD_RET_SUCCESS;
}

static int ipq_do_dump_data(unsigned int dump_level)
{
	unsigned int indx;
	int ret = CMD_RET_SUCCESS;

	for (indx = 0; indx < *dump_entries; indx++) {
		if (dump_level != dumpinfo[indx].dump_level)
			continue;

		printf("\nProcessing %s:\n", dumpinfo[indx].name);
		ret = dump_to_dst(dumpinfo[indx].is_aligned_access,
				dumpinfo[indx].start, dumpinfo[indx].size,
				dumpinfo[indx].name);
		if (ret == CMD_RET_FAILURE)
			goto stop_dump;

	}

stop_dump:
	return ret;
}

static void ipq_dump_func(unsigned int dump_level)
{
	uint64_t etime;

	etime = get_timer(0) + (10 * CONFIG_SYS_HZ);
	printf("\nHit any key within 10s to stop dump activity...");
	while (!tstc()) {       /* while no incoming data */
		if (get_timer(0) >= etime) {
			if (ipq_do_dump_data(dump_level) == CMD_RET_FAILURE)
				printf("Crashdump saving failed!\n");
			break;
		}
	}

	/* reset the system, some images might not be loaded
	 * when crashmagic is found
	 */
	reset_cpu();
}

void reset_crashdump(void)
{
	int ret = 0;
	unsigned int cookie = ipq_read_tcsr_boot_misc();

	qca_scm_sdi();
	if (cookie & DLOAD_ENABLE)
		cookie |= CRASHDUMP_RESET;

	cookie &= DLOAD_DISABLE;
	ret = qca_scm_dload(TCSR_BOOT_MISC_REG, cookie);
	if (ret)
		printf ("Error in reseting the Magic cookie\n");
	return;
}


int do_crashdump(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	if (ipq_iscrashed()) {
		printf("Crashdump magic found, "
				"initializing dump activity..\n");
		ipq_dump_func(FULL_DUMP);
	}

	if (ipq_iscrashed_crashdump_disabled()) {
		printf("Crashdump disabled, resetting the board..\n");
		reset_cpu();
	}

	return 0;
}

U_BOOT_CMD(
	crashdump, 1, 0, do_crashdump,
	"Collect crashdump from ipq",
	"crashdump - Collect crashdump if IPQ is crashed\n"
	);
