/* Copyright (c) 2015-2017 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __MACH_SCM_H
#define __MACH_SCM_H

#include <linux/errno.h>
#define SCM_SVC_BOOT			0x1
#define SCM_SVC_PIL			0x2
#define SCM_SVC_UTIL			0x3
#define SCM_SVC_TZ			0x4
#define SCM_SVC_IO			0x5
#define SCM_SVC_INFO			0x6
#define SCM_SVC_SSD			0x7
#define SCM_SVC_FUSE			0x8
#define SCM_SVC_PWR			0x9
#define SCM_SVC_CRYPTO			0xA
#define SCM_SVC_CP			0xC
#define SCM_SVC_DCVS			0xD
#define SCM_SVC_TZSCHEDULER		0xFC
#define SCM_SVC_WR			0x10
#define SCM_SVC_RD			0x12
#define QFPROM_IS_AUTHENTICATE_CMD	0x7
#define TZBSP_BUILD_VER_QUERY_CMD	0x4
#define SCM_BOOT_ADDR			0x1
#define SCM_FLAG_COLDBOOT_CPU1		0x1
#define SCM_SVC_ID_SHIFT		0xA
#define IS_CALL_AVAIL_CMD		0x1
#define PART_INFO_CMD			0x22
#define GET_SECURE_STATE_CMD		0x4
#define QTI_SCM_SVC_RESETTYPE_CMD	0x18

#ifdef CONFIG_IPQ_BT_SUPPORT
#define SCM_PAS_INIT_IMAGE_CMD		0x1
#define SCM_PAS_AUTH_AND_RESET_CMD	0x5
#define SCM_CMD_OTP  			0x15
#define SCM_SVC_BT_ECO_BIT		0x2
#define SCM_BT_ECO_BIT_TOGGLE_CMD	0x21
#endif

#define KERNEL_AUTH_CMD				0x1E
#define SCM_CMD_SEC_AUTH			0x1F
#define SCM_CMD_TZ_CONFIG_HW_FOR_RAM_DUMP_ID	0x9
#define SCM_CMD_TZ_FORCE_DLOAD_ID		0x10

/* scm_v8 */
#define SCM_VAL				0x0
#define SCM_IO_READ			0x1
#define SCM_IO_WRITE			0x2

#define CE_CHN_SWITCH_CMD		0x2

#define SCM_EBUSY_WAIT_MS	30
#define SCM_EBUSY_MAX_RETRY	20
#define SCM_V2_EBUSY		-12
#define SCM_EBUSY		-6
#define SCM_ENOMEM		-5
#define SCM_EOPNOTSUPP		-4
#define SCM_EINVAL_ADDR		-3
#define SCM_EINVAL_ARG		-2
#define SCM_ERROR		-1
#define SCM_INTERRUPTED		 1

/* OWNER IDs */
#define SCM_OWNR_SIP		2
#define SCM_OWNR_QSEE_OS	50
#define SCM_OWNR_TEE_HLOS	51

/* SVC IDs  */
#define SCM_SVC_APP_MGR		1	/* Application service manager */
#define SCM_SVC_LISTENER	2	/* Listener service manager */
#define SCM_SVC_EXTERNAL	3	/* External Image loading */
#define SCM_SVC_MON_SAT		252	/* Monitor SAT test calls */
#define SCM_SVC_TEST_1		253	/* TZ test calls (continued). */
#define SCM_SVC_TEST_0		254	/* TZ test calls */

/* to align the pointer to the (next) page boundary */
#define PAGE_ALIGN(addr) ALIGN(addr, PAGE_SIZE)

/**
 * struct scm_command - one SCM command buffer
 * @len: total available memory for command and response
 * @buf_offset: start of command buffer
 * @resp_hdr_offset: start of response buffer
 * @id: command to be executed
 * @buf: buffer returned from scm_get_command_buffer()
 *
 * An SCM command is laid out in memory as follows:
 *
 *	------------------- <--- struct scm_command
 *	| command header  |
 *	------------------- <--- scm_get_command_buffer()
 *	| command buffer  |
 *	------------------- <--- struct scm_response and
 *	| response header |      scm_command_to_response()
 *	------------------- <--- scm_get_response_buffer()
 *	| response buffer |
 *	-------------------
 *
 * There can be arbitrary padding between the headers and buffers so
 * you should always use the appropriate scm_get_*_buffer() routines
 * to access the buffers in a safe manner.
 */
struct scm_command {
	u32	len;
	u32	buf_offset;
	u32	resp_hdr_offset;
	u32	id;
	u32	buf[0];
};

/**
 * struct scm_response - one SCM response buffer
 * @len: total available memory for response
 * @buf_offset: start of response data relative to start of scm_response
 * @is_complete: indicates if the command has finished processing
 */
struct scm_response {
	u32	len;
	u32	buf_offset;
	u32	is_complete;
};


int qti_scm_call_read(u32, u32, u32 *, u32*);
#define MAX_QTI_SCM_RETS		3
#define MAX_QTI_SCM_ARGS		10
#define SCM_READ_OP			1

/**
 * struct qti_scm_desc
 *  <at> arginfo: Metadata describi`ng the arguments in args[]
 *  <at> args: The array of arguments for the secure syscall
 *  <at> ret: The values returned by the secure syscall
 *  <at> extra_arg_buf: The buffer containing extra arguments
                        (that don't fit in available registers)
 *  <at> x5: The 4rd argument to the secure syscall or physical address of
                extra_arg_buf
 */
struct qti_scm_desc {
	u32 arginfo;
	u64 args[MAX_QTI_SCM_ARGS];
	u32 ret[MAX_QTI_SCM_RETS];

	/* private */
	void *extra_arg_buf;
	u64 x5;
};

#define QCOM_SCM_FNID(s, c) ((((s) & 0xFF) << 8) | ((c) & 0xFF))

#define QCOM_SCM_ARGS_IMPL(num, a, b, c, d, e, f, g, h, i, j, ...) (\
			(((a) & 0x3) << 4) | \
			(((b) & 0x3) << 6) | \
			(((c) & 0x3) << 8) | \
			(((d) & 0x3) << 10) | \
			(((e) & 0x3) << 12) | \
			(((f) & 0x3) << 14) | \
			(((g) & 0x3) << 16) | \
			(((h) & 0x3) << 18) | \
			(((i) & 0x3) << 20) | \
			(((j) & 0x3) << 22) | \
			((num) & 0xf))

#define QCOM_SCM_ARGS(...) QCOM_SCM_ARGS_IMPL(__VA_ARGS__,\
				 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

#define QCOM_SCM_SIP_FNID(s, c) (((((s) & 0xFF) << 8) | \
			((c) & 0xFF)) | 0x02000000)


#define QCOM_SMC_ATOMIC_MASK            0x80000000


#define MAX_QCOM_SCM_ARGS 10
#define N_EXT_QCOM_SCM_ARGS 7
#define FIRST_EXT_ARG_IDX 3
#define N_REGISTER_ARGS (MAX_QCOM_SCM_ARGS - N_EXT_QCOM_SCM_ARGS + 1)

typedef struct {
#ifdef CONFIG_CPU_V7A
	u64 reg_x0;
	u64 reg_x1;
	u64 reg_x2;
	u64 reg_x3;
	u64 reg_x4;
	u64 reg_x5;
	u64 reg_x6;
	u64 reg_x7;
	u64 reg_x8;
	u64 kernel_start;
#endif
#ifdef CONFIG_ARM64
	uintptr_t reg_x0;
	uintptr_t reg_x1;
	uintptr_t reg_x2;
	uintptr_t reg_x3;
	uintptr_t reg_x4;
	uintptr_t reg_x5;
	uintptr_t reg_x6;
	uintptr_t reg_x7;
	uintptr_t reg_x8;
	uintptr_t kernel_start;
#endif
} kernel_params;

#define SCM_ARCH64_SWITCH_ID	0x1
#define SCM_EL1SWITCH_CMD_ID	0xf

#define SCM_NULL_OP 0
#define	SCM_RW_OP   2
#define	SCM_BUF_VAL 3

#define PHYA0_RFA_RFA_RFA_OTP_OTP_OV_1		0xC5D4484

void __attribute__ ((noreturn)) jump_kernel(void *kernel_entry,
		void *fdt_addr);
int qca_scm_sdi(void);
int qca_scm_dload(u32 *tcsr_addr, u32 magic_cookie);
#endif
