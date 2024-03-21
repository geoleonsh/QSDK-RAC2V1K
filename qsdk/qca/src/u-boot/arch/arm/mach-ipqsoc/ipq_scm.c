/*
 * Copyright (c) 2015-2017 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#include <ubi_uboot.h>
#include <linux/stddef.h>
#include <linux/compat.h>
#include <linux/dma-mapping.h>
#include <linux/arm-smccc.h>
#include <errno.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <hang.h>
#include <mach/ipq_scm.h>
#include <common.h>
#include <asm/system.h>

static u64 qcom_smccc_convention = -1;

void __qcom_scm_init(void)
{
	u64 cmd;
	struct arm_smccc_res res;
	u32 function = QCOM_SCM_FNID(SCM_SVC_INFO, IS_CALL_AVAIL_CMD);

	/* First try a SMC64 call */
	cmd = ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64,
				 ARM_SMCCC_OWNER_SIP, function);

	arm_smccc_smc(cmd, QCOM_SCM_ARGS(1), cmd & (~BIT(ARM_SMCCC_TYPE_SHIFT)),
		      0, 0, 0, 0, 0, &res);

	if (!res.a0 && res.a1)
		qcom_smccc_convention = ARM_SMCCC_SMC_64;
	else
		qcom_smccc_convention = ARM_SMCCC_SMC_32;
}

static int scm_remap_error(int err)
{
	switch (err) {
	case SCM_ERROR:
		return -EIO;
	case SCM_EINVAL_ADDR:
	case SCM_EINVAL_ARG:
		return -EINVAL;
	case SCM_EOPNOTSUPP:
		return -EOPNOTSUPP;
	case SCM_ENOMEM:
		return -ENOMEM;
	case SCM_EBUSY:
		return -EBUSY;
	}
	return -EINVAL;
}

/**
 * qcom_scm_call() - Invoke a syscall in the secure world
 * @dev:	device
 * @owner_id:	owner identifier
 * @svc_id:	service identifier
 * @cmd_id:	command identifier
 * @desc:	Descriptor structure containing arguments and return values
 *
 * Sends a command to the SCM and waits for the command to finish processing.
 * This should *only* be called in pre-emptible context.
*/
static int qcom_scm_call(u32 owner_id, u32 svc_id,
			 u32 cmd_id, const struct qti_scm_desc *desc,
			 struct arm_smccc_res *res)
{
	int arglen = desc->arginfo & 0xf;
	int retry_count = 0, i;
	u32 fn_id = QCOM_SCM_FNID(svc_id, cmd_id);
	u64 cmd, x5 = desc->args[FIRST_EXT_ARG_IDX];
	dma_addr_t args_phys = 0;
	void *args_virt = NULL;
	size_t alloc_len;
	struct arm_smccc_quirk quirk = {.id = ARM_SMCCC_QUIRK_QCOM_A6};

	__qcom_scm_init();

	if (unlikely(arglen > N_REGISTER_ARGS)) {
		alloc_len = N_EXT_QCOM_SCM_ARGS * sizeof(u64);
		args_virt = kzalloc(PAGE_ALIGN(alloc_len), GFP_KERNEL);

		if (!args_virt)
			return -ENOMEM;

		if (qcom_smccc_convention == ARM_SMCCC_SMC_32) {
			__le32 *args = args_virt;

			for (i = 0; i < N_EXT_QCOM_SCM_ARGS; i++)
				args[i] = cpu_to_le32(desc->args[i +
						      FIRST_EXT_ARG_IDX]);
		} else {
			__le64 *args = args_virt;

			for (i = 0; i < N_EXT_QCOM_SCM_ARGS; i++)
				args[i] = cpu_to_le64(desc->args[i +
						      FIRST_EXT_ARG_IDX]);
		}

		args_phys = dma_map_single(args_virt, alloc_len,
					   DMA_TO_DEVICE);

		x5 = args_phys;
	}

	do {

		cmd = ARM_SMCCC_CALL_VAL(ARM_SMCCC_STD_CALL,
					 qcom_smccc_convention,
					 owner_id, fn_id);

		quirk.state.a6 = 0;

		do {
			arm_smccc_smc_quirk(cmd, desc->arginfo, desc->args[0],
				      desc->args[1], desc->args[2], x5,
				      quirk.state.a6, 0, res, &quirk);

			if (res->a0 == SCM_INTERRUPTED)
				cmd = res->a0;

		} while (res->a0 == SCM_INTERRUPTED);


		if (res->a0 == SCM_V2_EBUSY) {
			if (retry_count++ > SCM_EBUSY_MAX_RETRY)
				break;
			mdelay(SCM_EBUSY_WAIT_MS);
		}
	}  while (res->a0 == SCM_V2_EBUSY);

	if (args_virt) {
		dma_unmap_single(args_phys, alloc_len, DMA_TO_DEVICE);
		kfree(args_virt);
	}

	if ((long)res->a0 < 0)
		return scm_remap_error(res->a0);

	return 0;
}

void __attribute__ ((noreturn)) jump_kernel(void *kernel_entry,
				void *fdt_addr)
{
	struct qti_scm_desc desc = {0};
	int ret = 0;
	kernel_params param = {0};
	struct arm_smccc_res res;

	desc.arginfo = QCOM_SCM_ARGS(2, SCM_READ_OP);
	desc.args[1] = sizeof(param);
#ifdef CONFIG_CPU_V7A
	param.kernel_start = (u32)kernel_entry;
	param.reg_x0 = (u32)fdt_addr;
	desc.args[0] = (u32) &param;
	printf("Jumping to AARCH64 kernel via monitor\n");
#elif CONFIG_ARM64
	param.kernel_start = (ulong)kernel_entry;
	param.reg_x2 = (uintptr_t)fdt_addr;
	desc.args[0] = (uintptr_t) &param;
	printf("Jumping to AARCH32 kernel via monitor\n");
#endif
	ret = qcom_scm_call(SCM_OWNR_SIP, SCM_ARCH64_SWITCH_ID,\
				SCM_EL1SWITCH_CMD_ID, &desc, &res);

	printf("Can't boot kernel: %d\n", ret);
	hang();
}

int qti_scm_call_read(u32 svc_id, u32 cmd_id, u32 *addr, u32 *val)
{
        int ret = 0;
        struct qti_scm_desc desc = {0};
	struct arm_smccc_res res;

        /* In ipq807x, this SCM call is called as a Fast
         * SCM call which means it will get executed in
         * EL3 monitor mode itself without jumping to QSEE.
         * But, In ipq6018, We need to jump into QSEE which
         * will execute the SCM call, as we do not have
         * support for Fast SCM call in ipq6018.
         */

        desc.arginfo = QCOM_SCM_ARGS(1, SCM_VAL);

        desc.args[0] = (uintptr_t)addr;
	ret = qcom_scm_call(SCM_OWNR_SIP, svc_id,\
				cmd_id, &desc, &res);
	*val = res.a1;
        return ret;
}

int qti_scm_call_write(u32 svc_id, u32 cmd_id, u32 *addr, u32 val)
{
	int ret = 0;
        struct qti_scm_desc desc = {0};
	struct arm_smccc_res res;

	/* In ipq807x, this SCM call is called as a Fast
	 * SCM call which means it will get executed in
	 * EL3 monitor mode itself without jumping to QSEE.
	 * But, In ipq6018, We need to jump into QSEE which
	 * will execute the SCM call, as we do not have
	 * support for Fast SCM call in ipq6018.
	 */

	desc.arginfo = QCOM_SCM_ARGS(2, SCM_VAL, SCM_VAL);

	desc.args[0] = (uintptr_t)addr;
	desc.args[1] = val;
	ret = qcom_scm_call(SCM_OWNR_SIP, svc_id,\
				cmd_id, &desc, &res);
	return ret;
}

int qca_scm_sdi(void)
{
        int ret;
        struct qti_scm_desc desc = {0};
	struct arm_smccc_res res;

	desc.args[0] = 1ul;    /* Disable wdog debug */
	desc.args[1] = 0ul;    /* SDI Enable */
	desc.arginfo = QCOM_SCM_ARGS(2, SCM_VAL, SCM_VAL);
	ret = qcom_scm_call(SCM_OWNR_SIP, SCM_SVC_BOOT,\
				SCM_CMD_TZ_CONFIG_HW_FOR_RAM_DUMP_ID,
				&desc, &res);
	if (ret)
		return ret;

	return desc.ret[0];
}

int qca_scm_dload(u32 *tcsr_addr, u32 magic_cookie)
{
	return qti_scm_call_write(SCM_SVC_IO, SCM_IO_WRITE,
			tcsr_addr, magic_cookie);
}
