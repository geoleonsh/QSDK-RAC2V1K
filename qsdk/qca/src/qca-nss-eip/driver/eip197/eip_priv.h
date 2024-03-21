/*
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef __EIP_PRIV_H
#define __EIP_PRIV_H

#include <linux/platform_device.h>
#include <linux/ip.h>
#include "eip.h"
#include "eip_dma.h"
#include "eip_hw.h"
#include "eip_ctx.h"
#include "eip_tk.h"
#include "eip_tr.h"
#include "eip_flow.h"

#define EIP_MAX_STR_LEN 25U		/* Maximum string length */
#define EIP_DEBUGFS_MAX_NAME 128U	/* Maximum string length for debugfs */

/*
 * TODO: what should be correct value?
 */
#define EIP_RX_BUFFER_HEADROOM 128U
#define EIP_RX_BUFFER_TAILROOM 192U
#define EIP_RX_BUFFER_DATA_LEN 1500U
#define EIP_RX_BUFFER_SIZE ((EIP_RX_BUFFER_HEADROOM) + (EIP_RX_BUFFER_TAILROOM) + (EIP_RX_BUFFER_DATA_LEN))

#define ASSERT(x) \
do { if (unlikely(!(x))) { panic("ASSERT FAILED at (%s:%d): %s\n", __FILE__, __LINE__, #x); } } while (0)

/*
 * eip_svc_entry
 *	Algorithm information.
 */
struct eip_svc_entry {
	char name[CRYPTO_MAX_ALG_NAME];		/* Algo name as per cra-driver name */

	eip_tk_proc_t enc_tk_fill;		/* Token fill method for encode operation */
	eip_tk_proc_t dec_tk_fill;		/* Token fill method for decode operation */
	eip_tk_proc_t auth_tk_fill;		/* Token fill method for auth operation */
	eip_tr_init_t tr_init;			/* TR initialization */

	uint32_t ctrl_words_0;			/* Initial control word 0 for HW */
	uint32_t ctrl_words_1;			/* Initial control word 1 for HW */

	uint16_t auth_block_len;		/* Authentication Block length */
	uint16_t auth_digest_len;		/* Authentication digest size */
	uint16_t auth_state_len;		/* Intermidiate state length of hash operation */

	uint16_t iv_len;			/* Cipher IV length */
	uint8_t cipher_blk_len;			/* Cipher block length */
};

/*
 * eip_frag
 *	Fragment information. Usally allocated on stack.
 */
struct eip_frag {
	void *data;		/* Virtual address of data fragment */
	uint32_t flag;		/* flags to use in descriptor configuration */
	uint32_t idx;		/* descriptor index */
	uint16_t len;		/* Length of data fragment */
};

/*
 * eip_sw_desc
 *	SW descriptor for storing Meta information.
 */
struct eip_sw_desc {
	struct eip_tr *tr;		/* Transform record */
	struct eip_tk *tk;		/* Transform token */
	eip_dma_callback_t comp;		/* HW Rx completion callback */
	eip_dma_err_callback_t err_comp;	/* HW Rx completion with error callback */

	uint32_t tk_hdr;		/* Command token[2] */
	uint32_t tk_addr;		/* Control token address (EIP96 instructions) */
	uint32_t tr_addr_type;		/* TR physical address with lower 2 bits OR'd with type */
	uint32_t tk_words;		/* Command Frag[0] Contol token length in words */
	uint32_t hw_svc;		/* HW service */

	uint16_t src_nsegs;		/* Source data segments */
	uint16_t dst_nsegs;		/* Destination data segments */

	eip_req_t req;			/* Request associated with desc */
};

/*
 * eip_drv
 *	Global Driver structure.
 */
struct eip_drv {
	struct dentry *dentry;			/* Root package debugfs dentry */
	struct platform_device *pdev;		/* Device associated with the driver */
};

/*
 * eip_pdev
 *	Platform device data.
 */
struct eip_pdev {
	struct platform_device *pdev;		/* Device associated with the driver */
	struct eip_dma la[NR_CPUS];		/* Lookaside DMA object per CPU */
	struct eip_dma hy[NR_CPUS];		/* Hybrid DMA object per CPU */
	struct kmem_cache *tr_cache;		/* Transform Record cache */
	struct kmem_cache *flow_swcache;	/* Flow Record cache */
	struct eip_flow_hw *flow_hwcache;       /* DMA memory for HW flow */
	struct dentry *dentry;			/* Driver debugfs dentry */
	void __iomem *dev_vaddr;		/* starting virtual address of device */
	dma_addr_t dev_paddr;			/* starting physical address of device */
	dma_addr_t hwcache_paddr;		/* Physical address for HW flow */
	bool redirect_en;			/* Redirection / Hybrid mode is configured */
	bool dma_refill_req;			/* Hybrid DMA refill required */
	struct eip_flow_tbl flow_table;		/* Flow table */
};

extern struct eip_drv eip_drv_g;	/* Global Driver object */

#endif /* __EIP_PRIV_H */
