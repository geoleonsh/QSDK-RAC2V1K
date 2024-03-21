/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
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

#include <fal/fal_acl.h>
#include <fal/fal_api.h>
#include <ppe_drv_acl.h>

/*
 * ACL list ID macros.
 */
#define PPE_DRV_ACL_LIST_ID_MAX 		1024
#define PPE_DRV_ACL_LIST_ID_USED		1
#define PPE_DRV_ACL_LIST_ID_FREE		2
#define PPE_DRV_ACL_LIST_ID_IPO_MAX		512
#define PPE_DRV_ACL_LIST_ID_IPO_START 		0
#define PPE_DRV_ACL_LIST_ID_IPO_END		(PPE_DRV_ACL_LIST_ID_IPO_MAX - 1)
#define PPE_DRV_ACL_LIST_ID_PRE_IPO_START	(PPE_DRV_ACL_LIST_ID_IPO_MAX)
#define PPE_DRV_ACL_LIST_ID_PRE_IPO_END		(PPE_DRV_ACL_LIST_ID_MAX - 1)

/*
 * RULE macros.
 */
#define PPE_DRV_ACL_RULE_ID 1
#define PPE_DRV_ACL_RULE_NR 1
#define PPE_DRV_ACL_HW_INDEX_MAX 1024

/*
 * ppe_drv_acl_slice
 *	ACL slice
 */
struct ppe_drv_acl_slice {
	uint16_t index;				/* ACL slice index. */
	struct ppe_drv_acl_rule_match_one rule;	/* ACL single rule. */
};

/*
 * ppe_drv_acl_ctx
 *	ACL rule context
 */
struct ppe_drv_acl_ctx {
	uint8_t req_slices;			/* Required number of slices. */
	uint8_t total_slices;			/* Total number of slices used for this rule. */
	int16_t list_id;			/* Associated list ID. */
	bool rule_valid;			/* Rule valid flag to handle failure with partial configuration. */
	bool rule_type_valid;			/* Indicate if rule type is already set. */
	ppe_drv_acl_ipo_t type;			/* IPO type - IPO or pre-IPO? */
	fal_acl_rule_t fal_rule;		/* FAL rule structure */
};

/*
 * ppe_drv_acl_row
 *	ACL row
 */
struct ppe_drv_acl_row {
	uint8_t row_num;			/* ACL rule number. */
	uint8_t slice_bitmap;			/* Used slice bitmap. */
	struct ppe_drv_acl_slice slices[PPE_DRV_ACL_SLICE_PER_ROW];
						/* Individual slices per row. */
};

/*
 * ppe_drv_acl_sc
 *	FLOW ACL service code
 */
struct ppe_drv_acl_sc {
	ppe_drv_sc_t sc;			/* Associated service code for flow binding. */
	uint8_t in_use;				/* Indicate if service code is in use. */
};

/*
 * ppe_drv_acl
 *	Complete list of rows and actions
 */
struct ppe_drv_acl {
	uint8_t list_id[PPE_DRV_ACL_LIST_ID_MAX];		/* List IDs */
	struct ppe_drv_acl_sc acl_sc[PPE_DRV_SC_FLOW_ACL_MAX];	/* List of service codes for flow/policer binding. */
	ppe_drv_acl_flow_callback_t flow_add_cb;		/* Flow add callback when flow needs to be attached with ACL. */
	ppe_drv_acl_flow_callback_t flow_del_cb;		/* Flow delete callback when flow needs to detached from ACL. */
	void *flow_app_data;					/* Flow callback app data. */
	ppe_drv_acl_mirror_core_select_cb_t mirror_core_cb;	/* DP callback to select core for mirrored packets. */
	void *mirror_core_app_data;				/* Mirror core selection callback app data. */
};

/*
 * ppe_drv_acl_tbl
 *	PPE drv ACL based actions framework
 *	TODO: move it to the PPE RULE module.
 */
struct ppe_drv_acl_tbl {
	uint16_t hw_index;				/* Hardware ACL index */
	uint16_t acl_id;				/* Software ACL id */
	void *app_data;					/* App data */
	void *mirror_app_data;				/* Mirror callback App data */
	ppe_drv_acl_process_callback_t cb;		/* Per ACL index registered callback */
	ppe_drv_acl_process_callback_t mirror_cb;	/* Per ACL mirrored registered callback */
};

/*
 * Internal APIs.
 */
void ppe_drv_acl_entries_free(struct ppe_drv_acl *acl);
void ppe_drv_acl_tbl_entries_free(struct ppe_drv_acl_tbl *acl_tbl);
struct ppe_drv_acl *ppe_drv_acl_entries_alloc(void);
struct ppe_drv_acl_tbl *ppe_drv_acl_tbl_entries_alloc(void);
