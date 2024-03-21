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

#include <linux/module.h>
#include <ppe_drv_public.h>
#include <ppe_acl.h>
#include "ppe_acl_stats.h"

/*
 * Max number of ACL rules IDs.
 */
#define PPE_ACL_USER_RULE_ID_MAX 1024		/* Maximum number of user rule ID */
#define PPE_ACL_POLICER_RULE_ID_BASE (PPE_ACL_USER_RULE_ID_MAX + 1)
						/* Policer rule ID */
#define PPE_ACL_RULE_ID_MAX 2048		/* Maximum number of ACL rule ID */

/*
 * PPE ACL debug macros
 */
#if (PPE_ACL_DEBUG_LEVEL == 3)
#define ppe_acl_assert(c, s, ...)
#else
#define ppe_acl_assert(c, s, ...) if (!(c)) { printk(KERN_CRIT "%s[%d]:" s, __FUNCTION__, __LINE__, ##__VA_ARGS__); BUG_ON(!(c)); }
#endif

#if defined(CONFIG_DYNAMIC_DEBUG)
/*
 * If dynamic debug is enabled, use pr_debug.
 */
#define ppe_acl_warn(s, ...) pr_debug("%s[%d]:" s, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define ppe_acl_info(s, ...) pr_debug("%s[%d]:" s, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define ppe_acl_trace(s, ...) pr_debug("%s[%d]:" s, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else

/*
 * Statically compile messages at different levels, when dynamic debug is disabled.
 */
#if (PPE_ACL_DEBUG_LEVEL < 2)
#define ppe_acl_warn(s, ...)
#else
#define ppe_acl_warn(s, ...) pr_warn("%s[%d]:" s, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#endif

#if (PPE_ACL_DEBUG_LEVEL < 3)
#define ppe_acl_info(s, ...)
#else
#define ppe_acl_info(s, ...) pr_notice("%s[%d]:" s, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#endif

#if (PPE_ACL_DEBUG_LEVEL < 4)
#define ppe_acl_trace(s, ...)
#else
#define ppe_acl_trace(s, ...) pr_info("%s[%d]:" s, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#endif
#endif

/*
 * ppe_acl
 *	Structure for acl rule.
 */
struct ppe_acl {
	/*
	 * list head for active list
	 */
	struct list_head list;			/* List head for active list */

	/*
	 * basic info
	 */
	uint32_t rule_valid_flag;		/* Rule valid flags */
	uint32_t rule_id;			/* Associated rule ID */
	struct ppe_drv_acl_ctx *ctx;		/* PPE driver rule context */
	uint16_t pri;				/* Rule priority */
	ppe_drv_acl_ipo_t ipo;			/* IPO type */

	/*
	 * Rule information extracted from user rule.
	 */
	struct ppe_drv_acl_rule info;

	/*
	 * References
	 */
	struct kref ref_cnt;			/* Reference count */

	/*
	 * service code for flow combination
	 */
	uint8_t sc;

	/*
	 * callback information for this acl rule.
	 */
	ppe_acl_rule_callback_t cb;		/* callback for this ACL id. */
	void *app_data;				/* app data to be passed in the cb. */

	/*
	 * TODO: Other book keeping info for stats and ACL dump.
	 */
};

/*
 * ppe_acl_base
 *	PPE ACL base structure
 */
struct ppe_acl_base {
	spinlock_t lock;				/* ACL lock */

	/*
	 * General stats
	 */
	struct ppe_acl_stats stats;			/* ACL statistics */
	struct dentry *dentry;				/* Debugfs entry */

	/*
	 * Pointer to memory pool for rule table
	 */
	struct ppe_acl_rule_id *rule_id_tbl;		/* Memory for ACL rule ID table */
	struct kref ref;				/* Reference count */

	/*
	 * Active list of ACL rules
	 */
	struct list_head active_rules;			/* List of active ACL rules */
};

extern struct ppe_acl_base ppe_acl_gbl;

/*
 * ACL manager APIs.
 */
void ppe_acl_deinit(void);
void ppe_acl_init(struct dentry *d_rule);

