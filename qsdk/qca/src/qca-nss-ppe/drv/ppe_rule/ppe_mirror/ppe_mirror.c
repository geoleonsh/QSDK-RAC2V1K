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
#include <linux/skbuff.h>
#include <linux/vmalloc.h>
#include <linux/debugfs.h>
#include <linux/netdevice.h>
#include <ppe_drv.h>
#include <ppe_drv_cc.h>
#include <ppe_drv_acl.h>
#include "ppe_mirror.h"

struct ppe_mirror gbl_ppe_mirror = {0};

/*
 * ppe_mirror_process_skb()
 *	Process mirrored packets from PPE.
 */
bool ppe_mirror_process_skb(void *appdata, struct sk_buff *skb, void *info)
{
	struct ppe_mirror *mirror_g = &gbl_ppe_mirror;
	struct ppe_drv_acl_metadata *acl_info = (struct ppe_drv_acl_metadata *)info;
	struct ppe_mirror_acl_map *mirror_mapping = NULL;
	struct ppe_mirror_group_info *group_info = NULL;
	struct ppe_mirror_acl_stats *acl_stats, *acl_group_stats = NULL;
	uint16_t acl_id = acl_info->acl_id;
	ppe_mirror_capture_callback_t cb = NULL;

	spin_lock_bh(&mirror_g->lock);
	mirror_mapping = &mirror_g->mirror_mapping[acl_id];

	/*
	 * Check if ACL mirror mapping is valid or not.
	 */
	if (!mirror_mapping->is_valid) {
		spin_unlock_bh(&mirror_g->lock);
		ppe_mirror_stats_inc(&mirror_g->stats.acl_mirror_process_mapping_invalid);
		ppe_mirror_warn("%p: Mirror mapping is not valid for ACL id %d\n", mirror_g, acl_id);
		return false;
	}

	/*
	 * Get the group information from the mapping.
	 */
	group_info = mirror_mapping->group_info;
	if (!group_info) {
		spin_unlock_bh(&mirror_g->lock);
		ppe_mirror_stats_inc(&mirror_g->stats.acl_mirror_process_group_invalid);
		ppe_mirror_warn("%p: ACL id is not associated with a group %d\n", mirror_g, acl_id);
		return false;
	}

	/*
	 * Get the callback registered for this group and
	 * send the mirrored packet for further processing.
	 */
	cb = group_info->cb;
	if (!cb) {
		spin_unlock_bh(&mirror_g->lock);
		ppe_mirror_warn("%p: Callback is not associated with a group %d\n", mirror_g, acl_id);
		return false;
	}

	skb->dev = group_info->group_dev;
	acl_stats = &mirror_mapping->acl_stats;
	acl_group_stats = &group_info->acl_stats;
	spin_unlock_bh(&mirror_g->lock);

	/*
	 * Increment the packet count for this ACL ID also for the
	 * group to which this mirrored packet belongs to.
	 * Hand over the mirrored packet to user registered callback.
	 */
	ppe_mirror_update_acl_stats(acl_stats, skb->len);
	ppe_mirror_update_acl_stats(acl_group_stats, skb->len);

	cb(group_info->app_data, skb, group_info->group_dev);

	return true;
}

/*
 * ppe_mirror_group_free()
 *	Free memory for a mirror group.
 */
static void ppe_mirror_group_free(struct ppe_mirror_group_info *group_info)
{
	/*
	 * Free memory for a group
	 */
	kfree(group_info);
}

/*
 * ppe_mirror_group_alloc()
 *	Allocate memory for a mirror group.
 */
static inline struct ppe_mirror_group_info *ppe_mirror_group_alloc(void)
{
	/*
	 * Allocate memory for a new group
	 *
	 * kzalloc with GFP_ATOMIC is used assuming ACL groups can be created
	 * in softirq context as well while processing an SKB in the system.
	 */
	return kzalloc(sizeof(struct ppe_mirror_group_info), GFP_ATOMIC);
}

/*
 * ppe_mirror_get_group_info_for_acl()
 *	Serach the group info for the ACL index.
 *	Create one if it does not exists.
 */
ppe_mirror_ret_t ppe_mirror_get_group_info_for_acl(struct ppe_mirror *mirror_g,
							struct ppe_mirror_acl_mapping_info *map_info,
							struct ppe_mirror_group_info **group_info)
{
	struct ppe_mirror_group_info *cur_group = NULL;
	uint16_t acl_id = map_info->acl_id;
	/*
	 * If active group list is empty, create a new group
	 * for this ACL id.
	 */
	if (list_empty(&mirror_g->active_mirror_groups))
		goto create_group;

	/*
	 * Traverse the list of active groups to find if group info
	 * is present or not - If not present, create one.
	 */
	list_for_each_entry(cur_group, &mirror_g->active_mirror_groups, list) {
		if (cur_group->group_dev == map_info->capture_dev) {
			ppe_mirror_info("%p: Group info: %p Group dev: %p found for ACL ID: %d",
						mirror_g, cur_group, cur_group->group_dev, acl_id);

			/*
			 * Check if in the new request the callback is different
			 * for this mirror group.
			 */
			if (cur_group->cb != map_info->cb) {
				ppe_mirror_warn("%p: Group info: %p cb: %p Missmatch for ACL ID: %d new cb %p:",
						mirror_g, cur_group, cur_group->cb, acl_id, map_info->cb);
				ppe_mirror_stats_inc(&mirror_g->stats.acl_mapping_add_invalid_group_info);
				return PPE_MIRROR_RET_ADD_FAIL_GROUP_INFO_INVALID;
			}

			*group_info = cur_group;
			return PPE_MIRROR_RET_SUCCESS;
		}
	}

create_group:
	/*
	 * Allocate a new group information for the ACL index.
	 */
	*group_info = ppe_mirror_group_alloc();
	if (!(*group_info)) {
		ppe_mirror_warn("%p: Falied to allocate a group for ACL id %d\n", mirror_g, acl_id);
		ppe_mirror_stats_inc(&mirror_g->stats.acl_mapping_add_map_nomem);
		return PPE_MIRROR_RET_ADD_FAIL_NO_MEM;
	}

	/*
	 * Fill the group information and add the group in the
	 * active group list.
	 */
	(*group_info)->group_dev = map_info->capture_dev;
	(*group_info)->cb = map_info->cb;
	(*group_info)->app_data = map_info->app_data;
	list_add(&(*group_info)->list, &mirror_g->active_mirror_groups);
	mirror_g->no_of_active_mirror_groups++;

	return PPE_MIRROR_RET_SUCCESS;
}

/*
 * ppe_mirror_acl_destroy_mapping_table()
 *	Destroy mapping table for an ACL index.
 */
static ppe_mirror_ret_t ppe_mirror_destroy_mapping_tbl(uint16_t acl_id)
{
	struct ppe_mirror *mirror_g = &gbl_ppe_mirror;
	struct ppe_mirror_acl_map *mirror_map = NULL;
	struct ppe_mirror_group_info *group_info = NULL;
	ppe_mirror_ret_t ret;

	spin_lock_bh(&mirror_g->lock);

	/*
	 * Check if this ACL rule has a valid mapping or not.
	 */
	mirror_map = &mirror_g->mirror_mapping[acl_id];
	if (!mirror_map->is_valid) {
		spin_unlock_bh(&mirror_g->lock);
		ppe_mirror_warn("%p: Mirror mapping is not valid for ACL index %d\n", mirror_g, acl_id);
		ret = PPE_MIRROR_RET_DELETE_FAIL_MAPPING_NOT_FOUND;
		ppe_mirror_stats_inc(&mirror_g->stats.acl_mapping_del_fail_map_not_found);
		goto fail;
	}

	/*
	 * Unmap the ACL index to group mapping.
	 * If That group does not have any ACLs mapped, delete the group.
	 */
	group_info = mirror_map->group_info;
	if (!group_info) {
		spin_unlock_bh(&mirror_g->lock);
		ppe_mirror_warn("%p:No mirror group found for ACL index %d\n", mirror_g, acl_id);
		ret = PPE_MIRROR_RET_DELETE_FAIL_GROUP_NOT_FOUND;
		ppe_mirror_stats_inc(&mirror_g->stats.acl_mapping_del_fail_group_not_found);
		goto fail;
	}

	/*
	 * Decrement the number of mappings for the group.
	 * If there are no mappings, destroy the group.
	 */
	group_info->number_of_mappings -= 1;
	if (!group_info->number_of_mappings) {
		list_del(&group_info->list);
		ppe_mirror_group_free(group_info);
		mirror_g->no_of_active_mirror_groups--;
		ppe_mirror_assert(mirror_g->no_of_active_mirror_groups >= 0, "Bad number of active mappings\n");
	}

	/*
	 * At this point, release the ACL to Group mapping.
	 */
	mirror_map->group_info = NULL;
	mirror_map->is_valid = false;

	spin_unlock_bh(&mirror_g->lock);
	ppe_mirror_info("%p:Released the mirror mapping for ACL index %d\n", mirror_g, acl_id);
	return PPE_MIRROR_RET_SUCCESS;

fail:
	return ret;
}

/*
 * ppe_mirror_acl_configure_mapping_table()
 *	Configure mapping table for an ACL ID.
 */
static ppe_mirror_ret_t ppe_mirror_configure_mapping_tbl(struct ppe_mirror_acl_mapping_info *map_info)
{
	struct ppe_mirror *mirror_g = &gbl_ppe_mirror;
	struct ppe_mirror_acl_map *mirror_map = NULL;
	struct ppe_mirror_group_info *group_info = NULL;
	uint16_t acl_id = map_info->acl_id;
	ppe_mirror_ret_t ret;

	spin_lock_bh(&mirror_g->lock);

	/*
	 * Check if this ACL rule has a valid mapping or not.
	 */
	mirror_map = &mirror_g->mirror_mapping[acl_id];
	if (mirror_map->is_valid) {
		spin_unlock_bh(&mirror_g->lock);
		ppe_mirror_warn("%p: Mirror mapping is already valid for ACL index %d\n", mirror_g, acl_id);
		ret = PPE_MIRROR_RET_ADD_FAIL_MAPPING_EXIST;
		ppe_mirror_stats_inc(&mirror_g->stats.acl_mapping_add_map_exist);
		goto fail;
	}

	/*
	 * Check if the group dev info is establish for the device linked with
	 * this ACL id.
	 * If not create a group dev and link the same with this ACL index.
	 */
	ret = ppe_mirror_get_group_info_for_acl(mirror_g, map_info, &group_info);
	if (ret != PPE_MIRROR_RET_SUCCESS) {
		spin_unlock_bh(&mirror_g->lock);
		ppe_mirror_warn("%p: Failed to get group info for ACL %d ret %d\n", mirror_g, acl_id, ret);
		goto fail;
	}

	/*
	 * At this point, establish the mapping and increment the count.
	 */
	mirror_map->group_info = group_info;
	mirror_map->acl_rule_id = acl_id;
	mirror_map->is_valid = true;
	group_info->number_of_mappings += 1;

	spin_unlock_bh(&mirror_g->lock);

	ppe_mirror_info("%p:Establish the mirror mapping for ACL index %d\n", mirror_g, acl_id);
	return PPE_MIRROR_RET_SUCCESS;

fail:
	return ret;
}

/*
 * ppe_mirror_acl_mapping_delete()
 *	PPE mirror mapping delete exported API.
 */
ppe_mirror_ret_t ppe_mirror_acl_mapping_delete(uint16_t acl_id)
{
	struct ppe_mirror *mirror_g = &gbl_ppe_mirror;
	uint16_t hw_index;
	ppe_mirror_ret_t ret;

	ppe_mirror_info("%p: Mapping delete request received for ACL ID: %d", mirror_g, acl_id);
	ppe_mirror_stats_inc(&mirror_g->stats.acl_mapping_del_req);

	/*
	 * Check for Invalid ACL index.
	 */
	if (acl_id < 0 || acl_id >= PPE_MIRROR_ACL_RULE_MAX) {
		ppe_mirror_warn("%p: Invalid rule id received for ACL rule mapping delete %d", mirror_g, acl_id);
		ret = PPE_MIRROR_RET_DELETE_FAIL_MAPPING_INVALID_ACL_ID;
		ppe_mirror_stats_inc(&mirror_g->stats.acl_mapping_del_fail_invalid_rule_id);
		goto fail;
	}

	/*
	 * Destroy the mirror mapping for the ACL index.
	 */
	ret = ppe_mirror_destroy_mapping_tbl(acl_id);
	if (ret != PPE_MIRROR_RET_SUCCESS) {
		ppe_mirror_warn("%p: Failed to destroy mapping table ACL id %d ret %d\n", mirror_g, acl_id, ret);
		goto fail;
	}

	/*
	 * Get the hardware index for the ACL id and dereference the
	 * ACL rule.
	 */
	hw_index = ppe_acl_rule_get_and_deref_hw_idx(acl_id);
	if (hw_index == PPE_ACL_INVALID_HW_INDEX) {
		ppe_mirror_warn("%p: Invalid hw index for ACL rule delete %d", mirror_g, acl_id);
		ret = PPE_MIRROR_RET_DELETE_FAIL_MAPPING_INVALID_ACL_RULE;
		ppe_mirror_stats_inc(&mirror_g->stats.acl_mapping_del_fail_rule_not_found);
		goto fail;
	}

	/*
	 * Unregister the callback for the ACL index.
	 */
	ppe_drv_acl_unregister_mirror_cb(hw_index);

	ppe_mirror_info("%p: Mapping deleted succesfully for ACL rule %d\n", mirror_g, acl_id);
	ppe_mirror_stats_inc(&mirror_g->stats.acl_mapping_del_success);
	ppe_mirror_stats_dec(&mirror_g->stats.acl_mapping_count);
	return PPE_MIRROR_RET_SUCCESS;

fail:
	return ret;
}
EXPORT_SYMBOL(ppe_mirror_acl_mapping_delete);

/*
 * ppe_mirror_acl_mapping_add()
 *	PPE mirror mapping add API.
 */
ppe_mirror_ret_t ppe_mirror_acl_mapping_add(struct ppe_mirror_acl_mapping_info *mapping_info)
{
	struct ppe_mirror *mirror_g = &gbl_ppe_mirror;
	uint16_t acl_id = mapping_info->acl_id;
	uint16_t hw_index;
	ppe_mirror_ret_t ret;

	ppe_mirror_info("%p: Mapping add request: %d", mirror_g, acl_id);
	ppe_mirror_stats_inc(&mirror_g->stats.acl_mapping_add_req);

	/*
	 * Check for invalid ACL ID received.
	 */
	if (mapping_info->acl_id < 0 || mapping_info->acl_id >= PPE_MIRROR_ACL_RULE_MAX) {
		ppe_mirror_warn("%p: Invalid rule id received for ACL rule mapping %d", mirror_g, mapping_info->acl_id);
		ret = PPE_MIRROR_RET_ADD_FAIL_MAPPING_INVALID_ACL_ID;
		ppe_mirror_stats_inc(&mirror_g->stats.acl_mapping_add_fail_invalid_rule_id);
		goto fail3;
	}

	/*
	 * Get the hardware index for the ACL id and take one reference for the
	 * ACL rule.
	 */
	hw_index = ppe_acl_rule_get_and_ref_hw_idx(acl_id);
	if (hw_index == PPE_ACL_INVALID_HW_INDEX) {
		ppe_mirror_warn("%p: ACL Rule not found for ACL rule mapping %d", mirror_g, mapping_info->acl_id);
		ret = PPE_MIRROR_RET_ADD_FAIL_MAPPING_INVALID_ACL_RULE;
		ppe_mirror_stats_inc(&mirror_g->stats.acl_mapping_add_fail_rule_not_found);
		goto fail3;
	}

	/*
	 * Establish the mirror mapping for this ACL ID.
	 */
	ret = ppe_mirror_configure_mapping_tbl(mapping_info);
	if (ret != PPE_MIRROR_RET_SUCCESS) {
		ppe_mirror_warn("%p: Failed to configure mapping table ACL id %d ret %d\n", mirror_g, acl_id, ret);
		goto fail2;
	}

	/*
	 * Register the callback for the ACL index.
	 */
	if (!ppe_drv_acl_register_mirror_cb(hw_index, acl_id, ppe_mirror_process_skb, NULL)) {
		ppe_mirror_warn("%p: Failed to register callback for ACL id %d\n", mirror_g, acl_id);
		ret = PPE_MIRROR_RET_ADD_FAIL_MAPPING_CB_REG;
		ppe_mirror_stats_inc(&mirror_g->stats.acl_mapping_add_fail_cb_reg);
		goto fail1;
	}

	ppe_mirror_info("%p: Mapping added succesfully for ACL rule %d\n", mirror_g, acl_id);
	ppe_mirror_stats_inc(&mirror_g->stats.acl_mapping_add_success);
	ppe_mirror_stats_inc(&mirror_g->stats.acl_mapping_count);
	return PPE_MIRROR_RET_SUCCESS;

fail1:
	ppe_mirror_destroy_mapping_tbl(acl_id);
fail2:
	ppe_acl_rule_get_and_deref_hw_idx(acl_id);
fail3:
	ppe_mirror_warn("%p: Failed to add a mapping for an ACL rule %d Ret %d\n", mirror_g, acl_id, ret);
	return ret;
}
EXPORT_SYMBOL(ppe_mirror_acl_mapping_add);

/*
 * ppe_mirror_enable_capture_core()
 */
ppe_mirror_ret_t ppe_mirror_enable_capture_core(uint8_t core_id)
{
	struct ppe_mirror *mirror_g = &gbl_ppe_mirror;
	uint8_t ret;

	if (core_id < 0 || core_id >= PPE_MIRROR_CAPTURE_CORE_MAX) {
		ppe_mirror_warn("invalid core ID received %d\n", core_id);
		ret = PPE_MIRROR_RET_INVALID_CAPTURE_CORE;
                ppe_mirror_stats_inc(&mirror_g->stats.acl_mapping_invalid_capture_core);
		return ret;
	}

	if (!ppe_drv_acl_enable_mirror_capture_core(core_id)) {
		ppe_mirror_warn("Failed to enable capture core %d\n", core_id);
		ret = PPE_MIRROR_RET_FAIL_EN_CAPTURE_CORE;
		ppe_mirror_stats_inc(&mirror_g->stats.acl_mapping_fail_en_capture_core);
		return ret;
	}

	return PPE_MIRROR_RET_SUCCESS;
}
EXPORT_SYMBOL(ppe_mirror_enable_capture_core);

/*
 * ppe_mirror_deinit()
 *	PPE mirror deinit API.
 */
void ppe_mirror_deinit(void)
{
	ppe_mirror_stats_debugfs_exit();
}

/*
 * ppe_mirror_init()
 *	PPE Mirror init API.
 */
void ppe_mirror_init(struct dentry *d_rule)
{
	struct ppe_mirror *mirror_g = &gbl_ppe_mirror;
	spin_lock_init(&mirror_g->lock);

	/*
	 * Initialize active group list
	 */
	INIT_LIST_HEAD(&mirror_g->active_mirror_groups);

	/*
	 * Create debugfs directories/files.
	 */
	ppe_mirror_stats_debugfs_init(d_rule);
}
