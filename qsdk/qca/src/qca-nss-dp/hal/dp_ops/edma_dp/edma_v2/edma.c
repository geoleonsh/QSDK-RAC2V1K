/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 *
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER
 * RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE
 * USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/irq.h>
#include <linux/reset.h>
#include <fal/fal_qm.h>
#include <fal/fal_rss_hash.h>
#include <fal/fal_servcode.h>
#include <ppe_drv_sc.h>
#include <ppe_drv_acl.h>
#include <ppe_drv.h>
#include <linux/clk.h>
#include "edma.h"
#include "edma_cfg_tx.h"
#include "edma_cfg_rx.h"
#include "edma_regs.h"
#include "edma_debug.h"
#include "edma_debugfs.h"
#include "nss_dp_dev.h"

/*
 * EDMA hardware instance
 */
struct edma_gbl_ctx edma_gbl_ctx;

static char edma_txcmpl_irq_name[EDMA_MAX_TXCMPL_RINGS][EDMA_IRQ_NAME_SIZE];
static char edma_rxdesc_irq_name[EDMA_MAX_RXDESC_RINGS][EDMA_IRQ_NAME_SIZE];

#if defined(NSS_DP_POINT_OFFLOAD)
/*
 * nss_dp_point_offload_info_get()
 *	Get point offload ring information
 */
void nss_dp_point_offload_info_get(uint32_t *txdesc_num, uint32_t *txcmpl_num,
		uint32_t *rxfill_num, uint32_t *rxdesc_num)
{
	*txdesc_num = edma_gbl_ctx.txdesc_point_offload_ring;
	*txcmpl_num = edma_gbl_ctx.txcmpl_point_offload_ring;
	*rxfill_num = edma_gbl_ctx.rxfill_point_offload_ring;
	*rxdesc_num = edma_gbl_ctx.rxdesc_point_offload_ring;
}
EXPORT_SYMBOL(nss_dp_point_offload_info_get);
#endif

/*
 * edma_nsm_sawf_sc_stats_read()
 *	Read stats for NSM for a given service class.
 */
bool edma_nsm_sawf_sc_stats_read(struct nss_dp_hal_nsm_sawf_sc_stats *nsm_stats, uint8_t service_class)
{
	struct edma_sawf_sc_stats *sawf_sc_stats;
	unsigned int start;

	if (!PPE_DRV_SERVICE_CLASS_IS_VALID(service_class)) {
		edma_warn("%u Invalid SAWF service class.", service_class);
		return false;
	}

	sawf_sc_stats = &edma_gbl_ctx.sawf_sc_stats[service_class];
	do {
		start = u64_stats_fetch_begin_irq(&sawf_sc_stats->syncp);
		nsm_stats->rx_packets = sawf_sc_stats->rx_packets;
		nsm_stats->rx_bytes = sawf_sc_stats->rx_bytes;
	} while (u64_stats_fetch_retry_irq(&sawf_sc_stats->syncp, start));

	return true;
}

/*
 * edma_disable_interrupts()
 *	Disable EDMA RX/TX interrupt masks.
 */
void edma_disable_interrupts(struct edma_gbl_ctx *egc)
{
	uint32_t i;

	for (i = 0; i < egc->num_rxdesc_rings; i++) {
		struct edma_rxdesc_ring *rxdesc_ring =
				&egc->rxdesc_rings[i];
		edma_reg_write(EDMA_REG_RXDESC_INT_MASK(rxdesc_ring->ring_id),
				EDMA_MASK_INT_CLEAR);
	}

	for (i = 0; i < egc->num_txcmpl_rings; i++) {
		struct edma_txcmpl_ring *txcmpl_ring =
				&egc->txcmpl_rings[i];
		edma_reg_write(EDMA_REG_TX_INT_MASK(txcmpl_ring->id),
				EDMA_MASK_INT_CLEAR);
	}

	/*
	 * Clear MISC interrupt mask.
	 */
	edma_reg_write(EDMA_REG_MISC_INT_MASK, EDMA_MASK_INT_CLEAR);
}

/*
 * edma_enable_interrupts()
 *	Enable RX/TX EDMA interrupt masks.
 */
void edma_enable_interrupts(struct edma_gbl_ctx *egc)
{
	uint32_t i;

	for (i = 0; i < egc->num_rxdesc_rings; i++) {
		struct edma_rxdesc_ring *rxdesc_ring =
				&egc->rxdesc_rings[i];
		edma_reg_write(EDMA_REG_RXDESC_INT_MASK(rxdesc_ring->ring_id),
				egc->rxdesc_intr_mask);
	}

	for (i = 0; i < egc->num_txcmpl_rings; i++) {
		struct edma_txcmpl_ring *txcmpl_ring =
				&egc->txcmpl_rings[i];
		edma_reg_write(EDMA_REG_TX_INT_MASK(txcmpl_ring->id),
				egc->txcmpl_intr_mask);
	}

	/*
	 * Enable MISC interrupt mask.
	 */
	edma_reg_write(EDMA_REG_MISC_INT_MASK, egc->misc_intr_mask);
}

/*
 * edma_disable_port()
 *	EDMA disable port
 */
static void edma_disable_port(void)
{
	edma_reg_write(EDMA_REG_PORT_CTRL, EDMA_DISABLE);
}

/*
 * edma_cfg_sc_bypass
 *	Set service code to disable PPE processing
 *
 * TODO: Use PPE APIs when they are available.
 */
static sw_error_t edma_cfg_sc_bypass(struct edma_gbl_ctx *egc)
{
	sw_error_t ret;
	fal_servcode_config_t entry = {0};
	entry.bypass_bitmap[0] = ~((1 << FAKE_MAC_HEADER_BYP)
					| (1 << SERVICE_CODE_BYP)
					| (1 << FAKE_L2_PROTO_BYP));
	entry.bypass_bitmap[1] = ~(1 << ACL_POST_ROUTING_CHECK_BYP);

	ret = fal_servcode_config_set(0, EDMA_SC_BYPASS, &entry);
	if (ret < 0) {
		edma_err("%px: Error in configuring service code %d\n", egc, ret);
	}

	return ret;
}

/*
 * edma_cleanup()
 *	EDMA cleanup
 */
void edma_cleanup(bool is_dp_override)
{
	/*
	 * The cleanup can happen from data plane override
	 * or from module_exit, we want to cleanup only once
	 *
	 * On cleanup, disable EDMA only at module exit time, since
	 * NSS firmware depends on this setting.
	 */
	if (!edma_gbl_ctx.edma_initialized) {
		if (!is_dp_override) {
			edma_disable_port();
		}
		return;
	}

	if (edma_gbl_ctx.ctl_table_hdr) {
		unregister_sysctl_table(edma_gbl_ctx.ctl_table_hdr);
		edma_gbl_ctx.ctl_table_hdr = NULL;
	}

	/*
	 * TODO: Check with HW team about the state of in-flight
	 * packets when the descriptor rings are disabled.
	 */
	edma_cfg_tx_rings_disable(&edma_gbl_ctx);
	edma_cfg_rx_rings_disable(&edma_gbl_ctx);

	edma_disable_interrupts(&edma_gbl_ctx);

	/*
	 * Remove interrupt handlers and NAPI
	 */
	if (edma_gbl_ctx.napi_added) {
		uint32_t i;

		/*
		 * Free IRQ for TXCMPL rings
		 */
		for (i = 0; i < edma_gbl_ctx.num_txcmpl_rings; i++) {
			synchronize_irq(edma_gbl_ctx.txcmpl_intr[i]);

			free_irq(edma_gbl_ctx.txcmpl_intr[i],
					(void *)&(edma_gbl_ctx.txcmpl_rings[i]));
		}

		/*
		 * Free IRQ for RXDESC rings
		 */
		for (i = 0; i < edma_gbl_ctx.num_rxdesc_rings; i++) {
			synchronize_irq(edma_gbl_ctx.rxdesc_intr[i]);
			free_irq(edma_gbl_ctx.rxdesc_intr[i],
					(void *)&(edma_gbl_ctx.rxdesc_rings[i]));
		}

		/*
		 * Free Misc IRQ
		 */
		synchronize_irq(edma_gbl_ctx.misc_intr);
		free_irq(edma_gbl_ctx.misc_intr, (void *)(edma_gbl_ctx.pdev));

		edma_cfg_rx_napi_delete(&edma_gbl_ctx);
		edma_cfg_tx_napi_delete(&edma_gbl_ctx);
		edma_gbl_ctx.napi_added = false;
	}

	/*
	 * Disable EDMA only at module exit time, since NSS firmware
	 * depends on this setting.
	 */
	if (!is_dp_override) {
		edma_disable_port();
	}

	/*
	 * cleanup rings and free
	 */
	edma_cfg_tx_rings_cleanup(&edma_gbl_ctx);
	edma_cfg_rx_rings_cleanup(&edma_gbl_ctx);

	iounmap(edma_gbl_ctx.reg_base);
	release_mem_region((edma_gbl_ctx.reg_resource)->start,
			resource_size(edma_gbl_ctx.reg_resource));

	/*
	 * Clean the debugfs entries for the EDMA
	 */
	edma_debugfs_exit();

	/*
	 * Unregister PTP service code callback function
	 */
	ppe_drv_sc_unregister_cb(PPE_DRV_SC_PTP);

	/*
	 * Unregister mirror core selection API callback with PPE driver
	 */
	ppe_drv_acl_mirror_core_select_unregister_cb();

	/*
	 * Mark initialize false, so that we do not
	 * try to cleanup again
	 */
	edma_gbl_ctx.edma_initialized = false;
}

/*
 * edma_validate_desc_map()
 *	Validate descriptor maps received from the dtsi
 */
static bool edma_validate_desc_map(void)
{
	uint32_t i, j, desc_bitmap = 0;
	int32_t txdesc_port_arr[EDMA_MAX_TXDESC_RINGS] = {0};

	for (i = 0; i < EDMA_TX_RING_PER_CORE_MAX; i++) {
		for_each_possible_cpu(j) {
			int32_t desc_num = edma_gbl_ctx.tx_map[i][j];
			if (desc_num < 0) {
				continue;
			}

			if (desc_num >= EDMA_MAX_TXDESC_RINGS) {
				edma_err("desc number (%d) at tx_map[%d][%d] greater"
						" than the max txdesc ring count\n",
						 desc_num, i, j);
				return false;
			}

			/*
			 * Check if the user is trying to map the same descriptor
			 * for multiple ports.
			 * If that is the case, then return failure.
			 */
			if (desc_bitmap & (1 << desc_num)) {
				if (txdesc_port_arr[desc_num] == i) {
					continue;
				} else {
					edma_err("desc number (%d) already in use by other"
						       " port (%d)\n", desc_num,
						       txdesc_port_arr[desc_num]);
					return false;
				}
			}

			desc_bitmap |= (1 << desc_num);
			txdesc_port_arr[desc_num] = i;
		}
	}

	desc_bitmap = 0;

	for_each_possible_cpu(i) {
		int32_t desc_num = edma_gbl_ctx.rxdesc_ring_map[0][i];
		if (desc_num < 0) {
			continue;
		}

		if (desc_num >= EDMA_MAX_RXDESC_RINGS) {
			edma_err("desc number (%d) at rxdesc_map[%d][%d] is greater than"
				       " the max rxdesc rings allowed\n",
				       desc_num, 0, i);
			return false;
		}

		if (desc_bitmap & (1 << desc_num)) {
			edma_err("desc num (%d) already present in the rexdesc ring map\n",
					desc_num);
			return false;
		}

		desc_bitmap |= (1 << desc_num);
	}

	return true;
}

/*
 * edma_of_get_pdata()
 *	Read the device tree details for EDMA
 */
static int edma_of_get_pdata(struct resource *edma_res)
{
	int ret;
	uint32_t i, j;

	/*
	 * Find EDMA node in device tree
	 */
	edma_gbl_ctx.device_node = of_find_node_by_name(NULL,
				EDMA_DEVICE_NODE_NAME);
	if (!edma_gbl_ctx.device_node) {
		edma_err("EDMA device tree node (%s) not found\n",
				EDMA_DEVICE_NODE_NAME);
		return -EINVAL;
	}

	/*
	 * Get EDMA device node
	 */
	edma_gbl_ctx.pdev = of_find_device_by_node(edma_gbl_ctx.device_node);
	if (!edma_gbl_ctx.pdev) {
		edma_err("Platform device for node %px(%s) not found\n",
				edma_gbl_ctx.device_node,
				(edma_gbl_ctx.device_node)->name);
		return -EINVAL;
	}

	/*
	 * Get EDMA register resource
	 */
	if (of_address_to_resource(edma_gbl_ctx.device_node, 0, edma_res) != 0) {
		edma_err("Unable to get register address for edma device: "
			  EDMA_DEVICE_NODE_NAME"\n");
		return -EINVAL;
	}

	/*
	 * Get id of first TXDESC ring
	 */
	if (of_property_read_u32(edma_gbl_ctx.device_node, "qcom,txdesc-ring-start",
				&edma_gbl_ctx.txdesc_ring_start) != 0) {
		edma_err("Read error 1st TXDESC ring (txdesc_ring_start)\n");
		return -EINVAL;
	}

	if (edma_gbl_ctx.txdesc_ring_start >= EDMA_MAX_TXDESC_RINGS) {
		edma_err("Incorrect txdesc-ring-start value (%d) received as input\n",
				edma_gbl_ctx.txdesc_ring_start);
		return -EINVAL;
	}
	edma_debug("txdesc ring start: %d\n", edma_gbl_ctx.txdesc_ring_start);

	/*
	 * Get number of TXDESC rings
	 */
	if (of_property_read_u32(edma_gbl_ctx.device_node, "qcom,txdesc-rings",
				&edma_gbl_ctx.num_txdesc_rings) != 0) {
		edma_err("Unable to read number of txdesc rings.\n");
		return -EINVAL;
	}

	if (edma_gbl_ctx.num_txdesc_rings > EDMA_MAX_TXDESC_RINGS) {
		edma_err("Invalid txdesc-rings value (%d) received as input\n",
				edma_gbl_ctx.num_txdesc_rings);
		return -EINVAL;
	}

	edma_gbl_ctx.txdesc_ring_end = edma_gbl_ctx.txdesc_ring_start +
					edma_gbl_ctx.num_txdesc_rings;
	if (edma_gbl_ctx.txdesc_ring_end > EDMA_MAX_TXDESC_RINGS) {
		edma_err("Invalid Txdesc ring configuration: txdesc-ring-start (%d)"
				" and txdesc-rings (%d)\n",
				edma_gbl_ctx.txdesc_ring_start,
				edma_gbl_ctx.num_txdesc_rings);
		return -EINVAL;
	}
	edma_debug("txdesc rings count: %d, txdesc ring end: %d\n",
			edma_gbl_ctx.num_txdesc_rings, edma_gbl_ctx.txdesc_ring_end);

	/*
	 * Get id of first TXCMPL ring
	 */
	if (of_property_read_u32(edma_gbl_ctx.device_node, "qcom,txcmpl-ring-start",
				&edma_gbl_ctx.txcmpl_ring_start) != 0) {
		edma_err("Read error 1st TXCMPL ring (txcmpl_ring_start)\n");
		return -EINVAL;
	}

	if (edma_gbl_ctx.txcmpl_ring_start >= EDMA_MAX_TXCMPL_RINGS) {
		edma_err("Incorrect txcmpl-ring-start value (%d) received as input\n",
				edma_gbl_ctx.txcmpl_ring_start);
		return -EINVAL;
	}
	edma_debug("txcmpl ring start: %d\n", edma_gbl_ctx.txcmpl_ring_start);

	/*
	 * Get number of TXCMPL rings
	 */
	if (of_property_read_u32(edma_gbl_ctx.device_node, "qcom,txcmpl-rings",
				&edma_gbl_ctx.num_txcmpl_rings) != 0) {
		edma_err("Unable to read number of txcmpl rings.\n");
		return -EINVAL;
	}

	if (edma_gbl_ctx.num_txcmpl_rings > EDMA_MAX_TXCMPL_RINGS) {
		edma_err("Invalid txcmpl-rings value (%d) received as input\n",
				edma_gbl_ctx.num_txcmpl_rings);
		return -EINVAL;
	}

	edma_gbl_ctx.txcmpl_ring_end = edma_gbl_ctx.txcmpl_ring_start +
					edma_gbl_ctx.num_txcmpl_rings;
	if (edma_gbl_ctx.txcmpl_ring_end > EDMA_MAX_TXCMPL_RINGS) {
		edma_err("Invalid Txcmpl ring configuration: txcmpl-ring-start (%d)"
				" and txcmpl-rings (%d)\n",
				edma_gbl_ctx.txcmpl_ring_start,
				edma_gbl_ctx.num_txcmpl_rings);
		return -EINVAL;
	}
	edma_debug("txcmpl rings count: %d, txcmpl ring end: %d\n",
			edma_gbl_ctx.num_txcmpl_rings, edma_gbl_ctx.txcmpl_ring_end);

	/*
	 * Get id of first RXFILL ring
	 */
	if (of_property_read_u32(edma_gbl_ctx.device_node, "qcom,rxfill-ring-start",
				&edma_gbl_ctx.rxfill_ring_start) != 0) {
		edma_err("Read error 1st RXFILL ring (rxfill-ring-start)\n");
		return -EINVAL;
	}

	if (edma_gbl_ctx.rxfill_ring_start >= EDMA_MAX_RXFILL_RINGS) {
		edma_err("Incorrect rxfill-ring-start value (%d) received as input\n",
				edma_gbl_ctx.rxfill_ring_start);
		return -EINVAL;
	}
	edma_debug("rxfill ring start: %d\n", edma_gbl_ctx.rxfill_ring_start);

	/*
	 * Get number of RXFILL rings
	 */
	if (of_property_read_u32(edma_gbl_ctx.device_node, "qcom,rxfill-rings",
					&edma_gbl_ctx.num_rxfill_rings) != 0) {
		edma_err("Unable to read number of rxfill rings.\n");
		return -EINVAL;
	}

	if (edma_gbl_ctx.num_rxfill_rings > EDMA_MAX_RXFILL_RINGS) {
		edma_err("Invalid rxfill-rings value (%d) received as input\n",
				edma_gbl_ctx.num_rxfill_rings);
		return -EINVAL;
	}

	edma_gbl_ctx.rxfill_ring_end = edma_gbl_ctx.rxfill_ring_start +
					edma_gbl_ctx.num_rxfill_rings;
	if (edma_gbl_ctx.rxfill_ring_end > EDMA_MAX_RXFILL_RINGS) {
		edma_err("Invalid Rxfill ring configuration: rxfill-ring-start (%d)"
				" and rxfill-rings (%d)\n",
				edma_gbl_ctx.rxfill_ring_start,
				edma_gbl_ctx.num_rxfill_rings);
		return -EINVAL;
	}
	edma_debug("rxfill rings count: %d, rxfill ring end: %d\n",
			edma_gbl_ctx.num_rxfill_rings, edma_gbl_ctx.rxfill_ring_end);

	/*
	 * Get page_mode of RXFILL rings
	 * TODO: Move this setting to DP common node
	 */
#if !defined(NSS_DP_MEM_PROFILE_LOW) && !defined(NSS_DP_MEM_PROFILE_MEDIUM)
	of_property_read_u32(edma_gbl_ctx.device_node, "qcom,rx-page-mode",
					&edma_gbl_ctx.rx_page_mode);
#endif

	/*
	 * Get id of first RXDESC ring
	 */
	if (of_property_read_u32(edma_gbl_ctx.device_node, "qcom,rxdesc-ring-start",
				&edma_gbl_ctx.rxdesc_ring_start) != 0) {
		edma_err("Read error 1st RXDESC ring (rxdesc-ring-start)\n");
		return -EINVAL;
	}

	if (edma_gbl_ctx.rxdesc_ring_start >= EDMA_MAX_RXDESC_RINGS) {
		edma_err("Incorrect rxdesc-ring-start value (%d) received as input\n",
				edma_gbl_ctx.rxdesc_ring_start);
		return -EINVAL;
	}
	edma_debug("rxdesc ring start: %d\n", edma_gbl_ctx.rxdesc_ring_start);

	/*
	 * Get number of RXDESC rings
	 */
	if (of_property_read_u32(edma_gbl_ctx.device_node, "qcom,rxdesc-rings",
					&edma_gbl_ctx.num_rxdesc_rings) != 0) {
		edma_err("Unable to read number of rxdesc rings.\n");
		return -EINVAL;
	}

	if (edma_gbl_ctx.num_rxdesc_rings > EDMA_MAX_RXDESC_RINGS) {
		edma_err("Invalid rxdesc-rings value (%d) received as input\n",
				edma_gbl_ctx.num_rxdesc_rings);
		return -EINVAL;
	}

	edma_gbl_ctx.rxdesc_ring_end = edma_gbl_ctx.rxdesc_ring_start +
					edma_gbl_ctx.num_rxdesc_rings;
	if (edma_gbl_ctx.rxdesc_ring_end > EDMA_MAX_RXDESC_RINGS) {
		edma_err("Invalid Rxdesc ring configuration: rxdesc-ring-start (%d)"
				" and rxdesc-rings (%d)\n",
				edma_gbl_ctx.rxdesc_ring_start,
				edma_gbl_ctx.num_rxdesc_rings);
		return -EINVAL;
	}
	edma_debug("rxdesc rings count: %d, rxdesc ring end: %d\n",
			edma_gbl_ctx.num_rxdesc_rings, edma_gbl_ctx.rxdesc_ring_end);

	/*
	 * Get Tx Map priority level
	 */
	if (of_property_read_u32(edma_gbl_ctx.device_node, "qcom,tx-map-priority-level",
					&edma_gbl_ctx.tx_priority_level) != 0) {
		edma_err("Unable to read Tx map priority level.\n");
		return -EINVAL;
	}

	if (edma_gbl_ctx.tx_priority_level > EDMA_TX_MAX_PRIORITY_LEVEL) {
		edma_err("Invalid tx priority value (%d), maximum possible"
				" priority value is %u\n", edma_gbl_ctx.tx_priority_level,
				EDMA_TX_MAX_PRIORITY_LEVEL);
		return -EINVAL;
	}
	edma_debug("tx map priority level: %d\n", edma_gbl_ctx.tx_priority_level);

	/*
	 * Get Rx Map priority level
	 */
	if (of_property_read_u32(edma_gbl_ctx.device_node,
					"qcom,rx-map-priority-level",
					&edma_gbl_ctx.rx_priority_level) != 0) {
		edma_err("Unable to read Rx map priority level.\n");
		return -EINVAL;
	}

	if (edma_gbl_ctx.rx_priority_level > EDMA_RX_MAX_PRIORITY_LEVEL) {
		edma_err("Invalid tx priority value (%d), maximum possible"
				" priority value is %u\n",
				edma_gbl_ctx.rx_priority_level,
				EDMA_RX_MAX_PRIORITY_LEVEL);
		return -EINVAL;
	}
	edma_debug("rx map priority level: %d\n",
				edma_gbl_ctx.rx_priority_level);

	/*
	 * Get TXDESC Map
	 */
	ret = of_property_read_u32_array(edma_gbl_ctx.device_node,
			"qcom,txdesc-map",
			(int32_t *)edma_gbl_ctx.tx_map,
			(EDMA_MAX_PORTS * EDMA_TX_MAX_PRIORITY_LEVEL * NR_CPUS));
	if (ret) {
		edma_err("Unable to read Tx map array. ret: %d\n", ret);
		return -EINVAL;
	}

	for (i = 0; i < EDMA_TX_RING_PER_CORE_MAX; i++) {
		for_each_possible_cpu(j) {
			edma_debug("txmap[%d][%d] = %d\n", i, j,
					edma_gbl_ctx.tx_map[i][j]);
		}
	}

	/*
	 * Get Rx queue start
	 */
	ret = of_property_read_u8(edma_gbl_ctx.device_node,
			"qcom,rx-queue-start",
			&edma_gbl_ctx.rx_queue_start);
	if (ret) {
		edma_err("Unable to read Rx queue start.\n");
		return -EINVAL;
	}
	edma_debug("rx queue start: %d\n",
			edma_gbl_ctx.rx_queue_start);

	/*
	 * Get rx_ring to queue mapping
	 */
	ret = of_property_read_u32_array(edma_gbl_ctx.device_node,
			"qcom,rx-ring-queue-map",
			(int32_t *)edma_gbl_ctx.rx_ring_queue_map,
			(EDMA_MAX_PRI_PER_CORE * NR_CPUS));
	if (ret) {
		edma_err("Unable to read Rx ring to queue map array. ret: %d\n", ret);
		return -EINVAL;
	}

	for (i = 0; i < EDMA_MAX_PRI_PER_CORE; i++) {
		for_each_possible_cpu(j) {
			edma_debug("Rx ring to queue map[%d][%d] = %d\n", i, j,
					edma_gbl_ctx.rx_ring_queue_map[i][j]);
		}
	}

	/*
	 * Get TXDESC flow control Group ID Map
	 */
	ret = of_property_read_u32_array(edma_gbl_ctx.device_node,
			"qcom,txdesc-fc-grp-map",
			(int32_t *)edma_gbl_ctx.tx_fc_grp_map, EDMA_MAX_GMACS);
	if (ret) {
		edma_err("Unable to read TxDesc-Fc-Grp map array. \
			ret: %d\n", ret);
		return -EINVAL;
	}

	/*
	 * Get RXDESC Map
	 */
	ret = of_property_read_u32_array(edma_gbl_ctx.device_node,
			"qcom,rxdesc-map",
			(int32_t *)edma_gbl_ctx.rxdesc_ring_map,
			(EDMA_RXDESC_RING_PER_CORE_MAX * NR_CPUS));
	if (ret) {
		edma_err("Unable to read Rx ring map array. Return: %d\n", ret);
		return -EINVAL;
	}

	for_each_possible_cpu(i) {
		edma_debug("rxdesc_ring_map[%d] = %d\n", i,
				edma_gbl_ctx.rxdesc_ring_map[0][i]);
	}

#if defined(NSS_DP_POINT_OFFLOAD)
	ret = of_property_read_u32(edma_gbl_ctx.device_node, "qcom,txdesc_point_offload_ring", &edma_gbl_ctx.txdesc_point_offload_ring);
	if (ret) {
		edma_err("Unable to parse Tx point offload ring with err: %d\n", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(edma_gbl_ctx.device_node, "qcom,txcmpl_point_offload_ring", &edma_gbl_ctx.txcmpl_point_offload_ring);
	if (ret) {
		edma_err("Unable to read Tx completion point offload ring with err: %d\n", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(edma_gbl_ctx.device_node, "qcom,rxfill_point_offload_ring", &edma_gbl_ctx.rxfill_point_offload_ring);
	if (ret) {
		edma_err("Unable to read RX fill point offload ring with err: %d\n", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(edma_gbl_ctx.device_node, "qcom,rxdesc_point_offload_ring", &edma_gbl_ctx.rxdesc_point_offload_ring);
	if (ret) {
		edma_err("Unable to read RX desc point offload ring with err: %d\n", ret);
		return -EINVAL;
	}
#endif

#ifdef NSS_DP_PPEDS_SUPPORT
	if (of_property_read_u32(edma_gbl_ctx.device_node, "qcom,ppeds-num",
					&edma_gbl_ctx.ppeds_drv.num_nodes) != 0) {
		edma_err("Unable to read number of PPE-DS nodes\n");
		return -EINVAL;
	}

	if (edma_gbl_ctx.ppeds_drv.num_nodes > EDMA_PPEDS_MAX_NODES) {
		edma_err("Invalid number of ppeds nodes (%d), maximum possible"
				" ppeds node count is %u\n",
				edma_gbl_ctx.ppeds_drv.num_nodes,
				EDMA_PPEDS_MAX_NODES);
		return -EINVAL;
	}

#if defined(NSS_DP_POINT_OFFLOAD)
	if (edma_gbl_ctx.ppeds_drv.num_nodes == EDMA_PPEDS_MAX_NODES) {
		/*
		 * Only enable EDMA_PPEDS_MAX_NODES - 1 PPE-DS nodes when the
		 * point offload feature is enabled (because one pair of EDMA
		 * Rx/Tx rings will be shared between the point offload feature
		 * and the PPE-DS feature).
		 */
		edma_gbl_ctx.ppeds_drv.num_nodes = EDMA_PPEDS_MAX_NODES - 1;
		edma_warn("Error: PPE node count is %d when point offload is enabled.",
				 EDMA_PPEDS_MAX_NODES);
	}
#endif
	edma_debug("PPE-DS num nodes: %d\n", edma_gbl_ctx.ppeds_drv.num_nodes);

	if (edma_gbl_ctx.ppeds_drv.num_nodes > 0) {
		ret = of_property_read_u32_array(edma_gbl_ctx.device_node,
				"qcom,ppeds-map",
				(int32_t *)edma_gbl_ctx.ppeds_node_map,
				(edma_gbl_ctx.ppeds_drv.num_nodes * EDMA_PPEDS_NUM_ENTRY));
		if (ret) {
			edma_err("Unable to read PPE-DS map array. ret: %d\n", ret);
			return -EINVAL;
		}
	}
#endif
	return 0;
}

/*
 * edma_alloc_rings()
 *	Allocate and initialize EDMA rings
 */
static int edma_alloc_rings(struct edma_gbl_ctx *egc)
{
	if (edma_cfg_tx_rings_alloc(egc)) {
		edma_err("Error in allocating tx rings\n");
		return -ENOMEM;
	}

	if (edma_cfg_rx_rings_alloc(egc)) {
		edma_err("Error in allocating rx rings\n");
		goto rx_rings_alloc_fail;
	}

	return 0;

rx_rings_alloc_fail:
	edma_cfg_tx_rings_cleanup(egc);
	return -ENOMEM;
}

/*
 * edma_hw_reset()
 *	Reset EDMA Hardware during initialization
 */
static inline int edma_hw_reset(struct edma_gbl_ctx *egc)
{

	/*
	 * Soc Specific Reset
	 */
	nss_dp_hal_hw_reset(egc->pdev);

	edma_info("EDMA HW Reset completed succesfully\n");

	return 0;
}

/*
 * edma_init_ring_maps()
 *	API to initialize TX/RX ring maps in the global context
 */
static void edma_init_ring_maps(void)
{
	uint32_t i, j;

	for (i = 0; i < EDMA_MAX_TXDESC_RINGS; i++) {
		edma_gbl_ctx.tx_to_txcmpl_map[i] = -1;
	}

	for (i = 0; i < EDMA_TX_RING_PER_CORE_MAX; i++) {
		for_each_possible_cpu(j) {
			edma_gbl_ctx.tx_map[i][j] = -1;
		}
	}

	for (i = 0; i < EDMA_TXCMPL_RING_PER_CORE_MAX; i++) {
		for_each_possible_cpu(j) {
			edma_gbl_ctx.txcmpl_map[i][j] = -1;
		}
	}

	for (i = 0; i < EDMA_MAX_GMACS; i++) {
		edma_gbl_ctx.tx_fc_grp_map[i] = -1;
	}
}

/*
 * edma_configure_ucast_prio_map_tbl()
 *	Configure unicast priority map table
 *
 * Map int_priority values to priority class and initialize
 * unicast priority map table for default profile_id.
 */
static sw_error_t edma_configure_ucast_prio_map_tbl(void)
{
	uint8_t pri_class;
	uint8_t int_pri;
	sw_error_t ret;

	/*
	 * Set the priority class value for every possible priority.
	 */
	for (int_pri = 0; int_pri < EDMA_PRI_MAX; int_pri++) {
		pri_class = nss_dp_pri_map[int_pri];

		/*
		 * Priority offset should be less than maximum supported queue priority
		 */
		if (pri_class > (EDMA_MAX_PRI_PER_CORE - 1)) {
			edma_err("Configured incorrect priority offset: %d\n", pri_class);
			return SW_BAD_PARAM;
		}

		ret = fal_ucast_priority_class_set(EDMA_SWITCH_DEV_ID, EDMA_CPU_PORT_PROFILE_ID, int_pri, pri_class);
		if (unlikely(ret != SW_OK)) {
			edma_err("Failed with error: %d to set queue priority class for int_pri: %d for profile_id: %d\n",
				  ret, int_pri, EDMA_CPU_PORT_PROFILE_ID);
			return ret;
		}

		edma_info("profile_id: %d, int_priority: %d, pri_class: %d\n", EDMA_CPU_PORT_PROFILE_ID, int_pri, pri_class);
	}

	return ret;
}

/*
 * edma_configure_rps_hash_map()
 *	Configure RPS hash map
 *
 * Map all possible hash values to queues used by the EDMA
 * Rx rings in a round robin fashion. These queues are expected
 * to be mapped to different Rx rings which are assigned to different
 * cores using IRQ affinity configuration.
 */
void edma_configure_rps_hash_map(struct edma_gbl_ctx *egc)
{
	uint32_t hash = 0;
	uint32_t q_off = egc->rx_queue_start;

	/*
	 * Initialize the store
	 */
	for (hash = 0; hash < EDMA_RSS_HASH_MAX; hash++) {
		fal_ucast_hash_map_set(0, EDMA_CPU_PORT_PROFILE_ID, hash, q_off);
		edma_info("profile_id: %u, hash: %u, q_off: %u\n",
				EDMA_CPU_PORT_PROFILE_ID, hash, q_off);

		q_off += EDMA_MAX_PRI_PER_CORE;
		q_off %= (EDMA_MAX_PRI_PER_CORE * edma_cfg_rx_rps_num_cores);
	}
}

/*
 * edma_configure_mirror_pkt_capture_core()
 *	Configure capture core for mirrored packets.
 */
void edma_configure_mirror_pkt_capture_core(uint8_t core_id, void *app_data)
{
	edma_cfg_rx_mcast_qid_to_core_mapping(&edma_gbl_ctx, core_id);
}

/*
 * edma_hw_init()
 *	EDMA hardware initialization
 */
static int edma_hw_init(struct edma_gbl_ctx *egc)
{
	int ret = 0;
	uint32_t data;

	data = edma_reg_read(EDMA_REG_MAS_CTRL);
	edma_info("EDMA ver %d hw init\n", data);

	/*
	 * Setup private data structure
	 */
	egc->rxfill_intr_mask = EDMA_RXFILL_INT_MASK;
	egc->rxdesc_intr_mask = EDMA_RXDESC_INT_MASK_PKT_INT;
	egc->txcmpl_intr_mask = EDMA_TX_INT_MASK_PKT_INT;
	egc->edma_initialized = false;
	atomic_set(&egc->active_port_count, 0);

	/*
	 * Reset EDMA
	 */
	ret = edma_hw_reset(egc);
	if (ret) {
		edma_err("Error in resetting the hardware. ret: %d\n", ret);
		return ret;
	}

	ret = (int)edma_cfg_sc_bypass(egc);
	if (ret) {
		edma_err("Error in configuring service code: %d\n", ret);
		return ret;
	}

	/*
	 * Set EDMA global page mode and jumbo MRU
	 */
	edma_cfg_rx_page_mode_and_jumbo(egc);

	ret = edma_alloc_rings(egc);
	if (ret) {
		edma_err("Error in initializaing the rings. ret: %d\n", ret);
		return ret;
	}

	/*
	 * Disable interrupts
	 */
	edma_disable_interrupts(egc);

	edma_cfg_rx_rings_disable(egc);
	edma_cfg_tx_rings_disable(egc);

	edma_cfg_tx_mapping(egc);
	edma_cfg_rx_mapping(egc);
#if defined(NSS_DP_POINT_OFFLOAD)
	edma_cfg_tx_point_offload_mapping(egc);
	edma_cfg_rx_point_offload_mapping(egc);
#endif

	edma_cfg_tx_rings(egc);
	edma_cfg_rx_rings(egc);
#if defined(NSS_DP_POINT_OFFLOAD)
	edma_cfg_rx_point_offload_rings(egc);
#endif

	/*
	 * Configure DMA request priority, DMA read burst length,
	 * and AXI write size.
	 */
	data = EDMA_DMAR_BURST_LEN_SET(EDMA_BURST_LEN_ENABLE)
		| EDMA_DMAR_REQ_PRI_SET(0)
		| EDMA_DMAR_TXDATA_OUTSTANDING_NUM_SET(31)
		| EDMA_DMAR_TXDESC_OUTSTANDING_NUM_SET(7)
		| EDMA_DMAR_RXFILL_OUTSTANDING_NUM_SET(7);
	edma_reg_write(EDMA_REG_DMAR_CTRL, data);

	/*
	 * Configure Tx Timeout Threshold
	 */
#if defined(NSS_DP_IPQ53XX)
	data = EDMA_TX_TIMEOUT_THRESH_VAL;
	edma_reg_write(EDMA_REG_TX_TIMEOUT_THRESH, data);
#endif

	/*
	 * Misc error mask
	 */
	data = EDMA_MISC_AXI_RD_ERR_MASK |
		EDMA_MISC_AXI_WR_ERR_MASK |
		EDMA_MISC_RX_DESC_FIFO_FULL_MASK |
		EDMA_MISC_RX_ERR_BUF_SIZE_MASK |
		EDMA_MISC_TX_SRAM_FULL_MASK |
		EDMA_MISC_TX_CMPL_BUF_FULL_MASK |
		EDMA_MISC_DATA_LEN_ERR_MASK;
	data |= EDMA_MISC_TX_TIMEOUT_MASK;
	egc->misc_intr_mask = data;

	edma_cfg_rx_rings_enable(egc);
	edma_cfg_tx_rings_enable(egc);

	/*
	 * Global EDMA enable and padding enable
	 */
	data = EDMA_PORT_PAD_EN | EDMA_PORT_EDMA_EN;
	edma_reg_write(EDMA_REG_PORT_CTRL, data);

	/*
	 * Initialize unicast priority map table
	 */
	ret = (int)edma_configure_ucast_prio_map_tbl();
	if (ret) {
		edma_err("Failed to initialize unicast priority map table: %d\n", ret);
		edma_cfg_rx_rings_disable(egc);
		edma_cfg_tx_rings_disable(egc);
		edma_cfg_tx_rings_cleanup(egc);
		edma_cfg_rx_rings_cleanup(egc);
		return ret;
	}

	/*
	 * Initialize RPS hash map table
	 */
	edma_configure_rps_hash_map(egc);

	egc->edma_initialized = true;

	return 0;
}

/*
 * edma_configure_clocks()
 *	API to configure EDMA common clocks
 */
static int32_t edma_configure_clocks(void)
{
	struct platform_device *pdev = edma_gbl_ctx.pdev;
	int32_t err;

	/*
	 * Configure SoC specific EDMA/NSS clocks
	 */
	err = nss_dp_hal_configure_clocks(pdev);
	if (err) {
		edma_err("DP hal clock config failed\n");
		return -1;
	}

	return 0;
}

/*
 * edma_rx_flow_control_table
 *	EDMA Rx flow control sysctl table
 */
static struct ctl_table edma_rx_flow_control_table[] = {
	{
		.procname	=	"rx_fc_enable",
		.data		=	&edma_cfg_rx_fc_enable,
		.maxlen		=	sizeof(int),
		.mode		=	0644,
		.proc_handler	=	edma_cfg_rx_fc_enable_handler
	},
	{
		.procname	=	"rx_queue_tail_drop_enable",
		.data		=	&edma_cfg_rx_queue_tail_drop_enable,
		.maxlen		=	sizeof(int),
		.mode		=	0644,
		.proc_handler	=	edma_cfg_rx_queue_tail_drop_handler
	},
	{}
};

/*
 * edma_sub
 *	EDMA sub directory
 */
static struct ctl_table edma_sub[] = {
	{
		.procname	=	"rx_fc",
		.mode		=	0555,
		.child		=	edma_rx_flow_control_table,
	},
	{
		.procname	=	"rps_num_cores",
		.data		=	&edma_cfg_rx_rps_num_cores,
		.maxlen		=	sizeof(int),
		.mode		=	0644,
		.proc_handler	=	edma_cfg_rx_rps,
	},
	{
		.procname	=	"edma_sec_desc_feature_enable",
		.data		=	&edma_cfg_rx_sec_desc_inval,
		.maxlen		=	sizeof(int),
		.mode		=	0644,
		.proc_handler	=	edma_cfg_rx_inval,
	},
	{}
};

/*
 * edma_main
 *	EDMA main directory
 */
static struct ctl_table edma_main[] = {
	{
		.procname	=	"edma",
		.mode		=	0555,
		.child		=	edma_sub,
	},
	{}
};

/*
 * edma_root
 *	EDMA root directory
 */
static struct ctl_table edma_root[] = {
	{
		.procname	=	"net",
		.mode		=	0555,
		.child		=	edma_main,
	},
	{}
};

/*
 * edma_init()
 *	EDMA init
 */
int edma_init(void)
{
	int ret = 0, i;
	struct resource res_edma;
	uint8_t queue_start = 0;

	/*
	 * Check the EDMA state
	 */
	if (likely(edma_gbl_ctx.edma_initialized)) {
		edma_debug("EDMA is already initialized");
		return 0;
	}

	edma_init_ring_maps();

	/*
	 * Get all the DTS data needed
	 */
	if (edma_of_get_pdata(&res_edma) < 0) {
		edma_err("Unable to get EDMA DTS data.\n");
		return -EINVAL;
	}

	if (!edma_validate_desc_map()) {
		edma_err("Incorrect desc map received\n");
		return -EINVAL;
	}

	edma_gbl_ctx.ctl_table_hdr = register_sysctl_table(edma_root);
	if (!edma_gbl_ctx.ctl_table_hdr) {
		edma_err("sysctl table configuration failed");
		return -EINVAL;
	}

	/*
	 * Request memory region for EDMA registers
	 */
	edma_gbl_ctx.reg_resource = request_mem_region(res_edma.start,
				resource_size(&res_edma),
				EDMA_DEVICE_NODE_NAME);
	if (!edma_gbl_ctx.reg_resource) {
		edma_err("Unable to request EDMA register memory.\n");
		unregister_sysctl_table(edma_gbl_ctx.ctl_table_hdr);
		edma_gbl_ctx.ctl_table_hdr = NULL;
		return -EFAULT;
	}

	/*
	 * Remap register resource
	 */
	edma_gbl_ctx.reg_base = ioremap((edma_gbl_ctx.reg_resource)->start,
			resource_size(edma_gbl_ctx.reg_resource));
	if (!edma_gbl_ctx.reg_base) {
		edma_err("Unable to remap EDMA register memory.\n");
		ret = -EFAULT;
		goto edma_init_remap_fail;
	}

	/*
	 * Initialize EDMA debugfs entry
	 */
	ret = edma_debugfs_init();
	if (ret < 0) {
		edma_err("Error in EDMA debugfs init API. ret: %d\n", ret);
		ret = -EINVAL;
		goto edma_debugfs_init_fail;
	}

#ifdef NSS_DP_PPEDS_SUPPORT
	if (edma_ppeds_init(&edma_gbl_ctx.ppeds_drv) != 0) {
		edma_err("Error in edma ppeds initialization\n");
		ret = -EFAULT;
		goto edma_init_ppeds_init_fail;
	}
#endif

	/*
	 * Configure the EDMA common clocks
	 */
	ret = edma_configure_clocks();
	if (ret) {
		edma_err("Error in configuring the common EDMA clocks\n");
		ret = -EFAULT;
		goto edma_hw_init_fail;
	}

	edma_info("EDMA common clocks are configured\n");

	if (edma_hw_init(&edma_gbl_ctx) != 0) {
		edma_err("Error in edma initialization\n");
		ret = -EFAULT;
		goto edma_hw_init_fail;
	}

	/*
	 * Register PTP service code callback function
	 */
	ppe_drv_sc_register_cb(PPE_DRV_SC_PTP, edma_rx_phy_tstamp_buf, NULL);

	/*
	 * Register mirror core selection API callback with PPE driver
	 */
	ppe_drv_acl_mirror_core_select_register_cb(edma_configure_mirror_pkt_capture_core, NULL);

	/*
	 * We add NAPIs and register IRQs at the time of the first netdev open
	 */
	edma_gbl_ctx.napi_added = false;

	/*
	 * DP module maintains queue to ring mapping, and the rings are mapped
	 * to specific host cores. Similar mapping is needed in ppe driver to
	 * redirect packets/flows to specific host cores.
	 */
	for (i = 0; i < NR_CPUS; i++) {
		queue_start = edma_gbl_ctx.rx_ring_queue_map[edma_gbl_ctx.rx_queue_start][i];
		ppe_drv_core2queue_mapping(i, queue_start);
	}

	return 0;

edma_hw_init_fail:
#ifdef NSS_DP_PPEDS_SUPPORT
	edma_ppeds_deinit(&edma_gbl_ctx.ppeds_drv);
edma_init_ppeds_init_fail:
#endif
	edma_debugfs_exit();

edma_debugfs_init_fail:
	iounmap(edma_gbl_ctx.reg_base);

edma_init_remap_fail:
	release_mem_region((edma_gbl_ctx.reg_resource)->start,
			resource_size(edma_gbl_ctx.reg_resource));
	unregister_sysctl_table(edma_gbl_ctx.ctl_table_hdr);
	edma_gbl_ctx.ctl_table_hdr = NULL;
	return ret;
}

/*
 * edma_irq_init()
 *	Initialize interrupt handlers for the driver
 */
int edma_irq_init(void)
{
	int err;
	uint32_t entry_num, i;

	/*
	 * Get TXCMPL rings IRQ numbers
	 */
	entry_num = 0;
	for (i = 0; i < edma_gbl_ctx.num_txcmpl_rings; i++, entry_num++) {
		edma_gbl_ctx.txcmpl_intr[i] =
			platform_get_irq(edma_gbl_ctx.pdev, entry_num);
		if (edma_gbl_ctx.txcmpl_intr[i] < 0) {
			edma_err("%s: txcmpl_intr[%u] irq get failed\n",
					(edma_gbl_ctx.device_node)->name, i);
			return -1;
		}

		edma_debug("%s: txcmpl_intr[%u] = %u\n",
				 (edma_gbl_ctx.device_node)->name,
				 i, edma_gbl_ctx.txcmpl_intr[i]);
	}

	/*
	 * Get RXDESC rings IRQ numbers
	 *
	 */
	for (i = 0; i < edma_gbl_ctx.num_rxdesc_rings; i++, entry_num++) {
		edma_gbl_ctx.rxdesc_intr[i] =
			platform_get_irq(edma_gbl_ctx.pdev, entry_num);
		if (edma_gbl_ctx.rxdesc_intr[i] < 0) {
			edma_err("%s: rxdesc_intr[%u] irq get failed\n",
					(edma_gbl_ctx.device_node)->name, i);
			return -1;
		}

		edma_debug("%s: rxdesc_intr[%u] = %u\n",
				 (edma_gbl_ctx.device_node)->name,
				 i, edma_gbl_ctx.rxdesc_intr[i]);
	}

	/*
	 * Get misc IRQ number
	 */
	edma_gbl_ctx.misc_intr = platform_get_irq(edma_gbl_ctx.pdev, entry_num);
	if (edma_gbl_ctx.misc_intr < 0) {
		edma_err("%s: misc_intr irq get failed\n", (edma_gbl_ctx.device_node)->name);
		return -1;
	}

	edma_debug("%s: misc IRQ:%u\n", (edma_gbl_ctx.device_node)->name,
						edma_gbl_ctx.misc_intr);

#ifdef NSS_DP_PPEDS_SUPPORT
	/*
	 * Get PPE-DS IRQ numbers
	 */
	for (i = 0; i < edma_gbl_ctx.ppeds_drv.num_nodes; i++) {
		int32_t val;

		entry_num++;
		val = platform_get_irq(edma_gbl_ctx.pdev, entry_num);
		if (val < 0) {
			edma_err("%s: Invalid value: ppeds_txcomp_intr[%u]: %d\n",
					(edma_gbl_ctx.device_node)->name, i, val);
			return -1;
		}
		edma_gbl_ctx.ppeds_drv.ppeds_node_cfg[i].irq_map[EDMA_PPEDS_TXCOMP_IRQ_IDX] = val;

		entry_num++;
		val = platform_get_irq(edma_gbl_ctx.pdev, entry_num);
		if (val < 0) {
			edma_err("%s: Invalid value: ppeds_rxdesc_intr[%u]: %d\n",
					(edma_gbl_ctx.device_node)->name, i, val);
			return -1;
		}
		edma_gbl_ctx.ppeds_drv.ppeds_node_cfg[i].irq_map[EDMA_PPEDS_RXDESC_IRQ_IDX] = val;

		entry_num++;
		val = platform_get_irq(edma_gbl_ctx.pdev, entry_num);
		if (val < 0) {
			edma_err("%s: Invalid value: ppeds_rxfill_intr[%u]: %d\n",
					(edma_gbl_ctx.device_node)->name, i, val);
			return -1;
		}
		edma_gbl_ctx.ppeds_drv.ppeds_node_cfg[i].irq_map[EDMA_PPEDS_RXFILL_IRQ_IDX] = val;

		edma_debug("PPE-DS IRQ: TxComplete: %d, Rx: %d, Rxfill: %d\n",
			edma_gbl_ctx.ppeds_drv.ppeds_node_cfg[i].irq_map[EDMA_PPEDS_TXCOMP_IRQ_IDX],
			edma_gbl_ctx.ppeds_drv.ppeds_node_cfg[i].irq_map[EDMA_PPEDS_RXDESC_IRQ_IDX],
			edma_gbl_ctx.ppeds_drv.ppeds_node_cfg[i].irq_map[EDMA_PPEDS_RXFILL_IRQ_IDX]);
	}
#endif

	/*
	 * Request IRQ for Tx complete rings
	 */
	for (i = 0; i < edma_gbl_ctx.num_txcmpl_rings; i++) {
		snprintf(edma_txcmpl_irq_name[i], 32, "edma_txcmpl_%d", edma_gbl_ctx.txcmpl_ring_start + i);

		irq_set_status_flags(edma_gbl_ctx.txcmpl_intr[i], IRQ_DISABLE_UNLAZY);

		err = request_irq(edma_gbl_ctx.txcmpl_intr[i],
				edma_tx_handle_irq, IRQF_SHARED,
				edma_txcmpl_irq_name[i],
				(void *)&(edma_gbl_ctx.txcmpl_rings[i]));
		if (err) {
			edma_err("TXCMPL ring IRQ:%d request %d failed\n",
					edma_gbl_ctx.txcmpl_intr[i], i);
			return -1;

		}

		edma_debug("TXCMPL ring(%d) IRQ:%d request success(%s)\n",
					edma_gbl_ctx.txcmpl_ring_start + i,
					edma_gbl_ctx.txcmpl_intr[i],
					edma_txcmpl_irq_name[i]);
	}

	/*
	 * Request IRQ for RXDESC rings
	 */
	for (i = 0; i < edma_gbl_ctx.num_rxdesc_rings; i++) {
		snprintf(edma_rxdesc_irq_name[i], 20, "edma_rxdesc_%d", edma_gbl_ctx.rxdesc_ring_start + i);

		irq_set_status_flags(edma_gbl_ctx.rxdesc_intr[i], IRQ_DISABLE_UNLAZY);

		err = request_irq(edma_gbl_ctx.rxdesc_intr[i],
				edma_rx_handle_irq, IRQF_SHARED,
				edma_rxdesc_irq_name[i],
				(void *)&(edma_gbl_ctx.rxdesc_rings[i]));
		if (err) {
			edma_err("RXDESC ring IRQ:%d request failed\n",
					edma_gbl_ctx.rxdesc_intr[i]);
			goto rx_desc_ring_intr_req_fail;
		}

		edma_debug("RXDESC ring(%d) IRQ:%d request success(%s)\n",
					edma_gbl_ctx.rxdesc_ring_start + i,
					edma_gbl_ctx.rxdesc_intr[i],
					edma_rxdesc_irq_name[i]);
	}

	/*
	 * Request Misc IRQ
	 */
	err = request_irq(edma_gbl_ctx.misc_intr, edma_misc_handle_irq,
						IRQF_SHARED, "edma_misc",
						(void *)edma_gbl_ctx.pdev);
	if (err) {
		edma_err("MISC IRQ:%d request failed\n",
				edma_gbl_ctx.misc_intr);
		goto misc_intr_req_fail;
	}

	return 0;

misc_intr_req_fail:
	/*
	 * Free IRQ for RXDESC rings
	 */
	for (i = 0; i < edma_gbl_ctx.num_rxdesc_rings; i++) {
		synchronize_irq(edma_gbl_ctx.rxdesc_intr[i]);

		free_irq(edma_gbl_ctx.rxdesc_intr[i],
				(void *)&(edma_gbl_ctx.rxdesc_rings[i]));
	}

rx_desc_ring_intr_req_fail:
	/*
	 * Free IRQ for TXCMPL rings
	 */
	for (i = 0; i < edma_gbl_ctx.num_txcmpl_rings; i++) {

		synchronize_irq(edma_gbl_ctx.txcmpl_intr[i]);
		free_irq(edma_gbl_ctx.txcmpl_intr[i],
				(void *)&(edma_gbl_ctx.txcmpl_rings[i]));
	}

	return -1;
}
