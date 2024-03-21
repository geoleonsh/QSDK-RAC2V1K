/*
 **************************************************************************
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
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
 **************************************************************************
 */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/ip.h>
#include <net/ipv6.h>
#include <linux/tcp.h>
#include <linux/module.h>
#include <linux/inet.h>
#include <linux/etherdevice.h>

#define DEBUG_LEVEL ECM_PPE_COMMON_DEBUG_LEVEL

#include "ecm_types.h"
#include "ecm_db_types.h"
#include "ecm_state.h"
#include "ecm_tracker.h"
#include "ecm_classifier.h"
#include "ecm_front_end_types.h"
#include "ecm_tracker_datagram.h"
#include "ecm_tracker_udp.h"
#include "ecm_tracker_tcp.h"
#include "ecm_db.h"
#include "ecm_interface.h"
#include "ecm_ppe_common.h"
#include "ecm_front_end_common.h"
#include "ecm_ppe_ipv6.h"
#include "ecm_ppe_ipv4.h"
#ifdef ECM_INTERFACE_VXLAN_ENABLE
#include <net/vxlan.h>
#endif

#ifdef ECM_IPV6_ENABLE
/*
 * ecm_ppe_ipv6_is_conn_limit_reached()
 *	Connection limit is reached or not ?
 */
bool ecm_ppe_ipv6_is_conn_limit_reached(void)
{
#if !defined(ECM_FRONT_END_CONN_LIMIT_ENABLE)
	return false;
#endif

	if (likely(!((ecm_front_end_is_feature_supported(ECM_FE_FEATURE_CONN_LIMIT)) && ecm_front_end_conn_limit))) {
		return false;
	}

	spin_lock_bh(&ecm_ppe_ipv6_lock);
	if (ecm_ppe_ipv6_accelerated_count == PPE_DRV_V6_MAX_CONN_COUNT) {
		spin_unlock_bh(&ecm_ppe_ipv6_lock);
		DEBUG_INFO("ECM DB connection limit %d reached, for PPE frontend \
			   new flows cannot be accelerated.\n",
			   ecm_ppe_ipv6_accelerated_count);
		return true;
	}
	spin_unlock_bh(&ecm_ppe_ipv6_lock);
	return false;
}
#endif

/*
 * ecm_ppe_ipv4_is_conn_limit_reached()
 *	Connection limit is reached or not ?
 */
bool ecm_ppe_ipv4_is_conn_limit_reached(void)
{
#if !defined(ECM_FRONT_END_CONN_LIMIT_ENABLE)
	return false;
#endif

	if (likely(!((ecm_front_end_is_feature_supported(ECM_FE_FEATURE_CONN_LIMIT)) && ecm_front_end_conn_limit))) {
		return false;
	}

	spin_lock_bh(&ecm_ppe_ipv4_lock);
	if (ecm_ppe_ipv4_accelerated_count == PPE_DRV_V4_MAX_CONN_COUNT) {
		spin_unlock_bh(&ecm_ppe_ipv4_lock);
		DEBUG_INFO("ECM DB connection limit %d reached, for PPE frontend \
			   new flows cannot be accelerated.\n",
			   ecm_ppe_ipv4_accelerated_count);
		return true;
	}
	spin_unlock_bh(&ecm_ppe_ipv4_lock);
	return false;
}

#ifdef ECM_INTERFACE_VXLAN_ENABLE
/*
 * ecm_ppe_ported_get_vxlan_ppe_dev_index()
 *	Check and get whether the given ecm iface instance of type VXLAN virtual port is created in PPE.
 */
int ecm_ppe_ported_get_vxlan_ppe_dev_index(struct ecm_front_end_connection_instance *feci, struct ecm_db_iface_instance *ii,
											ecm_db_obj_dir_t dir, enum nss_ppe_vxlanmgr_vp_creation *vp_status)
{
	int if_index = -1;
	uint32_t remote_ip[4] = {0};
	struct ecm_db_interface_info_vxlan vxlan_info = {0};
	struct net_device *dev;
	struct vxlan_config *cfg;
	struct vxlan_dev *priv;
	union vxlan_addr *src_ip;
	uint8_t ip_type;

	dev = dev_get_by_index(&init_net, ecm_db_iface_interface_identifier_get(ii));
	if (!dev) {
		DEBUG_TRACE("%px: VXLAN: could not get the dev", feci);
		return -1;
	}

	priv = netdev_priv(dev);
	cfg = &priv->cfg;
	src_ip = &cfg->saddr;
	if (src_ip->sa.sa_family == AF_INET) {
		ip_type = AF_INET;
	} else {
		ip_type = AF_INET6;
	}

	ecm_db_iface_vxlan_info_get(ii, &vxlan_info);
	if (!vxlan_info.if_type) {
		ip_addr_t addr = {0};

		DEBUG_TRACE("%px: VXLAN: It is an outer rule", feci);
		ecm_db_connection_address_get(feci->ci, ECM_DB_OBJ_DIR_TO_NAT, addr);
		if (ip_type == AF_INET) {
			ECM_IP_ADDR_TO_NIN4_ADDR(remote_ip[0], addr);
		} else {
			uint32_t temp[4] = {0};

			ECM_IP_ADDR_TO_PPE_IPV6_ADDR(temp, addr);
			remote_ip[0] = ntohl(temp[0]);
			remote_ip[1] = ntohl(temp[1]);
			remote_ip[2] = ntohl(temp[2]);
			remote_ip[3] = ntohl(temp[3]);
		}
	} else {
		uint8_t mac_addr[ETH_ALEN] = {0};
		union vxlan_addr rip = {0};

		DEBUG_TRACE("%px: VXLAN: It is an inner rule", feci);
		ecm_db_connection_node_address_get(feci->ci, dir, mac_addr);
		if (vxlan_find_remote_ip(priv, mac_addr, priv->cfg.vni, &rip) < 0) {
			DEBUG_WARN("%px: VXLAN: failed to get the remote IP from kernel", feci);
			dev_put(dev);
			return -1;
		}

		if (ip_type == AF_INET) {
			remote_ip[0] = rip.sin.sin_addr.s_addr;
			DEBUG_TRACE("%px: VXLAN: rip:%pI4h", feci, remote_ip);
		} else {
			memcpy(remote_ip, &rip.sin6.sin6_addr, sizeof(struct in6_addr));
			DEBUG_TRACE("%px: VXLAN: rip:%pI6h", feci, &remote_ip[0]);
		}
	}

	*vp_status = nss_ppe_vxlanmgr_get_ifindex_and_vp_status(dev, remote_ip, ip_type, &if_index);

	DEBUG_TRACE("%px: VXLAN: netdev:%s if_index:%d vp_status:%u\n", feci,dev->name, if_index, *vp_status);

	dev_put(dev);

	return if_index;
}
#endif
