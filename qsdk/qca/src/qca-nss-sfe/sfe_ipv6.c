/*
 * sfe_ipv6.c
 *	Shortcut forwarding engine - IPv6 support.
 *
 * Copyright (c) 2015-2016, 2019-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
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
#include <linux/sysfs.h>
#include <linux/skbuff.h>
#include <linux/icmp.h>
#include <net/tcp.h>
#include <linux/etherdevice.h>
#include <linux/version.h>
#include <net/udp.h>
#include <net/vxlan.h>
#include <linux/refcount.h>
#include <linux/netfilter.h>
#include <linux/inetdevice.h>
#include <linux/netfilter_ipv6.h>
#include <linux/seqlock.h>
#include <net/protocol.h>
#include <net/addrconf.h>
#include <net/gre.h>

#if defined(SFE_RFS_SUPPORTED)
#include <ppe_rfs.h>
#endif
#include "sfe_debug.h"
#include "sfe_api.h"
#include "sfe.h"
#include "sfe_flow_cookie.h"
#include "sfe_ipv6.h"
#include "sfe_ipv6_udp.h"
#include "sfe_ipv6_tcp.h"
#include "sfe_ipv6_icmp.h"
#include "sfe_pppoe.h"
#include "sfe_pppoe_mgr.h"
#include "sfe_ipv6_pppoe_br.h"
#include "sfe_ipv6_tunipip6.h"
#include "sfe_ipv6_gre.h"
#include "sfe_ipv6_esp.h"
#include "sfe_ipv6_etherip.h"

#define sfe_ipv6_addr_copy(src, dest) memcpy((void *)(dest), (void *)(src), 16)

static char *sfe_ipv6_exception_events_string[SFE_IPV6_EXCEPTION_EVENT_LAST] = {
	"UDP_HEADER_INCOMPLETE",
	"UDP_NO_CONNECTION",
	"UDP_IP_OPTIONS_OR_INITIAL_FRAGMENT",
	"UDP_SMALL_TTL",
	"UDP_NEEDS_FRAGMENTATION",
	"TCP_HEADER_INCOMPLETE",
	"TCP_NO_CONNECTION_SLOW_FLAGS",
	"TCP_NO_CONNECTION_FAST_FLAGS",
	"TCP_IP_OPTIONS_OR_INITIAL_FRAGMENT",
	"TCP_SMALL_TTL",
	"TCP_NEEDS_FRAGMENTATION",
	"TCP_FLAGS",
	"TCP_SEQ_EXCEEDS_RIGHT_EDGE",
	"TCP_SMALL_DATA_OFFS",
	"TCP_BAD_SACK",
	"TCP_BIG_DATA_OFFS",
	"TCP_SEQ_BEFORE_LEFT_EDGE",
	"TCP_ACK_EXCEEDS_RIGHT_EDGE",
	"TCP_ACK_BEFORE_LEFT_EDGE",
	"ICMP_HEADER_INCOMPLETE",
	"ICMP_UNHANDLED_TYPE",
	"ICMP_IPV6_HEADER_INCOMPLETE",
	"ICMP_IPV6_NON_V6",
	"ICMP_IPV6_IP_OPTIONS_INCOMPLETE",
	"ICMP_IPV6_UDP_HEADER_INCOMPLETE",
	"ICMP_IPV6_TCP_HEADER_INCOMPLETE",
	"ICMP_IPV6_UNHANDLED_PROTOCOL",
	"ICMP_NO_CONNECTION",
	"ICMP_FLUSHED_CONNECTION",
	"HEADER_INCOMPLETE",
	"BAD_TOTAL_LENGTH",
	"NON_V6",
	"NON_INITIAL_FRAGMENT",
	"DATAGRAM_INCOMPLETE",
	"IP_OPTIONS_INCOMPLETE",
	"UNHANDLED_PROTOCOL",
	"FLOW_COOKIE_ADD_FAIL",
	"NO_HEADROOM",
	"UNCLONE_FAILED",
	"INVALID_PPPOE_SESSION",
	"INCORRECT_PPPOE_PARSING",
	"PPPOE_NOT_SET_IN_CME",
	"PPPOE_BR_NOT_IN_CME",
	"INGRESS_VLAN_TAG_MISMATCH",
	"INVALID_SOURCE_INTERFACE",
	"TUNIPIP6_HEADER_INCOMPLETE",
	"TUNIPIP6_NO_CONNECTION",
	"TUNIPIP6_IP_OPTIONS_OR_INITIAL_FRAGMENT",
	"TUNIPIP6_SMALL_TTL",
	"TUNIPIP6_NEEDS_FRAGMENTATION",
	"TUNIPIP6_SYNC_ON_FIND",
	"GRE_HEADER_INCOMPLETE",
	"GRE_NO_CONNECTION",
	"GRE_IP_OPTIONS_OR_INITIAL_FRAGMENT",
	"GRE_SMALL_TTL",
	"GRE_NEEDS_FRAGMENTATION",
	"ESP_NO_CONNECTION",
	"ESP_IP_OPTIONS_OR_INITIAL_FRAGMENT",
	"ESP_NEEDS_FRAGMENTATION",
	"ESP_SMALL_TTL",
	"INGRESS_TRUSTSEC_SGT_MISMATCH",
	"GSO_NOT_SUPPORTED",
	"TSO_SEG_MAX_NOT_SUPPORTED",
	"ETHERIP_NO_CONNECTION",
	"ETHERIP_IP_OPTIONS_OR_INITIAL_FRAGMENT",
	"ETHERIP_NEEDS_FRAGMENTATION",
	"ETHERIP_SMALL_TTL"
};

static struct sfe_ipv6 __si6;
struct sfe_ipv6_msg *sfe_ipv6_sync_many_msg;
uint32_t sfe_ipv6_sync_max_number;

/*
 * sfe_ipv6_get_debug_dev()
 */
static ssize_t sfe_ipv6_get_debug_dev(struct device *dev, struct device_attribute *attr, char *buf);

/*
 * sysfs attributes.
 */
static const struct device_attribute sfe_ipv6_debug_dev_attr =
	__ATTR(debug_dev, S_IWUSR | S_IRUGO, sfe_ipv6_get_debug_dev, NULL);

/*
 * sfe_ipv6_get_connection_match_hash()
 *	Generate the hash used in connection match lookups.
 */
static inline unsigned int sfe_ipv6_get_connection_match_hash(struct net_device *dev, u8 protocol,
							      struct sfe_ipv6_addr *src_ip, __be16 src_port,
							      struct sfe_ipv6_addr *dest_ip, __be16 dest_port)
{
	u32 idx, hash = 0;

	for (idx = 0; idx < 4; idx++) {
		hash ^= src_ip->addr[idx] ^ dest_ip->addr[idx];
	}
	hash =  hash ^ protocol ^ ntohs(src_port ^ dest_port);
	return ((hash >> SFE_IPV6_CONNECTION_HASH_SHIFT) ^ hash) & SFE_IPV6_CONNECTION_HASH_MASK;
}

/*
 * sfe_ipv6_find_connection_match_rcu()
 *	Get the IPv6 flow match info that corresponds to a particular 5-tuple.
 */
struct sfe_ipv6_connection_match *
sfe_ipv6_find_connection_match_rcu(struct sfe_ipv6 *si, struct net_device *dev, u8 protocol,
					struct sfe_ipv6_addr *src_ip, __be16 src_port,
					struct sfe_ipv6_addr *dest_ip, __be16 dest_port)
{
	struct sfe_ipv6_connection_match *cm = NULL;
	unsigned int conn_match_idx;
	struct hlist_head *lhead;
	WARN_ON_ONCE(!rcu_read_lock_held());

	conn_match_idx = sfe_ipv6_get_connection_match_hash(dev, protocol, src_ip, src_port, dest_ip, dest_port);

	lhead = &si->hlist_conn_match_hash_head[conn_match_idx];

	/*
	 * Hopefully the first entry is the one we want.
	 */
	hlist_for_each_entry_rcu(cm, lhead, hnode) {
		if ((!sfe_ipv6_addr_equal(cm->match_src_ip, src_ip)) ||
		    (!sfe_ipv6_addr_equal(cm->match_dest_ip, dest_ip)) ||
		    (cm->match_dest_port != dest_port) ||
		    (cm->match_src_port != src_port) ||
		    (cm->match_protocol != protocol)) {
			continue;
		}

		this_cpu_inc(si->stats_pcpu->connection_match_hash_hits64);

		break;

	}

	return cm;
}

/*
 * sfe_ipv6_connection_mc_dest_compute_translations()
 *	Compute port and address translations for a connection match entry.
 */
static void sfe_ipv6_connection_mc_dest_compute_translations(struct sfe_ipv6_connection_match *cm, struct sfe_ipv6_mc_dest *dest)
{
	u32 diff[9];
	u32 *idx_32;
	u16 *idx_16;

	/*
	 * Before we insert the entry look to see if this is tagged as doing address
	 * translations.  If it is then work out the adjustment that we need to apply
	 * to the transport checksum.
	 */
	if (dest->flags & SFE_IPV6_CONNECTION_MATCH_FLAG_XLATE_SRC) {
		u32 adj = 0;
		u32 carry = 0;

		/*
		 * Precompute an incremental checksum adjustment so we can
		 * edit packets in this stream very quickly.  The algorithm is from RFC1624.
		 */
		idx_32 = diff;
		*(idx_32++) = cm->match_src_ip[0].addr[0];
		*(idx_32++) = cm->match_src_ip[0].addr[1];
		*(idx_32++) = cm->match_src_ip[0].addr[2];
		*(idx_32++) = cm->match_src_ip[0].addr[3];

		idx_16 = (u16 *)idx_32;
		*(idx_16++) = cm->match_src_port;
		*(idx_16++) = ~dest->xlate_src_ident;
		idx_32 = (u32 *)idx_16;

		*(idx_32++) = ~dest->xlate_src_ip[0];
		*(idx_32++) = ~dest->xlate_src_ip[1];
		*(idx_32++) = ~dest->xlate_src_ip[2];
		*(idx_32++) = ~dest->xlate_src_ip[3];

		/*
		 * When we compute this fold it down to a 16-bit offset
		 * as that way we can avoid having to do a double
		 * folding of the twos-complement result because the
		 * addition of 2 16-bit values cannot cause a double
		 * wrap-around!
		 */
		for (idx_32 = diff; idx_32 < diff + 9; idx_32++) {
			u32 w = *idx_32;
			adj += carry;
			adj += w;
			carry = (w > adj);
		}
		adj += carry;
		adj = (adj & 0xffff) + (adj >> 16);
		adj = (adj & 0xffff) + (adj >> 16);
		dest->xlate_src_csum_adjustment = (u16)adj;
	}
}

/*
 * sfe_ipv6_connection_match_update_summary_stats()
 *	Update the summary stats for a connection match entry.
 */
static inline void sfe_ipv6_connection_match_update_summary_stats(struct sfe_ipv6_connection_match *cm,
					       u32 *packets, u32 *bytes)

{
	u32 packet_count, byte_count;

	packet_count = atomic_read(&cm->rx_packet_count);
	cm->rx_packet_count64 += packet_count;
	atomic_sub(packet_count, &cm->rx_packet_count);

	byte_count = atomic_read(&cm->rx_byte_count);
	cm->rx_byte_count64 += byte_count;
	atomic_sub(byte_count, &cm->rx_byte_count);

	*packets = packet_count;
	*bytes = byte_count;
}

/*
 * sfe_ipv6_connection_match_compute_translations()
 *	Compute port and address translations for a connection match entry.
 */
static void sfe_ipv6_connection_match_compute_translations(struct sfe_ipv6_connection_match *cm)
{
	u32 diff[9];
	u32 *idx_32;
	u16 *idx_16;

	/*
	 * Before we insert the entry look to see if this is tagged as doing address
	 * translations.  If it is then work out the adjustment that we need to apply
	 * to the transport checksum.
	 */
	if (cm->flags & SFE_IPV6_CONNECTION_MATCH_FLAG_XLATE_SRC) {
		u32 adj = 0;
		u32 carry = 0;

		/*
		 * Precompute an incremental checksum adjustment so we can
		 * edit packets in this stream very quickly.  The algorithm is from RFC1624.
		 */
		idx_32 = diff;
		*(idx_32++) = cm->match_src_ip[0].addr[0];
		*(idx_32++) = cm->match_src_ip[0].addr[1];
		*(idx_32++) = cm->match_src_ip[0].addr[2];
		*(idx_32++) = cm->match_src_ip[0].addr[3];

		idx_16 = (u16 *)idx_32;
		*(idx_16++) = cm->match_src_port;
		*(idx_16++) = ~cm->xlate_src_port;
		idx_32 = (u32 *)idx_16;

		*(idx_32++) = ~cm->xlate_src_ip[0].addr[0];
		*(idx_32++) = ~cm->xlate_src_ip[0].addr[1];
		*(idx_32++) = ~cm->xlate_src_ip[0].addr[2];
		*(idx_32++) = ~cm->xlate_src_ip[0].addr[3];

		/*
		 * When we compute this fold it down to a 16-bit offset
		 * as that way we can avoid having to do a double
		 * folding of the twos-complement result because the
		 * addition of 2 16-bit values cannot cause a double
		 * wrap-around!
		 */
		for (idx_32 = diff; idx_32 < diff + 9; idx_32++) {
			u32 w = *idx_32;
			adj += carry;
			adj += w;
			carry = (w > adj);
		}
		adj += carry;
		adj = (adj & 0xffff) + (adj >> 16);
		adj = (adj & 0xffff) + (adj >> 16);
		cm->xlate_src_csum_adjustment = (u16)adj;
	}

	if (cm->flags & SFE_IPV6_CONNECTION_MATCH_FLAG_XLATE_DEST) {
		u32 adj = 0;
		u32 carry = 0;

		/*
		 * Precompute an incremental checksum adjustment so we can
		 * edit packets in this stream very quickly.  The algorithm is from RFC1624.
		 */
		idx_32 = diff;
		*(idx_32++) = cm->match_dest_ip[0].addr[0];
		*(idx_32++) = cm->match_dest_ip[0].addr[1];
		*(idx_32++) = cm->match_dest_ip[0].addr[2];
		*(idx_32++) = cm->match_dest_ip[0].addr[3];

		idx_16 = (u16 *)idx_32;
		*(idx_16++) = cm->match_dest_port;
		*(idx_16++) = ~cm->xlate_dest_port;
		idx_32 = (u32 *)idx_16;

		*(idx_32++) = ~cm->xlate_dest_ip[0].addr[0];
		*(idx_32++) = ~cm->xlate_dest_ip[0].addr[1];
		*(idx_32++) = ~cm->xlate_dest_ip[0].addr[2];
		*(idx_32++) = ~cm->xlate_dest_ip[0].addr[3];

		/*
		 * When we compute this fold it down to a 16-bit offset
		 * as that way we can avoid having to do a double
		 * folding of the twos-complement result because the
		 * addition of 2 16-bit values cannot cause a double
		 * wrap-around!
		 */
		for (idx_32 = diff; idx_32 < diff + 9; idx_32++) {
			u32 w = *idx_32;
			adj += carry;
			adj += w;
			carry = (w > adj);
		}
		adj += carry;
		adj = (adj & 0xffff) + (adj >> 16);
		adj = (adj & 0xffff) + (adj >> 16);
		cm->xlate_dest_csum_adjustment = (u16)adj;
	}
}

/*
 * sfe_ipv6_update_summary_stats()
 *	Update the summary stats.
 */
static void sfe_ipv6_update_summary_stats(struct sfe_ipv6 *si, struct sfe_ipv6_stats *stats)
{
	int i = 0;

	memset(stats, 0, sizeof(*stats));

	for_each_possible_cpu(i) {
		const struct sfe_ipv6_stats *s = per_cpu_ptr(si->stats_pcpu, i);

		stats->connection_create_requests64 += s->connection_create_requests64;
		stats->connection_create_collisions64 += s->connection_create_collisions64;
		stats->connection_create_failures64 += s->connection_create_failures64;
		stats->connection_destroy_requests64 += s->connection_destroy_requests64;
		stats->connection_destroy_misses64 += s->connection_destroy_misses64;
		stats->connection_match_hash_hits64 += s->connection_match_hash_hits64;
		stats->connection_match_hash_reorders64 += s->connection_match_hash_reorders64;
		stats->connection_flushes64 += s->connection_flushes64;
		stats->packets_dropped64 += s->packets_dropped64;
		stats->packets_forwarded64 += s->packets_forwarded64;
		stats->packets_fast_xmited64 += s->packets_fast_xmited64;
		stats->packets_fast_qdisc_xmited64 += s->packets_fast_qdisc_xmited64;
		stats->packets_not_forwarded64 += s->packets_not_forwarded64;
		stats->pppoe_encap_packets_forwarded64 += s->pppoe_encap_packets_forwarded64;
		stats->pppoe_decap_packets_forwarded64 += s->pppoe_decap_packets_forwarded64;
		stats->pppoe_bridge_packets_forwarded64 += s->pppoe_bridge_packets_forwarded64;
		stats->pppoe_bridge_packets_3tuple_forwarded64 += s->pppoe_bridge_packets_3tuple_forwarded64;
		stats->connection_create_requests_overflow64 += s->connection_create_requests_overflow64;
	}
}

/*
 * sfe_ipv6_insert_connection_match()
 *	Insert a connection match into the hash.
 *
 * On entry we must be holding the lock that protects the hash table.
 */
static inline void sfe_ipv6_insert_connection_match(struct sfe_ipv6 *si,
						    struct sfe_ipv6_connection_match *cm)
{
	unsigned int conn_match_idx
		= sfe_ipv6_get_connection_match_hash(cm->match_dev, cm->match_protocol,
						     cm->match_src_ip, cm->match_src_port,
						     cm->match_dest_ip, cm->match_dest_port);

	lockdep_assert_held(&si->lock);

	hlist_add_head_rcu(&cm->hnode, &si->hlist_conn_match_hash_head[conn_match_idx]);
#ifdef CONFIG_NF_FLOW_COOKIE
	if (!si->flow_cookie_enable)
		return;

	/*
	 * Configure hardware to put a flow cookie in packet of this flow,
	 * then we can accelerate the lookup process when we received this packet.
	 */
	for (conn_match_idx = 1; conn_match_idx < SFE_FLOW_COOKIE_SIZE; conn_match_idx++) {
		struct sfe_ipv6_flow_cookie_entry *entry = &si->sfe_flow_cookie_table[conn_match_idx];

		if ((NULL == entry->match) && time_is_before_jiffies(entry->last_clean_time + HZ)) {
			sfe_ipv6_flow_cookie_set_func_t func;

			rcu_read_lock();
			func = rcu_dereference(si->flow_cookie_set_func);
			if (func) {
				if (!func(cm->match_protocol, cm->match_src_ip->addr, cm->match_src_port,
					 cm->match_dest_ip->addr, cm->match_dest_port, conn_match_idx)) {
					entry->match = cm;
					cm->flow_cookie = conn_match_idx;
				} else {
					si->exception_events[SFE_IPV6_EXCEPTION_EVENT_FLOW_COOKIE_ADD_FAIL]++;
				}
			}
			rcu_read_unlock();

			break;
		}
	}
#endif
}

/*
 * sfe_ipv6_remove_connection_match()
 *	Remove a connection match object from the hash.
 */
static inline void sfe_ipv6_remove_connection_match(struct sfe_ipv6 *si, struct sfe_ipv6_connection_match *cm)
{

	lockdep_assert_held(&si->lock);
#ifdef CONFIG_NF_FLOW_COOKIE
	if (si->flow_cookie_enable) {
		/*
		 * Tell hardware that we no longer need a flow cookie in packet of this flow
		 */
		unsigned int conn_match_idx;

		for (conn_match_idx = 1; conn_match_idx < SFE_FLOW_COOKIE_SIZE; conn_match_idx++) {
			struct sfe_ipv6_flow_cookie_entry *entry = &si->sfe_flow_cookie_table[conn_match_idx];

			if (cm == entry->match) {
				sfe_ipv6_flow_cookie_set_func_t func;

				rcu_read_lock();
				func = rcu_dereference(si->flow_cookie_set_func);
				if (func) {
					func(cm->match_protocol, cm->match_src_ip->addr, cm->match_src_port,
					     cm->match_dest_ip->addr, cm->match_dest_port, 0);
				}
				rcu_read_unlock();

				cm->flow_cookie = 0;
				entry->match = NULL;
				entry->last_clean_time = jiffies;
				break;
			}
		}
	}
#endif
	hlist_del_init_rcu(&cm->hnode);

}

/*
 * sfe_ipv6_get_connection_hash()
 *	Generate the hash used in connection lookups.
 */
static inline unsigned int sfe_ipv6_get_connection_hash(u8 protocol, struct sfe_ipv6_addr *src_ip, __be16 src_port,
							struct sfe_ipv6_addr *dest_ip, __be16 dest_port)
{
	u32 idx, hash = 0;

	for (idx = 0; idx < 4; idx++) {
		hash ^= src_ip->addr[idx] ^ dest_ip->addr[idx];
	}
	hash = hash ^ protocol ^ ntohs(src_port) ^ dest_port;
	return ((hash >> SFE_IPV6_CONNECTION_HASH_SHIFT) ^ hash) & SFE_IPV6_CONNECTION_HASH_MASK;
}

/*
 * sfe_ipv6_find_connection()
 *	Get the IPv6 connection info that corresponds to a particular 5-tuple.
 *
 * On entry we must be holding the lock that protects the hash table.
 */
static inline struct sfe_ipv6_connection *sfe_ipv6_find_connection(struct sfe_ipv6 *si, u32 protocol,
								   struct sfe_ipv6_addr *src_ip, __be16 src_port,
								   struct sfe_ipv6_addr *dest_ip, __be16 dest_port)
{
	struct sfe_ipv6_connection *c;

	unsigned int conn_idx = sfe_ipv6_get_connection_hash(protocol, src_ip, src_port, dest_ip, dest_port);

	lockdep_assert_held(&si->lock);
	c = si->conn_hash[conn_idx];

	while (c) {
		if ((c->src_port == src_port)
		    && (c->dest_port == dest_port)
		    && (sfe_ipv6_addr_equal(c->src_ip, src_ip))
		    && (sfe_ipv6_addr_equal(c->dest_ip, dest_ip))
		    && (c->protocol == protocol)) {
			return c;
		}
		c = c->next;
	}

	return NULL;
}

/*
 * sfe_ipv6_insert_connection()
 *	Insert a connection into the hash.
 *
 * On entry we must be holding the lock that protects the hash table.
 */
static void sfe_ipv6_insert_connection(struct sfe_ipv6 *si, struct sfe_ipv6_connection *c)
{
	struct sfe_ipv6_connection **hash_head;
	struct sfe_ipv6_connection *prev_head;
	unsigned int conn_idx;

	lockdep_assert_held(&si->lock);

	/*
	 * Insert entry into the connection hash.
	 */
	conn_idx = sfe_ipv6_get_connection_hash(c->protocol, c->src_ip, c->src_port,
						c->dest_ip, c->dest_port);
	hash_head = &si->conn_hash[conn_idx];
	prev_head = *hash_head;
	c->prev = NULL;
	if (prev_head) {
		prev_head->prev = c;
	}

	c->next = prev_head;
	*hash_head = c;

	/*
	 * Insert entry into the "all connections" list.
	 */
	if (si->all_connections_tail) {
		c->all_connections_prev = si->all_connections_tail;
		si->all_connections_tail->all_connections_next = c;
	} else {
		c->all_connections_prev = NULL;
		si->all_connections_head = c;
	}

	si->all_connections_tail = c;
	c->all_connections_next = NULL;
	si->num_connections++;

	/*
	 * Insert the connection match objects too.
	 */
	sfe_ipv6_insert_connection_match(si, c->original_match);
	if (c->reply_match) {
		sfe_ipv6_insert_connection_match(si, c->reply_match);
	}
}

/*
 * sfe_ipv6_remove_connection()
 *	Remove a sfe_ipv6_connection object from the hash.
 *
 * On entry we must be holding the lock that protects the hash table.
 */
bool sfe_ipv6_remove_connection(struct sfe_ipv6 *si, struct sfe_ipv6_connection *c)
{
	sfe_fls_conn_delete_t delete_cb;

	lockdep_assert_held(&si->lock);
	if (c->removed) {
		DEBUG_ERROR("%px: Connection has been removed already\n", c);
		return false;
	}

	/*
	 * dereference the decap direction top_interface_dev
	 */
	if (c->reply_match) {
		if (c->reply_match->top_interface_dev) {
			dev_put(c->reply_match->top_interface_dev);
		}

		/*
		 * If qdisc_xmit_dev is present, dereference qdisc net dev.
		 */
		if (c->reply_match->qdisc_xmit_dev) {
			dev_put(c->reply_match->xmit_dev);
		}
		rcu_read_lock();
		delete_cb = rcu_dereference(sfe_fls_info.delete_cb);
		if (c->reply_match->fls_conn && delete_cb) {
			delete_cb(c->reply_match->fls_conn);
		}
		rcu_read_unlock();

		sfe_ipv6_remove_connection_match(si, c->reply_match);
	}

	/*
	 * If qdisc_xmit_dev is present, dereference qdisc net dev.
	 */
	if (c->original_match->qdisc_xmit_dev) {
		dev_put(c->original_match->xmit_dev);
	}

	rcu_read_lock();
	delete_cb = rcu_dereference(sfe_fls_info.delete_cb);
	if (c->original_match->fls_conn && delete_cb) {
		delete_cb(c->original_match->fls_conn);
	}
	rcu_read_unlock();

	sfe_ipv6_remove_connection_match(si, c->original_match);

	/*
	 * Unlink the connection.
	 */
	if (c->prev) {
		c->prev->next = c->next;
	} else {
		unsigned int conn_idx = sfe_ipv6_get_connection_hash(c->protocol, c->src_ip, c->src_port,
								     c->dest_ip, c->dest_port);
		si->conn_hash[conn_idx] = c->next;
	}

	if (c->next) {
		c->next->prev = c->prev;
	}

	/*
	 * Unlink connection from all_connections list
	 */
	if (c->all_connections_prev) {
		c->all_connections_prev->all_connections_next = c->all_connections_next;
	} else {
		si->all_connections_head = c->all_connections_next;
	}

	if (c->all_connections_next) {
		c->all_connections_next->all_connections_prev = c->all_connections_prev;
	} else {
		si->all_connections_tail = c->all_connections_prev;
	}

	/*
	 * If I am the next sync connection, move the sync to my next or head.
	 */
	if (unlikely(si->wc_next == c)) {
		si->wc_next = c->all_connections_next;
	}

	c->removed = true;
	si->num_connections--;
	return true;
}

/*
 * sfe_ipv6_gen_sync_connection()
 *	Sync a connection.
 *
 * On entry to this function we expect that the lock for the connection is either
 * already held (while called from sfe_ipv6_periodic_sync() or isn't required
 * (while called from sfe_ipv6_flush_sfe_ipv6_connection())
 */
static void sfe_ipv6_gen_sync_connection(struct sfe_ipv6 *si, struct sfe_ipv6_connection *c,
					struct sfe_connection_sync *sis, sfe_sync_reason_t reason,
					u64 now_jiffies)
{
	struct sfe_ipv6_connection_match *original_cm;
	struct sfe_ipv6_connection_match *reply_cm;
	u32 packet_count, byte_count;

	/*
	 * Fill in the update message.
	 */
	sis->is_v6 = 1;
	sis->protocol = c->protocol;
	sis->src_ip.ip6[0] = c->src_ip[0];
	sis->src_ip_xlate.ip6[0] = c->src_ip_xlate[0];
	sis->dest_ip.ip6[0] = c->dest_ip[0];
	sis->dest_ip_xlate.ip6[0] = c->dest_ip_xlate[0];
	sis->src_port = c->src_port;
	sis->src_port_xlate = c->src_port_xlate;
	sis->dest_port = c->dest_port;
	sis->dest_port_xlate = c->dest_port_xlate;

	original_cm = c->original_match;
	reply_cm = c->reply_match;
	sis->src_td_max_window = original_cm->protocol_state.tcp.max_win;
	sis->src_td_end = original_cm->protocol_state.tcp.end;
	sis->src_td_max_end = original_cm->protocol_state.tcp.max_end;

	sfe_ipv6_connection_match_update_summary_stats(original_cm, &packet_count, &byte_count);
	sis->src_new_packet_count = packet_count;
	sis->src_new_byte_count = byte_count;

	sis->src_dev = original_cm->match_dev;
	sis->src_packet_count = original_cm->rx_packet_count64;
	sis->src_byte_count = original_cm->rx_byte_count64;

	if (reply_cm) {
		sis->dest_td_max_window = reply_cm->protocol_state.tcp.max_win;
		sis->dest_td_end = reply_cm->protocol_state.tcp.end;
		sis->dest_td_max_end = reply_cm->protocol_state.tcp.max_end;

		sfe_ipv6_connection_match_update_summary_stats(reply_cm, &packet_count, &byte_count);
		sis->dest_new_packet_count = packet_count;
		sis->dest_new_byte_count = byte_count;

		sis->dest_dev = reply_cm->match_dev;
		sis->dest_packet_count = reply_cm->rx_packet_count64;
		sis->dest_byte_count = reply_cm->rx_byte_count64;
	}

	sis->reason = reason;

	/*
	 * Get the time increment since our last sync.
	 */
	sis->delta_jiffies = now_jiffies - c->last_sync_jiffies;
	c->last_sync_jiffies = now_jiffies;
}

/*
 * sfe_ipv6_free_mc_dest_rcu
 *	Delay to free the multicast dest.
 */
static void sfe_ipv6_free_mc_dest_rcu(struct rcu_head *head)
{
	struct sfe_ipv6_mc_dest *mc_dest;
	mc_dest = container_of(head, struct sfe_ipv6_mc_dest, rcu);
	kfree(mc_dest);
	return;
}

/*
 * sfe_ipv6_free_sfe_ipv6_connection_rcu()
 *	Called at RCU qs state to free the connection object.
 */
static void sfe_ipv6_free_sfe_ipv6_connection_rcu(struct rcu_head *head)
{
	struct sfe_ipv6_connection *c;
	struct sock *sk;

	/*
	 * We dont need spin lock as the connection is already removed from link list
	 */
	c = container_of(head, struct sfe_ipv6_connection, rcu);
	BUG_ON(!c->removed);

	DEBUG_TRACE("%px: connecton has been deleted\n", c);

	/*
	 * Decrease the refcount taken in function sfe_ipv6_create_rule()
	 * during call of __udp6_lib_lookup()
	 */
	if (c->reply_match) {
		if (c->reply_match->up) {
			sk = (struct sock *)c->reply_match->up;
			sock_put(sk);
		}
		dev_put(c->reply_dev);
		kfree(c->reply_match);
	}

	/*
	 * Release our hold of the source and dest devices and free the memory
	 * for our connection objects.
	 */
	dev_put(c->original_dev);
	if (c->original_match->flags & SFE_IPV6_CONNECTION_MATCH_FLAG_MULTICAST) {
		struct sfe_ipv6_mc_dest *cur, *tmp;
		list_for_each_entry_safe(cur, tmp, &(c->original_match->mc_list), list) {
			list_del_rcu(&cur->list);

			/*
			 * The connection c has been quiescent after being
			 * removed from connection list. During the grace
			 * period, mc_create_msg and the forwarding datapath
			 * could not get access this c any more and therefore
			 * mc list could not be accessed too. So we could
			 * replace call_rcu with kfree safely.
			 */
			kfree(cur);
		}
	}

	kfree(c->original_match);
	kfree(c);
}

/*
 * sfe_ipv6_fls_clear()
 *	Clear all flow statistics connection pointers.
 */
void sfe_ipv6_fls_clear(void)
{
	struct sfe_ipv6 *si6 = &__si6;
	uint32_t i;
	struct sfe_ipv6_connection_match *cm = NULL;

	rcu_read_lock();
	for (i = 0; i < SFE_IPV6_CONNECTION_HASH_SIZE; i++) {
		struct hlist_head *lhead = &si6->hlist_conn_match_hash_head[i];

		hlist_for_each_entry_rcu(cm, lhead, hnode) {
			cm->fls_conn = NULL;
		}
		break;
	}
	rcu_read_unlock();
}

/*
 * sfe_ipv6_sync_status()
 *	update a connection status to its connection manager.
 *
 * si: the ipv6 context
 * c: which connection to be notified
 * reason: what kind of reason: flush, or destroy
 */
void sfe_ipv6_sync_status(struct sfe_ipv6 *si,
				      struct sfe_ipv6_connection *c,
				      sfe_sync_reason_t reason)
{
	struct sfe_connection_sync sis;
	u64 now_jiffies;
	sfe_sync_rule_callback_t sync_rule_callback;

	rcu_read_lock();
	sync_rule_callback = rcu_dereference(si->sync_rule_callback);
	rcu_read_unlock();
	if (unlikely(!sync_rule_callback)) {
		return;
	}

	/*
	 * Generate a sync message and then sync.
	 */
	now_jiffies = get_jiffies_64();
	sfe_ipv6_gen_sync_connection(si, c, &sis, reason, now_jiffies);
	sync_rule_callback(&sis);
}

/*
 * sfe_ipv6_flush_connection()
 *	Flush a connection and free all associated resources.
 *
 * We need to be called with bottom halves disabled locally as we need to acquire
 * the connection hash lock and release it again.  In general we're actually called
 * from within a BH and so we're fine, but we're also called when connections are
 * torn down.
 */
void sfe_ipv6_flush_connection(struct sfe_ipv6 *si,
				      struct sfe_ipv6_connection *c,
				      sfe_sync_reason_t reason)
{
	BUG_ON(!c->removed);

	this_cpu_inc(si->stats_pcpu->connection_flushes64);
	sfe_ipv6_sync_status(si, c, reason);

#if defined(SFE_RFS_SUPPORTED)
	if (sfe_is_ppe_rfs_feature_enabled()) {
		struct ppe_rfs_ipv6_rule_destroy_msg pr6rd;

		pr6rd.tuple.flow_ip[0] = ntohl(c->src_ip[0].addr[3]);
		pr6rd.tuple.flow_ip[1] = ntohl(c->src_ip[0].addr[2]);
		pr6rd.tuple.flow_ip[2] = ntohl(c->src_ip[0].addr[1]);
		pr6rd.tuple.flow_ip[3] = ntohl(c->src_ip[0].addr[0]);
		pr6rd.tuple.flow_ident = ntohs(c->src_port);
		pr6rd.tuple.return_ip[0] = ntohl(c->dest_ip[0].addr[3]);
		pr6rd.tuple.return_ip[1] = ntohl(c->dest_ip[0].addr[2]);
		pr6rd.tuple.return_ip[2] = ntohl(c->dest_ip[0].addr[1]);
		pr6rd.tuple.return_ip[3] = ntohl(c->dest_ip[0].addr[0]);
		pr6rd.tuple.return_ident = ntohs(c->dest_port);
		pr6rd.tuple.protocol = c->protocol;

		pr6rd.original_dev = c->original_dev;
		pr6rd.reply_dev = c->reply_dev;

		if (ppe_rfs_ipv6_rule_destroy(&pr6rd) != PPE_RFS_RET_SUCCESS) {
			DEBUG_INFO("%p: Error in deleting IPv6 PPE RFS rules\n", &pr6rd);
		}
	}
#endif

	/*
	 * Release our hold of the source and dest devices and free the memory
	 * for our connection objects.
	 */
	call_rcu(&c->rcu, sfe_ipv6_free_sfe_ipv6_connection_rcu);
}

/*
 * sfe_ipv6_service_class_stats_pcpu_get()
 *	Gets one CPU's service class statistics.
 */
static inline bool sfe_ipv6_service_class_stats_pcpu_get(struct sfe_ipv6_per_service_class_stats *sc_stats, uint64_t *bytes, uint64_t *packets)
{
	uint32_t retries = 0;
	uint32_t seq;
	uint64_t bytes_tmp, packets_tmp;

	do {
		seq = read_seqcount_begin(&sc_stats->seq);
		bytes_tmp = sc_stats->tx_bytes;
		packets_tmp = sc_stats->tx_packets;
	} while (read_seqcount_retry(&sc_stats->seq, seq) && ++retries < SFE_SERVICE_CLASS_STATS_MAX_RETRY);

	*bytes += bytes_tmp;
	*packets += packets_tmp;

	return retries < SFE_SERVICE_CLASS_STATS_MAX_RETRY;
}

/*
 * sfe_ipv6_service_class_stats_get()
 *	Copy the ipv6 statistics for the given service class.
 */
bool sfe_ipv6_service_class_stats_get(uint8_t sid, uint64_t *bytes, uint64_t *packets)
{
	struct sfe_ipv6 *si = &__si6;
	uint32_t cpu = 0;

	for_each_possible_cpu(cpu) {
		struct sfe_ipv6_service_class_stats_db *stats_db = per_cpu_ptr(si->stats_pcpu_psc, cpu);
		struct sfe_ipv6_per_service_class_stats *sc_stats = &stats_db->psc_stats[sid];

		if (!sfe_ipv6_service_class_stats_pcpu_get(sc_stats, bytes, packets)) {
			return false;
		}
	}

	return true;
}

/*
 * sfe_ipv6_service_class_stats_inc()
 *	Increment per cpu per service class stats.
 */
void sfe_ipv6_service_class_stats_inc(struct sfe_ipv6 *si, uint8_t sid, uint64_t bytes)
{
	struct sfe_ipv6_service_class_stats_db *sc_stats_db = this_cpu_ptr(si->stats_pcpu_psc);
	struct sfe_ipv6_per_service_class_stats *sc_stats = &sc_stats_db->psc_stats[sid];

	write_seqcount_begin(&sc_stats->seq);
	sc_stats->tx_bytes += bytes;
	sc_stats->tx_packets++;
	write_seqcount_end(&sc_stats->seq);
}

/*
 * sfe_ipv6_exception_stats_inc()
 *	Increment exception stats.
 */
void sfe_ipv6_exception_stats_inc(struct sfe_ipv6 *si, enum sfe_ipv6_exception_events reason)
{
	struct sfe_ipv6_stats *stats = this_cpu_ptr(si->stats_pcpu);

	stats->exception_events64[reason]++;
	stats->packets_not_forwarded64++;
}

/*
 * sfe_ipv6_is_local_ip()
 *	return true if it is local ip otherwise return false
 */
static bool sfe_ipv6_is_local_ip(struct sfe_ipv6 *si, uint8_t *addr)
{
	struct net_device *dev;
	struct in6_addr ip_addr;
	memcpy(ip_addr.s6_addr, addr, 16);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
	dev = ipv6_dev_find(&init_net, &ip_addr, 1);
#else
	dev = ipv6_dev_find(&init_net, &ip_addr, NULL);
#endif

	if (dev) {
		dev_put(dev);
		return true;
	}

	return false;
}

/*
 * sfe_ipv6_recv()
 *	Handle packet receives and forwaring.
 *
 * Returns 1 if the packet is forwarded or 0 if it isn't.
 */
int sfe_ipv6_recv(struct net_device *dev, struct sk_buff *skb, struct sfe_l2_info *l2_info, bool tun_outer)
{
	struct sfe_ipv6 *si = &__si6;
	unsigned int len;
	unsigned int payload_len;
	unsigned int ihl = sizeof(struct ipv6hdr);
	bool sync_on_find = false;
	struct ipv6hdr *iph;
	u8 next_hdr;

	/*
	 * Check that we have space for an IP header and an uplayer header here.
	 */
	len = skb->len;
	if (!pskb_may_pull(skb, ihl + sizeof(struct sfe_ipv6_ext_hdr))) {

		sfe_ipv6_exception_stats_inc(si, SFE_IPV6_EXCEPTION_EVENT_HEADER_INCOMPLETE);
		DEBUG_TRACE("len: %u is too short\n", len);
		return 0;
	}

	/*
	 * Is our IP version wrong?
	 */
	iph = (struct ipv6hdr *)skb->data;
	if (unlikely(iph->version != 6)) {

		sfe_ipv6_exception_stats_inc(si, SFE_IPV6_EXCEPTION_EVENT_NON_V6);
		DEBUG_TRACE("IP version: %u\n", iph->version);
		return 0;
	}

	/*
	 * Does our datagram fit inside the skb?
	 */
	payload_len = ntohs(iph->payload_len);
	if (unlikely(payload_len > (len - ihl))) {

		sfe_ipv6_exception_stats_inc(si, SFE_IPV6_EXCEPTION_EVENT_DATAGRAM_INCOMPLETE);
		DEBUG_TRACE("payload_len: %u, exceeds len: %u\n", payload_len, (len - (unsigned int)sizeof(struct ipv6hdr)));
		return 0;
	}

	next_hdr = iph->nexthdr;
	while (unlikely(sfe_ipv6_is_ext_hdr(next_hdr))) {
		struct sfe_ipv6_ext_hdr *ext_hdr;
		unsigned int ext_hdr_len;

		ext_hdr = (struct sfe_ipv6_ext_hdr *)(skb->data + ihl);

		ext_hdr_len = ext_hdr->hdr_len;
		ext_hdr_len <<= 3;
		ext_hdr_len += sizeof(struct sfe_ipv6_ext_hdr);
		ihl += ext_hdr_len;
		if (!pskb_may_pull(skb, ihl + sizeof(struct sfe_ipv6_ext_hdr))) {
			sfe_ipv6_exception_stats_inc(si, SFE_IPV6_EXCEPTION_EVENT_HEADER_INCOMPLETE);

			DEBUG_TRACE("extension header %d not completed\n", next_hdr);
			return 0;
		}
		/*
		 * Any packets have extend hdr, won't be handled in the fast
		 * path,sync its status and exception to the kernel.
		 */
		sync_on_find = true;
		next_hdr = ext_hdr->next_hdr;
	}

	/*
	 * Handle PPPoE bridge packets using 3-tuple acceleration if SFE_PPPOE_BR_ACCEL_MODE_EN_3T
	 */
	if (unlikely(sfe_l2_parse_flag_check(l2_info, SFE_L2_PARSE_FLAGS_PPPOE_INGRESS)) &&
	    unlikely(sfe_pppoe_get_br_accel_mode() == SFE_PPPOE_BR_ACCEL_MODE_EN_3T)) {
		struct ethhdr *eth = eth_hdr(skb);
		if (!sfe_pppoe_mgr_find_session(l2_info->pppoe_session_id, eth->h_source)) {
			return sfe_ipv6_recv_pppoe_bridge(si, skb, dev, len, iph, ihl, l2_info);
		}
	}

	if (IPPROTO_UDP == next_hdr) {
		return sfe_ipv6_recv_udp(si, skb, dev, len, iph, ihl, sync_on_find, l2_info, tun_outer);
	}

	if (IPPROTO_TCP == next_hdr) {
		return sfe_ipv6_recv_tcp(si, skb, dev, len, iph, ihl, sync_on_find, l2_info);
	}

	if (IPPROTO_ESP == next_hdr) {
		return sfe_ipv6_recv_esp(si, skb, dev, len, iph, ihl, sync_on_find, l2_info, tun_outer);
	}

	if (IPPROTO_ETHERIP == next_hdr) {
		return sfe_ipv6_recv_etherip(si, skb, dev, len, iph, ihl, sync_on_find, l2_info, tun_outer);
	}

	if (IPPROTO_ICMPV6 == next_hdr) {
		return sfe_ipv6_recv_icmp(si, skb, dev, len, iph, ihl);
	}

	if (IPPROTO_IPIP == next_hdr) {
		return sfe_ipv6_recv_tunipip6(si, skb, dev, len, iph, ihl, sync_on_find, l2_info, true);
	}

#ifdef SFE_GRE_TUN_ENABLE
	if (IPPROTO_GRE == next_hdr) {
		return sfe_ipv6_recv_gre(si, skb, dev, len, iph, ihl, sync_on_find, l2_info, tun_outer);
	}
#endif

	sfe_ipv6_exception_stats_inc(si, SFE_IPV6_EXCEPTION_EVENT_UNHANDLED_PROTOCOL);
	DEBUG_TRACE("not UDP, TCP or ICMP: %u\n", next_hdr);
	return 0;
}

/*
 * sfe_ipv6_update_tcp_state()
 *	update TCP window variables.
 */
static void
sfe_ipv6_update_tcp_state(struct sfe_ipv6_connection *c,
			  struct sfe_ipv6_rule_create_msg *msg)
{
	struct sfe_ipv6_connection_match *orig_cm;
	struct sfe_ipv6_connection_match *repl_cm;
	struct sfe_ipv6_tcp_connection_match *orig_tcp;
	struct sfe_ipv6_tcp_connection_match *repl_tcp;

	orig_cm = c->original_match;
	repl_cm = c->reply_match;
	orig_tcp = &orig_cm->protocol_state.tcp;
	repl_tcp = &repl_cm->protocol_state.tcp;

	/* update orig */
	if (orig_tcp->max_win < msg->tcp_rule.flow_max_window) {
		orig_tcp->max_win = msg->tcp_rule.flow_max_window;
	}
	if ((s32)(orig_tcp->end - msg->tcp_rule.flow_end) < 0) {
		orig_tcp->end = msg->tcp_rule.flow_end;
	}
	if ((s32)(orig_tcp->max_end - msg->tcp_rule.flow_max_end) < 0) {
		orig_tcp->max_end = msg->tcp_rule.flow_max_end;
	}

	/* update reply */
	if (repl_tcp->max_win < msg->tcp_rule.return_max_window) {
		repl_tcp->max_win = msg->tcp_rule.return_max_window;
	}
	if ((s32)(repl_tcp->end - msg->tcp_rule.return_end) < 0) {
		repl_tcp->end = msg->tcp_rule.return_end;
	}
	if ((s32)(repl_tcp->max_end - msg->tcp_rule.return_max_end) < 0) {
		repl_tcp->max_end = msg->tcp_rule.return_max_end;
	}

	/* update match flags */
	orig_cm->flags &= ~SFE_IPV6_CONNECTION_MATCH_FLAG_NO_SEQ_CHECK;
	repl_cm->flags &= ~SFE_IPV6_CONNECTION_MATCH_FLAG_NO_SEQ_CHECK;
	if (msg->rule_flags & SFE_RULE_CREATE_FLAG_NO_SEQ_CHECK) {
		orig_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_NO_SEQ_CHECK;
		repl_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_NO_SEQ_CHECK;
	}
}

/*
 * sfe_ipv6_update_protocol_state()
 *	update protocol specified state machine.
 */
static void
sfe_ipv6_update_protocol_state(struct sfe_ipv6_connection *c,
			       struct sfe_ipv6_rule_create_msg *msg)
{
	switch (msg->tuple.protocol) {
	case IPPROTO_TCP:
		sfe_ipv6_update_tcp_state(c, msg);
		break;
	}
}

/*
 * sfe_ipv6_match_entry_set_vlan_hdr()
 */
static bool sfe_ipv6_match_entry_set_vlan_hdr(u8 *vlan_hdr_cnt, struct sfe_vlan_hdr *vlan_hdr,
		u32 primary_vlan_tag, u32 secondary_vlan_tag) {
	u16 tpid;

	u8 idx = (*vlan_hdr_cnt);
	if ((primary_vlan_tag & VLAN_VID_MASK) != SFE_VLAN_ID_NOT_CONFIGURED) {
		if (idx == SFE_MAX_VLAN_DEPTH) {
			DEBUG_TRACE("%px: vlan_hdr_cnt overflow while filling vlan_tag=%d\n", vlan_hdr, (u16)primary_vlan_tag);
			return false;
		}

		tpid = (u16)(primary_vlan_tag >> 16);
		vlan_hdr[idx].tpid = ntohs(tpid);
		vlan_hdr[idx].tci = (u16)primary_vlan_tag;
		(*vlan_hdr_cnt)++;
	}

	idx = (*vlan_hdr_cnt);
	if ((secondary_vlan_tag & VLAN_VID_MASK) != SFE_VLAN_ID_NOT_CONFIGURED) {
		if (idx == SFE_MAX_VLAN_DEPTH) {
			DEBUG_TRACE("%px: vlan_hdr_cnt overflow while filling vlan_tag=%d\n", vlan_hdr, (u16)secondary_vlan_tag);
			return false;
		}

		tpid = (u16)(secondary_vlan_tag >> 16);
		vlan_hdr[idx].tpid = ntohs(tpid);
		vlan_hdr[idx].tci = (u16)secondary_vlan_tag;
		(*vlan_hdr_cnt)++;
	}

	return true;
}

/*
 * sfe_ipv6_match_entry_set_vlan()
 */
static bool sfe_ipv6_match_entry_set_vlan(
			struct sfe_ipv6_connection_match *cm,
			u32 primary_ingress_vlan_tag,
			u32 primary_egress_vlan_tag,
			u32 secondary_ingress_vlan_tag,
			u32 secondary_egress_vlan_tag)
{
	if (sfe_ipv6_match_entry_set_vlan_hdr(&(cm->ingress_vlan_hdr_cnt), cm->ingress_vlan_hdr,
			primary_ingress_vlan_tag, secondary_ingress_vlan_tag)) {
		if (sfe_ipv6_match_entry_set_vlan_hdr(&(cm->egress_vlan_hdr_cnt), cm->egress_vlan_hdr,
				primary_egress_vlan_tag, secondary_egress_vlan_tag)) {
			return true;
		}
	}

	return false;
}

/*
 * sfe_ipv6_match_entry_set_trustsec()
 */
static void sfe_ipv6_match_entry_set_trustsec(
			struct sfe_ipv6_connection_match *cm,
			u16 ingress_sgt,
			u16 egress_sgt
			)
{
	cm->ingress_trustsec_hdr.sgt = ingress_sgt;
	cm->egress_trustsec_hdr.sgt = egress_sgt;
}

/*
 * sfe_ipv6_update_rule()
 *	update forwarding rule after rule is created.
 */
void sfe_ipv6_update_rule(struct sfe_ipv6_rule_create_msg *msg)

{
	struct sfe_ipv6_connection *c;
	struct sfe_ipv6 *si = &__si6;

	spin_lock_bh(&si->lock);

	c = sfe_ipv6_find_connection(si,
				     msg->tuple.protocol,
				     (struct sfe_ipv6_addr *)msg->tuple.flow_ip,
				     msg->tuple.flow_ident,
				     (struct sfe_ipv6_addr *)msg->tuple.return_ip,
				     msg->tuple.return_ident);
	if (c != NULL) {
		sfe_ipv6_update_protocol_state(c, msg);
	}

	spin_unlock_bh(&si->lock);
}

/*
 * sfe_ipv6_mark_rule_update()
 *	Updates the mark values of match entries.
 */
void sfe_ipv6_mark_rule_update(struct sfe_connection_mark *mark)
{
	struct sfe_ipv6_connection *c;
	struct sfe_ipv6 *si = &__si6;

	spin_lock_bh(&si->lock);
	c = sfe_ipv6_find_connection(si, mark->protocol,
				     (struct sfe_ipv6_addr *)mark->src_ip,
				     mark->src_port,
				     (struct sfe_ipv6_addr *)mark->dest_ip,
				     mark->dest_port);
	if (!c) {
		spin_unlock_bh(&si->lock);
		DEBUG_WARN("%px: connection not found for mark update\n", mark);
		return;
	}

	switch (mark->type) {
	case SFE_CONNECTION_MARK_TYPE_CONNMARK:
		c->original_match ->mark = mark->flow_mark;
		if (c->reply_match) {
			c->reply_match->mark = mark->return_mark;
		}
		spin_unlock_bh(&si->lock);
		break;

	case SFE_CONNECTION_MARK_TYPE_SAWFMARK:
		if (mark->flags & SFE_SAWF_MARK_FLOW_VALID) {
			c->original_match->mark = mark->flow_mark;
			c->original_match->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_MARK;
			c->original_match->sawf_valid = true;
		}

		if (c->reply_match) {
			if (mark->flags & SFE_SAWF_MARK_RETURN_VALID) {
				c->reply_match->mark = mark->return_mark;
				c->reply_match->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_MARK;
				c->reply_match->sawf_valid = true;
			}
		}
		spin_unlock_bh(&si->lock);
		break;

	default:
		spin_unlock_bh(&si->lock);
		DEBUG_WARN("%px: unsupported mark type: %d\n", mark, mark->type);
		break;
	}

	DEBUG_TRACE("%px: flow/return mark updated with %x/%x\n",
			c, mark->flow_mark, mark->return_mark);
}
EXPORT_SYMBOL(sfe_ipv6_mark_rule_update);

/*
 * sfe_ipv6_xmit_eth_type_check
 *	Checking if MAC header has to be written.
 */
static inline bool sfe_ipv6_xmit_eth_type_check(struct net_device *dev, u32 cm_flags)
{
	if (!(dev->flags & IFF_NOARP)) {
		return true;
	}

	/*
	 * For PPPoE, since we are now supporting PPPoE encapsulation, we are writing L2 header.
	 */
	if (cm_flags & SFE_IPV6_CONNECTION_MATCH_FLAG_PPPOE_ENCAP) {
		return true;
	}

	return false;
}

/*
 * sfe_ipv6_add_mc_dest_tail()
 *	Add mc dest to the tail.
 * return 0 successful, return 1 failed.
 */
int sfe_ipv6_add_mc_dest_tail(struct sfe_ipv6_connection_match *cm, struct sfe_ipv6_mc_dest *mc_dest) {
	struct sfe_ipv6_mc_dest *cur;

	list_for_each_entry_rcu(cur, &(cm->mc_list), list) {
		if (cur->xmit_dev == mc_dest->xmit_dev) {
			DEBUG_WARN("%px: The tail new destination[%s] already exists in the list\n", cm, mc_dest->xmit_dev->name);
			return 1;
		}
	}
	list_add_tail_rcu(&mc_dest->list, &cm->mc_list);

	return 0;
}

/*
 * sfe_ipv6_add_mc_dest_head()
 *	Add mc dest to the head.
 * return 0 successful, return 1 failed.
 */
int sfe_ipv6_add_mc_dest_head(struct sfe_ipv6_connection_match *cm, struct sfe_ipv6_mc_dest *mc_dest) {

	struct sfe_ipv6_mc_dest *cur;
	list_for_each_entry_rcu(cur, &(cm->mc_list), list) {
		if (cur->xmit_dev == mc_dest->xmit_dev) {
			DEBUG_WARN("%px: The head new destination[%s] already exists in the list\n", cm, mc_dest->xmit_dev->name);
			return 1;
		}
	}
	list_add_rcu(&mc_dest->list, &cm->mc_list);

	return 0;
}

/*
 * sfe_ipv6_find_and_delete_mc_dest()
 *	Delete the mc dest entry in the list.
 * return 0 successful, return 1 failed.
 */
int sfe_ipv6_find_and_delete_mc_dest(struct sfe_ipv6_connection_match *cm, struct net_device *dest_dev)
{
	struct sfe_ipv6_mc_dest *cur, *tmp;

	list_for_each_entry_safe(cur, tmp, &(cm->mc_list), list) {
		if(cur->xmit_dev == dest_dev) {
			list_del_rcu(&cur->list);
			call_rcu(&cur->rcu, sfe_ipv6_free_mc_dest_rcu);
			return 0;
		}
	}
	return 1;
}

/*
 * sfe_ipv6_allocate_mc_dest()
 *	Allocate a new dest and fill the rule information.
 */
struct sfe_ipv6_mc_dest *sfe_ipv6_allocate_mc_dest(struct sfe_ipv6_connection_match *cm, struct sfe_ipv6_mc_rule_create_msg *msg,
		struct sfe_ipv6_mc_device_entry *if_rule, struct net_device *dest_dev)
{
	struct sfe_ipv6_mc_dest *dest;
	int skb_changed = 0;

	dest = (struct sfe_ipv6_mc_dest *)kzalloc(sizeof(struct sfe_ipv6_mc_dest), GFP_ATOMIC);
	if (!dest) {
		return NULL;
	}

	if (if_rule->rule_flags & SFE_RULE_CREATE_FLAG_FLOW_TRANSMIT_FAST) {
		dest->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_FAST_XMIT_DEV_ADMISSION;
	}

	if (if_rule->valid_flags & SFE_MC_RULE_CREATE_IF_FLAG_VLAN_VALID) {
		/*
		 * Initialize egress VLAN information.
		 */
		dest->egress_vlan_hdr_cnt = 0;
		memset(dest->egress_vlan_hdr, 0, sizeof(struct sfe_vlan_hdr) * SFE_MAX_VLAN_DEPTH);

		sfe_ipv6_match_entry_set_vlan_hdr(&(dest->egress_vlan_hdr_cnt),	dest->egress_vlan_hdr,
				if_rule->egress_vlan_tag[0],if_rule->egress_vlan_tag[1]);

		if ((if_rule->rule_flags & SFE_RULE_CREATE_FLAG_USE_RETURN_BOTTOM_INTERFACE) &&
				dest->egress_vlan_hdr_cnt > 0) {
			dest->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_INSERT_EGRESS_VLAN_TAG;
			dest->l2_hdr_size += dest->egress_vlan_hdr_cnt * VLAN_HLEN;
			skb_changed = 1;
		}
	}

	if (if_rule->valid_flags & SFE_RULE_CREATE_PPPOE_ENCAP_VALID) {
		dest->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PPPOE_ENCAP;
		dest->l2_hdr_size += PPPOE_SES_HLEN;
		dest->pppoe_session_id = if_rule->pppoe_session_id;
		ether_addr_copy(dest->pppoe_remote_mac, if_rule->pppoe_remote_mac);
		skb_changed = 1;
	}

	/*
	 * For the non-arp interface, we don't write L2 HDR.
	 */
	if (sfe_ipv6_xmit_eth_type_check(dest_dev, if_rule->rule_flags)) {
		/*
		 * Check whether the rule has configured a specific source MAC address to use.
		 * This is needed when virtual L3 interfaces such as br-lan, macvlan, vlan are used during egress
		 */
		if (if_rule->rule_flags & SFE_MC_RULE_CREATE_IF_FLAG_BRIDGE_FLOW) {
			ether_addr_copy((u8 *)dest->xmit_src_mac, (u8 *)msg->conn_rule.flow_mac);
		} else {
			ether_addr_copy((u8 *)dest->xmit_src_mac, (u8 *)if_rule->if_mac);
		}
		ether_addr_copy((u8 *)dest->xmit_dest_mac, (u8 *)msg->dest_mac);
		dest->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_WRITE_L2_HDR;
		dest->l2_hdr_size += ETH_HLEN;
		skb_changed = 1;
		/*
		 * If our dev writes Ethernet headers then we can write a really fast
		 * version.
		 */
		if (dest_dev->header_ops) {
			if (dest_dev->header_ops->create == eth_header) {
				dest->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_WRITE_FAST_ETH_HDR;
			}
		}
	}

	/*
	 * If l2_features are disabled and flow uses l2 features such as macvlan/bridge/pppoe/vlan,
	 * bottom interfaces are expected to be disabled in the flow rule and always top interfaces
	 * are used. In such cases, do not use HW csum offload. csum offload is used only when we
	 * are sending directly to the destination interface that supports it.
	 */
	if (likely(dest_dev->features & NETIF_F_HW_CSUM) && sfe_dev_has_hw_csum(dest_dev)) {
		if ((msg->conn_rule.return_top_interface_num == msg->conn_rule.return_interface_num) ||
				(if_rule->rule_flags & SFE_RULE_CREATE_FLAG_USE_RETURN_BOTTOM_INTERFACE)) {
			/*
			 * Enable CSUM offload
			 */
			dest->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_CSUM_OFFLOAD;
		}
	}

	dest->xmit_dev = dest_dev;

	if (if_rule->valid_flags & SFE_MC_RULE_CREATE_IF_FLAG_NAT_VALID) {

		DEBUG_TRACE("%px: SRC nat was applied to:%s\n", msg, dest_dev->name);
		dest->xlate_src_ip[0] = if_rule->xlate_src_ip[0];
		dest->xlate_src_ip[1] = if_rule->xlate_src_ip[1];
		dest->xlate_src_ip[2] = if_rule->xlate_src_ip[2];
		dest->xlate_src_ip[3] = if_rule->xlate_src_ip[3];
		dest->xlate_src_ident = if_rule->xlate_src_ident;
		dest->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_XLATE_SRC;
		sfe_ipv6_connection_mc_dest_compute_translations(cm, dest);
		skb_changed = 1;
	}

	if (skb_changed) {
		dest->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_MULTICAST_CHANGED;
	}
	return dest;
}

/*
 * sfe_ipv6_update_mc_dest()
 *	update the mc dest rules.
 */
int sfe_ipv6_update_mc_dest(struct sfe_ipv6_connection_match *cm, struct sfe_ipv6_mc_rule_create_msg *msg)
{
	int idx;
	u16 if_cnt = msg->if_cnt;
	for (idx = 0; idx < if_cnt; idx++) {
		struct sfe_ipv6_mc_device_entry *if_rule = &msg->if_rule[idx];
		struct sfe_ipv6_mc_dest *dest;
		struct net_device *dest_dev;
		s32 dest_interface_num;

		dest_interface_num = if_rule->if_num;
		dest_dev = dev_get_by_index(&init_net, dest_interface_num);
		if (!dest_dev) {
			DEBUG_WARN("%px: Unable to find dest_dev corresponding to %d\n", msg, dest_interface_num);
			return -EINVAL;
		}

		if (if_rule->rule_flags & SFE_MC_RULE_CREATE_IF_FLAG_LEAVE) {
			sfe_ipv6_find_and_delete_mc_dest(cm, dest_dev);
			dev_put(dest_dev);
			continue;
		}

		dest = sfe_ipv6_allocate_mc_dest(cm, msg, if_rule, dest_dev);
		if (!dest) {
			DEBUG_WARN("%px: Unable to allocate dest corresponding to %d\n", msg, dest_interface_num);
			dev_put(dest_dev);
			return -EINVAL;

		}

		/*
		 * make the destination that need change the skb before the
		 * destinations that doesn't.
		 * so the last destination of the list could safely use the
		 * original skb.
		 */
		if (dest->flags & SFE_IPV6_CONNECTION_MATCH_FLAG_MULTICAST_CHANGED) {
			if (sfe_ipv6_add_mc_dest_head(cm, dest)) {
				DEBUG_WARN("%px: Unable to insert dest[%s] to the head\n", msg, dest_dev->name);
				dev_put(dest_dev);
				kfree(dest);
				return -EINVAL;
			}
		} else {
			if (sfe_ipv6_add_mc_dest_tail(cm, dest)) {
				DEBUG_WARN("%px: Unable to insert dest[%s] to the tail\n", msg, dest_dev->name);
				dev_put(dest_dev);
				kfree(dest);
				return -EINVAL;
			}
		}
	}
	return 0;
}

/*
 * sfe_ipv6_create_mc_dest()
 *	create the mc dest rules.
 */
int sfe_ipv6_create_mc_dest(struct sfe_ipv6_connection_match *cm, struct sfe_ipv6_mc_rule_create_msg *msg)
{
	int idx;
	u16 if_cnt = msg->if_cnt;
	for (idx = 0; idx < if_cnt; idx++) {
		struct sfe_ipv6_mc_device_entry *if_rule = &msg->if_rule[idx];
		struct sfe_ipv6_mc_dest *dest;
		struct net_device *dest_dev;
		s32 dest_interface_num;

		dest_interface_num = if_rule->if_num;
		dest_dev = dev_get_by_index(&init_net, dest_interface_num);
		if (!dest_dev) {
			DEBUG_WARN("%px: Unable to find dest_dev corresponding to %d\n", msg,
					dest_interface_num);
			return -EINVAL;
		}

		dest = sfe_ipv6_allocate_mc_dest(cm, msg, if_rule, dest_dev);
		if (!dest) {
			DEBUG_WARN("%px: Unable to allocate dest corresponding to %d\n", msg, dest_interface_num);
			dev_put(dest_dev);
			return -EINVAL;

		}

		/*
		 * make the destination that need change the skb before the
		 * destinations that doesn't.
		 * so the last destination of the list could safely use the
		 * original skb.
		 */
		if (dest->flags & SFE_IPV6_CONNECTION_MATCH_FLAG_MULTICAST_CHANGED) {
			if (sfe_ipv6_add_mc_dest_head(cm, dest)) {
				DEBUG_WARN("%px: Unable to insert dest[%s] to the head\n", msg, dest_dev->name);
				dev_put(dest_dev);
				kfree(dest);
				return -EINVAL;
			}
		} else {
			if (sfe_ipv6_add_mc_dest_tail(cm, dest)) {
				DEBUG_WARN("%px: Unable to insert dest[%s] to the tail\n", msg, dest_dev->name);
				dev_put(dest_dev);
				kfree(dest);
				return -EINVAL;
			}
		}
	}
	return 0;
}

/*
 * sfe_ipv6_create_mc_rule()
 *	Create a multicast forwarding rule.
 */
int sfe_ipv6_create_mc_rule(struct sfe_ipv6_mc_rule_create_msg *msg)
{
	struct sfe_ipv6 *si = &__si6;
	struct sfe_ipv6_connection *c, *c_old;
	struct sfe_ipv6_connection_match *original_cm;
	struct net_device *dest_dev;
	struct net_device *src_dev;
	struct sfe_ipv6_5tuple *tuple = &msg->tuple;
	s32 flow_interface_num = msg->conn_rule.flow_top_interface_num;
	unsigned int src_if_idx;
	uint32_t if_min_mtu;
	int idx;
	u32 flow_sawf_tag;

	if (IPPROTO_UDP != tuple->protocol) {
		this_cpu_inc(si->stats_pcpu->connection_create_failures64);
		return -EINVAL;
	}

	if (msg->rule_flags & SFE_RULE_CREATE_FLAG_USE_FLOW_BOTTOM_INTERFACE) {
		flow_interface_num = msg->conn_rule.flow_interface_num;
	}

	src_dev = dev_get_by_index(&init_net, flow_interface_num);
	if (!src_dev) {
		DEBUG_WARN("%px: Unable to find src_dev corresponding to %d\n", msg,
						flow_interface_num);
		this_cpu_inc(si->stats_pcpu->connection_create_failures64);
		return -EINVAL;
	}

	if_min_mtu = msg->if_rule[0].if_mtu;

	for (idx = 0; idx < msg->if_cnt; idx ++) {
		s32 dest_interface_num;
		struct sfe_ipv6_mc_device_entry *if_rule = &msg->if_rule[idx];
		dest_interface_num = if_rule->if_num;
		dest_dev = dev_get_by_index(&init_net, dest_interface_num);
		if (!dest_dev) {
			DEBUG_WARN("%px: Unable to find dest_dev corresponding to %d\n", msg,
					dest_interface_num);
			this_cpu_inc(si->stats_pcpu->connection_create_failures64);
			dev_put(src_dev);
			return -EINVAL;
		}

		if (unlikely((dest_dev->reg_state != NETREG_REGISTERED) ||
					(src_dev->reg_state != NETREG_REGISTERED))) {
			dev_put(src_dev);
			dev_put(dest_dev);
			DEBUG_WARN("%px: src_dev=%s and dest_dev=%s are unregistered\n", msg,
					src_dev->name, dest_dev->name);
			this_cpu_inc(si->stats_pcpu->connection_create_failures64);
			return -EINVAL;
		}
		dev_put(dest_dev);
		if (if_rule->if_mtu < if_min_mtu) {
			if_min_mtu = if_rule->if_mtu;
		}
	}

#if (defined(SFE_MEM_PROFILE_MEDIUM) || defined(SFE_MEM_PROFILE_LOW))
	if (si->num_connections  >= sfe_ipv6_max_conn_count()) {
		spin_unlock_bh(&si->lock);
		this_cpu_inc(si->stats_pcpu->connection_create_requests_overflow64);
		dev_put(src_dev);
		DEBUG_WARN("%px: Maximum connection count(%d), reached %d\n", msg, sfe_ipv6_max_conn_count(), si->num_connections);
		return -EPERM;
	}
#endif

	/*
	 * Allocate the various connection tracking objects.
	 */
	c = (struct sfe_ipv6_connection *)kzalloc(sizeof(struct sfe_ipv6_connection), GFP_ATOMIC);
	if (unlikely(!c)) {
		DEBUG_WARN("%px: memory allocation of connection entry failed\n", msg);
		this_cpu_inc(si->stats_pcpu->connection_create_failures64);
		dev_put(src_dev);
		return -ENOMEM;
	}

	original_cm = (struct sfe_ipv6_connection_match *)kzalloc(sizeof(struct sfe_ipv6_connection_match), GFP_ATOMIC);
	if (unlikely(!original_cm)) {
		DEBUG_WARN("%px: memory allocation of connection match entry failed\n", msg);
		this_cpu_inc(si->stats_pcpu->connection_create_failures64);
		kfree(c);
		dev_put(src_dev);
		return -ENOMEM;
	}

	INIT_LIST_HEAD_RCU(&(original_cm->mc_list));
	this_cpu_inc(si->stats_pcpu->connection_create_requests64);

	spin_lock_bh(&si->lock);

	/*
	 * Check to see if there is already a flow that matches the rule we're
	 * trying to create.  If there is then we can't create a new one.
	 */
	c_old = sfe_ipv6_find_connection(si,
					tuple->protocol,
					(struct sfe_ipv6_addr *)tuple->flow_ip,
					tuple->flow_ident,
					(struct sfe_ipv6_addr *)tuple->return_ip,
					tuple->return_ident);

	if (c_old != NULL) {
		if (!(msg->rule_flags & SFE_MC_RULE_CREATE_FLAG_MC_UPDATE)) {
			DEBUG_WARN("%px: Rule existing but not update message\n", msg);
			goto update_done;
		}

		/*
		 * If we already have the flow then it's likely that this
		 * request to create the connection rule contains more
		 * up-to-date information. Check and update accordingly.
		 */
		if (sfe_ipv6_update_mc_dest(c_old->original_match, msg)) {
			this_cpu_inc(si->stats_pcpu->connection_create_failures64);
		}
update_done:
		spin_unlock_bh(&si->lock);

		kfree(original_cm);
		kfree(c);

		dev_put(src_dev);

		DEBUG_TRACE("%px: connection update the dest interface list -  p:%d\n"
			    "  s: %s:%pM:%pI4:%u, d: Multicast:%pM:%pI4:%u\n",
			    msg, tuple->protocol,
			    src_dev->name, msg->conn_rule.flow_mac, &tuple->flow_ip, ntohs(tuple->flow_ident),
			    msg->dest_mac, &tuple->return_ip, ntohs(tuple->return_ident));

		return 0;
	}

	/*
	 * Fill in the "original" direction connection matching object.
	 * Note that the transmit MAC address is "dest_mac_xlate" because
	 * we always know both ends of a connection by their translated
	 * addresses and not their public addresses.
	 */
	original_cm->match_dev = src_dev;
	original_cm->match_protocol = tuple->protocol;
	original_cm->match_src_ip[0] = *(struct sfe_ipv6_addr *)tuple->flow_ip;
	original_cm->match_src_port = (msg->rule_flags & SFE_RULE_CREATE_FLAG_NO_SRC_IDENT) ? 0 : tuple->flow_ident;
	original_cm->match_dest_ip[0] = *(struct sfe_ipv6_addr *)tuple->return_ip;
	original_cm->match_dest_port = tuple->return_ident;

	original_cm->xlate_src_ip[0] = *(struct sfe_ipv6_addr *)tuple->flow_ip;
	original_cm->xlate_src_port = tuple->flow_ident;

	original_cm->xmit_dev_mtu = if_min_mtu;

	original_cm->connection = c;
	original_cm->counter_match = NULL;

	if (msg->valid_flags & SFE_RULE_CREATE_MARK_VALID) {
		original_cm->mark = msg->mark_rule.flow_mark;
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_MARK;
	}
	if (msg->valid_flags & SFE_RULE_CREATE_QOS_VALID) {
		original_cm->priority =  msg->qos_rule.flow_qos_tag;
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PRIORITY_REMARK;
	}
	if (msg->valid_flags & SFE_RULE_CREATE_DSCP_MARKING_VALID) {
		original_cm->dscp = msg->dscp_rule.flow_dscp << SFE_IPV6_DSCP_SHIFT;
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_DSCP_REMARK;
	}
	if (msg->rule_flags & SFE_RULE_CREATE_FLAG_BRIDGE_FLOW) {
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_BRIDGE_FLOW;
	}


	/*
	 * Mark SAWF metadata if the sawf tag is valid and set.
	 */
	original_cm->sawf_valid = false;
	flow_sawf_tag = SFE_GET_SAWF_TAG(msg->sawf_rule.flow_mark);
	if (likely(SFE_SAWF_TAG_IS_VALID(flow_sawf_tag))) {
		original_cm->mark = msg->sawf_rule.flow_mark;
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_MARK;
		original_cm->sawf_valid = true;
	}

	/*
	 * Add VLAN rule to original_cm
	 */
	if (msg->valid_flags & SFE_RULE_CREATE_VLAN_VALID) {
		struct sfe_vlan_rule *vlan_primary_rule = &msg->vlan_primary_rule;
		struct sfe_vlan_rule *vlan_secondary_rule = &msg->vlan_secondary_rule;

		/*
		 * Initialize ingress VLAN information.
		 */
		original_cm->ingress_vlan_hdr_cnt = 0;
		memset(original_cm->ingress_vlan_hdr, 0, sizeof(struct sfe_vlan_hdr) * SFE_MAX_VLAN_DEPTH);

		sfe_ipv6_match_entry_set_vlan_hdr(&(original_cm->ingress_vlan_hdr_cnt),
				original_cm->ingress_vlan_hdr,
				vlan_primary_rule->ingress_vlan_tag,
				vlan_secondary_rule->ingress_vlan_tag);
	}

	if (((IPPROTO_GRE == tuple->protocol) || (IPPROTO_ESP == tuple->protocol)) &&
					!sfe_ipv6_is_local_ip(si, (uint8_t *)original_cm->match_dest_ip)) {
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PASSTHROUGH;
	}

#ifdef CONFIG_NF_FLOW_COOKIE
	original_cm->flow_cookie = 0;
#endif
#ifdef CONFIG_XFRM
	if (msg->valid_flags & SFE_RULE_CREATE_DIRECTION_VALID) {
		original_cm->flow_accel = msg->direction_rule.flow_accel;
	} else {
		original_cm->flow_accel = 1;
	}
#endif


	if (msg->rule_flags & SFE_RULE_CREATE_FLAG_FLOW_SRC_INTERFACE_CHECK) {
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_SRC_INTERFACE_CHECK;
	}

	if (msg->rule_flags & SFE_RULE_CREATE_FLAG_FLOW_SRC_INTERFACE_CHECK_NO_FLUSH) {
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_SRC_INTERFACE_CHECK_NO_FLUSH;
	}

	/*
	 * For multicast, it is only valid in the flow direction
	 */
	if (msg->valid_flags & SFE_RULE_CREATE_PPPOE_DECAP_VALID) {
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PPPOE_DECAP;
		original_cm->pppoe_session_id = msg->pppoe_rule.flow_pppoe_session_id;
		ether_addr_copy(original_cm->pppoe_remote_mac, msg->pppoe_rule.flow_pppoe_remote_mac);
	}


	src_if_idx = src_dev->ifindex;

	/*
	 * the net_protocol handler will be used only in decap path
	 * for non passthrough case.
	 */
	original_cm->proto = NULL;
	original_cm->top_interface_dev = NULL;

	original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_MULTICAST;

	if (0 != sfe_ipv6_create_mc_dest(original_cm, msg)) {
		struct sfe_ipv6_mc_dest *cur, *next;
		this_cpu_inc(si->stats_pcpu->connection_create_failures64);
		list_for_each_entry_safe(cur, next, &original_cm->mc_list, list) {
			list_del_rcu(&cur->list);
			call_rcu(&cur->rcu, sfe_ipv6_free_mc_dest_rcu);
		}
		spin_unlock_bh(&si->lock);

		kfree(original_cm);
		kfree(c);
		dev_put(src_dev);
		return 0;
	}
	/*
	 * Fill in the ipv6_connection object.
	 */
	c->protocol = tuple->protocol;
	c->src_ip[0] = *(struct sfe_ipv6_addr *)tuple->flow_ip;
	c->src_ip_xlate[0] = *(struct sfe_ipv6_addr *)msg->conn_rule.flow_ip_xlate;
	c->src_port = tuple->flow_ident;
	c->src_port_xlate = msg->conn_rule.flow_ident_xlate;
	c->original_dev = src_dev;
	c->original_match = original_cm;
	c->dest_ip[0] = *(struct sfe_ipv6_addr *)tuple->return_ip;
	c->dest_ip_xlate[0] = *(struct sfe_ipv6_addr *)msg->conn_rule.return_ip_xlate;
	c->dest_port = tuple->return_ident;
	c->dest_port_xlate = msg->conn_rule.return_ident_xlate;
	c->debug_read_seq = 0;
	c->last_sync_jiffies = get_jiffies_64();
	c->removed = false;

	sfe_ipv6_insert_connection(si, c);

	spin_unlock_bh(&si->lock);

	/*
	 * We have everything we need!
	 */
	DEBUG_INFO("%px: new multicast connection - p: %d\n"
		   "  s: %s:%pxM(%pxM):%pI6(%pI6):%u(%u)\n"
		   "  d: %s:%pxM(%pxM):%pI6(%pI6):%u(%u)\n",
		   c, tuple->protocol,
		   src_dev->name, msg->conn_rule.flow_mac, NULL,
		   (void *)tuple->flow_ip, (void *)&msg->conn_rule.flow_ip_xlate, ntohs(tuple->flow_ident), ntohs(msg->conn_rule.flow_ident_xlate),
		   dest_dev->name, NULL, msg->dest_mac,
		   (void *)tuple->return_ip, (void *)&msg->conn_rule.return_ip_xlate, ntohs(tuple->return_ident), ntohs(msg->conn_rule.flow_ident_xlate));
	return 0;
}

/*
 * sfe_ipv6_destroy_mc_rule()
 *	destory a multicast forwarding rule.
 */
void sfe_ipv6_destroy_mc_rule(struct sfe_ipv6_mc_rule_destroy_msg *msg)
{
	return sfe_ipv6_destroy_rule((struct sfe_ipv6_rule_destroy_msg *)msg);
}

/*
 * sfe_ipv6_create_rule()
 *	Create a forwarding rule.
 */
int sfe_ipv6_create_rule(struct sfe_ipv6_rule_create_msg *msg)
{
	struct sfe_ipv6 *si = &__si6;
	struct sfe_ipv6_connection *c, *old_c;
	struct sfe_ipv6_connection_match *original_cm;
	struct sfe_ipv6_connection_match *reply_cm;
	struct net_device *dest_dev;
	struct net_device *src_dev;
	struct sfe_ipv6_5tuple *tuple = &msg->tuple;
	struct sock *sk;
	struct net *net;
	unsigned int src_if_idx;
	void *orig_conn;
	void *reply_conn;
	sfe_fls_conn_create_t create_cb;

	s32 flow_interface_num = msg->conn_rule.flow_top_interface_num;
	s32 return_interface_num = msg->conn_rule.return_top_interface_num;
	u32 flow_sawf_tag;
	u32 return_sawf_tag;
	bool disable_l2_flow = false;
	bool disable_l2_return = false;

	/*
	 * Check if L2 features need to be disabled
	 */
	if (sfe_is_l2_feature_enabled()) {
		if ((msg->rule_flags & SFE_RULE_CREATE_FLAG_FLOW_L2_DISABLE)) {
			disable_l2_flow = true;
			DEBUG_TRACE("disable_l2_flow is set to true\n");
		}

		if ((msg->rule_flags & SFE_RULE_CREATE_FLAG_RETURN_L2_DISABLE)) {
			disable_l2_return = true;
			DEBUG_TRACE("disable_l2_return is set to true\n");
		}
	}

	/*
	 * Set the xmit interface to bottom interface if applicable.
	 * If we are explicitly asked to disable L2 processing, then for the receive direction,
	 * we need to store the bottom interface to use it as match_dev.
	 */
	if (msg->rule_flags & SFE_RULE_CREATE_FLAG_USE_FLOW_BOTTOM_INTERFACE || disable_l2_flow) {
		flow_interface_num = msg->conn_rule.flow_interface_num;
	}

	if (msg->rule_flags & SFE_RULE_CREATE_FLAG_USE_RETURN_BOTTOM_INTERFACE || disable_l2_return) {
		return_interface_num = msg->conn_rule.return_interface_num;
	}

	src_dev = dev_get_by_index(&init_net, flow_interface_num);
	if (!src_dev) {
		DEBUG_WARN("%px: Unable to find src_dev corresponding to %d\n", msg,
						flow_interface_num);
		this_cpu_inc(si->stats_pcpu->connection_create_failures64);
		return -EINVAL;
	}

	dest_dev = dev_get_by_index(&init_net, return_interface_num);
	if (!dest_dev) {
		DEBUG_WARN("%px: Unable to find dest_dev corresponding to %d\n", msg,
						return_interface_num);
		this_cpu_inc(si->stats_pcpu->connection_create_failures64);
		dev_put(src_dev);
		return -EINVAL;
	}

	if (unlikely((dest_dev->reg_state != NETREG_REGISTERED) ||
		     (src_dev->reg_state != NETREG_REGISTERED))) {
		DEBUG_WARN("%px: src_dev=%s and dest_dev=%s are unregistered\n", msg,
						src_dev->name, dest_dev->name);
		this_cpu_inc(si->stats_pcpu->connection_create_failures64);
		dev_put(src_dev);
		dev_put(dest_dev);
		return -EINVAL;
	}

	/*
	 * Allocate the various connection tracking objects.
	 */
	c = (struct sfe_ipv6_connection *)kzalloc(sizeof(struct sfe_ipv6_connection), GFP_ATOMIC);
	if (unlikely(!c)) {
		DEBUG_WARN("%px: memory allocation of connection entry failed\n", msg);
		this_cpu_inc(si->stats_pcpu->connection_create_failures64);
		dev_put(src_dev);
		dev_put(dest_dev);
		return -ENOMEM;
	}

	original_cm = (struct sfe_ipv6_connection_match *)kzalloc(sizeof(struct sfe_ipv6_connection_match), GFP_ATOMIC);
	if (unlikely(!original_cm)) {
		this_cpu_inc(si->stats_pcpu->connection_create_failures64);
		DEBUG_WARN("%px: memory allocation of connection match entry failed\n", msg);
		kfree(c);
		dev_put(src_dev);
		dev_put(dest_dev);
		return -ENOMEM;
	}

	reply_cm = (struct sfe_ipv6_connection_match *)kzalloc(sizeof(struct sfe_ipv6_connection_match), GFP_ATOMIC);
	if (unlikely(!reply_cm)) {
		this_cpu_inc(si->stats_pcpu->connection_create_failures64);
		DEBUG_WARN("%px: memory allocation of connection match entry failed\n", msg);
		kfree(original_cm);
		kfree(c);
		dev_put(src_dev);
		dev_put(dest_dev);
		return -ENOMEM;
	}

	this_cpu_inc(si->stats_pcpu->connection_create_requests64);

	spin_lock_bh(&si->lock);

	/*
	 * Check to see if max number of connection limit reached.
	 */
	if (si->num_connections  >= sfe_ipv6_max_conn_count()) {
		spin_unlock_bh(&si->lock);
		this_cpu_inc(si->stats_pcpu->connection_create_requests_overflow64);
		kfree(reply_cm);
		kfree(original_cm);
		kfree(c);
		dev_put(src_dev);
		dev_put(dest_dev);
		DEBUG_WARN("%px: Maximum connection count(%d), reached %d\n", msg, sfe_ipv6_max_conn_count(), si->num_connections);
		return -EPERM;
	}

	/*
	 * Check to see if there is already a flow that matches the rule we're
	 * trying to create.  If there is then we can't create a new one.
	 */
	old_c = sfe_ipv6_find_connection(si,
					tuple->protocol,
					(struct sfe_ipv6_addr *)tuple->flow_ip,
					tuple->flow_ident,
					(struct sfe_ipv6_addr *)tuple->return_ip,
					tuple->return_ident);

	if (old_c != NULL) {
		this_cpu_inc(si->stats_pcpu->connection_create_collisions64);

		/*
		 * If we already have the flow then it's likely that this
		 * request to create the connection rule contains more
		 * up-to-date information. Check and update accordingly.
		 */
		sfe_ipv6_update_protocol_state(old_c, msg);
		spin_unlock_bh(&si->lock);

		kfree(reply_cm);
		kfree(original_cm);
		kfree(c);
		dev_put(src_dev);
		dev_put(dest_dev);

		DEBUG_TRACE("connection already exists -  p: %d\n"
			    "  s: %s:%pxM:%pI6:%u, d: %s:%pxM:%pI6:%u\n",
			    tuple->protocol,
			    src_dev->name, msg->conn_rule.flow_mac, tuple->flow_ip, ntohs(tuple->flow_ident),
			   dest_dev->name, msg->conn_rule.return_mac, tuple->return_ip, ntohs(tuple->return_ident));
		return -EADDRINUSE;
	}

	/*
	 * Fill in the "original" direction connection matching object.
	 * Note that the transmit MAC address is "dest_mac_xlate" because
	 * we always know both ends of a connection by their translated
	 * addresses and not their public addresses.
	 */
	original_cm->match_dev = src_dev;
	original_cm->match_protocol = tuple->protocol;
	original_cm->match_src_ip[0] = *(struct sfe_ipv6_addr *)tuple->flow_ip;
	original_cm->match_src_port = (msg->rule_flags & SFE_RULE_CREATE_FLAG_NO_SRC_IDENT) ? 0 : tuple->flow_ident;
	original_cm->match_dest_ip[0] = *(struct sfe_ipv6_addr *)tuple->return_ip;
	original_cm->match_dest_port = tuple->return_ident;

	original_cm->xlate_src_ip[0] = *(struct sfe_ipv6_addr *)msg->conn_rule.flow_ip_xlate;
	original_cm->xlate_src_port = msg->conn_rule.flow_ident_xlate;
	original_cm->xlate_dest_ip[0] = *(struct sfe_ipv6_addr *)msg->conn_rule.return_ip_xlate;
	original_cm->xlate_dest_port =  msg->conn_rule.return_ident_xlate;

	original_cm->xmit_dev = dest_dev;

	/*
	 * Enable qdisc fast xmit path if single qdisc is present on non-bottom interface.
	 * For qdisc is enabled on bottom interface alone, we use dev_queue_xmit() instead to transmit to bottom interface.
	 */
	if ((msg->valid_flags & SFE_RULE_CREATE_QDISC_RULE_VALID) &&
			(msg->qdisc_rule.valid_flags & SFE_QDISC_RULE_RETURN_VALID) &&
			(msg->qdisc_rule.return_qdisc_interface != msg->conn_rule.return_interface_num)) {

		original_cm->xmit_dev = dev_get_by_index(&init_net, msg->qdisc_rule.return_qdisc_interface);
		original_cm->qdisc_xmit_dev = dest_dev;
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_FAST_QDISC_XMIT;
		DEBUG_TRACE("%p: Fast qdisc xmit flag is set for original_cm, xmit_dev: %s, qdisc_xmit_dev: %s\n",
				msg, original_cm->xmit_dev->name, original_cm->qdisc_xmit_dev->name);
	} else if (disable_l2_return) {
		/*
		 * Set the xmit dev to the top interface, and store it in top_interface_dev,
		 * to take the reference.
		 * TODO: Store the top_interface in CE instead to take the reference.
		 */
		if (!(reply_cm->top_interface_dev)) {
			reply_cm->top_interface_dev = dev_get_by_index(&init_net, msg->conn_rule.return_top_interface_num);
		}

		original_cm->xmit_dev = reply_cm->top_interface_dev;
		DEBUG_TRACE("%p: Multiple qdisc found on return hierarchy, setting original_cm xmit_dev as top,\
				original_cm->xmit_dev: %s, original_cm->match_dev: %s\n",
				msg, original_cm->xmit_dev->name, original_cm->match_dev->name);
	}

	original_cm->xmit_dev_mtu = msg->conn_rule.return_mtu;

	original_cm->connection = c;
	original_cm->counter_match = reply_cm;

	/*
	 * Valid in decap direction only
	 */
	RCU_INIT_POINTER(original_cm->up, NULL);

	if (msg->valid_flags & SFE_RULE_CREATE_MARK_VALID) {
		original_cm->mark =  msg->mark_rule.flow_mark;
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_MARK;
	}

	if (msg->valid_flags & SFE_RULE_CREATE_QOS_VALID) {
#if defined(SFE_PPE_QOS_SUPPORTED)

		/*
		 * SFE_QDISC_RULE_RETURN_PPE_QDISC_FAST_XMIT flag is set when qdisc is configured in flow direction
		 */
		if(msg->qdisc_rule.valid_flags & SFE_QDISC_RULE_RETURN_PPE_QDISC_FAST_XMIT) {
			original_cm->int_pri = msg->qos_rule.flow_int_pri;
		}
#endif
		original_cm->priority = msg->qos_rule.flow_qos_tag;
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PRIORITY_REMARK;
	}

	if (msg->valid_flags & SFE_RULE_CREATE_DSCP_MARKING_VALID) {
		original_cm->dscp = msg->dscp_rule.flow_dscp << SFE_IPV6_DSCP_SHIFT;
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_DSCP_REMARK;
	}
	if (msg->rule_flags & SFE_RULE_CREATE_FLAG_BRIDGE_FLOW) {
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_BRIDGE_FLOW;
	}
	if (msg->rule_flags & SFE_RULE_CREATE_FLAG_FLOW_TRANSMIT_FAST) {
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_FAST_XMIT_DEV_ADMISSION;
	}

	/*
	 * Mark SAWF metadata if the sawf tag is valid.
	 */
	original_cm->sawf_valid = false;
	flow_sawf_tag = SFE_GET_SAWF_TAG(msg->sawf_rule.flow_mark);
	if (likely(SFE_SAWF_TAG_IS_VALID(flow_sawf_tag))) {
		original_cm->mark = msg->sawf_rule.flow_mark;
		original_cm->sawf_valid = true;
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_MARK;
	}

	/*
	 * Add VLAN rule to original_cm
	 */
	if (msg->valid_flags & SFE_RULE_CREATE_VLAN_VALID) {
		struct sfe_vlan_rule *vlan_primary_rule = &msg->vlan_primary_rule;
		struct sfe_vlan_rule *vlan_secondary_rule = &msg->vlan_secondary_rule;
		sfe_ipv6_match_entry_set_vlan(original_cm,
					     vlan_primary_rule->ingress_vlan_tag,
					     vlan_primary_rule->egress_vlan_tag,
					     vlan_secondary_rule->ingress_vlan_tag,
					     vlan_secondary_rule->egress_vlan_tag);

		if ((msg->rule_flags & SFE_RULE_CREATE_FLAG_USE_RETURN_BOTTOM_INTERFACE) &&
				original_cm->egress_vlan_hdr_cnt > 0) {
			original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_INSERT_EGRESS_VLAN_TAG;
			original_cm->l2_hdr_size += original_cm->egress_vlan_hdr_cnt * VLAN_HLEN;
		}
	}

#ifdef SFE_BRIDGE_VLAN_FILTERING_ENABLE
	/*
	 * Add Bridge VLAN Filter rule in original_cm
	 */
	if (msg->valid_flags & SFE_RULE_CREATE_VLAN_FILTER_VALID) {
		DEBUG_INFO("%px: Bridge VLAN Filter rule configuration received from connection manager in original dir\n"
				" orig_cm:  vlan_filter_ingress_tag: %x flags: %x \n"
				" orig_cm:  vlan_filter_egress_tag:  %x flags: %x \n",
				msg, msg->flow_vlan_filter_rule.ingress_vlan_tag, msg->flow_vlan_filter_rule.ingress_flags,
				msg->flow_vlan_filter_rule.egress_vlan_tag, msg->flow_vlan_filter_rule.egress_flags);

		/*
		 * Populate the VLAN Filter rule in the connection match entry.
		 */
		original_cm->vlan_filter_rule.ingress_vlan_tag = msg->flow_vlan_filter_rule.ingress_vlan_tag;
		original_cm->vlan_filter_rule.ingress_flags = msg->flow_vlan_filter_rule.ingress_flags;

		/*
		 * Add VLAN Filter rule for ingress validation / egress tagging.
		 * We might be stacking Bridge VLAN Filter headers after traditional VLAN headers. (not clearing out the arry).
		 * This might fail, when both primary and secondary traditional VLANs have been configured.
		 */
		if (!sfe_ipv6_match_entry_set_vlan(original_cm, msg->flow_vlan_filter_rule.ingress_vlan_tag, msg->flow_vlan_filter_rule.egress_vlan_tag,
					SFE_VLAN_ID_NOT_CONFIGURED, SFE_VLAN_ID_NOT_CONFIGURED)) {
			this_cpu_inc(si->stats_pcpu->connection_create_failures64);
			spin_unlock_bh(&si->lock);
			kfree(reply_cm);
			kfree(original_cm);
			kfree(c);
			dev_put(src_dev);
			dev_put(dest_dev);
			DEBUG_WARN("%px: More than %d VLAN & VLAN Filter rules are not allowed\n", msg, SFE_MAX_VLAN_DEPTH);
			return -EPERM;
		}

		if (!(msg->flow_vlan_filter_rule.egress_flags & SFE_VLAN_FILTER_FLAG_EGRESS_UNTAGGED) &&
			(original_cm->egress_vlan_hdr_cnt > 0)) {
			original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_INSERT_EGRESS_VLAN_TAG;
			original_cm->l2_hdr_size += original_cm->egress_vlan_hdr_cnt * VLAN_HLEN;
			DEBUG_TRACE("%px: original_cm: Bridge VLAN insert egress VLAN Tag found\n", msg);
		}
	}
#endif

	if ((IPPROTO_GRE == tuple->protocol) && !sfe_ipv6_is_local_ip(si, (uint8_t *)original_cm->match_dest_ip)) {
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PASSTHROUGH;
	}

	if ((IPPROTO_ETHERIP == tuple->protocol) && !sfe_ipv6_is_local_ip(si, (uint8_t *)original_cm->match_dest_ip)) {
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PASSTHROUGH;
	}

	/*
	 * In decap case, we should not set cm->proto for bypass ipip6 packets.
	 */
	if(IPPROTO_IPIP == tuple->protocol && !sfe_dev_is_ipip6(src_dev)) {
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PASSTHROUGH;
	}

#ifdef CONFIG_NF_FLOW_COOKIE
	original_cm->flow_cookie = 0;
#endif
#ifdef CONFIG_XFRM
	if (msg->valid_flags & SFE_RULE_CREATE_DIRECTION_VALID) {
		original_cm->flow_accel = msg->direction_rule.flow_accel;
	} else {
		original_cm->flow_accel = 1;
	}
#endif

	/*
	 * Check if TSO is enabled on the bottom interface. This flag is used for PPPoE TCP flows.
	 * For PPPoE TCP flows, we can only use HW TSO since kernel does not support GSO for this path.
	 */
	if (unlikely(dest_dev->features & NETIF_F_TSO) && (msg->rule_flags & SFE_RULE_CREATE_FLAG_USE_RETURN_BOTTOM_INTERFACE)) {
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_TSO_ENABLE;
		DEBUG_TRACE("%px: TSO is enabled on the destination interface: %s", msg, dest_dev->name);
	}

	/*
	 * If l2_features are disabled and flow uses l2 features such as macvlan/bridge/pppoe/vlan,
	 * bottom interfaces are expected to be disabled in the flow rule and always top interfaces
	 * are used. In such cases, do not use HW csum offload. csum offload is used only when we
	 * are sending directly to the destination interface that supports it.
	 */
	if (likely(dest_dev->features & NETIF_F_HW_CSUM) && sfe_dev_has_hw_csum(dest_dev)) {
		if ((msg->conn_rule.return_top_interface_num == msg->conn_rule.return_interface_num) ||
			(msg->rule_flags & SFE_RULE_CREATE_FLAG_USE_RETURN_BOTTOM_INTERFACE)) {
			/*
			 * Dont enable CSUM offload
			 */
			 original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_CSUM_OFFLOAD;
		}
	}

	/*
	 * Adding PPPoE parameters to original and reply entries based on the direction where
	 * PPPoE header is valid in ECM rule.
	 *
	 * If PPPoE is valid in flow direction (from interface is PPPoE), then
	 *	original cm will have PPPoE at ingress (strip PPPoE header)
	 *	reply cm will have PPPoE at egress (add PPPoE header)
	 *
	 * If PPPoE is valid in return direction (to interface is PPPoE), then
	 *	original cm will have PPPoE at egress (add PPPoE header)
	 *	reply cm will have PPPoE at ingress (strip PPPoE header)
	 */
	if (msg->valid_flags & SFE_RULE_CREATE_PPPOE_DECAP_VALID) {
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PPPOE_DECAP;
		original_cm->pppoe_session_id = msg->pppoe_rule.flow_pppoe_session_id;
		ether_addr_copy(original_cm->pppoe_remote_mac, msg->pppoe_rule.flow_pppoe_remote_mac);

		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PPPOE_ENCAP;
		reply_cm->l2_hdr_size += PPPOE_SES_HLEN;
		reply_cm->pppoe_session_id = msg->pppoe_rule.flow_pppoe_session_id;
		ether_addr_copy(reply_cm->pppoe_remote_mac, msg->pppoe_rule.flow_pppoe_remote_mac);
	}

	if (msg->valid_flags & SFE_RULE_CREATE_PPPOE_ENCAP_VALID) {
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PPPOE_ENCAP;
		original_cm->l2_hdr_size += PPPOE_SES_HLEN;
		original_cm->pppoe_session_id = msg->pppoe_rule.return_pppoe_session_id;
		ether_addr_copy(original_cm->pppoe_remote_mac, msg->pppoe_rule.return_pppoe_remote_mac);

		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PPPOE_DECAP;
		reply_cm->pppoe_session_id = msg->pppoe_rule.return_pppoe_session_id;
		ether_addr_copy(reply_cm->pppoe_remote_mac, msg->pppoe_rule.return_pppoe_remote_mac);
	}

	if (msg->rule_flags & SFE_RULE_CREATE_FLAG_FLOW_SRC_INTERFACE_CHECK) {
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_SRC_INTERFACE_CHECK;
	}

	if (msg->rule_flags & SFE_RULE_CREATE_FLAG_FLOW_SRC_INTERFACE_CHECK_NO_FLUSH) {
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_SRC_INTERFACE_CHECK_NO_FLUSH;
	}

	if (sfe_is_l2_feature_enabled() && !(msg->rule_flags & SFE_RULE_CREATE_FLAG_BRIDGE_FLOW)) {
		if (msg->rule_flags & SFE_RULE_CREATE_FLAG_USE_FLOW_BOTTOM_INTERFACE) {
			if (sfe_dev_is_bridge(original_cm->top_interface_dev)) {
				original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PACKET_HOST;
			}
		}
		if (msg->rule_flags & SFE_RULE_CREATE_FLAG_USE_RETURN_BOTTOM_INTERFACE) {
			if (sfe_dev_is_bridge(reply_cm->top_interface_dev)) {
				reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PACKET_HOST;
			}
		}
	}

	/*
	 * Disable HW CSUM Offload for connection with trustsec header
	 */
	if (msg->valid_flags & SFE_RULE_CREATE_TRUSTSEC_VALID) {
		struct sfe_trustsec_rule *trustsec_rule = &msg->trustsec_rule;
		sfe_ipv6_match_entry_set_trustsec(original_cm,
						trustsec_rule->ingress_sgt,
						trustsec_rule->egress_sgt);
		original_cm->flags &= ~SFE_IPV6_CONNECTION_MATCH_FLAG_CSUM_OFFLOAD;

		if (trustsec_rule->sgt_valid_flags & SFE_TRUSTSEC_INGRESS_SGT_VALID) {
			original_cm->ingress_trustsec_valid = true;
		}

		if (trustsec_rule->sgt_valid_flags & SFE_TRUSTSEC_EGRESS_SGT_VALID) {
			original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_INSERT_EGRESS_TRUSTSEC_SGT;
		}
	}

	/*
	 * For the non-arp interface, we don't write L2 HDR.
	 * Excluding PPPoE from this, since we are now supporting PPPoE encap/decap.
	 */
	if (sfe_ipv6_xmit_eth_type_check(dest_dev, original_cm->flags)) {

		/*
		 * Check whether the rule has configured a specific source MAC address to use.
		 * This is needed when virtual L3 interfaces such as br-lan, macvlan, vlan are used during egress
		 */
		if (msg->rule_flags & SFE_RULE_CREATE_FLAG_BRIDGE_FLOW) {
			ether_addr_copy((u8 *)original_cm->xmit_src_mac, (u8 *)msg->conn_rule.flow_mac);
		} else {
			if ((msg->valid_flags & SFE_RULE_CREATE_SRC_MAC_VALID) &&
			    (msg->src_mac_rule.mac_valid_flags & SFE_SRC_MAC_RETURN_VALID)) {
				ether_addr_copy((u8 *)original_cm->xmit_src_mac, (u8 *)msg->src_mac_rule.return_src_mac);
			} else {
				ether_addr_copy((u8 *)original_cm->xmit_src_mac, (u8 *)dest_dev->dev_addr);
			}
		}

		/*
		 * In route mode, the return mac address could be wrong since it
		 * use to_nat device if destination is PPPoE device, using pppoe_remote_mac
		 * is more safe
		 */
		if (original_cm->flags & SFE_IPV6_CONNECTION_MATCH_FLAG_PPPOE_ENCAP) {
			ether_addr_copy((u8 *)original_cm->xmit_dest_mac, (u8 *)original_cm->pppoe_remote_mac);
		} else {
			ether_addr_copy((u8 *)original_cm->xmit_dest_mac, (u8 *)msg->conn_rule.return_mac);
		}

		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_WRITE_L2_HDR;
		original_cm->l2_hdr_size += ETH_HLEN;

		/*
		 * If our dev writes Ethernet headers then we can write a really fast
		 * version
		 */
		if (dest_dev->header_ops) {
			if (dest_dev->header_ops->create == eth_header) {
				original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_WRITE_FAST_ETH_HDR;
			}
		}
	}

	/*
	 * Fill in the "reply" direction connection matching object.
	 */
	reply_cm->match_dev = dest_dev;
	reply_cm->match_protocol = tuple->protocol;
	reply_cm->match_src_ip[0] = *(struct sfe_ipv6_addr *)msg->conn_rule.return_ip_xlate;
	reply_cm->match_dest_ip[0] = *(struct sfe_ipv6_addr *)msg->conn_rule.flow_ip_xlate;
	reply_cm->match_dest_port = msg->conn_rule.flow_ident_xlate;
	reply_cm->xlate_src_ip[0] = *(struct sfe_ipv6_addr *)tuple->return_ip;
	reply_cm->xlate_src_port = tuple->return_ident;
	reply_cm->xlate_dest_ip[0] = *(struct sfe_ipv6_addr *)tuple->flow_ip;
	reply_cm->xlate_dest_port = tuple->flow_ident;

	/*
	 * Keep source port as 0 for tunnels requiring 4-tuple match (eg: VxLAN).
	 */
	reply_cm->match_src_port = (msg->rule_flags & SFE_RULE_CREATE_FLAG_NO_SRC_IDENT) ? 0 : msg->conn_rule.return_ident_xlate;

	reply_cm->xmit_dev = src_dev;

	rcu_read_lock();
	create_cb = rcu_dereference(sfe_fls_info.create_cb);
	if (create_cb) {
		create_cb(6, original_cm->match_protocol,
					(uint32_t *)original_cm->match_src_ip,
					original_cm->match_src_port,
					(uint32_t *)original_cm->match_dest_ip,
					original_cm->match_dest_port,
					(uint32_t *)reply_cm->match_src_ip,
					reply_cm->match_src_port,
					(uint32_t *)reply_cm->match_dest_ip,
					reply_cm->match_dest_port,
					&orig_conn,
					&reply_conn);

		if (orig_conn && reply_conn) {
			original_cm->fls_conn = orig_conn;
			reply_cm->fls_conn = reply_conn;
		}
	}
	rcu_read_unlock();

	/*
	 * Enable qdisc fast xmit path if single qdisc is present on non-bottom interface.
	 * For qdisc is enabled on bottom interface alone, we use dev_queue_xmit() instead to transmit to bottom interface.
	 */
	if ((msg->valid_flags & SFE_RULE_CREATE_QDISC_RULE_VALID) &&
			(msg->qdisc_rule.valid_flags & SFE_QDISC_RULE_FLOW_VALID) &&
			(msg->qdisc_rule.flow_qdisc_interface != msg->conn_rule.flow_interface_num)) {

		reply_cm->xmit_dev = dev_get_by_index(&init_net, msg->qdisc_rule.flow_qdisc_interface);
		reply_cm->qdisc_xmit_dev = src_dev;
		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_FAST_QDISC_XMIT;
		DEBUG_TRACE("%p: Fast qdisc xmit flag is set for original_cm, xmit_dev: %s, qdisc_xmit_dev: %s\n",
				msg, reply_cm->xmit_dev->name, reply_cm->qdisc_xmit_dev->name);
	} else if (disable_l2_flow) {
		/*
		 * Set the xmit dev to the top interface, and store it in top_interface_dev,
		 * to take the reference.
		 * TODO: Store the top_interface in CE instead to take the reference.
		 */
		if (!(original_cm->top_interface_dev)) {
			original_cm->top_interface_dev = dev_get_by_index(&init_net, msg->conn_rule.flow_top_interface_num);
		}

		reply_cm->xmit_dev = original_cm->top_interface_dev;
		DEBUG_TRACE("%p: Multiple qdisc found on flow hierarchy, setting xmit_dev as top,\
				reply_cm->xmit_dev: %s, reply_cm->match_dev: %s\n",
				msg, reply_cm->xmit_dev->name, reply_cm->match_dev->name);
	}

	reply_cm->xmit_dev_mtu = msg->conn_rule.flow_mtu;

	reply_cm->connection = c;
	reply_cm->counter_match = original_cm;

	if (msg->valid_flags & SFE_RULE_CREATE_MARK_VALID) {
		reply_cm->mark =  msg->mark_rule.return_mark;
		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_MARK;
	}
	if (msg->valid_flags & SFE_RULE_CREATE_QOS_VALID) {
#if defined(SFE_PPE_QOS_SUPPORTED)

		/*
		 * SFE_QDISC_RULE_FLOW_PPE_QDISC_FAST_XMIT flag is set when qdisc is configured in return direction
		 */
		if(msg->qdisc_rule.valid_flags & SFE_QDISC_RULE_FLOW_PPE_QDISC_FAST_XMIT) {
			reply_cm->int_pri = msg->qos_rule.return_int_pri;
		}
#endif
		reply_cm->priority = msg->qos_rule.return_qos_tag;
		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PRIORITY_REMARK;
	}
	if (msg->valid_flags & SFE_RULE_CREATE_DSCP_MARKING_VALID) {
		reply_cm->dscp = msg->dscp_rule.return_dscp << SFE_IPV6_DSCP_SHIFT;
		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_DSCP_REMARK;
	}
	if (msg->rule_flags & SFE_RULE_CREATE_FLAG_BRIDGE_FLOW) {
		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_BRIDGE_FLOW;
	}
	if (msg->rule_flags & SFE_RULE_CREATE_FLAG_RETURN_TRANSMIT_FAST) {
		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_FAST_XMIT_DEV_ADMISSION;
	}

	if ((IPPROTO_GRE == tuple->protocol) && !sfe_ipv6_is_local_ip(si, (uint8_t *)reply_cm->match_dest_ip)) {
		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PASSTHROUGH;
	}

	if ((IPPROTO_ETHERIP == tuple->protocol) && !sfe_ipv6_is_local_ip(si, (uint8_t *)reply_cm->match_dest_ip)) {
		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PASSTHROUGH;
	}

        /*
         * In decap case, we should not set cm->proto for bypass ipip6 packets.
         */
	if(IPPROTO_IPIP == tuple->protocol && !sfe_dev_is_ipip6(src_dev)) {
		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_PASSTHROUGH;
	}

	/*
	 * Mark return SAWF metadata if the sawf tag is valid.
	 */
	reply_cm->sawf_valid = false;
	return_sawf_tag = SFE_GET_SAWF_TAG(msg->sawf_rule.return_mark);
	if (likely(SFE_SAWF_TAG_IS_VALID(return_sawf_tag))) {
		reply_cm->mark = msg->sawf_rule.return_mark;
		reply_cm->sawf_valid = true;
		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_MARK;
	}

	/*
	 * Strip off flags if L2 processing is disabled in the rule.
	 * SFE transmits the packet to the top interface for L2 processing
	 */
	if (disable_l2_flow) {
		reply_cm->flags &= ~SFE_IPV6_CONNECTION_MATCH_FLAG_PPPOE_ENCAP;
		reply_cm->flags &= ~SFE_IPV6_CONNECTION_MATCH_FLAG_WRITE_L2_HDR;
		reply_cm->flags &= ~SFE_IPV6_CONNECTION_MATCH_FLAG_WRITE_FAST_ETH_HDR;
		reply_cm->flags &= ~SFE_IPV6_CONNECTION_MATCH_FLAG_INSERT_EGRESS_VLAN_TAG;
	}

	if (disable_l2_return) {
		original_cm->flags &= ~SFE_IPV6_CONNECTION_MATCH_FLAG_PPPOE_ENCAP;
		original_cm->flags &= ~SFE_IPV6_CONNECTION_MATCH_FLAG_WRITE_L2_HDR;
		original_cm->flags &= ~SFE_IPV6_CONNECTION_MATCH_FLAG_WRITE_FAST_ETH_HDR;
		original_cm->flags &= ~SFE_IPV6_CONNECTION_MATCH_FLAG_INSERT_EGRESS_VLAN_TAG;
	}

	/*
	 * Setup UDP Socket if found to be valid for decap.
	 */
	RCU_INIT_POINTER(reply_cm->up, NULL);
	net = dev_net(reply_cm->match_dev);
	src_if_idx = src_dev->ifindex;

	rcu_read_lock();

	/*
	 * Look for the associated sock object.
	 * __udp6_lib_lookup() holds a reference for this sock object,
	 * which will be released in sfe_ipv6_flush_connection()
	 */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 13, 0))
	sk = __udp6_lib_lookup(net, (const struct in6_addr *)reply_cm->match_dest_ip,
			reply_cm->match_dest_port, (const struct in6_addr *)reply_cm->xlate_src_ip,
			reply_cm->xlate_src_port, src_if_idx, &udp_table);
#else
	sk = __udp6_lib_lookup(net, (const struct in6_addr *)reply_cm->match_dest_ip,
			reply_cm->match_dest_port, (const struct in6_addr *)reply_cm->xlate_src_ip,
			reply_cm->xlate_src_port, src_if_idx, 0, &udp_table, NULL);
#endif
	rcu_read_unlock();

	/*
	 * We set the UDP sock pointer as valid only for decap direction.
	 */
	if (sk && udp_sk(sk)->encap_type) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 13, 0))
		if (!atomic_add_unless(&sk->sk_refcnt, 1, 0)) {
#else
		if (!refcount_inc_not_zero(&sk->sk_refcnt)) {
#endif
			this_cpu_inc(si->stats_pcpu->connection_create_failures64);
			spin_unlock_bh(&si->lock);
			kfree(reply_cm);
			kfree(original_cm);
			kfree(c);

			DEBUG_INFO("sfe: unable to take reference for socket  p:%d\n", tuple->protocol);
			DEBUG_INFO("SK: connection - \n"
					"  s: %s:%pI6(%pI6):%u(%u)\n"
				   "  d: %s:%pI6(%pI6):%u(%u)\n",
					reply_cm->match_dev->name, &reply_cm->match_src_ip, &reply_cm->xlate_src_ip,
					ntohs(reply_cm->match_src_port), ntohs(reply_cm->xlate_src_port),
					reply_cm->xmit_dev->name, &reply_cm->match_dest_ip, &reply_cm->xlate_dest_ip,
					ntohs(reply_cm->match_dest_port), ntohs(reply_cm->xlate_dest_port));

			dev_put(src_dev);
			dev_put(dest_dev);

			return -ESHUTDOWN;
		}

		rcu_assign_pointer(reply_cm->up, udp_sk(sk));
		DEBUG_INFO("Sock lookup success with reply_cm direction(%p)\n", sk);
		DEBUG_INFO("SK: connection - \n"
			   "  s: %s:%pI6(%pI6):%u(%u)\n"
			   "  d: %s:%pI6(%pI6):%u(%u)\n",
			reply_cm->match_dev->name, &reply_cm->match_src_ip, &reply_cm->xlate_src_ip,
			ntohs(reply_cm->match_src_port), ntohs(reply_cm->xlate_src_port),
			reply_cm->xmit_dev->name, &reply_cm->match_dest_ip, &reply_cm->xlate_dest_ip,
			ntohs(reply_cm->match_dest_port), ntohs(reply_cm->xlate_dest_port));
	}

	/*
	 * Add VLAN rule to reply_cm
	 */
	if (msg->valid_flags & SFE_RULE_CREATE_VLAN_VALID) {
		struct sfe_vlan_rule *vlan_primary_rule = &msg->vlan_primary_rule;
		struct sfe_vlan_rule *vlan_secondary_rule = &msg->vlan_secondary_rule;
		sfe_ipv6_match_entry_set_vlan(reply_cm,
					     vlan_primary_rule->egress_vlan_tag,
					     vlan_primary_rule->ingress_vlan_tag,
					     vlan_secondary_rule->egress_vlan_tag,
					     vlan_secondary_rule->ingress_vlan_tag);

		if ((msg->rule_flags & SFE_RULE_CREATE_FLAG_USE_FLOW_BOTTOM_INTERFACE) &&
				reply_cm->egress_vlan_hdr_cnt > 0) {
			reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_INSERT_EGRESS_VLAN_TAG;
			reply_cm->l2_hdr_size += reply_cm->egress_vlan_hdr_cnt * VLAN_HLEN;
		}
	}

#ifdef SFE_BRIDGE_VLAN_FILTERING_ENABLE
	/*
	 * Add Bridge VLAN Filter rule in reply_cm
	 */
	if (msg->valid_flags & SFE_RULE_CREATE_VLAN_FILTER_VALID) {
		DEBUG_INFO("%px: Bridge VLAN Filter rule configuration received from connection manager in reply dir\n"
				" reply_cm: vlan_filter_ingress_tag: %x flags: %x \n"
				" reply_cm: vlan_filter_egress_tag:  %x flags: %x \n",
				msg, msg->return_vlan_filter_rule.ingress_vlan_tag, msg->return_vlan_filter_rule.ingress_flags,
				msg->return_vlan_filter_rule.egress_vlan_tag, msg->return_vlan_filter_rule.egress_flags);

		/*
		 * Populate the VLAN Filter rule in the connection match entry.
		 */
		reply_cm->vlan_filter_rule.ingress_vlan_tag = msg->return_vlan_filter_rule.ingress_vlan_tag;
		reply_cm->vlan_filter_rule.ingress_flags = msg->return_vlan_filter_rule.ingress_flags;

		/*
		 * Add VLAN Filter rule.
		 * This might fail, when both primary and secondary traditional vlan has been configured.
		 */
		if (!sfe_ipv6_match_entry_set_vlan(reply_cm, msg->return_vlan_filter_rule.ingress_vlan_tag, msg->return_vlan_filter_rule.egress_vlan_tag,
					SFE_VLAN_ID_NOT_CONFIGURED, SFE_VLAN_ID_NOT_CONFIGURED)) {
			this_cpu_inc(si->stats_pcpu->connection_create_failures64);
			spin_unlock_bh(&si->lock);
			kfree(reply_cm);
			kfree(original_cm);
			kfree(c);
			dev_put(src_dev);
			dev_put(dest_dev);
			DEBUG_WARN("%px: More than %d VLAN & VLAN Filter rules are not allowed\n", msg, SFE_MAX_VLAN_DEPTH);
			return -EPERM;
		}

		if (!(msg->return_vlan_filter_rule.egress_flags & SFE_VLAN_FILTER_FLAG_EGRESS_UNTAGGED) &&
			(reply_cm->egress_vlan_hdr_cnt > 0)) {
			reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_INSERT_EGRESS_VLAN_TAG;
			reply_cm->l2_hdr_size += reply_cm->egress_vlan_hdr_cnt * VLAN_HLEN;
			DEBUG_TRACE("%px: reply_cm: Bridge VLAN insert egress VLAN Tag found\n", msg);
		}
	}
#endif

#ifdef CONFIG_NF_FLOW_COOKIE
	reply_cm->flow_cookie = 0;
#endif
#ifdef CONFIG_XFRM
	if (msg->valid_flags & SFE_RULE_CREATE_DIRECTION_VALID) {
		reply_cm->flow_accel = msg->direction_rule.return_accel;
	} else {
		reply_cm->flow_accel = 1;
	}
#endif

	/*
	 * the inet6_protocol handler will be used only in decap path
	 * for non passthrough case.
	 */
	original_cm->proto = NULL;
	reply_cm->proto = NULL;
	original_cm->top_interface_dev = NULL;
	reply_cm->top_interface_dev = NULL;

#ifdef SFE_GRE_TUN_ENABLE
	if ((IPPROTO_GRE == tuple->protocol) && !(reply_cm->flags & SFE_IPV6_CONNECTION_MATCH_FLAG_PASSTHROUGH)) {
		rcu_read_lock();
		reply_cm->proto = rcu_dereference(inet6_protos[IPPROTO_GRE]);
		rcu_read_unlock();

		if (unlikely(!reply_cm->proto)) {
			this_cpu_inc(si->stats_pcpu->connection_create_failures64);
			spin_unlock_bh(&si->lock);
			kfree(reply_cm);
			kfree(original_cm);
			kfree(c);
			dev_put(src_dev);
			dev_put(dest_dev);
			DEBUG_WARN("sfe: GRE proto handler is not registered\n");
			return -EPERM;
		}
	}
#endif

	if ((IPPROTO_ESP == tuple->protocol) && !(reply_cm->flags & SFE_IPV6_CONNECTION_MATCH_FLAG_PASSTHROUGH)) {
		rcu_read_lock();
		reply_cm->proto = rcu_dereference(inet6_protos[IPPROTO_ESP]);
		rcu_read_unlock();

		if (unlikely(!reply_cm->proto)) {
			this_cpu_inc(si->stats_pcpu->connection_create_failures64);
			spin_unlock_bh(&si->lock);
			kfree(reply_cm);
			kfree(original_cm);
			kfree(c);
			dev_put(src_dev);
			dev_put(dest_dev);
			DEBUG_WARN("sfe: ESP proto handler is not registered\n");
			return -EPERM;
		}
	}

	if ((IPPROTO_ETHERIP == tuple->protocol) && !(reply_cm->flags & SFE_IPV6_CONNECTION_MATCH_FLAG_PASSTHROUGH)) {
		rcu_read_lock();
		reply_cm->proto = rcu_dereference(inet6_protos[IPPROTO_ETHERIP]);
		rcu_read_unlock();

		if (unlikely(!reply_cm->proto)) {
			this_cpu_inc(si->stats_pcpu->connection_create_failures64);
			spin_unlock_bh(&si->lock);
			kfree(reply_cm);
			kfree(original_cm);
			kfree(c);
			dev_put(src_dev);
			dev_put(dest_dev);
			DEBUG_WARN("sfe: Etherip proto handler is not registered\n");
			return -EPERM;
		}
	}

	/*
	 * Decapsulation path have proto set.
	 * This is used to differentiate de/encap, and call protocol specific handler.
	 */
	if (IPPROTO_IPIP == tuple->protocol && !(reply_cm->flags & SFE_IPV6_CONNECTION_MATCH_FLAG_PASSTHROUGH)) {
		original_cm->proto = NULL;
		rcu_read_lock();
		reply_cm->proto = rcu_dereference(inet6_protos[tuple->protocol]);
		rcu_read_unlock();
		reply_cm->top_interface_dev = dev_get_by_index(&init_net, msg->conn_rule.return_top_interface_num);

		if (unlikely(!reply_cm->top_interface_dev)) {
			DEBUG_WARN("%px: Unable to find top_interface_dev corresponding to %d\n", msg,
						msg->conn_rule.return_top_interface_num);
			this_cpu_inc(si->stats_pcpu->connection_create_failures64);
			spin_unlock_bh(&si->lock);
			kfree(reply_cm);
			kfree(original_cm);
			kfree(c);
			dev_put(src_dev);
			dev_put(dest_dev);
			return -EINVAL;
		}
	}

	/*
	 * Check if TSO is enabled on the bottom interface. This flag is used for PPPoE TCP flows.
	 * For PPPoE TCP flows, we can only use HW TSO since kernel does not support GSO for this path.
	 */
	if (unlikely(src_dev->features & NETIF_F_TSO) && (msg->rule_flags & SFE_RULE_CREATE_FLAG_USE_FLOW_BOTTOM_INTERFACE)) {
		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_TSO_ENABLE;
		DEBUG_TRACE("%px: TSO is enabled on the source interface: %s", msg, src_dev->name);
	}

	/*
	 * If l2_features are disabled and flow uses l2 features such as macvlan/bridge/pppoe/vlan,
	 * bottom interfaces are expected to be disabled in the flow rule and always top interfaces
	 * are used. In such cases, do not use HW csum offload. csum offload is used only when we
	 * are sending directly to the destination interface that supports it.
	 */
	if (likely(src_dev->features & NETIF_F_HW_CSUM) && sfe_dev_has_hw_csum(src_dev)) {
		if ((msg->conn_rule.flow_top_interface_num == msg->conn_rule.flow_interface_num) ||
			(msg->rule_flags & SFE_RULE_CREATE_FLAG_USE_FLOW_BOTTOM_INTERFACE)) {
			/*
			 * Dont enable CSUM offload
			 */
			 reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_CSUM_OFFLOAD;
		}
	}

	/*
	 * Disable HW CSUM Offload for connection with trustsec header
	 */
	if (msg->valid_flags & SFE_RULE_CREATE_TRUSTSEC_VALID) {
		struct sfe_trustsec_rule *trustsec_rule = &msg->trustsec_rule;
		sfe_ipv6_match_entry_set_trustsec(reply_cm,
						trustsec_rule->egress_sgt,
						trustsec_rule->ingress_sgt);
		reply_cm->flags &= ~SFE_IPV6_CONNECTION_MATCH_FLAG_CSUM_OFFLOAD;

		if (trustsec_rule->sgt_valid_flags & SFE_TRUSTSEC_EGRESS_SGT_VALID) {
			reply_cm->ingress_trustsec_valid = true;
		}

		if (trustsec_rule->sgt_valid_flags & SFE_TRUSTSEC_INGRESS_SGT_VALID) {
			original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_INSERT_EGRESS_TRUSTSEC_SGT;
		}
	}


	if (msg->rule_flags & SFE_RULE_CREATE_FLAG_RETURN_SRC_INTERFACE_CHECK) {
		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_SRC_INTERFACE_CHECK;
	}

	if (msg->rule_flags & SFE_RULE_CREATE_FLAG_RETURN_SRC_INTERFACE_CHECK_NO_FLUSH) {
		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_SRC_INTERFACE_CHECK_NO_FLUSH;
	}

	/*
	 * For the non-arp interface, we don't write L2 HDR.
	 * Excluding PPPoE from this, since we are now supporting PPPoE encap/decap.
	 */
	if (sfe_ipv6_xmit_eth_type_check(src_dev, reply_cm->flags)) {

		/*
		 * Check whether the rule has configured a specific source MAC address to use.
		 * This is needed when virtual L3 interfaces such as br-lan, macvlan, vlan are used during egress
		 */
		if (msg->rule_flags & SFE_RULE_CREATE_FLAG_BRIDGE_FLOW) {
			ether_addr_copy((u8 *)reply_cm->xmit_src_mac, (u8 *)msg->conn_rule.return_mac);
		} else {
			if ((msg->valid_flags & SFE_RULE_CREATE_SRC_MAC_VALID) &&
			    (msg->src_mac_rule.mac_valid_flags & SFE_SRC_MAC_FLOW_VALID)) {
				ether_addr_copy((u8 *)reply_cm->xmit_src_mac, (u8 *)msg->src_mac_rule.flow_src_mac);
			} else {
				ether_addr_copy((u8 *)reply_cm->xmit_src_mac, (u8 *)src_dev->dev_addr);
			}
		}

		ether_addr_copy((u8 *)reply_cm->xmit_dest_mac, (u8 *)msg->conn_rule.flow_mac);

		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_WRITE_L2_HDR;
		reply_cm->l2_hdr_size += ETH_HLEN;

		/*
		 * If our dev writes Ethernet headers then we can write a really fast
		 * version.
		 */
		if (src_dev->header_ops) {
			if (src_dev->header_ops->create == eth_header) {
				reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_WRITE_FAST_ETH_HDR;
			}
		}
	}

	if ((!sfe_ipv6_addr_equal((struct sfe_ipv6_addr *)tuple->return_ip, (struct sfe_ipv6_addr *)msg->conn_rule.return_ip_xlate)) ||
				(tuple->return_ident != msg->conn_rule.return_ident_xlate)) {
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_XLATE_DEST;
		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_XLATE_SRC;
	}

	if ((!sfe_ipv6_addr_equal((struct sfe_ipv6_addr *)tuple->flow_ip, (struct sfe_ipv6_addr *)msg->conn_rule.flow_ip_xlate)) ||
				(tuple->flow_ident != msg->conn_rule.flow_ident_xlate)) {
		original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_XLATE_SRC;
		reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_XLATE_DEST;
	}

	/*
	 * Initialize the protocol-specific information that we track.
	 */
	switch (tuple->protocol) {
	case IPPROTO_TCP:
		original_cm->protocol_state.tcp.win_scale = msg->tcp_rule.flow_window_scale;
		original_cm->protocol_state.tcp.max_win = msg->tcp_rule.flow_max_window ? msg->tcp_rule.flow_max_window : 1;
		original_cm->protocol_state.tcp.end = msg->tcp_rule.flow_end;
		original_cm->protocol_state.tcp.max_end = msg->tcp_rule.flow_max_end;
		reply_cm->protocol_state.tcp.win_scale = msg->tcp_rule.return_window_scale;
		reply_cm->protocol_state.tcp.max_win = msg->tcp_rule.return_max_window ? msg->tcp_rule.return_max_window : 1;
		reply_cm->protocol_state.tcp.end = msg->tcp_rule.return_end;
		reply_cm->protocol_state.tcp.max_end = msg->tcp_rule.return_max_end;
		if (msg->rule_flags & SFE_RULE_CREATE_FLAG_NO_SEQ_CHECK) {
			original_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_NO_SEQ_CHECK;
			reply_cm->flags |= SFE_IPV6_CONNECTION_MATCH_FLAG_NO_SEQ_CHECK;
		}
		break;

	case IPPROTO_RAW:
		/*
		 * Set src_port to 0 to avoid hash collision in connection match lookups.
		 */
		original_cm->match_src_port = 0;
		original_cm->xlate_src_port = 0;
		reply_cm->match_src_port = 0;
		reply_cm->xlate_src_port = 0;
		break;
	}

	/*
	 * Fill in the ipv6_connection object.
	 */
	c->protocol = tuple->protocol;
	c->src_ip[0] = *(struct sfe_ipv6_addr *)tuple->flow_ip;
	c->src_ip_xlate[0] = *(struct sfe_ipv6_addr *)msg->conn_rule.flow_ip_xlate;
	c->src_port = tuple->flow_ident;
	c->src_port_xlate = msg->conn_rule.flow_ident_xlate;
	c->original_dev = src_dev;
	c->original_match = original_cm;

	c->dest_ip[0] = *(struct sfe_ipv6_addr *)tuple->return_ip;
	c->dest_ip_xlate[0] = *(struct sfe_ipv6_addr *)msg->conn_rule.return_ip_xlate;
	c->dest_port = tuple->return_ident;
	c->dest_port_xlate = msg->conn_rule.return_ident_xlate;

	c->reply_dev = dest_dev;
	c->reply_match = reply_cm;
	c->debug_read_seq = 0;
	c->last_sync_jiffies = get_jiffies_64();
	c->removed = false;

	sfe_ipv6_connection_match_compute_translations(original_cm);
	sfe_ipv6_connection_match_compute_translations(reply_cm);
	sfe_ipv6_insert_connection(si, c);

	spin_unlock_bh(&si->lock);

	/*
	 * We have everything we need!
	 */
	DEBUG_INFO("%px: new connection - p: %d\n"
		   "  s: %s:%pxM(%pxM):%pI6(%pI6):%u(%u)\n"
		   "  d: %s:%pxM(%pxM):%pI6(%pI6):%u(%u)\n"
		   "msg flags: valid_flags=%x rule_flags=%x\n"
#ifdef SFE_BRIDGE_VLAN_FILTERING_ENABLE
		   " orig_cm:  vlan_filter_ingress_tag: %x flags: %x \n"
		   " reply_cm: vlan_filter_ingress_tag: %x flags: %x \n"
#endif
		   "qdisc_rule: valid=%x flow_qdisc_interface=%d return_qdisc_interface=%d",
		   c, tuple->protocol,
		   src_dev->name, msg->conn_rule.flow_mac, NULL,
		   (void *)tuple->flow_ip, (void *)msg->conn_rule.flow_ip_xlate, ntohs(tuple->flow_ident), ntohs(msg->conn_rule.flow_ident_xlate),
		   dest_dev->name, NULL, msg->conn_rule.return_mac,
		   (void *)tuple->return_ip, (void *)msg->conn_rule.return_ip_xlate, ntohs(tuple->return_ident), ntohs(msg->conn_rule.return_ident_xlate),
		   msg->valid_flags, msg->rule_flags,
#ifdef SFE_BRIDGE_VLAN_FILTERING_ENABLE
		   original_cm->vlan_filter_rule.ingress_vlan_tag, original_cm->vlan_filter_rule.ingress_flags,
		   reply_cm->vlan_filter_rule.ingress_vlan_tag, reply_cm->vlan_filter_rule.ingress_flags,
#endif
		   msg->qdisc_rule.valid_flags, msg->qdisc_rule.flow_qdisc_interface, msg->qdisc_rule.return_qdisc_interface);

	return 0;
}

#if defined(SFE_RFS_SUPPORTED)
/*
 * sfe_ipv6_fill_connection_dev()
 */
void sfe_ipv6_fill_connection_dev(struct sfe_ipv6_rule_destroy_msg *msg, struct net_device **original_dev, struct net_device **reply_dev)
{
	struct sfe_ipv6 *si = &__si6;
	struct sfe_ipv6_connection *c;
	struct sfe_ipv6_5tuple *tuple = &msg->tuple;

	spin_lock_bh(&si->lock);
	c = sfe_ipv6_find_connection(si, tuple->protocol, (struct sfe_ipv6_addr *)tuple->flow_ip, tuple->flow_ident,
				     (struct sfe_ipv6_addr *)tuple->return_ip, tuple->return_ident);
	if (!c) {
		*original_dev = NULL;
		*reply_dev = NULL;
		spin_unlock_bh(&si->lock);
		return;
	}

	*original_dev = c->original_dev;
	*reply_dev = c->reply_dev;
	spin_unlock_bh(&si->lock);
}
#endif

/*
 * sfe_ipv6_destroy_rule()
 *	Destroy a forwarding rule.
 */
void sfe_ipv6_destroy_rule(struct sfe_ipv6_rule_destroy_msg *msg)
{
	struct sfe_ipv6 *si = &__si6;
	struct sfe_ipv6_connection *c;
	bool ret;
	struct sfe_ipv6_5tuple *tuple = &msg->tuple;

	this_cpu_inc(si->stats_pcpu->connection_destroy_requests64);

	spin_lock_bh(&si->lock);

	/*
	 * Check to see if we have a flow that matches the rule we're trying
	 * to destroy.  If there isn't then we can't destroy it.
	 */
	c = sfe_ipv6_find_connection(si, tuple->protocol, (struct sfe_ipv6_addr *)tuple->flow_ip, tuple->flow_ident,
				     (struct sfe_ipv6_addr *)tuple->return_ip, tuple->return_ident);
	if (!c) {
		spin_unlock_bh(&si->lock);

		this_cpu_inc(si->stats_pcpu->connection_destroy_misses64);

		DEBUG_TRACE("connection does not exist - p: %d, s: %pI6:%u, d: %pI6:%u\n",
			    tuple->protocol, tuple->flow_ip, ntohs(tuple->flow_ident),
			    tuple->return_ip, ntohs(tuple->return_ident));
		return;
	}

	/*
	 * Remove our connection details from the hash tables.
	 */
	ret = sfe_ipv6_remove_connection(si, c);
	spin_unlock_bh(&si->lock);

	if (ret) {
		sfe_ipv6_flush_connection(si, c, SFE_SYNC_REASON_DESTROY);
	}

	DEBUG_INFO("connection destroyed - p: %d, s: %pI6:%u, d: %pI6:%u\n",
		   tuple->protocol, tuple->flow_ip, ntohs(tuple->flow_ident),
		   tuple->return_ip, ntohs(tuple->return_ident));
}

/*
 * sfe_ipv6_sync_invoke()
 *	Schedule many sync stats.
 */
bool sfe_ipv6_sync_invoke(uint16_t index)
{
	struct sfe_ipv6 *si = &__si6;
	return schedule_delayed_work_on(si->work_cpu, &(si->sync_dwork), 0);
}

/*
 * sfe_ipv6_register_sync_rule_callback()
 *	Register a callback for rule synchronization.
 */
void sfe_ipv6_register_sync_rule_callback(sfe_sync_rule_callback_t sync_rule_callback)
{
	struct sfe_ipv6 *si = &__si6;

	spin_lock_bh(&si->lock);
	rcu_assign_pointer(si->sync_rule_callback, sync_rule_callback);
	spin_unlock_bh(&si->lock);
}

/*
 * sfe_ipv6_register_sync_rule_callback()
 *	Register a callback for rule synchronization.
 */
void sfe_ipv6_register_many_sync_callback(sfe_ipv6_many_sync_callback_t cb)
{
	struct sfe_ipv6 *si = &__si6;

	spin_lock_bh(&si->lock);
	rcu_assign_pointer(si->many_sync_callback, cb);
	spin_unlock_bh(&si->lock);
}

/*
 * sfe_ipv6_get_debug_dev()
 */
static ssize_t sfe_ipv6_get_debug_dev(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct sfe_ipv6 *si = &__si6;
	ssize_t count;
	int num;

	spin_lock_bh(&si->lock);
	num = si->debug_dev;
	spin_unlock_bh(&si->lock);

	count = snprintf(buf, (ssize_t)PAGE_SIZE, "%d\n", num);
	return count;
}

/*
 * sfe_ipv6_destroy_all_rules_for_dev()
 *	Destroy all connections that match a particular device.
 *
 * If we pass dev as NULL then this destroys all connections.
 */
void sfe_ipv6_destroy_all_rules_for_dev(struct net_device *dev)
{
	struct sfe_ipv6 *si = &__si6;
	struct sfe_ipv6_connection *c;
	bool ret;

another_round:
	spin_lock_bh(&si->lock);

	for (c = si->all_connections_head; c; c = c->all_connections_next) {
		/*
		 * Does this connection relate to the device we are destroying?
		 */
		if (!dev
		    || (dev == c->original_dev)
		    || (dev == c->reply_dev)) {
			break;
		}
	}

	if (c) {
		ret = sfe_ipv6_remove_connection(si, c);
	}

	spin_unlock_bh(&si->lock);

	if (c) {
		if (ret) {
			sfe_ipv6_flush_connection(si, c, SFE_SYNC_REASON_DESTROY);
		}
		goto another_round;
	}
}

/*
 * sfe_ipv6_periodic_sync()
 */
static void sfe_ipv6_periodic_sync(struct work_struct *work)
{
	struct sfe_ipv6 *si = container_of((struct delayed_work *)work, struct sfe_ipv6, sync_dwork);
	u64 now_jiffies;
	int quota;
	sfe_ipv6_many_sync_callback_t sync_rule_callback;
	struct sfe_ipv6_connection *c;
	struct sfe_ipv6_conn_sync *conn_sync;

	now_jiffies = get_jiffies_64();

	rcu_read_lock();
	sync_rule_callback = rcu_dereference(si->many_sync_callback);
	rcu_read_unlock();
	if (!sync_rule_callback) {
		return;
	}

	spin_lock_bh(&si->lock);

	/*
	 * If we have reached the end of the connection list, walk from
	 * the connection head.
	 */
	c = si->wc_next;
	if (unlikely(!c)) {
		c = si->all_connections_head;
	}

	/*
	 * Get the max number of connections to be put in this sync msg.
	 */
	quota = sfe_ipv6_sync_max_number;
	conn_sync = sfe_ipv6_sync_many_msg->msg.conn_stats_many.conn_sync;

	/*
	 * Walk the "all connection" list and sync the connection state.
	 */
	while (likely(c && quota)) {
		struct sfe_ipv6_connection_match *cm;
		struct sfe_ipv6_connection_match *counter_cm;
		struct sfe_connection_sync sis;

		cm = c->original_match;
		counter_cm = c->reply_match;

		/*
		 * Didn't receive packets in the origial direction or reply
		 * direction, move to the next connection.
		 */
		if (!atomic_read(&cm->rx_packet_count) && ((!counter_cm) || (!atomic_read(&counter_cm->rx_packet_count)))) {
			c = c->all_connections_next;
			continue;
		}

		/*
		 * Sync the connection state.
		 */
		sfe_ipv6_gen_sync_connection(si, c, &sis, SFE_SYNC_REASON_STATS, now_jiffies);
		sfe_ipv6_stats_convert(conn_sync, &sis);

		quota--;
		conn_sync++;
		c = c->all_connections_next;
	}

	/*
	 * At the end of loop, put wc_next to the connection we left
	 */
	si->wc_next = c;
	spin_unlock_bh(&si->lock);

	if (c == NULL) {
		DEBUG_INFO("Synced all connections\n");
		sfe_ipv6_sync_many_msg->msg.conn_stats_many.next = 0;
	} else {
		DEBUG_INFO("Some connections left\n");
		sfe_ipv6_sync_many_msg->msg.conn_stats_many.next = sfe_ipv6_sync_max_number - quota;
	}
	DEBUG_INFO("Synced [%d] connections\n", (sfe_ipv6_sync_max_number - quota));

	sfe_ipv6_sync_many_msg->msg.conn_stats_many.count = sfe_ipv6_sync_max_number - quota;
	sfe_ipv6_sync_many_msg->cm.response = SFE_CMN_RESPONSE_ACK;

	sync_rule_callback(sfe_ipv6_sync_many_msg);
}

/*
 * sfe_ipv6_debug_dev_read_start()
 *	Generate part of the XML output.
 */
static int sfe_ipv6_debug_dev_read_start(struct sfe_ipv6 *si, char *buffer, char *msg, size_t length,
					  int *total_read, struct sfe_ipv6_debug_xml_write_state *ws)
{
	int bytes_read;

	si->debug_read_seq++;

	bytes_read = snprintf(msg, length, "<sfe_ipv6>\n");

	if ((bytes_read + *total_read) >= length) {
		return -ENOMEM;
	}

	if (copy_to_user(buffer + *total_read, msg, bytes_read)) {
		return -EFAULT;
	}

	*total_read += bytes_read;

	ws->state++;
	return 0;
}

/*
 * sfe_ipv6_debug_dev_read_connections_start()
 *	Generate part of the XML output.
 */
static int sfe_ipv6_debug_dev_read_connections_start(struct sfe_ipv6 *si, char *buffer, char *msg, size_t length,
						      int *total_read, struct sfe_ipv6_debug_xml_write_state *ws)
{
	int bytes_read;

	bytes_read = snprintf(msg, length, "\t<connections>\n");

	if ((bytes_read + *total_read) >= length) {
		return -ENOMEM;
	}

	if (copy_to_user(buffer + *total_read, msg, bytes_read)) {
		return -EFAULT;
	}

	*total_read += bytes_read;

	ws->state++;
	return 0;
}

/*
 * sfe_ipv6_debug_dev_read_connections_connection()
 *	Generate part of the XML output.
 */
static int sfe_ipv6_debug_dev_read_connections_connection(struct sfe_ipv6 *si, char *buffer, char *msg, size_t length,
							   int *total_read, struct sfe_ipv6_debug_xml_write_state *ws)
{
	struct sfe_ipv6_connection *c;
	struct sfe_ipv6_connection_match *original_cm;
	struct sfe_ipv6_connection_match *reply_cm;
	int bytes_read = 0;
	int protocol;
	struct net_device *src_dev;
	struct sfe_ipv6_addr src_ip;
	struct sfe_ipv6_addr src_ip_xlate;
	__be16 src_port;
	__be16 src_port_xlate;
	u64 src_rx_packets;
	u64 src_rx_bytes;
	struct net_device *dest_dev;
	struct sfe_ipv6_addr dest_ip;
	struct sfe_ipv6_addr dest_ip_xlate;
	__be16 dest_port;
	__be16 dest_port_xlate;
	u64 dest_rx_packets;
	u64 dest_rx_bytes;
	u64 last_sync_jiffies;
	u32 src_mark, dest_mark,  src_priority, dest_priority, src_dscp, dest_dscp;
	bool original_cm_sawf_valid, reply_cm_sawf_valid;
	u32 flow_service_class, return_service_class;
	u32 flow_msduq, return_msduq;
	u32 packet, byte, original_cm_flags;
	u16 pppoe_session_id;
	u8 pppoe_remote_mac[ETH_ALEN];
	u32 original_fast_xmit, reply_fast_xmit;
#ifdef CONFIG_NF_FLOW_COOKIE
	int src_flow_cookie, dst_flow_cookie;
#endif

	spin_lock_bh(&si->lock);

	for (c = si->all_connections_head; c; c = c->all_connections_next) {
		if (c->debug_read_seq < si->debug_read_seq) {
			break;
		}
	}

	/*
	 * If there were no connections then move to the next state.
	 */
	if (!c) {
		spin_unlock_bh(&si->lock);
		ws->state++;
		return true;
	}

	original_cm = c->original_match;
	reply_cm = c->reply_match;

	protocol = c->protocol;
	src_dev = c->original_dev;
	src_ip = c->src_ip[0];
	src_ip_xlate = c->src_ip_xlate[0];
	src_port = c->src_port;
	src_port_xlate = c->src_port_xlate;
	src_priority = original_cm->priority;
	src_dscp = original_cm->dscp >> SFE_IPV6_DSCP_SHIFT;

	sfe_ipv6_connection_match_update_summary_stats(original_cm, &packet, &byte);

	src_rx_packets = original_cm->rx_packet_count64;
	src_rx_bytes = original_cm->rx_byte_count64;
	src_mark = original_cm->mark;
	original_fast_xmit = original_cm->flags & SFE_IPV6_CONNECTION_MATCH_FLAG_FAST_XMIT;
	dest_dev = c->reply_dev;
	dest_ip = c->dest_ip[0];
	dest_ip_xlate = c->dest_ip_xlate[0];
	dest_port = c->dest_port;
	dest_port_xlate = c->dest_port_xlate;
	if (reply_cm) {
		sfe_ipv6_connection_match_update_summary_stats(reply_cm, &packet, &byte);
		dest_priority = reply_cm->priority;
		dest_dscp = reply_cm->dscp >> SFE_IPV6_DSCP_SHIFT;
		dest_rx_packets = reply_cm->rx_packet_count64;
		dest_rx_bytes = reply_cm->rx_byte_count64;
		dest_mark = reply_cm->mark;
		reply_fast_xmit = reply_cm->flags & SFE_IPV6_CONNECTION_MATCH_FLAG_FAST_XMIT;
		original_cm_sawf_valid = original_cm->sawf_valid;
		reply_cm_sawf_valid = reply_cm->sawf_valid;
		flow_service_class = SFE_GET_SAWF_SERVICE_CLASS(original_cm->mark);
		flow_msduq = SFE_GET_SAWF_MSDUQ(original_cm->mark);
		return_service_class = SFE_GET_SAWF_SERVICE_CLASS(reply_cm->mark);
		return_msduq = SFE_GET_SAWF_MSDUQ(reply_cm->mark);
#ifdef CONFIG_NF_FLOW_COOKIE
		dst_flow_cookie = reply_cm->flow_cookie;
#endif

	} else {
		dest_priority = 0;
		dest_dscp = 0;
		dest_rx_packets = 0;
		dest_rx_bytes = 0;
		dest_mark = 0;
		reply_fast_xmit = 0;
		original_cm_sawf_valid = 0;
		reply_cm_sawf_valid = 0;
		flow_service_class = 0;
		flow_msduq = 0;
		return_service_class = 0;
		return_msduq = 0;
#ifdef CONFIG_NF_FLOW_COOKIE
		dst_flow_cookie = 0;
#endif
	}
	last_sync_jiffies = get_jiffies_64() - c->last_sync_jiffies;
	original_cm_flags = original_cm->flags;
	pppoe_session_id = original_cm->pppoe_session_id;
	ether_addr_copy(pppoe_remote_mac, original_cm->pppoe_remote_mac);

#ifdef CONFIG_NF_FLOW_COOKIE
	src_flow_cookie = original_cm->flow_cookie;
#endif
	spin_unlock_bh(&si->lock);

	bytes_read = snprintf(msg, length - bytes_read, "\t\t<connection "
				"protocol=\"%u\" "
				"src_dev=\"%s\" "
				"src_ip=\"%pI6\" src_ip_xlate=\"%pI6\" "
				"src_port=\"%u\" src_port_xlate=\"%u\" "
				"src_priority=\"%u\" src_dscp=\"%u\" "
				"src_rx_pkts=\"%llu\" src_rx_bytes=\"%llu\" "
				"src_mark=\"%08x\" "
				"src_fast_xmit=\"%s\" "
				"dest_dev=\"%s\" "
				"dest_ip=\"%pI6\" dest_ip_xlate=\"%pI6\" "
				"dest_port=\"%u\" dest_port_xlate=\"%u\" "
				"dest_priority=\"%u\" dest_dscp=\"%u\" "
				"dest_rx_pkts=\"%llu\" dest_rx_bytes=\"%llu\" "
				"dest_mark=\"%08x\" "
				"reply_fast_xmit=\"%s\" "
#ifdef CONFIG_NF_FLOW_COOKIE
				"src_flow_cookie=\"%d\" dst_flow_cookie=\"%d\" "
#endif
				"last_sync=\"%llu\" ",
				protocol,
				src_dev->name,
				&src_ip, &src_ip_xlate,
				ntohs(src_port), ntohs(src_port_xlate),
				src_priority, src_dscp,
				src_rx_packets, src_rx_bytes,
				src_mark,
				original_fast_xmit ? "Yes" : "No",
				dest_dev->name,
				&dest_ip, &dest_ip_xlate,
				ntohs(dest_port), ntohs(dest_port_xlate),
				dest_priority, dest_dscp,
				dest_rx_packets, dest_rx_bytes,
				dest_mark,
				reply_fast_xmit ? "Yes" : "No",
#ifdef CONFIG_NF_FLOW_COOKIE
				src_flow_cookie, dst_flow_cookie,
#endif
				last_sync_jiffies);

	if ((length - bytes_read) <= 0) {
		return -ENOMEM;
	}

	if (original_cm_flags &= (SFE_IPV6_CONNECTION_MATCH_FLAG_PPPOE_DECAP | SFE_IPV6_CONNECTION_MATCH_FLAG_PPPOE_ENCAP)) {
		bytes_read += snprintf(msg + bytes_read, length - bytes_read, "pppoe_session_id=\"%u\" pppoe_server_MAC=\"%pM\" ",
			pppoe_session_id, pppoe_remote_mac);
	}

	if ((length - bytes_read) <= 0) {
		return -ENOMEM;
	}

	if (original_cm_sawf_valid) {
		bytes_read += snprintf(msg + bytes_read, length - bytes_read, "flow_service_class=\"%d\" flow_msduq= \"0x%x\" ",
			flow_service_class, flow_msduq);
	}

	if ((length - bytes_read) <= 0) {
		return -ENOMEM;
	}

	if (reply_cm_sawf_valid) {
		bytes_read += snprintf(msg + bytes_read, length - bytes_read, "return_service_class=\"%d\" return_msduq= \"0x%x\" ",
			return_service_class, return_msduq);
	}

	if (original_cm->flags & SFE_IPV6_CONNECTION_MATCH_FLAG_MULTICAST) {
		struct sfe_ipv6_mc_dest *mc_xmit_dev;
		u32 fast_xmit;
		u32 vlan_hdr_cnt;
		list_for_each_entry_rcu(mc_xmit_dev, &original_cm->mc_list, list) {
			 fast_xmit = mc_xmit_dev->flags & SFE_IPV6_CONNECTION_MATCH_FLAG_FAST_XMIT;
			 vlan_hdr_cnt = mc_xmit_dev->egress_vlan_hdr_cnt;

			 if ((length - bytes_read) <= 0) {
				 return -ENOMEM;
			 }

			 bytes_read += snprintf(msg + bytes_read,
					  length - bytes_read, "mc_dev=\"%s\" mc_dmac=\"%pM\" mc_smac=\"%pM\" fast_xmit=\"%s\" vlan_hdr_cnt=\"%d\"",
					 mc_xmit_dev->xmit_dev->name,
					 mc_xmit_dev->xmit_dest_mac,
					 mc_xmit_dev->xmit_src_mac,
					 fast_xmit?"Yes":"No",
					 vlan_hdr_cnt);
		 }
	}

	if ((length - bytes_read) <= 0) {
		return -ENOMEM;
	}

	bytes_read += snprintf(msg + bytes_read, length - bytes_read, "/>\n");

	if ((bytes_read + *total_read) >= length) {
		return -ENOMEM;
	}

	if (copy_to_user(buffer + *total_read, msg, bytes_read)) {
		return -EFAULT;
	}

	*total_read += bytes_read;

	c->debug_read_seq = si->debug_read_seq;
	return 0;
}

/*
 * sfe_ipv6_debug_dev_read_connections_end()
 *	Generate part of the XML output.
 */
static int sfe_ipv6_debug_dev_read_connections_end(struct sfe_ipv6 *si, char *buffer, char *msg, size_t length,
						    int *total_read, struct sfe_ipv6_debug_xml_write_state *ws)
{
	int bytes_read;

	bytes_read = snprintf(msg, length, "\t</connections>\n");

	if ((bytes_read + *total_read) >= length) {
		return -ENOMEM;
	}

	if (copy_to_user(buffer + *total_read, msg, bytes_read)) {
		return -EFAULT;
	}

	*total_read += bytes_read;

	ws->state++;
	return 0;
}

/*
 * sfe_ipv6_debug_dev_read_exceptions_start()
 *	Generate part of the XML output.
 */
static int sfe_ipv6_debug_dev_read_exceptions_start(struct sfe_ipv6 *si, char *buffer, char *msg, size_t length,
						     int *total_read, struct sfe_ipv6_debug_xml_write_state *ws)
{
	int bytes_read;

	bytes_read = snprintf(msg, length, "\t<exceptions>\n");

	if ((bytes_read + *total_read) >= length) {
		return -ENOMEM;
	}

	if (copy_to_user(buffer + *total_read, msg, bytes_read)) {
		return -EFAULT;
	}

	*total_read += bytes_read;

	ws->state++;
	return 0;
}

/*
 * sfe_ipv6_debug_dev_read_exceptions_exception()
 *	Generate part of the XML output.
 */
static int sfe_ipv6_debug_dev_read_exceptions_exception(struct sfe_ipv6 *si, char *buffer, char *msg, size_t length,
							 int *total_read, struct sfe_ipv6_debug_xml_write_state *ws)
{
	int i;
	u64 val = 0;

	for_each_possible_cpu(i) {
		const struct sfe_ipv6_stats *s = per_cpu_ptr(si->stats_pcpu, i);
			val += s->exception_events64[ws->iter_exception];
	}

	if (val) {
		int bytes_read;

		bytes_read = snprintf(msg, length,
				      "\t\t<exception name=\"%s\" count=\"%llu\" />\n",
				      sfe_ipv6_exception_events_string[ws->iter_exception],
				      val);

		if ((bytes_read + *total_read) >= length) {
			return -ENOMEM;
		}

		if (copy_to_user(buffer + *total_read, msg, bytes_read)) {
			return -EFAULT;
		}

		*total_read += bytes_read;
	}

	ws->iter_exception++;
	if (ws->iter_exception >= SFE_IPV6_EXCEPTION_EVENT_LAST) {
		ws->iter_exception = 0;
		ws->state++;
	}

	return 0;
}

/*
 * sfe_ipv6_debug_dev_read_exceptions_end()
 *	Generate part of the XML output.
 */
static int sfe_ipv6_debug_dev_read_exceptions_end(struct sfe_ipv6 *si, char *buffer, char *msg, size_t length,
						   int *total_read, struct sfe_ipv6_debug_xml_write_state *ws)
{
	int bytes_read;

	bytes_read = snprintf(msg, length, "\t</exceptions>\n");

	if ((bytes_read + *total_read) >= length) {
		return -ENOMEM;
	}

	if (copy_to_user(buffer + *total_read, msg, bytes_read)) {
		return -EFAULT;
	}

	*total_read += bytes_read;

	ws->state++;
	return 0;
}

/*
 * sfe_ipv6_debug_dev_read_stats()
 *	Generate part of the XML output.
 */
static int sfe_ipv6_debug_dev_read_stats(struct sfe_ipv6 *si, char *buffer, char *msg, size_t length,				  int *total_read, struct sfe_ipv6_debug_xml_write_state *ws)
{
	int bytes_read;
	struct sfe_ipv6_stats stats;
	unsigned int num_conn;

	sfe_ipv6_update_summary_stats(si, &stats);

	spin_lock_bh(&si->lock);
	num_conn = si->num_connections;
	spin_unlock_bh(&si->lock);

	bytes_read = snprintf(msg, length, "\t<stats "
			      "num_connections=\"%u\" "
			      "pkts_dropped=\"%llu\" "
			      "pkts_fast_xmited=\"%llu\" "
			      "pkts_fast_qdisc_xmited=\"%llu\" "
			      "pkts_forwarded=\"%llu\" pkts_not_forwarded=\"%llu\" "
			      "create_requests=\"%llu\" create_collisions=\"%llu\" "
			      "create_failures=\"%llu\" "
			      "destroy_requests=\"%llu\" destroy_misses=\"%llu\" "
			      "flushes=\"%llu\" "
			      "hash_hits=\"%llu\" hash_reorders=\"%llu\" "
			      "pppoe_encap_pkts_fwded=\"%llu\" "
			      "pppoe_decap_pkts_fwded=\"%llu\" "
			      "pppoe_bridge_pkts_fwded=\"%llu\" "
			      "pppoe_bridge_pkts_3tuple_fwded=\"%llu\" "
			      "connection_create_requests_overflow64=\"%llu\" />\n",
				num_conn,
				stats.packets_dropped64,
				stats.packets_fast_xmited64,
				stats.packets_fast_qdisc_xmited64,
				stats.packets_forwarded64,
				stats.packets_not_forwarded64,
				stats.connection_create_requests64,
				stats.connection_create_collisions64,
				stats.connection_create_failures64,
				stats.connection_destroy_requests64,
				stats.connection_destroy_misses64,
				stats.connection_flushes64,
				stats.connection_match_hash_hits64,
				stats.connection_match_hash_reorders64,
				stats.pppoe_encap_packets_forwarded64,
				stats.pppoe_decap_packets_forwarded64,
				stats.pppoe_bridge_packets_forwarded64,
				stats.pppoe_bridge_packets_3tuple_forwarded64,
				stats.connection_create_requests_overflow64);

	if ((bytes_read + *total_read) >= length) {
		return -ENOMEM;
	}

	if (copy_to_user(buffer + *total_read, msg, bytes_read)) {
		return -EFAULT;
	}

	*total_read += bytes_read;

	ws->state++;
	return 0;
}

/*
 * sfe_ipv6_debug_dev_read_end()
 *	Generate part of the XML output.
 */
static int sfe_ipv6_debug_dev_read_end(struct sfe_ipv6 *si, char *buffer, char *msg, size_t length,
					int *total_read, struct sfe_ipv6_debug_xml_write_state *ws)
{
	int bytes_read;

	bytes_read = snprintf(msg, length, "</sfe_ipv6>\n");

	if ((bytes_read + *total_read) >= length) {
		return -ENOMEM;
	}

	if (copy_to_user(buffer + *total_read, msg, bytes_read)) {
		return -EFAULT;
	}

	*total_read += bytes_read;

	ws->state++;
	return 0;
}

/*
 * Array of write functions that write various XML elements that correspond to
 * our XML output state machine.
 */
static sfe_ipv6_debug_xml_write_method_t sfe_ipv6_debug_xml_write_methods[SFE_IPV6_DEBUG_XML_STATE_DONE] = {
	sfe_ipv6_debug_dev_read_start,
	sfe_ipv6_debug_dev_read_connections_start,
	sfe_ipv6_debug_dev_read_connections_connection,
	sfe_ipv6_debug_dev_read_connections_end,
	sfe_ipv6_debug_dev_read_exceptions_start,
	sfe_ipv6_debug_dev_read_exceptions_exception,
	sfe_ipv6_debug_dev_read_exceptions_end,
	sfe_ipv6_debug_dev_read_stats,
	sfe_ipv6_debug_dev_read_end,
};

/*
 * sfe_ipv6_debug_dev_read()
 *	Send info to userspace upon read request from user
 */
static ssize_t sfe_ipv6_debug_dev_read(struct file *filp, char *buffer, size_t length, loff_t *offset)
{
	struct sfe_ipv6_debug_xml_write_state *ws;
	struct sfe_ipv6 *si = &__si6;
	int total_read = 0;
	int null_fill;
	int status = 0;
	char *msg;

	msg = kmalloc(length, GFP_KERNEL);
	if (!msg) {
		return -ENOMEM;
	}

	ws = (struct sfe_ipv6_debug_xml_write_state *)filp->private_data;
	while ((ws->state != SFE_IPV6_DEBUG_XML_STATE_DONE)) {
		status = sfe_ipv6_debug_xml_write_methods[ws->state](si, buffer, msg, length, &total_read, ws);
		if (!status) {
			continue;
		}

		break;
	}

	if (status == -ENOMEM) {
		/*
		 * To request more buffer when we are short of it. Indicate that we have consumed whole buffer.
		 * Since there is some unusable space left, fill them with null string to avoid
		 * displaying junk characters.
		 */
		null_fill = length - total_read;
		memset(msg, '\0', null_fill);

		if (copy_to_user(buffer + total_read, msg, null_fill)) {
			kfree(msg);
			return -EFAULT;
		}

		total_read += null_fill;
	}

	if (status == -EFAULT) {
		total_read = -EFAULT;
	}

	kfree(msg);
	return total_read;
}

/*
 * sfe_ipv6_debug_dev_open()
 */
static int sfe_ipv6_debug_dev_open(struct inode *inode, struct file *file)
{
	struct sfe_ipv6_debug_xml_write_state *ws;

	ws = (struct sfe_ipv6_debug_xml_write_state *)file->private_data;
	if (ws) {
		return 0;
	}

	ws = kzalloc(sizeof(struct sfe_ipv6_debug_xml_write_state), GFP_KERNEL);
	if (!ws) {
		return -ENOMEM;
	}

	ws->state = SFE_IPV6_DEBUG_XML_STATE_START;
	file->private_data = ws;

	return 0;
}

/*
 * sfe_ipv6_debug_dev_release()
 */
static int sfe_ipv6_debug_dev_release(struct inode *inode, struct file *file)
{
	struct sfe_ipv6_debug_xml_write_state *ws;

	ws = (struct sfe_ipv6_debug_xml_write_state *)file->private_data;
	if (ws) {
		/*
		 * We've finished with our output so free the write state.
		 */
		kfree(ws);
		file->private_data = NULL;
	}

	return 0;
}

/*
 * File operations used in the debug char device
 */
static struct file_operations sfe_ipv6_debug_dev_fops = {
	.read = sfe_ipv6_debug_dev_read,
	.open = sfe_ipv6_debug_dev_open,
	.release = sfe_ipv6_debug_dev_release
};

#ifdef CONFIG_NF_FLOW_COOKIE
/*
 * sfe_ipv6_register_flow_cookie_cb
 *	register a function in SFE to let SFE use this function to configure flow cookie for a flow
 *
 * Hardware driver which support flow cookie should register a callback function in SFE. Then SFE
 * can use this function to configure flow cookie for a flow.
 * return: 0, success; !=0, fail
 */
int sfe_ipv6_register_flow_cookie_cb(sfe_ipv6_flow_cookie_set_func_t cb)
{
	struct sfe_ipv6 *si = &__si6;

	BUG_ON(!cb);

	if (si->flow_cookie_set_func) {
		return -1;
	}

	rcu_assign_pointer(si->flow_cookie_set_func, cb);
	return 0;
}

/*
 * sfe_ipv6_unregister_flow_cookie_cb
 *	unregister function which is used to configure flow cookie for a flow
 *
 * return: 0, success; !=0, fail
 */
int sfe_ipv6_unregister_flow_cookie_cb(sfe_ipv6_flow_cookie_set_func_t cb)
{
	struct sfe_ipv6 *si = &__si6;

	RCU_INIT_POINTER(si->flow_cookie_set_func, NULL);
	return 0;
}

/*
 * sfe_ipv6_get_flow_cookie()
 */
static ssize_t sfe_ipv6_get_flow_cookie(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct sfe_ipv6 *si = &__si6;
	return snprintf(buf, (ssize_t)PAGE_SIZE, "%d\n", si->flow_cookie_enable);
}

/*
 * sfe_ipv6_set_flow_cookie()
 */
static ssize_t sfe_ipv6_set_flow_cookie(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct sfe_ipv6 *si = &__si6;
	si->flow_cookie_enable = strict_strtol(buf, NULL, 0);

	return size;
}

/*
 * sysfs attributes.
 */
static const struct device_attribute sfe_ipv6_flow_cookie_attr =
	__ATTR(flow_cookie_enable, S_IWUSR | S_IRUGO, sfe_ipv6_get_flow_cookie, sfe_ipv6_set_flow_cookie);
#endif /*CONFIG_NF_FLOW_COOKIE*/

/*
 * sfe_ipv6_get_cpu()
 */
static ssize_t sfe_ipv6_get_cpu(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct sfe_ipv6 *si = &__si6;
	return snprintf(buf, (ssize_t)PAGE_SIZE, "%d\n", si->work_cpu);
}

/*
 * sfe_ipv6_set_cpu()
 */
static ssize_t sfe_ipv6_set_cpu(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct sfe_ipv6 *si = &__si6;
	int work_cpu;

	work_cpu = simple_strtol(buf, NULL, 0);
	if ((work_cpu >= 0) && (work_cpu <= NR_CPUS)) {
		si->work_cpu = work_cpu;
	} else {
		dev_err(dev, "%s is not in valid range[0,%d]", buf, NR_CPUS);
	}

	return size;
}
/*
 * sysfs attributes.
 */
static const struct device_attribute sfe_ipv6_cpu_attr =
	__ATTR(stat_work_cpu, S_IWUSR | S_IRUGO, sfe_ipv6_get_cpu, sfe_ipv6_set_cpu);

 /*
 * sfe_ipv6_hash_init()
 *	Initialize conn match hash lists
 */
static void sfe_ipv6_conn_match_hash_init(struct sfe_ipv6 *si, int len)
{
	struct hlist_head *hash_list = si->hlist_conn_match_hash_head;
	int i;

	for (i = 0; i < len; i++) {
		INIT_HLIST_HEAD(&hash_list[i]);
	}
}

#ifdef SFE_PROCESS_LOCAL_OUT
/*
 * sfe_ipv6_local_out()
 *	Called for packets from ip_local_out() - post encapsulation & other packets
 */
static unsigned int sfe_ipv6_local_out(void *priv,
				struct sk_buff *skb,
				const struct nf_hook_state *nhs)
{
	struct sfe_l2_info l2_info = {0};

	DEBUG_TRACE("sfe: sfe_ipv6_local_out hook called.\n");

	if (likely(skb->skb_iif)) {
		return sfe_ipv6_recv(skb->dev, skb, &l2_info, true) ? NF_STOLEN : NF_ACCEPT;
	}

	return NF_ACCEPT;
}

/*
 * struct nf_hook_ops sfe_ipv6_ops_local_out[]
 *	Hooks into netfilter local out packet monitoring points.
 */
static struct nf_hook_ops sfe_ipv6_ops_local_out[] __read_mostly = {

	/*
	 * Local out routing hook is used to monitor packets.
	 */
	{
		.hook           = sfe_ipv6_local_out,
		.pf             = PF_INET6,
		.hooknum        = NF_INET_LOCAL_OUT,
		.priority       = NF_IP6_PRI_FIRST,
	},
};
#endif

/*
 * sfe_ipv6_cancel_delayed_work_sync()
 */
bool sfe_ipv6_cancel_delayed_work_sync(void)
{
	struct sfe_ipv6 *si = &__si6;
	return cancel_delayed_work_sync(&si->sync_dwork);
}

/*
 * sfe_ipv6_init()
 */
int sfe_ipv6_init(void)
{
	struct sfe_ipv6 *si = &__si6;
	int result = -1;

	DEBUG_INFO("SFE IPv6 init\n");

	sfe_ipv6_conn_match_hash_init(si, ARRAY_SIZE(si->hlist_conn_match_hash_head));

	si->stats_pcpu = alloc_percpu_gfp(struct sfe_ipv6_stats, GFP_KERNEL | __GFP_ZERO);
	if (!si->stats_pcpu) {
		DEBUG_ERROR("failed to allocate stats memory for sfe_ipv6\n");
		goto exit0;
	}

	/*
	 * Allocate per cpu per service class memory.
	 */
	si->stats_pcpu_psc = alloc_percpu_gfp(struct sfe_ipv6_service_class_stats_db,
						GFP_KERNEL | __GFP_ZERO);
	if (!si->stats_pcpu_psc) {
		DEBUG_ERROR("failed to allocate per cpu per service clas stats memory\n");
		goto exit1;
	}

	/*
	 * Create sys/sfe_ipv6
	 */
	si->sys_ipv6 = kobject_create_and_add("sfe_ipv6", NULL);
	if (!si->sys_ipv6) {
		DEBUG_ERROR("failed to register sfe_ipv6\n");
		goto exit2;
	}

	/*
	 * Create files, one for each parameter supported by this module.
	 */
	result = sysfs_create_file(si->sys_ipv6, &sfe_ipv6_debug_dev_attr.attr);
	if (result) {
		DEBUG_ERROR("failed to register debug dev file: %d\n", result);
		goto exit3;
	}

	result = sysfs_create_file(si->sys_ipv6, &sfe_ipv6_cpu_attr.attr);
	if (result) {
		DEBUG_ERROR("failed to register debug dev file: %d\n", result);
		goto exit4;
	}

#ifdef CONFIG_NF_FLOW_COOKIE
	result = sysfs_create_file(si->sys_ipv6, &sfe_ipv6_flow_cookie_attr.attr);
	if (result) {
		DEBUG_ERROR("failed to register flow cookie enable file: %d\n", result);
		goto exit5;
	}
#endif /* CONFIG_NF_FLOW_COOKIE */

#ifdef SFE_PROCESS_LOCAL_OUT
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 13, 0))
	result = nf_register_hooks(sfe_ipv6_ops_local_out, ARRAY_SIZE(sfe_ipv6_ops_local_out));
#else
	result = nf_register_net_hooks(&init_net, sfe_ipv6_ops_local_out, ARRAY_SIZE(sfe_ipv6_ops_local_out));
#endif
	if (result < 0) {
		DEBUG_ERROR("can't register nf local out hook: %d\n", result);
		goto exit6;
	}
	DEBUG_INFO("Register nf local out hook success: %d\n", result);
#endif

	/*
	 * Register our debug char device.
	 */
	result = register_chrdev(0, "sfe_ipv6", &sfe_ipv6_debug_dev_fops);
	if (result < 0) {
		DEBUG_ERROR("Failed to register chrdev: %d\n", result);
		goto exit7;
	}

	si->debug_dev = result;
	si->work_cpu = WORK_CPU_UNBOUND;

	/*
	 * Create work to handle periodic statistics.
	 */
	INIT_DELAYED_WORK(&(si->sync_dwork), sfe_ipv6_periodic_sync);

	sfe_ipv6_sync_many_msg = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if(!sfe_ipv6_sync_many_msg) {
		goto exit8;
	}

	sfe_ipv6_msg_init(sfe_ipv6_sync_many_msg, SFE_SPECIAL_INTERFACE_IPV6,
			SFE_TX_CONN_STATS_SYNC_MANY_MSG,
			sizeof(struct sfe_ipv6_conn_sync_many_msg),
			NULL,
			NULL);
	sfe_ipv6_sync_max_number = (PAGE_SIZE - sizeof(struct sfe_ipv6_msg)) / sizeof(struct sfe_ipv6_conn_sync);

	spin_lock_init(&si->lock);
	return 0;

exit8:
	unregister_chrdev(si->debug_dev, "sfe_ipv6");

exit7:
#ifdef SFE_PROCESS_LOCAL_OUT
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 13, 0))
	DEBUG_TRACE("sfe: Unregister local out hook\n");
	nf_unregister_hooks(sfe_ipv6_ops_local_out, ARRAY_SIZE(sfe_ipv6_ops_local_out));
#else
	DEBUG_TRACE("sfe: Unregister local out hook\n");
	nf_unregister_net_hooks(&init_net, sfe_ipv6_ops_local_out, ARRAY_SIZE(sfe_ipv6_ops_local_out));
#endif
exit6:
#endif
#ifdef CONFIG_NF_FLOW_COOKIE
	sysfs_remove_file(si->sys_ipv6, &sfe_ipv6_flow_cookie_attr.attr);

exit5:
#endif /* CONFIG_NF_FLOW_COOKIE */
	sysfs_remove_file(si->sys_ipv6, &sfe_ipv6_cpu_attr.attr);

exit4:
	sysfs_remove_file(si->sys_ipv6, &sfe_ipv6_debug_dev_attr.attr);

exit3:
	kobject_put(si->sys_ipv6);

exit2:
	free_percpu(si->stats_pcpu_psc);

exit1:
	free_percpu(si->stats_pcpu);

exit0:
	return result;
}

/*
 * sfe_ipv6_exit()
 */
void sfe_ipv6_exit(void)
{
	struct sfe_ipv6 *si = &__si6;

	DEBUG_INFO("SFE IPv6 exit\n");

	/*
	 * Destroy all connections.
	 */
	sfe_ipv6_destroy_all_rules_for_dev(NULL);

	cancel_delayed_work_sync(&si->sync_dwork);

	unregister_chrdev(si->debug_dev, "sfe_ipv6");

	free_percpu(si->stats_pcpu);
	free_percpu(si->stats_pcpu_psc);

#ifdef SFE_PROCESS_LOCAL_OUT
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 13, 0))
	DEBUG_TRACE("sfe: Unregister local out hook\n");
	nf_unregister_hooks(sfe_ipv6_ops_local_out, ARRAY_SIZE(sfe_ipv6_ops_local_out));
#else
	DEBUG_TRACE("sfe: Unregister local out hook\n");
	nf_unregister_net_hooks(&init_net, sfe_ipv6_ops_local_out, ARRAY_SIZE(sfe_ipv6_ops_local_out));
#endif
#endif

#ifdef CONFIG_NF_FLOW_COOKIE
	sysfs_remove_file(si->sys_ipv6, &sfe_ipv6_flow_cookie_attr.attr);
#endif /* CONFIG_NF_FLOW_COOKIE */

	sysfs_remove_file(si->sys_ipv6, &sfe_ipv6_cpu_attr.attr);

	sysfs_remove_file(si->sys_ipv6, &sfe_ipv6_debug_dev_attr.attr);

	kobject_put(si->sys_ipv6);
}

#ifdef CONFIG_NF_FLOW_COOKIE
EXPORT_SYMBOL(sfe_ipv6_register_flow_cookie_cb);
EXPORT_SYMBOL(sfe_ipv6_unregister_flow_cookie_cb);
#endif
