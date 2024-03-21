/*
 * Copyright (c) 2022, Qualcomm Innovation Center, Inc. All rights reserved.
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

#ifndef __EIP_IPSEC_SA_H
#define __EIP_IPSEC_SA_H

#include <net/dst_cache.h>

struct eip_ipsec_dev;

/*
 * IPsec SA statistics.
 */
struct eip_ipsec_sa_stats {
	uint64_t tx_pkts;		/* Packet enqueued to DMA */
	uint64_t tx_bytes;		/* Bytes enqueued to DMA */
	uint64_t rx_pkts;		/* Packet completed by DMA */
	uint64_t rx_bytes;		/* Byte completed by DMA */

	uint64_t fail_route;		/* Route not found error */
	uint64_t fail_expand;		/* SKB expand failure */
	uint64_t fail_enqueue;		/* DMA transmit failure */
	uint64_t fail_transform;	/* transformation error */
	uint64_t fail_sp_alloc;		/* transformation error */
	uint64_t dst_cache_miss;	/* DST cache not present */
};

/*
 * IPsec SA object.
 */
struct eip_ipsec_sa {
	struct list_head node;		/* Node in device list */
	struct hlist_node hnode;	/* Node in global hash list */
	struct net_device *ndev;	/* Parent device object */
	struct eip_ipsec_dev *eid;	/* Cached device private pointer */

	struct kref ref;		/* Reference incremented for each flow & packet */
	struct dst_cache dst_cache;	/* Destination route cache */
	__be32 spi;			/* ESP Security Parameter Index */
	__be32 dst_ip[4];		/* Destination IP address */
	u8 ip_ver;			/* IP version */
	uint16_t head_room;		/* Headroom required for encapsulation */
	uint16_t tail_room;		/* Tailroom required for encapsulation */
	struct eip_tr *tr;		/* Transform record allocated by HW */
	struct eip_ipsec_sa_stats __percpu *stats_pcpu;	/* SA statistics */
	struct xfrm_state *xs;          /* offloaded xfrm sate to use in SKB after decap. */

#if defined(EIP_IPSEC_HYBRID)
	eip_tr_callback_t cb;		/* Callback for VP exception */
#endif

	struct eip_ipsec_tuple tuple;	/* SA tuple */
	struct completion completion;   /* Completion to wait for all deref */
	struct rcu_head rcu;            /* delay rcu free */
};

void eip_ipsec_sa_final(struct kref *kref);
struct eip_ipsec_sa *eip_ipsec_sa_ref_get_encap(struct net_device *ndev);
struct eip_ipsec_sa *eip_ipsec_sa_ref_get_decap_v6(__be32 *addr, __be32 spi);
struct eip_ipsec_sa *eip_ipsec_sa_ref_get_decap_v4(__be32 *addr, __be32 spi);
ssize_t eip_ipsec_sa_read(struct net_device *ndev, bool encap, char *buf, ssize_t max_len);

/*
 * Increment SA reference.
 */
static inline struct eip_ipsec_sa *eip_ipsec_sa_ref(struct eip_ipsec_sa *sa)
{
	kref_get(&sa->ref);
	return sa;
}

/*
 * Increment SA reference only if nonzero
 */
static inline struct eip_ipsec_sa *eip_ipsec_sa_ref_unless_zero(struct eip_ipsec_sa *sa)
{
	if (!kref_get_unless_zero(&sa->ref))
		return NULL;

	return sa;
}
/*
 * Decrement SA reference.
 */
static inline void eip_ipsec_sa_deref(struct eip_ipsec_sa *sa)
{
	kref_put(&sa->ref, eip_ipsec_sa_final);
}

#endif /* !__EIP_IPSEC_SA_H */
