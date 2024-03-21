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

#ifndef __EIP_IPSEC_DEV_H
#define __EIP_IPSEC_DEV_H

#define EIP_IPSEC_DEV_MAX_HDR_LEN 96U
#define EIP_IPSEC_DEV_MAX_HEADROOM 128U           /* Size of the buffer headroom. */
#define EIP_IPSEC_DEV_MAX_TAILROOM 192U           /* Size of the buffer tailroom. */
#define EIP_IPSEC_DEV_ARPHRD 31U	/* ARP (iana.org) hardware type for an IPsec tunnel. */

/*
 * IPsec device statistics.
 */
struct eip_ipsec_dev_stats {
	uint64_t sa_added;		/* Number of (rekeyed) SA added */
	uint64_t sa_removed;		/* Number of SA removed */

	/*
	 * Tx is for encapsulation packets.
	 */
	uint64_t tx_pkts;		/* Encap Packet transmitted */
	uint64_t tx_bytes;		/* Encap Bytes transmitted */
	uint64_t tx_vp_exp;		/* Encap Packet transmitted via VP exception */
	uint64_t tx_host;		/* Encap packet trasmitted via Host */
	uint64_t tx_fail;		/* Encapsulation failure */
	uint64_t tx_fail_sa;		/* sa not found failure */
	uint64_t tx_fail_vp_sa;		/* sa not found failure during VP exception */

	/*
	 * Rx is for decapsulation packets.
	 */
	uint64_t rx_pkts;		/* Decap Packet received */
	uint64_t rx_host;		/* Decap Packet received via Host */
	uint64_t rx_bytes;		/* Decap Bytes received */
	uint64_t rx_fail;		/* Decapsulation failure */
	uint64_t rx_fail_sa;		/* sa not found failure */
};


/*
 * IPsec device object.
 */
struct eip_ipsec_dev {
	struct list_head node;		/* Node in Global device list */
	struct net_device *ndev;	/* Linux netdevice representation for this device */
	int64_t devid;			/* Device ID */
	struct list_head enc_sa;	/* Encap SA list protected by RCU read lock and spinlock for modification */
	struct list_head dec_sa;	/* Decap SA list protected by RCU read lock and spinlock for modification */
	struct eip_ipsec_dev_stats __percpu *stats_pcpu;	/* Device statistics */
	struct dentry *dentry;		/* Driver debugfs dentry */

#if defined(EIP_IPSEC_HYBRID)
	ppe_vp_num_t vp_num;		/* VP number associated with this netdevice. */
#endif
};

void eip_ipsec_dev_enc_err(void *app_data, eip_req_t req, int err);
void eip_ipsec_dev_enc_done_v4(void *app_data, eip_req_t req);
void eip_ipsec_dev_enc_done_v6(void *app_data, eip_req_t req);

struct net_device *eip_ipsec_dev_get_by_id(int64_t devid);
void eip_ipsec_dev_link_id(struct net_device *dev, int64_t devid);
void eip_ipsec_dev_unlink_id(struct net_device *dev, int64_t devid);
bool eip_ipsec_dev_sa_exist(struct net_device *dev);

#endif /* !__EIP_IPSEC_DEV_H */
