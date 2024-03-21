/*
 * Copyright (c) 2014, 2018-2019 The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/phy.h>
#include <net/sock.h>
#include <asm/io.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0))
#include <mach/msm_iomap.h>
#else
#include <linux/of.h>
#include <linux/of_mdio.h>
#ifdef MACSEC_IPQ806X_SUPPORT
#include "socinfo.h"
#endif
#endif

#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/notifier.h>

#include "nss_macsec_emac.h"
#include "nss_macsec_sdk_api.h"
#include "nss_macsec_fal_api.h"
#include "nss_macsec_utility.h"
#include "nss_macsec_os.h"
#include "nss_macsec_ops.h"

#include "nss_gmac_dev.h"
#include "nss_gmac_clocks.h"
#include "nss_gmac_network_interface.h"

/* NSS MACSEC Base Addresses */
#define NSS_MACSEC_BASE			0x37800000
#define NSS_MACSEC_REG_LEN		0x00200000
#define MACSEC_DEVICE_NUM 4

/* Pause is 5000 slot-times where slot-time is the time taken to transmit 64 bytes on GMII */
#define NSS_GMAC_PAUSE_TIME 0x13880000

/* Global data */
struct nss_gmac_global_ctx ctx;

/* Initialize notifier list for NSS GMAC */
static BLOCKING_NOTIFIER_HEAD(nss_gmac_notifier_list);

static struct sock *sdk_nl_sk = NULL;

static bool phy_driver_flag[MACSEC_DEVICE_NUM] = {FALSE, FALSE, FALSE, FALSE};
static bool secy_phy_flag;
/**
 * @brief MACSEC driver context
 */
struct nss_macsec_ctx {
	char __iomem *macsec_base[MACSEC_DEVICE_NUM];
};
static struct nss_macsec_ctx macsec_ctx;
static uint32_t macsec_notifier_register_status = 0;
static uint32_t nss_macsec_pre_init_flag = 0;


#ifdef MACSEC_IPQ806X_SUPPORT
static int macsec_ncb(struct notifier_block *nb, unsigned long value,
		      void *priv)
{
	int result = NOTIFY_OK;
	struct nss_gmac_speed_ctx *gmac_speed_ctx_p = priv;
	printk("macsec_ncb  mac_id:0x%x speed:0x%x\n",
		gmac_speed_ctx_p->mac_id, gmac_speed_ctx_p->speed);
	switch (value) {
	case NSS_GMAC_SPEED_SET:
		nss_macsec_speed(gmac_speed_ctx_p->mac_id - 1, gmac_speed_ctx_p->speed);
		break;
	default:
		result = NOTIFY_BAD;
	}
	return result;
}

static struct notifier_block macsec_notifier __attribute__ ((unused)) = {
	.notifier_call = macsec_ncb,
};
#endif

unsigned int nss_macsec_device_exist(unsigned int dev_id)
{
	if(macsec_ctx.macsec_base[dev_id] == NULL) {
		printk("macsec[%d] doesn't exist, any operation for it is not permitted!\n", dev_id);
		return ERROR_NOT_SUPPORT;
	}
	return ERROR_OK;
}

int nss_macsec_reg_read(unsigned int dev_id, unsigned int reg_addr,
			unsigned int *pvalue)
{
	if(macsec_ctx.macsec_base[dev_id] == NULL) {
		return ERROR_NOT_SUPPORT;
	}

	*pvalue =
	    readl((unsigned char *)((char *) macsec_ctx.macsec_base[dev_id] +
				    (reg_addr)));
	return 0;
}

int nss_macsec_reg_write(unsigned int dev_id, unsigned int reg_addr,
			 unsigned int pvalue)
{
	if(macsec_ctx.macsec_base[dev_id] == NULL) {
		return ERROR_NOT_SUPPORT;
	}
	writel(pvalue,
	       (unsigned char *)((char *) macsec_ctx.macsec_base[dev_id] +
				 (reg_addr)));
	return 0;
}

static unsigned int nss_macsec_msg_handle(void *msg_data, unsigned int *sync)
{
	struct sdk_msg_header *header = (struct sdk_msg_header *)msg_data;
	unsigned short cmd_type = header->cmd_type;
	unsigned int ret = 0;

	*sync = 1;
	switch (cmd_type) {
	case SDK_FAL_CMD:{
			ret = nss_macsec_fal_msg_handle(header);
		}
		break;

	default:
		break;
	}

	return ret;
}

static void nss_macsec_netlink_recv(struct sk_buff *__skb)
{
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh = NULL;
	struct sdk_msg_header *header = NULL;
	void *msg_data;
	u32 pid = 0, msg_type = 0, sync = 0, ret = 0, msg_len = 0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
	if ((skb = skb_clone(__skb, GFP_KERNEL)) != NULL) {
#else
	if ((skb = skb_get(__skb)) != NULL) {
#endif
		nlh = nlmsg_hdr(skb);
		pid = nlh->nlmsg_pid;
		msg_data = NLMSG_DATA(nlh);
		msg_type = nlh->nlmsg_type;
		msg_len = sizeof(struct nlmsghdr) + sizeof(struct sdk_msg_header);
		header = (struct sdk_msg_header *)msg_data;
		if (header->buf_len > (U32_MAX - msg_len))
			return;
		msg_len += header->buf_len;

		if(skb->len < msg_len) {
			printk("Unexpected msg received! skb_len [0x%x] less than 0x%x\n",
				skb->len, msg_len);
			return;
		} else if (msg_type == SDK_CALL_MSG) {
			ret = nss_macsec_msg_handle(msg_data, &sync);
			header->ret = ret;
		} else {
			printk("Unexpected msg:0x%x received!\n", msg_type);
			return;
		}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
		NETLINK_CB(skb).portid = 0; /* from kernel */
#else
		NETLINK_CB(skb).pid = 0;		/* from kernel */
#endif
		NETLINK_CB(skb).dst_group = 0;	/* unicast */
		netlink_unicast(sdk_nl_sk, skb, pid, MSG_DONTWAIT);
	}

	return;
}

static int nss_macsec_netlink_init(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
	struct netlink_kernel_cfg cfg = {
			.input			= nss_macsec_netlink_recv,
	};

	sdk_nl_sk = netlink_kernel_create(&init_net, NETLINK_SDK, &cfg);
#else
	sdk_nl_sk = netlink_kernel_create(&init_net,
						NETLINK_SDK,
						0,
						nss_macsec_netlink_recv,
						NULL, THIS_MODULE);
#endif

	if (sdk_nl_sk == NULL) {
		printk("Create netlink socket fail\n");
		return -ENODEV;
	}

	return 0;
}

static struct mii_bus *nss_macsec_miibus[MAX_SECY_ID] = {
	NULL, NULL, NULL, NULL};

int nss_macsec_phy_mdio_write(u32 secy_id, u32 reg_addr, u16 val)
{
	u32 phy_addr;

	if (!nss_macsec_miibus[secy_id])
		return -EIO;

	phy_addr = secy_id_to_phy_addr(secy_id);
	mdiobus_write(nss_macsec_miibus[secy_id], phy_addr, reg_addr, val);

	return 0;
}

int nss_macsec_phy_mdio_read(u32 secy_id, u32 reg_addr, u16 *val)
{
	u32 phy_addr;

	if (!nss_macsec_miibus[secy_id])
		return -EIO;

	phy_addr = secy_id_to_phy_addr(secy_id);
	*val = mdiobus_read(nss_macsec_miibus[secy_id], phy_addr, reg_addr);

	return 0;
}


g_error_t nss_macsec_dt_secy_id_get(u8 *dev_name, u32 *secy_id)
{
	u32 secy_idx, phy_addr = -1;
	struct net_device *netdev = NULL;

	/* get net device by ifname */
	netdev = __dev_get_by_name(&init_net, (const char *)dev_name);
	if (unlikely(!netdev)) {
		*secy_id = INVALID_SECY_ID;
		return ERROR_NOT_FOUND;
	}

	/* get phy address by net device */
	if (netdev->phydev) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0))
		phy_addr = netdev->phydev->addr;
#else
		phy_addr = netdev->phydev->mdio.addr;
#endif
	}

	/* find secy_id by phy address */
	if (phy_addr != -1) {
		for (secy_idx = 0; secy_idx < MAX_SECY_ID; secy_idx++) {
			if (phy_addr == secy_id_to_phy_addr(secy_idx)) {
				*secy_id = secy_idx;
				return ERROR_OK;
			}
		}
	}

	/* fail to find secy_id */
	*secy_id = INVALID_SECY_ID;
	return ERROR_NOT_FOUND;
}

static int nss_macsec_mdiobus_init(struct device_node *macsec_node, u32 secy_id)
{
	struct device_node *mdio_node = NULL;

	if (secy_id >= MAX_SECY_ID) {
		macsec_warning("Invalid secy id:%d!\n", secy_id);
		return -EINVAL;
	}

	mdio_node = of_parse_phandle(macsec_node, "mdiobus", 0);
	if (!mdio_node) {
		macsec_warning("Failed to find mdio node!\n");
		return -EINVAL;
	}
	nss_macsec_miibus[secy_id] = of_mdio_find_bus(mdio_node);
	if (!nss_macsec_miibus[secy_id]) {
		macsec_warning("Failed to find mdio bus!\n");
		return -EIO;
	}

	return 0;
}

static void nss_macsec_netlink_fini(void)
{
	if (sdk_nl_sk) {
		sock_release(sdk_nl_sk->sk_socket);
		sdk_nl_sk = NULL;
	}
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
static int nss_macsec_dt_init(uint32_t dev_id)
{
	int ret = 0;
	unsigned int mem_start, mem_len;
	void __iomem *mmap_io_addr = NULL;
	struct device_node *nss_macsec_node = NULL;
	const __be32 *reg_cfg;
	unsigned int len = 0;
	char dev_name[32];
	unsigned int phy_address;

	memset(dev_name, 0, sizeof(dev_name));
	snprintf(dev_name, sizeof(dev_name), "nss-macsec%d", dev_id);

	nss_macsec_node = of_find_node_by_name(NULL, dev_name);
	if (!nss_macsec_node) {
		printk(KERN_ERR "cannot find nss-macsec%d node\n", dev_id);
		return -ENODEV;
	}
	printk(KERN_INFO "%s DT exist!\n", dev_name);

	nss_macsec_mdiobus_init(nss_macsec_node, dev_id);

/* add for Napa masec*/
	if (!of_property_read_u32(nss_macsec_node, "phy_addr", &phy_address)) {
		nss_macsec_secy_info_set(dev_id, phy_address);
		nss_macsec_secy_driver_init(dev_id);
		phy_driver_flag[dev_id] = TRUE;
		secy_phy_flag = TRUE;
		return ret;
	}
/* end for Napa masec*/

	reg_cfg = of_get_property(nss_macsec_node, "reg", &len);
	if (!reg_cfg) {
		printk(KERN_ERR "%s: error reading device node properties for reg\n", dev_name);
		return -EIO;
	}

	mem_start = be32_to_cpup(reg_cfg);
	mem_len = be32_to_cpup(reg_cfg + 1);

	if (!request_mem_region(mem_start, mem_len, dev_name)) {
		macsec_warning("%s Unable to request resource.", __func__);
		return -EIO;
	}

	mmap_io_addr = ioremap(mem_start, mem_len);
	if (!mmap_io_addr) {
		macsec_warning("%s ioremap fail.", __func__);
		return -EIO;
	}
	macsec_trace("macsec.%d phy_addr:0x%x len:0x%x mem_start:%p\n",
		     dev_id, mem_start, mem_len, mmap_io_addr);

	macsec_ctx.macsec_base[dev_id] = mmap_io_addr;

	if (nss_macsec_pre_init_flag == 0) {
#ifdef MACSEC_IPQ806X_SUPPORT
		nss_macsec_pre_init();
#endif
		nss_macsec_pre_init_flag = 1;
	}

	/* Invoke Macsec Initialization API */
	nss_macsec_secy_init(dev_id);

	if (macsec_notifier_register_status == 0) {
#ifdef MACSEC_IPQ806X_SUPPORT
		nss_gmac_link_state_change_notify_register(&macsec_notifier);
#endif
		macsec_notifier_register_status = 1;
	}

	macsec_trace("macsec.%d probe done\n", dev_id);
	return ret;

}

static int nss_macsec_clean(void)
{
	uint32_t dev_id = 0;
	int ret = 0, secy_id = 0;

	if (secy_phy_flag == TRUE) {
		for (secy_id = 0; secy_id < MACSEC_DEVICE_NUM; secy_id++) {
			if (phy_driver_flag[secy_id] == TRUE)
				nss_macsec_secy_driver_cleanup(secy_id);
		}
	} else {
		if (macsec_notifier_register_status) {
#ifdef MACSEC_IPQ806X_SUPPORT
			nss_gmac_link_state_change_notify_unregister(
				&macsec_notifier);
#endif
			macsec_notifier_register_status = 0;
		}

		for (dev_id = 0; dev_id < MACSEC_DEVICE_NUM; dev_id++) {
			iounmap(macsec_ctx.macsec_base[dev_id]);
			release_mem_region(
				NSS_MACSEC_BASE+(NSS_MACSEC_REG_LEN*dev_id),
					    NSS_MACSEC_REG_LEN);
		}
	}
	return ret;
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0))
static int nss_macsec_probe(struct platform_device *pdev)
{
	int ret = 0;
	unsigned int mem_start, mem_len;
	void __iomem *mmap_io_addr = NULL;

	if (nss_macsec_pre_init_flag == 0) {
#ifdef MACSEC_IPQ806X_SUPPORT
		nss_macsec_pre_init();
#endif
		nss_macsec_pre_init_flag = 1;
	}

	mem_start = pdev->resource[0].start;
	mem_len = pdev->resource[0].end - mem_start + 1;
	if (!request_mem_region(mem_start, mem_len, pdev->name)) {
		macsec_warning("%s Unable to request resource.", __func__);
		return -EIO;
	}

	mmap_io_addr = ioremap_nocache(mem_start, mem_len);
	if (!mmap_io_addr) {
		macsec_warning("%s ioremap fail.", __func__);
		return -EIO;
	}
	macsec_trace("macsec.%d phy_addr:0x%x len:0x%x mem_start:%p\n",
		     pdev->id, mem_start, mem_len, mmap_io_addr);

	macsec_ctx.macsec_base[pdev->id] = mmap_io_addr;

	/* Invoke Macsec Initialization API */
	nss_macsec_secy_init(pdev->id);

	if (macsec_notifier_register_status == 0) {
#ifdef MACSEC_IPQ806X_SUPPORT
		nss_gmac_link_state_change_notify_register(&macsec_notifier);
#endif
		macsec_notifier_register_status = 1;
	}

	macsec_trace("macsec.%d probe done\n", pdev->id);
	return ret;
}

static int nss_macsec_remove(struct platform_device *pdev)
{
	unsigned int mem_start, mem_len;

	macsec_trace("%s for dev_id:%d\n", __func__, pdev->id);
	mem_start = pdev->resource[0].start;
	mem_len = pdev->resource[0].end - mem_start + 1;

	if (macsec_notifier_register_status) {
#ifdef MACSEC_IPQ806X_SUPPORT
		nss_gmac_link_state_change_notify_unregister(&macsec_notifier);
#endif
		macsec_notifier_register_status = 0;
	}
	iounmap(macsec_ctx.macsec_base[pdev->id]);
	release_mem_region(mem_start, mem_len);
	return 0;
}

/**
 * @brief Linux Platform driver for MACSEC
 */
static struct platform_driver nss_macsec_drv __attribute__ ((unused)) = {
	.probe = nss_macsec_probe,
	.remove = nss_macsec_remove,
	.driver = {
		   .name = "nss-macsec",
		   .owner = THIS_MODULE,
		   },
};
#endif

static int __init nss_macsec_init_module(void)
{
	uint32_t dev_id = 0;
	int ret = 0;

	for (dev_id = 0; dev_id < MACSEC_DEVICE_NUM; dev_id++)
		macsec_ctx.macsec_base[dev_id] = NULL;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
		for (dev_id = 0; dev_id < MACSEC_DEVICE_NUM; dev_id++) {
			ret = nss_macsec_dt_init(dev_id);
			if (ret && (ret != -ENODEV))
				return -EIO;
		}
#else
	if (platform_driver_register(&nss_macsec_drv) != 0) {
		macsec_warning("platform drv reg failure\n");
		return -EIO;
	}
#endif
	macsec_trace("%s done!\n", __func__);

	return ret;
}

static int __init nss_macsec_init(void)
{
	nss_macsec_netlink_init();

	nss_macsec_mutex_init();

	nss_macsec_init_module();

	printk("nss_macsec init success\n");

	return 0;
}

static void __exit nss_macsec_fini(void)
{
	nss_macsec_netlink_fini();

	nss_macsec_mutex_destroy();
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0))
	platform_driver_unregister(&nss_macsec_drv);
#else
	nss_macsec_clean();
#endif
	if (nss_macsec_pre_init_flag) {
#ifdef MACSEC_IPQ806X_SUPPORT
		nss_macsec_pre_exit();
#endif
		nss_macsec_pre_init_flag = 0;
	}

	printk("nss_macsec exit success\n");
}

/**
 * @brief Return PCS Channel speed values
 * @param[in] nss_gmac_dev *
 * @return returns PCS speed values.
 */
static uint32_t get_pcs_speed(struct nss_gmac_dev *gmacdev)
{
	uint32_t speed;

	switch (gmacdev->speed) {
	case SPEED_1000:
		speed = PCS_CH_SPEED_1000;
		break;

	case SPEED_100:
		speed = PCS_CH_SPEED_100;
		break;

	case SPEED_10:
		speed = PCS_CH_SPEED_10;
		break;

	default:
		speed = PCS_CH_SPEED_1000;
		break;
	}

	return speed;
}

/*
 * @brief Return clock divider value for QSGMII PHY.
 * @param[in] nss_gmac_dev *
 * @return returns QSGMII clock divider value.
 */
static uint32_t clk_div_qsgmii(struct nss_gmac_dev *gmacdev)
{
	uint32_t div;

	switch (gmacdev->speed) {
	case SPEED_1000:
		div = QSGMII_CLK_DIV_1000;
		break;

	case SPEED_100:
		div = QSGMII_CLK_DIV_100;
		break;

	case SPEED_10:
		div = QSGMII_CLK_DIV_10;
		break;

	default:
		div = QSGMII_CLK_DIV_1000;
		break;
	}

	return div;
}

/**
 * @brief Return clock divider value for SGMII PHY.
 * @param[in] nss_gmac_dev *
 * @return returns SGMII clock divider value.
 */
static uint32_t clk_div_sgmii(struct nss_gmac_dev *gmacdev)
{
	uint32_t div;

	switch (gmacdev->speed) {
	case SPEED_1000:
		div = SGMII_CLK_DIV_1000;
		break;

	case SPEED_100:
		div = SGMII_CLK_DIV_100;
		break;

	case SPEED_10:
		div = SGMII_CLK_DIV_10;
		break;

	default:
		div = SGMII_CLK_DIV_1000;
		break;
	}

	return div;
}

/**
 * @brief Return clock divider value for RGMII PHY.
 * @param[in] nss_gmac_dev *
 * @return returns RGMII clock divider value.
 */
static uint32_t clk_div_rgmii(struct nss_gmac_dev *gmacdev)
{
	uint32_t div;

	switch (gmacdev->speed) {
	case SPEED_1000:
		div = RGMII_CLK_DIV_1000;
		break;

	case SPEED_100:
		div = RGMII_CLK_DIV_100;
		break;

	case SPEED_10:
		div = RGMII_CLK_DIV_10;
		break;

	default:
		div = RGMII_CLK_DIV_1000;
		break;
	}

	return div;
}

/**
 * @brief Set GMAC speed.
 * @param[in] nss_gmac_dev *
 * @return returns 0 on success.
 */
int32_t nss_gmac_dev_set_speed(struct nss_gmac_dev *gmacdev)
{
	uint32_t val = 0;
	uint32_t id = gmacdev->macid;
	uint32_t div = 0, pcs_speed = 0;
	uint32_t clk = 0;
	uint32_t *nss_base = (uint32_t *)(gmacdev->ctx->nss_base);
	uint32_t *qsgmii_base = (uint32_t *)(gmacdev->ctx->qsgmii_base);
	struct nss_gmac_speed_ctx gmac_speed_ctx = {0, 0};
	int force_speed = 0;

	switch (gmacdev->phy_mii_type) {
	case PHY_INTERFACE_MODE_RGMII:
		div = clk_div_rgmii(gmacdev);
		break;

	case PHY_INTERFACE_MODE_SGMII:
		div = clk_div_sgmii(gmacdev);
		break;

	case PHY_INTERFACE_MODE_QSGMII:
		div = clk_div_qsgmii(gmacdev);
		break;

	default:
		netdev_dbg(gmacdev->netdev, "%s: Invalid MII type\n", __func__);
		return -EINVAL;
	}

	if (gmacdev->forced_speed != SPEED_UNKNOWN)
		force_speed = 1;

	/*
	 * Set SGMII force speed control signal if necessary
	 */
	if (((gmacdev->phy_mii_type == PHY_INTERFACE_MODE_SGMII) ||
			(gmacdev->phy_mii_type == PHY_INTERFACE_MODE_QSGMII))
			&& (force_speed == 1)) {
			pcs_speed = get_pcs_speed(gmacdev);
			nss_gmac_set_reg_bits(qsgmii_base, PCS_ALL_CH_CTL,
						PCS_CHn_FORCE_SPEED(id));
			nss_gmac_clear_reg_bits(qsgmii_base, PCS_ALL_CH_CTL,
						PCS_CHn_SPEED_MASK(id));
			nss_gmac_set_reg_bits(qsgmii_base, PCS_ALL_CH_CTL,
						PCS_CHn_SPEED(id, pcs_speed));
	}

	clk = 0;
	/* Disable GMACn Tx/Rx clk */
	if (gmacdev->phy_mii_type == PHY_INTERFACE_MODE_RGMII)
		clk |= GMACn_RGMII_RX_CLK(id) | GMACn_RGMII_TX_CLK(id);
	else
		clk |= GMACn_GMII_RX_CLK(id) | GMACn_GMII_TX_CLK(id);
	nss_gmac_clear_reg_bits(nss_base, NSS_ETH_CLK_GATE_CTL, clk);

	/* set clock divider */
	val = nss_gmac_read_reg(nss_base, NSS_ETH_CLK_DIV0);
	val &= ~GMACn_CLK_DIV(id, GMACn_CLK_DIV_SIZE);
	val |= GMACn_CLK_DIV(id, div);
	nss_gmac_write_reg(nss_base, NSS_ETH_CLK_DIV0, val);

	/* Enable GMACn Tx/Rx clk */
	nss_gmac_set_reg_bits(nss_base, NSS_ETH_CLK_GATE_CTL, clk);

	val = nss_gmac_read_reg(nss_base, NSS_ETH_CLK_DIV0);
	netdev_dbg(gmacdev->netdev, "%s:NSS_ETH_CLK_DIV0(0x%x) - 0x%x\n",
		      __func__, NSS_ETH_CLK_DIV0, val);

	if (gmacdev->phy_mii_type == PHY_INTERFACE_MODE_SGMII
	    || gmacdev->phy_mii_type == PHY_INTERFACE_MODE_QSGMII) {
		nss_gmac_clear_reg_bits(qsgmii_base, PCS_MODE_CTL,
					PCS_MODE_CTL_CHn_AUTONEG_EN(id));

		/*
		 * Enable autonegotiation from MII register of PHY
		 * if the speed is not forced
		 */
		if (!force_speed) {
			nss_gmac_set_reg_bits(qsgmii_base, PCS_MODE_CTL,
					      PCS_MODE_CTL_CHn_AUTONEG_EN(id));
		}

		val = nss_gmac_read_reg(qsgmii_base, PCS_MODE_CTL);
		netdev_dbg(gmacdev->netdev, "%s: qsgmii_base(0x%x) + PCS_MODE_CTL(0x%x): 0x%x\n",
		       __func__, (uint32_t)qsgmii_base, (uint32_t)PCS_MODE_CTL, val);

	}

	/* Notify link speed change to notifier list */
	gmac_speed_ctx.mac_id = gmacdev->macid;
	gmac_speed_ctx.speed = gmacdev->speed;
	blocking_notifier_call_chain(&nss_gmac_notifier_list,
					NSS_GMAC_SPEED_SET, &gmac_speed_ctx);

	return 0;
}

/*
 * Disables multicast hash filtering.
 * When disabled GMAC performs perfect destination address filtering
 * for multicast frames, it compares DA field with the value programmed
 * in DA register.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_multicast_hash_filter_disable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_frame_filter, gmac_mcast_hash_filter);
}

/*
 * Enables forwarding of control frames.
 * When set forwards all the control frames
 * (incl. unicast and multicast PAUSE frames).
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] pass control.
 * @return void.
 * @note Depends on RFE of flow_control_register[2]
 */
void nss_gmac_set_pass_control(struct nss_gmac_dev *gmacdev,
						uint32_t passcontrol)
{
	uint32_t data;

	data =
	    nss_gmac_read_reg(gmacdev->mac_base, gmac_frame_filter);
	data &= (~gmac_pass_control);
	data |= passcontrol;
	nss_gmac_write_reg(gmacdev->mac_base, gmac_frame_filter,
			   data);
}

/*
 * Enables reception of all the frames to application.
 * GMAC passes all the frames received to application
 * irrespective of whether they pass SA/DA address filtering or not.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_frame_filter_enable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_frame_filter, gmac_filter);
}

/*
 * Enables Receive Own bit (Only in Half Duplex Mode).
 * When enaled GMAC receives all the packets given by phy while transmitting.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_rx_own_enable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_config, gmac_rx_own);
}

/*
 * Enables the normal Destination address filtering.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_dst_addr_filter_normal(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_frame_filter, gmac_dest_addr_filter_inv);
}

/*
 * Disable the ip checksum offloading in receive path.
 * Ip checksum offloading is disabled in the receive path.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_disable_rx_chksum_offload(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_config, gmac_rx_ipc_offload);
}

/*
 * GMAC tries only one transmission (Only in Half Duplex mode).
 * If collision occurs on the GMII/MII, GMAC will ignore the current frami
 * transmission and report a frame abort with excessive collision
 * in tranmit frame status.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_retry_disable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base,
			      gmac_config, gmac_retry);
}

/*
 * Disables Source address filtering.
 * When disabled GMAC forwards the received frames with updated
 * SAMatch bit in rx_status.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_src_addr_filter_disable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_frame_filter, gmac_src_addr_filter);
}

/*
 * GMAC programmed with the back off limit value.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 * @note This function is tightly coupled with
 * nss_gmac_retry_enable(nss_gmac_dev *gmacdev)
 */
void nss_gmac_back_off_limit(struct nss_gmac_dev *gmacdev, uint32_t value)
{
	uint32_t data;

	data = nss_gmac_read_reg(gmacdev->mac_base, gmac_config);
	data &= (~gmac_backoff_limit);
	data |= value;
	nss_gmac_write_reg(gmacdev->mac_base, gmac_config, data);
}

/*
 * Rx flow control disable.
 * When disabled GMAC will not decode pause frame.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_rx_flow_control_disable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_flow_control, gmac_rx_flow_control);
}

/*
 * Selects the MII port.
 * When called MII (10/100Mbps) port is selected (programmable only in
 * 10/100/1000 Mbps configuration).
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_select_mii(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base,
			      gmac_config, gmac_mii_gmii);

	if (gmacdev->speed == SPEED_100) {
		nss_gmac_set_reg_bits(gmacdev->mac_base,
				      gmac_config, gmac_fe_speed100);
		return;
	}

	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_config, gmac_fe_speed100);
}

/*
 * Instruct the DMA to drop the packets fails tcp ip checksum.
 * This is to instruct the receive DMA engine to drop the recevied
 * packet if they fails the tcp/ip checksum in hardware. Valid only when
 * full checksum offloading is enabled(type-2).
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_rx_tcpip_chksum_drop_enable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->dma_base,
				dma_control, dma_disable_drop_tcp_cs);
}

/*
 * Enables the ip checksum offloading in receive path.
 * When set GMAC calculates 16 bit 1's complement of all received
 * ethernet frame payload. It also checks IPv4 Header checksum is correct.
 * GMAC core appends the 16 bit checksum calculated for payload of IP
 * datagram and appends it to Ethernet frame transferred to the application.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_enable_rx_chksum_offload(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base,
			      gmac_config, gmac_rx_ipc_offload);
}

/*
 * Initialize IPC Checksum offloading.
 * @param[in] pointer to nss_gmac_dev.
 * @return void
 */
void nss_gmac_ipc_offload_init(struct nss_gmac_dev *gmacdev)
{
	if (test_bit(__NSS_GMAC_RXCSUM, &gmacdev->flags)) {
		/* Enable the offload engine in the receive path */
		nss_gmac_enable_rx_chksum_offload(gmacdev);

		/* DMA drops the packets if error in encapsulated ethernet
		 * payload.
		 */
		nss_gmac_rx_tcpip_chksum_drop_enable(gmacdev);
		netdev_dbg(gmacdev->netdev, "%s: enable Rx checksum\n", __func__);
	} else {
		nss_gmac_disable_rx_chksum_offload(gmacdev);
		netdev_dbg(gmacdev->netdev, "%s: disable Rx checksum\n", __func__);
	}
}

/*
 * Disable the MMC Tx interrupt.
 * The MMC tx interrupts are masked out as per the mask specified.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] tx interrupt bit mask for which interrupts needs to be disabled.
 * @return returns void.
 */
void nss_gmac_disable_mmc_tx_interrupt(struct nss_gmac_dev *gmacdev,
						uint32_t mask)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base,
			      gmac_mmc_intr_mask_tx, mask);
}

/*
 * Tx flow control disable.
 * When Disabled
 *	- In full duplex GMAC will not transmit any pause frames.
 *	- In Half duplex GMAC disables the back pressure feature.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_tx_flow_control_disable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
			gmac_flow_control, gmac_tx_flow_control);
}

/*
 * Flush Dma Tx fifo.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_flush_tx_fifo(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits(gmacdev->dma_base, dma_control,
						dma_flush_tx_fifo);
}

/*
 * Disable Jumbo frame support.
 * When Disabled GMAC does not supports jumbo frames.
 * Giant frame error is reported in receive frame status.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_jumbo_frame_disable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_config, gmac_jumbo_frame);
}

/*
 * GMAC tries retransmission (Only in Half Duplex mode).
 * If collision occurs on the GMII/MII, GMAC attempt retries based on the
 * back off limit configured.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 * @note This function is tightly coupled with
 * nss_gmac_back_off_limit(nss_gmac_dev *, uint32_t).
 */
void nss_gmac_retry_enable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_config, gmac_retry);
}

/*
 * Enable the watchdog timer on the receiver.
 * When enabled, Gmac enables Watchdog timer, and GMAC allows no more than
 * 2048 bytes of data (10,240 if Jumbo frame enabled).
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_wd_enable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_config, gmac_watchdog);
}

/*
 * Disable pause frame generation.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_rx_pause_disable(struct nss_gmac_dev *gmacdev)
{
	netdev_dbg(gmacdev->netdev, "%s: disable Rx flow control\n", __func__);

	nss_gmac_clear_reg_bits(gmacdev->dma_base,dma_control,
				dma_rx_frame_flush);

	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_flow_control, gmac_rx_flow_control);
}

/**
 * @brief Read a register from an external PHY
 * @param[in] pointer to gmac context
 * @param[in] phy id
 * @param[in] register id
 * @return Returns value read from phy register on success, 0 otherwise.
 */
uint16_t nss_gmac_mii_rd_reg(struct nss_gmac_dev *gmacdev, uint32_t phy,
			     uint32_t reg)
{
	uint16_t data = 0;

	if (IS_ERR(gmacdev->phydev)) {
		netdev_dbg(gmacdev->netdev, "Error: Reading uninitialized PHY...\n");
		return 0;
	}

	data = (uint16_t)phy_read(gmacdev->phydev, reg);

	return data;
}

/*
 * Enables Frame bursting (Only in Half Duplex Mode).
 * When enabled, GMAC allows frame bursting in GMII Half Duplex mode.
 * Reserved in 10/100 and Full-Duplex configurations.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_frame_burst_enable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base,
			      gmac_config, gmac_frame_burst);
}

static void nss_gmac_check_pcs_status(struct nss_gmac_dev *gmacdev)
{
	struct nss_gmac_global_ctx *ctx = NULL;
	uint32_t *qsgmii_base = NULL;
	uint32_t id = 0;
	uint32_t reg = 0;

	ctx = gmacdev->ctx;
	qsgmii_base = ctx->qsgmii_base;
	id = gmacdev->sgmii_pcs_chanid;

	gmacdev->link_state = LINKDOWN;

	/* confirm link is up in PCS_QSGMII_MAC_STATUS register */
	reg = nss_gmac_read_reg(qsgmii_base, PCS_QSGMII_MAC_STAT);
	if (!(reg & PCS_MAC_STAT_CHn_LINK(id)))
		return;

	gmacdev->link_state = LINKUP;

	/* save duplexity */
	if (reg & PCS_MAC_STAT_CHn_DUPLEX(id))
		gmacdev->duplex_mode = DUPLEX_FULL;
	else
		gmacdev->duplex_mode = DUPLEX_HALF;

	/* save speed */
	switch (PCS_MAC_STAT_CHn_SPEED(id, reg)) {
	case 0:
		gmacdev->speed = SPEED_10;
		break;

	case 1:
		gmacdev->speed = SPEED_100;
		break;

	case 2:
		gmacdev->speed = SPEED_1000;
		break;
	}
}

/*
 * Handle Q/SGMII linkup
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
static void nss_gmac_check_sgmii_link(struct nss_gmac_dev *gmacdev)
{
	struct nss_gmac_global_ctx *ctx = NULL;
	uint32_t *qsgmii_base = NULL;
	uint32_t id = 0;
	uint32_t reg = 0;
	uint32_t previous_linkup_duplex = 0;
	uint32_t previous_linkup_speed = 0;
	uint32_t new_duplex = 0;
	uint32_t new_speed = 0;
	int32_t timeout = 0;
	int32_t timeout_count = 0;

	ctx = gmacdev->ctx;
	qsgmii_base = ctx->qsgmii_base;
	id = gmacdev->sgmii_pcs_chanid;

	previous_linkup_speed = gmacdev->speed;
	previous_linkup_duplex = gmacdev->duplex_mode;

reheck_pcs_mac_status:
	nss_gmac_check_pcs_status(gmacdev);
	if (gmacdev->link_state == LINKDOWN) {
		if (gmacdev->phydev->link) {
			netdev_dbg(gmacdev->netdev, "SGMII PCS error. Resetting PHY using MDIO\n");
			phy_write(gmacdev->phydev, MII_BMCR,
				BMCR_RESET | phy_read(gmacdev->phydev, MII_BMCR));
		}

		return;
	}

	new_speed = gmacdev->speed;
	new_duplex = gmacdev->duplex_mode;

	/* reinitiate autoneg in QSGMII CSR. */
	nss_gmac_set_reg_bits(qsgmii_base, PCS_MODE_CTL,
				PCS_MODE_CTL_CHn_AUTONEG_RESTART(id));
	nss_gmac_clear_reg_bits(qsgmii_base, PCS_MODE_CTL,
				PCS_MODE_CTL_CHn_AUTONEG_RESTART(id));
	timeout = 50;
	reg = nss_gmac_read_reg(qsgmii_base, PCS_ALL_CH_STAT);
	while (!(reg & PCS_CHn_AUTONEG_COMPLETE(id)) && timeout > 0) {
		timeout--;
		usleep_range(10000, 12000);
		reg = nss_gmac_read_reg(qsgmii_base, PCS_ALL_CH_STAT);
	}

	/* handle autoneg timeout */
	if (timeout == 0) {
		netdev_dbg(gmacdev->netdev, "%s: PCS ch %d autoneg timeout\n",
							__func__, id);
		timeout_count++;
		if (timeout_count == 2) {
			gmacdev->link_state = LINKDOWN;
			nss_gmac_set_reg_bits(qsgmii_base, PCS_MODE_CTL,
					      PCS_MODE_CTL_CHn_PHY_RESET(id));
			return;
		}
		goto reheck_pcs_mac_status;
	}
	netdev_dbg(gmacdev->netdev, "%s: PCS ch %d autoneg complete\n",
							__func__, id);

	nss_gmac_check_pcs_status(gmacdev);

	if ((gmacdev->link_state == LINKDOWN) || (new_speed != gmacdev->speed)) {
		gmacdev->link_state = LINKDOWN;
			netdev_dbg(gmacdev->netdev, "SGMII PCS error. Resetting PHY using MDIO\n");
			phy_write(gmacdev->phydev, MII_BMCR,
				BMCR_RESET | phy_read(gmacdev->phydev, MII_BMCR));
		return;
	}

	/* check if initial speed has changed */
	if (previous_linkup_speed != gmacdev->speed) {
		/* switch clock dividers */
		nss_gmac_dev_set_speed(gmacdev);

		/* flush GMAC fifo */
		nss_gmac_flush_tx_fifo(gmacdev);
	}
}

/*
 * Function to program DMA Control register.
 * The Dma Control register is programmed with the value given.
 * The bits to be set are bit wise or'ed and sent as the second
 * argument to this function.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] the data to be programmed.
 * @return 0 on success else return the error status.
 */
int32_t nss_gmac_dma_control_init(struct nss_gmac_dev *gmacdev,
						uint32_t init_value)
{
	nss_gmac_write_reg(gmacdev->dma_base, dma_control, init_value);
	return 0;
}

/*
 * This enables pause frame generation after
 * programming the appropriate registers.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_rx_pause_enable(struct nss_gmac_dev *gmacdev)
{
	netdev_dbg(gmacdev->netdev, "%s: enable Rx flow control\n", __func__);

	/*
	 * We set the DFF bit to convert GMAC DMA FIFO behavior from head
	 * drop to tail drop. This gives us extra room to store frames before
	 * we start sending pause frames and achieve 0 drops for high rate flows
	 * with 64 bytes frames.
	 */
	nss_gmac_set_reg_bits(gmacdev->dma_base, dma_control,
				dma_rx_frame_flush);

	nss_gmac_set_reg_bits(gmacdev->mac_base,
				gmac_flow_control, gmac_rx_flow_control);
}

/*
 * Sets the GMAC core in Half-Duplex mode.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_set_half_duplex(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_config, gmac_duplex);
}

/*
 * This function checks to see if phy PHY autonegotiation is complete.
 * It reads PHY registers to retrieve current speed and duplexity settings.
 * @param[in] pointer to nss_gmac_dev.
 * @return 0 on success. If successful, it updates gmacdev->speed and
 *	   gmacdev->duplex_mode with current speed and duplex mode.
 */
int32_t nss_gmac_check_phy_init(struct nss_gmac_dev *gmacdev)
{
	struct phy_device *phydev = NULL;
	uint32_t phy_link_speed;
	uint32_t phy_link_duplex;

	/*
	 * We cannot have link polling off without forced speed configured!
	 */
	if (!test_bit(__NSS_GMAC_LINKPOLL, &gmacdev->flags)
		&& (gmacdev->forced_speed == SPEED_UNKNOWN)) {
		netdev_dbg(gmacdev->netdev,
				"%s: Forced link speed not configured when link polling disabled\n",
				__func__);
		return -EIO;
	}

	/*
	 * First read the PHY speed/duplex if link polling
	 * is enabled
	 */
	if (test_bit(__NSS_GMAC_LINKPOLL, &gmacdev->flags)) {
		phydev = gmacdev->phydev;
		if (gmacdev->phydev->is_c45 == false) {
			/*
			 * Read the speed/duplex for standard C22 PHYs
			 */
			genphy_read_status(phydev);
		}
	}

	/*
	 * Now find the GMAC speed. Check if we have a forced speed
	 * and duplex configured for the GMAC.
	 */
	if (gmacdev->forced_speed != SPEED_UNKNOWN) {
		/*
		 * Set the GMAC speed as configured
		 */
		gmacdev->speed = gmacdev->forced_speed;
		gmacdev->duplex_mode = gmacdev->forced_duplex;

		goto out;
	}


	/*
	 * Get the GMAC speed from the SGMII PCS for SGMII/QSGMII
	 * interfaces
	 */
	if (gmacdev->phy_mii_type == PHY_INTERFACE_MODE_SGMII
		|| gmacdev->phy_mii_type == PHY_INTERFACE_MODE_QSGMII) {
		nss_gmac_check_sgmii_link(gmacdev);
		if (gmacdev->link_state == LINKDOWN) {
			netdev_dbg(gmacdev->netdev, "%s: SGMII phy linkup ERROR.\n"
								, __func__);
			return -EIO;
		}

		netdev_dbg(gmacdev->netdev, "%s: SGMII phy linkup OK.\n",
								__func__);
		goto out;
	}

	/*
	 * Get the GMAC speed from the PHY for RGMII
	 * interfaces
	 */
	if (phydev) {
		gmacdev->speed = phydev->speed;
		gmacdev->duplex_mode = phydev->duplex;
	}

out:
	/*
	 * If link polling is on, print the link speed from PHY
	 * Otherwise the link speed must have been forced, so
	 * print the GMAC (xMII) forced speed
	 */
	if (test_bit(__NSS_GMAC_LINKPOLL, &gmacdev->flags) && phydev) {
		phy_link_speed = phydev->speed;
		phy_link_duplex = phydev->duplex;
	} else {
		phy_link_speed = gmacdev->speed;
		phy_link_duplex = gmacdev->duplex_mode;
	}

	netdev_info(gmacdev->netdev,
			"%d Mbps %s Duplex\n",
			phy_link_speed, (phy_link_duplex == DUPLEX_FULL) ? "Full" : "Half");

	return 0;
}

/*
 * Sets the Mac address in to GMAC register.
 * This function sets the MAC address to the MAC register in question.
 * @param[in] pointer to nss_gmac_dev to populate mac dma and phy addresses.
 * @param[in] Register offset for Mac address high
 * @param[in] Register offset for Mac address low
 * @param[in] buffer containing mac address to be programmed.
 * @return void
 */
void nss_gmac_set_mac_addr(struct nss_gmac_dev *gmacdev, uint32_t mac_high,
			      uint32_t mac_low, uint8_t *mac_addr)
{
	uint32_t data;

	netdev_dbg(gmacdev->netdev, "Set addr %02x:%02x:%02x:%02x:%02x:%02x\n",
		      mac_addr[0], mac_addr[1], mac_addr[2],
		      mac_addr[3], mac_addr[4], mac_addr[5]);

	data = (mac_addr[5] << 8) | mac_addr[4] | 0x80000000;
	nss_gmac_write_reg(gmacdev->mac_base, mac_high, data);
	data = (mac_addr[3] << 24) | (mac_addr[2] << 16)
	    | (mac_addr[1] << 8) | mac_addr[0];
	nss_gmac_write_reg(gmacdev->mac_base, mac_low, data);
}

/*
 * This enables processing of received pause frame.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_tx_pause_enable(struct nss_gmac_dev *gmacdev)
{
	netdev_dbg(gmacdev->netdev, "%s: enable Tx flow control\n", __func__);

	if (gmacdev->data_plane_ops->pause_on_off(gmacdev->data_plane_ctx, 1)
							!= NSS_GMAC_SUCCESS) {
		netdev_dbg(gmacdev->netdev, "%s: tx flow control enable failed\n", __func__);
		return;
	}

	/*
	 * We set the DFF bit to convert GMAC DMA FIFO behavior from head
	 * drop to tail drop. This gives us extra room to store frames before
	 * we start sending pause frames and achieve 0 drops for high rate flows
	 * with 64 bytes frames.
	 */
	nss_gmac_set_reg_bits(gmacdev->dma_base, dma_control,
				dma_rx_frame_flush);

	nss_gmac_set_reg_bits(gmacdev->mac_base,
			      gmac_flow_control, NSS_GMAC_PAUSE_TIME | gmac_tx_flow_control);
}

/*
 * Disables multicast hash filtering.
 * When disabled GMAC performs perfect destination address filtering for unicast
 * frames, it compares DA field with the value programmed in DA register.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_unicast_hash_filter_disable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_frame_filter, gmac_ucast_hash_filter);
}

/*
 * Disable twokpe SUPPORT.
 * When disabled gmac does not support frames of length > 1522 bytes.
 * Giant frame error is reported in receive frame status
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_twokpe_frame_disable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_config, gmac_twokpe);
}

/*
 * Selects the GMII port.
 * When called GMII (1000Mbps) port is selected (programmable only in
 * 10/100/1000 Mbps configuration).
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_select_gmii(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_config, gmac_mii_gmii);
}

/*
 * Enable Jumbo frame support.
 * When Enabled GMAC supports jumbo frames of 9018/9022(VLAN tagged).
 * Giant frame error is not reported in receive frame status.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_jumbo_frame_enable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base,
			      gmac_config, gmac_jumbo_frame);
}

/*
 * Enable the transmission of frames on GMII/MII.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_tx_enable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base, gmac_config,
			      gmac_tx);
}

/*
 * Enables promiscous mode.
 * When enabled Address filter modules pass all incoming frames
 * regardless of their Destination and source addresses.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_promisc_enable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base,
			      gmac_frame_filter, gmac_promiscuous_mode);
}

/*
 * Disables Receive Own bit (Only in Half Duplex Mode).
 * When enaled GMAC disables the reception of frames when
 * gmii_txen_o is asserted.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_rx_own_disable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base,
			      gmac_config, gmac_rx_own);
}

/*
 * Enable twokpe frame support.
 * When Enabled GMAC supports jumbo frames of <= 2000 bytes.
 * Giant frame error is not reported in receive frame status.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_twokpe_frame_enable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base,
			      gmac_config, gmac_twokpe);
}

/**
 * @brief set gmac link status into expected state
 * @param[in] gmac_id
 * @param[in] link_state
 * @return void
 */
static void nss_gmac_link_status_set(uint32_t gmac_id, uint32_t link_state)
{
	struct nss_gmac_dev *gmac_dev = ctx.nss_gmac[gmac_id];
	if (gmac_dev == NULL)
		return;

	if (!test_bit(__NSS_GMAC_UP, &gmac_dev->flags))
		return;

	if (link_state == LINKDOWN && gmac_dev->link_state == LINKUP)
		nss_gmac_linkdown(gmac_dev);
	else if (link_state == LINKUP && gmac_dev->link_state == LINKDOWN)
		nss_gmac_linkup(gmac_dev);
}

/*
 * Enable the reception of frames on GMII/MII.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_rx_enable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base, gmac_config,
			      gmac_rx);
}

/*
 * Enables Multicast frames.
 * When enabled all multicast frames are passed.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_multicast_enable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base,
			      gmac_frame_filter, gmac_multicast_filter);
}

/*
 * Sets the GMAC core in Full-Duplex mode.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_set_full_duplex(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base,
			      gmac_config, gmac_duplex);
}

/*
 * Sets the GMAC in Normal mode.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_loopback_off(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_config, gmac_loopback);
}

/*
 * Disables the Deferral check in GMAC (Only in Half Duplex mode).
 * GMAC defers until the CRS signal goes inactive.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_deferral_check_disable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_config, gmac_deferral_check);
}

/*
 * GMAC doesnot strips the Pad/FCS field of incoming frames.
 * GMAC will pass all the incoming frames to Host unmodified.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_pad_crc_strip_disable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_config, gmac_pad_crc_strip);
}

/*
 * Enables Broadcast frames.
 * When enabled Address filtering module passes all incoming broadcast frames.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_broadcast_enable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_frame_filter, gmac_broadcast);
}

/*
 * disable processing of received pause frame.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_tx_pause_disable(struct nss_gmac_dev *gmacdev)
{
	netdev_dbg(gmacdev->netdev, "%s: disable Tx flow control\n", __func__);

	if (gmacdev->data_plane_ops->pause_on_off(gmacdev->data_plane_ctx, 0)
							!= NSS_GMAC_SUCCESS) {
		netdev_dbg(gmacdev->netdev, "%s: tx flow control disable failed\n", __func__);
		return;
	}

	nss_gmac_clear_reg_bits(gmacdev->dma_base, dma_control,
				dma_rx_frame_flush);

	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_flow_control, NSS_GMAC_PAUSE_TIME |
				gmac_tx_flow_control);

}

/*
 * Enables the Jabber frame support.
 * When enabled, GMAC disabled the jabber timer, and can transfer
 * 16,384 byte frames.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_jab_enable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base,
			      gmac_config, gmac_jabber);
}

/*
 * Disables detection of pause frames with stations unicast address.
 * When disabled GMAC only detects with the unique multicast address (802.3x).
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_unicast_pause_frame_detect_disable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
				gmac_flow_control, gmac_unicast_pause_frame);
}

/*
 * Configure and set Tx/Rx flow control
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_config_flow_control(struct nss_gmac_dev *gmacdev)
{
	uint16_t phyreg;

	netdev_dbg(gmacdev->netdev, "%s:\n", __func__);

	if (gmacdev->pause == 0) {
		nss_gmac_rx_pause_disable(gmacdev);
		nss_gmac_tx_pause_disable(gmacdev);
		return;
	}

	if (!test_bit(__NSS_GMAC_LINKPOLL, &gmacdev->flags))
		return;

	phyreg = nss_gmac_mii_rd_reg(gmacdev, gmacdev->phy_base, MII_LPA);

	if (phyreg & LPA_PAUSE_CAP) {
		/* link partner can do Tx/Rx flow control */
		netdev_dbg(gmacdev->netdev,
			      "%s: Link partner supports Tx/Rx flow control\n",
			      __func__);

		if (gmacdev->pause & FLOW_CTRL_RX)
			nss_gmac_rx_pause_enable(gmacdev);

		if (gmacdev->pause & FLOW_CTRL_TX)
			nss_gmac_tx_pause_enable(gmacdev);

		return;
	}

	if (phyreg & LPA_PAUSE_ASYM) {
		/* link partner can do Rx flow control only */
		netdev_dbg(gmacdev->netdev,
			      "%s: Link partner supports Rx flow control only\n",
			      __func__);

		/* disable Rx flow control as link
		 * partner cannot process pause frames
		 */
		nss_gmac_rx_pause_disable(gmacdev);
		if (gmacdev->pause & FLOW_CTRL_TX)
			nss_gmac_tx_pause_enable(gmacdev);

		return;
	}

	/* link partner does not support Tx/Rx flow control */
	netdev_dbg(gmacdev->netdev,
		      "%s: Link partner does not support Tx/Rx flow control\n",
		      __func__);
	nss_gmac_rx_flow_control_disable(gmacdev);
	nss_gmac_tx_flow_control_disable(gmacdev);
}

/*
 * Make MMC stats Clear-on-Read
 * @param[in] pointer to nss_gmac_dev.
 * @return void
 */
void nss_gmac_mmc_stats_cor_enable(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base, gmac_mmc_cntrl, gmac_mmc_cor);
}

/*
 * Disable Carrier sense.
 * When Disabled GMAC ignores CRS signal during frame transmission
 * in half duplex mode.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_disable_crs(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base,
			      gmac_config, gmac_disable_crs);
}

/*
 * Enable Carrier sense.
 * When Carrier sense is enabled GMAC generates Loss of Carier
 * or No carrier errors and can abort transmissions.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_enable_crs(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits(gmacdev->mac_base,
			      gmac_config, gmac_disable_crs);
}

/*
 * Programs the dma_tx_base_address with the Tx descriptor base address.
 * Tx Descriptor's base address is available in the gmacdev structure.
 * This function progrms the Dma Tx Base address with the starting
 * address of the descriptor ring or chain.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_init_tx_desc_base(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_write_reg(gmacdev->dma_base, dma_tx_base_addr,
					(uint32_t)gmacdev->tx_desc_dma);
}

/*
 * Mac initialization sequence.
 * This function calls the initialization routines
 * to initialize the GMAC register.
 * @param[in] pointer to nss_gmac_dev.
 * @return void
 */
void nss_gmac_mac_init(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_wd_enable(gmacdev);
	nss_gmac_jab_enable(gmacdev);
	nss_gmac_frame_burst_enable(gmacdev);
	nss_gmac_loopback_off(gmacdev);

	if (gmacdev->speed == SPEED_1000)
		nss_gmac_select_gmii(gmacdev);
	else
		nss_gmac_select_mii(gmacdev);

	if (gmacdev->duplex_mode == DUPLEX_FULL) {
		nss_gmac_set_full_duplex(gmacdev);
		nss_gmac_rx_own_enable(gmacdev);
		nss_gmac_retry_disable(gmacdev);
		nss_gmac_enable_crs(gmacdev);
	} else {
		nss_gmac_set_half_duplex(gmacdev);
		nss_gmac_rx_own_disable(gmacdev);
		nss_gmac_retry_enable(gmacdev);
		nss_gmac_disable_crs(gmacdev);
	}

	nss_gmac_pad_crc_strip_disable(gmacdev);
	nss_gmac_back_off_limit(gmacdev, gmac_backoff_limit0);
	nss_gmac_deferral_check_disable(gmacdev);

	nss_gmac_set_mac_addr(gmacdev, gmac_addr0_high,
			      gmac_addr0_low, gmacdev->netdev->dev_addr);

	/*Frame Filter Configuration */
	nss_gmac_frame_filter_enable(gmacdev);
	nss_gmac_set_pass_control(gmacdev, gmac_pass_control0);
	nss_gmac_broadcast_enable(gmacdev);
	nss_gmac_src_addr_filter_disable(gmacdev);
	nss_gmac_multicast_enable(gmacdev);
	gmacdev->netdev->flags |= IFF_ALLMULTI;
	nss_gmac_dst_addr_filter_normal(gmacdev);
	nss_gmac_multicast_hash_filter_disable(gmacdev);
	nss_gmac_promisc_enable(gmacdev);
	nss_gmac_unicast_hash_filter_disable(gmacdev);

	nss_gmac_ipc_offload_init(gmacdev);
	nss_gmac_mmc_stats_cor_enable(gmacdev);

	/* Flow Control Configuration */
	nss_gmac_unicast_pause_frame_detect_disable(gmacdev);
	nss_gmac_config_flow_control(gmacdev);

	nss_gmac_tx_enable(gmacdev);
	nss_gmac_rx_enable(gmacdev);
}

/*
 * Function to reset the GMAC core.
 * This reests the DMA and GMAC core. After reset all the
 * registers holds their respective reset value.
 * @param[in] pointer to nss_gmac_dev.
 * @return 0 on success else return the error status.
 */
void nss_gmac_reset(struct nss_gmac_dev *gmacdev)
{
	uint32_t data = 0;
	uint32_t reset_time __attribute__ ((unused)) = jiffies;
	struct nss_gmac_global_ctx *ctx;
	struct net_device *netdev = NULL;

	netdev = gmacdev->netdev;
	ctx = gmacdev->ctx;

	netdev_dbg(netdev, "%s: %s resetting...\n",
		      __func__, netdev->name);

	reset_time = jiffies;
	nss_gmac_write_reg(gmacdev->dma_base, dma_bus_mode, dma_reset_on);
	do {
		msleep(DEFAULT_LOOP_VARIABLE);
		data =
		    nss_gmac_read_reg(gmacdev->dma_base, dma_bus_mode);
	} while (data & dma_reset_on);

	msleep(1000);
	data = nss_gmac_read_reg(gmacdev->dma_base, dma_bus_mode);

	netdev_dbg(netdev, "GMAC reset completed in %d jiffies; dma_bus_mode - 0x%x\n", (int)(jiffies - reset_time), data);
}

/*
 * Disable the MMC Rx interrupt.
 * The MMC rx interrupts are masked out as per the mask specified.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] rx interrupt bit mask for which interrupts needs to be disabled.
 * @return returns void.
 */
void nss_gmac_disable_mmc_rx_interrupt(struct nss_gmac_dev *gmacdev,
						uint32_t mask)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base,
			      gmac_mmc_intr_mask_rx, mask);
}

/*
 * Function to program DMA bus mode register.
 * The Bus Mode register is programmed with the value given.
 * The bits to be set are bit wise or'ed and sent as the second
 * argument to this function.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] the data to be programmed.
 * @return 0 on success else return the error status.
 */
int32_t nss_gmac_dma_bus_mode_init(struct nss_gmac_dev *gmacdev,
							uint32_t init_value)
{
	nss_gmac_write_reg(gmacdev->dma_base, dma_bus_mode, init_value);
	return 0;
}

/*
 * Programs the dma_rx_base_address with the Rx descriptor base address.
 * Rx Descriptor's base address is available in the gmacdev structure.
 * This function progrms the Dma Rx Base address with the starting address
 * of the descriptor ring or chain.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_init_rx_desc_base(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_write_reg(gmacdev->dma_base, dma_rx_base_addr,
					(uint32_t)gmacdev->rx_desc_dma);
}

/*
 * Function to program DMA AXI bus mode register.
 * The Bus Mode register is programmed with the value given.
 * The bits to be set are bit wise or'ed and sent as the second
 * argument to this function.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] the data to be programmed.
 * @return 0 on success else return the error status.
 */
int32_t nss_gmac_dma_axi_bus_mode_init(struct nss_gmac_dev *gmacdev,
						uint32_t init_value)
{
	nss_gmac_write_reg(gmacdev->dma_base, dma_axi_bus_mode, init_value);
	return 0;
}

/**
 * @brief Function to change the Maximum Transfer Unit.
 * @param[in] pointer to net_device structure.
 * @param[in] New value for maximum frame size.
 * @return Returns 0 on success Errorcode on failure.
 */
int32_t nss_gmac_change_mtu(struct net_device *netdev, int32_t newmtu)
{
	struct nss_gmac_dev *gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);
	if (!gmacdev)
		return -EINVAL;

	if (newmtu > NSS_GMAC_JUMBO_MTU)
		return -EINVAL;

	if (gmacdev->data_plane_ops->change_mtu(gmacdev->data_plane_ctx, newmtu)
							 != NSS_GMAC_SUCCESS)
		return -EAGAIN;

	if (newmtu <= NSS_GMAC_NORMAL_FRAME_MTU) {
		nss_gmac_jumbo_frame_disable(gmacdev);
		nss_gmac_twokpe_frame_disable(gmacdev);
	} else if (newmtu <= NSS_GMAC_MINI_JUMBO_FRAME_MTU) {
		nss_gmac_jumbo_frame_disable(gmacdev);
		nss_gmac_twokpe_frame_enable(gmacdev);
	} else if (newmtu <= NSS_GMAC_FULL_JUMBO_FRAME_MTU) {
		nss_gmac_jumbo_frame_enable(gmacdev);
	}

	netdev->mtu = newmtu;
	return 0;
}

/*
 * Disable the MMC ipc rx checksum offload interrupt.
 * The MMC ipc rx checksum offload interrupts are masked out as
 * per the mask specified.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] rx interrupt bit mask for which interrupts needs to be disabled.
 * @return returns void.
 */
void nss_gmac_disable_mmc_ipc_rx_interrupt(struct nss_gmac_dev *gmacdev,
					   uint32_t mask)
{
	nss_gmac_set_reg_bits(gmacdev->mac_base,
			      gmac_mmc_rx_ipc_intr_mask, mask);
}

/**
 * @brief Notify linkup event to NSS
 * @param[in] pointer to gmac context
 * @return Returns void.
 */
static void nss_notify_linkup(struct nss_gmac_dev *gmacdev)
{
	uint32_t link = 0;

	if (!test_bit(__NSS_GMAC_UP, &gmacdev->flags))
		return;

	link = 0x1;
	if (gmacdev->speed == SPEED_1000)
		link |= 0x4;
	else if (gmacdev->speed == SPEED_100)
		link |= 0x2;

	gmacdev->data_plane_ops->link_state(gmacdev->data_plane_ctx, link);
}

/**
 * This function checks for completion of PHY init
 * and proceeds to initialize mac based on parameters
 * read from PHY registers. It indicates presence of carrier to OS.
 * @param[in] pointer to gmac context
 * @return Returns void.
 */
void nss_gmac_linkup(struct nss_gmac_dev *gmacdev)
{
	struct net_device *netdev = gmacdev->netdev;
	uint32_t gmac_tx_desc = 0, gmac_rx_desc = 0;
	uint32_t mode = NSS_GMAC_MODE0;

#ifdef RUMI_EMULATION_SUPPORT
	nss_gmac_spare_ctl(gmacdev);
#endif

	if (nss_gmac_check_phy_init(gmacdev) != 0) {
		gmacdev->link_state = LINKDOWN;
		return;
	}

	gmacdev->link_state = LINKUP;
	if (nss_gmac_dev_set_speed(gmacdev) != 0)
		return;

	if (gmacdev->first_linkup_done == 0) {
		nss_gmac_reset(gmacdev);
		nss_gmac_disable_interrupt_all(gmacdev);
		nss_gmac_clear_interrupt(gmacdev);

		/* Program Tx/Rx descriptor base addresses */
		nss_gmac_init_tx_desc_base(gmacdev);
		nss_gmac_init_rx_desc_base(gmacdev);
		nss_gmac_dma_bus_mode_init(gmacdev, dma_bus_mode_val);
		nss_gmac_dma_axi_bus_mode_init(gmacdev, dma_axi_bus_mode_val);
		nss_gmac_dma_control_init(gmacdev, dma_omr);
		nss_gmac_disable_mmc_tx_interrupt(gmacdev, 0xFFFFFFFF);
		nss_gmac_disable_mmc_rx_interrupt(gmacdev, 0xFFFFFFFF);
		nss_gmac_disable_mmc_ipc_rx_interrupt(gmacdev, 0xFFFFFFFF);

		/* Restore the Jumbo support settings as per corresponding
		 * interface mtu
		 */
		nss_gmac_change_mtu(gmacdev->netdev, gmacdev->netdev->mtu);
		gmacdev->first_linkup_done = 1;
	}

	nss_gmac_mac_init(gmacdev);

	if (gmacdev->data_plane_ops->open(gmacdev->data_plane_ctx, gmac_tx_desc,
				gmac_rx_desc, mode) != NSS_GMAC_SUCCESS) {
		netdev_dbg(netdev, "%s: data plane open command un-successful\n",
								__func__);
		gmacdev->link_state = LINKDOWN;
		return;
	}
	netdev_dbg(netdev, "%s: data plane open command successfully issued\n",
								__func__);

	nss_notify_linkup(gmacdev);

	netif_carrier_on(netdev);
}

/**
 * Save current state of link and
 * indicate absence of carrier to OS.
 * @param[in] nss_gmac_dev *
 * @return Returns void.
 */
void nss_gmac_linkdown(struct nss_gmac_dev *gmacdev)
{
	struct net_device *netdev = gmacdev->netdev;

	netdev_info(netdev, "Link down\n");

	if (test_bit(__NSS_GMAC_UP, &gmacdev->flags)) {
		netif_carrier_off(netdev);

		gmacdev->data_plane_ops->link_state(gmacdev->data_plane_ctx, 0);
	}
	gmacdev->link_state = LINKDOWN;
	gmacdev->duplex_mode = 0;
	gmacdev->speed = 0;
}

/**
 * @brief reset MACSEC IFG register
 * @param[in] gmac_id
 * @return void
 */
static void nss_gmac_ifg_reset(uint32_t gmac_id)
{
	uint32_t val = 0;
	uint32_t *nss_base = (uint32_t *)ctx.nss_base;

	val = nss_gmac_read_reg(nss_base, NSS_GMACn_CTL(gmac_id));
	val &= ~(IFG_MASK | GMAC_IFG_LIMIT(IFG_MASK));
	val |= (GMAC_IFG_CTL(GMAC_IFG) | GMAC_IFG_LIMIT(GMAC_IFG));
	nss_gmac_write_reg(nss_base, NSS_GMACn_CTL(gmac_id), val);
}

/**
 * @brief enable or disable MACSEC bypass function
 * @param[in] gmac_id
 * @param[in] enable
 * @return void
 */
void nss_macsec_bypass_en_set(uint32_t gmac_id, bool enable)
{
	uint32_t val = 0;
	uint32_t *nss_base = (uint32_t *)ctx.nss_base;
	struct nss_gmac_dev *gmac_dev = NULL;
	uint32_t link_reset_flag = 0;
	struct nss_gmac_speed_ctx gmac_speed_ctx = {0, 0};

	if ((gmac_id == 0) || (gmac_id > 3))
		return;

	gmac_dev = ctx.nss_gmac[gmac_id];
	if (gmac_dev == NULL)
		return;

	mutex_lock(&gmac_dev->link_mutex);

	/* If gmac is in link up state, it need to simulate link down event
	 * before setting IFG and simulate link up event after the operation
	 */
	if (gmac_dev->link_state == LINKUP)
		link_reset_flag = 1;

	/* simulate a gmac link down event */
	if (link_reset_flag)
		nss_gmac_link_status_set(gmac_id, LINKDOWN);

	/* Set MACSEC_IFG value */
	if (enable) {
		nss_gmac_ifg_reset(gmac_id);
	} else {
		val = nss_gmac_read_reg(nss_base, NSS_GMACn_CTL(gmac_id));
		val &= ~(IFG_MASK | GMAC_IFG_LIMIT(IFG_MASK));
		val |= (GMAC_IFG_CTL(MACSEC_IFG) | GMAC_IFG_LIMIT(MACSEC_IFG));
		nss_gmac_write_reg(nss_base, NSS_GMACn_CTL(gmac_id), val);
	}

	/* Enable/Disable MACSEC for related port */
	val = nss_gmac_read_reg(nss_base, NSS_MACSEC_CTL);
	val |= MACSEC_DP_RST_VAL;
	if (enable)
		val |= (1<<(gmac_id - 1));
	else
		val &= ~(1<<(gmac_id - 1));
	nss_gmac_write_reg(nss_base, NSS_MACSEC_CTL, val);

	/* simulate a gmac link up event */
	if (link_reset_flag)
		nss_gmac_link_status_set(gmac_id, LINKUP);

	mutex_unlock(&gmac_dev->link_mutex);

	/* Set MACSEC speed */
	gmac_speed_ctx.mac_id = gmac_dev->macid;
	gmac_speed_ctx.speed = gmac_dev->speed;
	blocking_notifier_call_chain(&nss_gmac_notifier_list,
					NSS_GMAC_SPEED_SET, &gmac_speed_ctx);
}
EXPORT_SYMBOL(nss_macsec_bypass_en_set);

void nss_gmac_link_state_change_notify_unregister(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&nss_gmac_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(nss_gmac_link_state_change_notify_unregister);

void nss_gmac_link_state_change_notify_register(struct notifier_block *nb)
{
	blocking_notifier_chain_register(&nss_gmac_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(nss_gmac_link_state_change_notify_register);

void nss_macsec_pre_init(void)
{
	uint32_t val = 0;
	uint32_t *nss_base = (uint32_t *)ctx.nss_base;

	/*
	 * Initialize wake and sleep counter values of
	 * MACSEC memory footswitch control.
	 */
	nss_gmac_write_reg(nss_base, NSS_MACSEC1_CORE_CLK_FS_CTL,
						MACSEC_CLK_FS_CTL_S_W_VAL);
	nss_gmac_write_reg(nss_base, NSS_MACSEC2_CORE_CLK_FS_CTL,
						MACSEC_CLK_FS_CTL_S_W_VAL);
	nss_gmac_write_reg(nss_base, NSS_MACSEC3_CORE_CLK_FS_CTL,
						MACSEC_CLK_FS_CTL_S_W_VAL);

	/* MACSEC reset */
	nss_gmac_write_reg(ctx.clk_ctl_base, MACSEC_CORE1_RESET, 1);
	nss_gmac_write_reg(ctx.clk_ctl_base, MACSEC_CORE2_RESET, 1);
	nss_gmac_write_reg(ctx.clk_ctl_base, MACSEC_CORE3_RESET, 1);
	msleep(100);

	/* Deassert MACSEC reset */
	nss_gmac_write_reg(ctx.clk_ctl_base, MACSEC_CORE1_RESET, 0);
	nss_gmac_write_reg(ctx.clk_ctl_base, MACSEC_CORE2_RESET, 0);
	nss_gmac_write_reg(ctx.clk_ctl_base, MACSEC_CORE3_RESET, 0);

	/* Enable MACSEC clocks */
	val = nss_gmac_read_reg(nss_base, NSS_ETH_CLK_GATE_CTL);
	val |= (MACSEC_CORE_CLKEN_VAL | MACSEC_GMII_RX_CLKEN_VAL |
						MACSEC_GMII_TX_CLKEN_VAL);
	nss_gmac_write_reg(nss_base, NSS_ETH_CLK_GATE_CTL, val);

	/* Bypass all MACSECs */
	nss_gmac_write_reg(nss_base, NSS_MACSEC_CTL, MACSEC_EXT_BYPASS_EN_MASK |
						MACSEC_DP_RST_VAL);
}
EXPORT_SYMBOL(nss_macsec_pre_init);

void nss_macsec_pre_exit(void)
{
	uint32_t *nss_base = (uint32_t *)ctx.nss_base;
	struct nss_gmac_dev *gmac_dev = NULL;
	uint32_t gmac_id = 0;
	uint32_t link_reset_flag = 0;

	/* MACSEC reset */
	nss_gmac_write_reg(ctx.clk_ctl_base, MACSEC_CORE1_RESET, 1);
	nss_gmac_write_reg(ctx.clk_ctl_base, MACSEC_CORE2_RESET, 1);
	nss_gmac_write_reg(ctx.clk_ctl_base, MACSEC_CORE3_RESET, 1);

	/* Bypass all MACSECs */
	nss_gmac_write_reg(nss_base, NSS_MACSEC_CTL,
				MACSEC_EXT_BYPASS_EN_MASK | MACSEC_DP_RST_VAL);

	/* Reset GMAC_IFG value */
	for (gmac_id = 1; gmac_id < 4; gmac_id++) {
		gmac_dev = ctx.nss_gmac[gmac_id];
		if (gmac_dev == NULL)
			continue;

		/*
		 * If gmac is in link up state, it need to simulate link down
		 * event before setting IFG and simulate link up event after the
		 * operation
		 */
		link_reset_flag = 0;

		mutex_lock(&gmac_dev->link_mutex);

		if (gmac_dev->link_state == LINKUP)
			link_reset_flag = 1;

		/* simulate a gmac link down event */
		if (link_reset_flag)
			nss_gmac_link_status_set(gmac_id, LINKDOWN);

		nss_gmac_ifg_reset(gmac_id);

		/* simulate a gmac link up event */
		if (link_reset_flag)
			nss_gmac_link_status_set(gmac_id, LINKUP);

		mutex_unlock(&gmac_dev->link_mutex);
	}
}
EXPORT_SYMBOL(nss_macsec_pre_exit);

module_init(nss_macsec_init);
module_exit(nss_macsec_fini);
#ifdef MODULE_LICENSE
MODULE_LICENSE("Dual BSD/GPL");
#endif
