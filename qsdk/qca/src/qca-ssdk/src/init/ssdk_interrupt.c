/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * Copyright (c) 2022, Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all copies.
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#include "fal_init.h"
#include "fal_reg_access.h"
#include "sw.h"
#include "ssdk_init.h"
#include "fal_init.h"
#include <linux/phy.h>
#include <linux/kernel.h>
#include <linux/kconfig.h>
#include <linux/version.h>
#include <linux/module.h>
#include <generated/autoconf.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/if_arp.h>
#include <linux/inetdevice.h>
#include <linux/netdevice.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/gpio.h>
#if defined(ISIS)
#include <isis/isis_reg.h>
#elif defined(ISISC)
#include <isisc/isisc_reg.h>
#else
#include <dess/dess_reg.h>
#endif
#if defined(CONFIG_OF) && (LINUX_VERSION_CODE >= KERNEL_VERSION(4,1,0))
#include <linux/of.h>
#elif defined(CONFIG_OF) && (LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0))
#include <linux/of.h>
#include <drivers/leds/leds-ipq40xx.h>
#include <linux/of_platform.h>
#include <linux/reset.h>
#else
#include <linux/ar8216_platform.h>
#endif
#include "ssdk_plat.h"
#include "ref_port_ctrl.h"
#include "fal_misc.h"
#include "fal_port_ctrl.h"
#ifdef MHT
#include "ssdk_mht.h"
#endif

extern void qca_ar8327_sw_mac_polling_task(struct qca_phy_priv *priv);

static int qca_phy_disable_intr(struct qca_phy_priv *priv)
{
	a_uint32_t  port_id = 0, phy_intr_status = 0;

	for(port_id = SSDK_PHYSICAL_PORT1; port_id < priv->ports; port_id++)
	{
		fal_intr_port_link_mask_set(priv->device_id, port_id, 0);
		fal_intr_port_link_status_get(priv->device_id, port_id,
			&phy_intr_status);
	}

	return 0;
}

static int qca_mac_disable_intr(struct qca_phy_priv *priv)
{
	a_uint32_t port_id = 0;

	for(port_id = SSDK_PHYSICAL_PORT1; port_id < priv->ports;  port_id++)
	{
		fal_intr_mask_mac_linkchg_set(priv->device_id, port_id, A_FALSE);
	}

	fal_intr_mask_set(priv->device_id, 0);
	fal_intr_status_mac_linkchg_clear(priv->device_id);

	return 0;
}

static int qca_phy_enable_intr(struct qca_phy_priv *priv)
{
	a_uint32_t port_id = 0, phy_intr_status = 0;

	for(port_id = SSDK_PHYSICAL_PORT1; port_id < priv->ports; port_id++)
	{
		fal_intr_port_link_status_get(priv->device_id, port_id,
			&phy_intr_status);
		/*enable link change intr*/
		fal_intr_port_link_mask_set(priv->device_id, port_id,
			FAL_PHY_INTR_STATUS_UP_CHANGE | FAL_PHY_INTR_STATUS_DOWN_CHANGE);
	}

	return 0;
}

int qca_mac_enable_intr(struct qca_phy_priv *priv)
{
	a_uint32_t port_id = 0;

	/*enable link change intr*/
	for(port_id = SSDK_PHYSICAL_PORT1; port_id < priv->ports;  port_id++)
	{
		fal_intr_mask_mac_linkchg_set(priv->device_id, port_id, A_TRUE);
	}

	fal_intr_mask_set(priv->device_id, FAL_SWITCH_INTR_LINK_STATUS);

	return 0;
}

static int qca_phy_clean_intr(struct qca_phy_priv *priv)
{
	a_uint32_t  port_id = 0, phy_intr_status = 0;

	for(port_id = SSDK_PHYSICAL_PORT1; port_id < priv->ports;  port_id++)
	{
		fal_intr_port_link_status_get(priv->device_id, port_id,
			&phy_intr_status);
	}

	return 0;
}

static int qca_mac_clean_intr(struct qca_phy_priv *priv)
{
	fal_intr_status_mac_linkchg_clear(priv->device_id);

	return 0;
}

static void
qca_link_change_task(struct qca_phy_priv *priv)
{
	SSDK_DEBUG("qca_link_change_task is running\n");
	mutex_lock(&priv->qm_lock);

	switch(priv->version)
	{
#ifdef ISISC
		case QCA_VER_AR8337:
			qca_ar8327_sw_mac_polling_task(priv);
			break;
#endif
#ifdef MHT
		case QCA_VER_MHT:
			qca_mht_sw_mac_polling_task(priv);
			break;
#endif
		default:
			break;
	}

	mutex_unlock(&priv->qm_lock);

	return;
}

static void
qca_intr_workqueue_task(struct work_struct *work)
{
	a_uint32_t intr_mask;
	struct qca_phy_priv *priv = container_of(work, struct qca_phy_priv,  intr_workqueue);

	fal_intr_mask_get(priv->device_id, &intr_mask);
	qca_phy_clean_intr(priv);
	qca_mac_clean_intr(priv);
	SSDK_DEBUG("intr_mask:%x, priv->version:%x\n", intr_mask, priv->version);
	if((intr_mask & FAL_SWITCH_INTR_LINK_STATUS))
		qca_link_change_task(priv);
	enable_irq(priv->link_interrupt_no);
}

 static irqreturn_t  qca_link_intr_handle(int irq, void *phy_priv)
 {
	struct qca_phy_priv *priv = (struct qca_phy_priv *)phy_priv;

	 disable_irq_nosync(irq);
	 schedule_work(&priv->intr_workqueue);
        SSDK_DEBUG("irq number is :%x\n",irq);

	 return IRQ_HANDLED;
 }

 int qca_intr_init(struct qca_phy_priv *priv)
{
	SSDK_DEBUG("start to  init the interrupt!\n");
	mutex_init(&priv->qm_lock);
	INIT_WORK(&priv->intr_workqueue, qca_intr_workqueue_task);
	qca_phy_disable_intr(priv);
	qca_mac_disable_intr(priv);
	if(request_irq(priv->link_interrupt_no, qca_link_intr_handle, priv->interrupt_flag, priv->link_intr_name, priv))
		return -1;
	qca_phy_enable_intr(priv);
	qca_mac_enable_intr(priv);

	return 0;
}

