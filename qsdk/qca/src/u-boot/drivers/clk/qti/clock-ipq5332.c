// SPDX-License-Identifier: BSD-3-Clause
/*
 * Clock drivers for QTI IPQ5332
 *
 * (C) Copyright 2022 Sumit Garg <sumit.garg@linaro.org>
 *
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 *
 */

#include <common.h>
#include <clk-uclass.h>
#include <dm.h>
#include <errno.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <dm/device-internal.h>
#include "clock-snapdragon.h"

#include <dt-bindings/clock/gcc-ipq5332.h>

/* GPLL0 clock control registers */
#define GPLL0_STATUS_ACTIVE BIT(31)

static struct vote_clk gcc_blsp1_ahb_clk = {
	.cbcr_reg = BLSP1_AHB_CBCR,
	.ena_vote = APCS_CLOCK_BRANCH_ENA_VOTE,
	.vote_bit = BIT(4) | BIT(2) | BIT(1),
};

static const struct bcr_regs sdc_regs = {
	.cfg_rcgr = SDCC1_APPS_CFG_RCGR,
	.cmd_rcgr = SDCC1_APPS_CMD_RCGR,
	.M = SDCC1_APPS_M,
	.N = SDCC1_APPS_N,
	.D = SDCC1_APPS_D,
};

static const struct bcr_regs uart1_regs = {
	.cfg_rcgr = BLSP1_UART_APPS_CFG_RCGR(0),
	.cmd_rcgr = BLSP1_UART_APPS_CMD_RCGR(0),
	.M = BLSP1_UART_APPS_M(0),
	.N = BLSP1_UART_APPS_N(0),
	.D = BLSP1_UART_APPS_D(0),
};

static const struct bcr_regs uart2_regs = {
	.cfg_rcgr = BLSP1_UART_APPS_CFG_RCGR(1),
	.cmd_rcgr = BLSP1_UART_APPS_CMD_RCGR(1),
	.M = BLSP1_UART_APPS_M(1),
	.N = BLSP1_UART_APPS_N(1),
	.D = BLSP1_UART_APPS_D(1),
};

static const struct bcr_regs uart3_regs = {
	.cfg_rcgr = BLSP1_UART_APPS_CFG_RCGR(2),
	.cmd_rcgr = BLSP1_UART_APPS_CMD_RCGR(2),
	.M = BLSP1_UART_APPS_M(2),
	.N = BLSP1_UART_APPS_N(2),
	.D = BLSP1_UART_APPS_D(2),
};

static const struct bcr_regs qup1_spi_regs = {
	.cfg_rcgr = BLSP1_QUP_SPI_APPS_CFG_RCGR(0),
	.cmd_rcgr = BLSP1_QUP_SPI_APPS_CMD_RCGR(0),
	.M = BLSP1_QUP_SPI_APPS_M(0),
	.N = BLSP1_QUP_SPI_APPS_N(0),
	.D = BLSP1_QUP_SPI_APPS_D(0),
};

static const struct bcr_regs qup2_spi_regs = {
	.cfg_rcgr = BLSP1_QUP_SPI_APPS_CFG_RCGR(1),
	.cmd_rcgr = BLSP1_QUP_SPI_APPS_CMD_RCGR(1),
	.M = BLSP1_QUP_SPI_APPS_M(1),
	.N = BLSP1_QUP_SPI_APPS_N(1),
	.D = BLSP1_QUP_SPI_APPS_D(1),
};

static const struct bcr_regs qup3_spi_regs = {
	.cfg_rcgr = BLSP1_QUP_SPI_APPS_CFG_RCGR(2),
	.cmd_rcgr = BLSP1_QUP_SPI_APPS_CMD_RCGR(2),
	.M = BLSP1_QUP_SPI_APPS_M(2),
	.N = BLSP1_QUP_SPI_APPS_N(2),
	.D = BLSP1_QUP_SPI_APPS_D(2),
};

static const struct bcr_regs_v2 gcc_qdss_at_regs = {
	.cfg_rcgr = GCC_QDSS_AT_CFG_RCGR,
	.cmd_rcgr = GCC_QDSS_AT_CMD_RCGR,
};

static const struct bcr_regs_v2 gcc_pcnoc_bfdcd_regs = {
	.cfg_rcgr = GCC_PCNOC_BFDCD_CFG_RCGR,
	.cmd_rcgr = GCC_PCNOC_BFDCD_CMD_RCGR,
};

static const struct bcr_regs_v2 nss_cc_cfg_regs = {
	.cfg_rcgr = NSS_CC_CFG_CFG_RCGR,
	.cmd_rcgr = NSS_CC_CFG_CMD_RCGR,
};

static const struct bcr_regs_v2 gcc_system_noc_bfdcd_regs = {
	.cfg_rcgr = GCC_SYSTEM_NOC_BFDCD_CFG_RCGR,
	.cmd_rcgr = GCC_SYSTEM_NOC_BFDCD_CMD_RCGR,
};

static const struct bcr_regs_v2 nss_cc_ppe_regs = {
	.cfg_rcgr = NSS_CC_PPE_CFG_RCGR,
	.cmd_rcgr = NSS_CC_PPE_CMD_RCGR,
};

static const struct bcr_regs_v2 gcc_qpic_io_macro_regs = {
	.cfg_rcgr = GCC_QPIC_IO_MACRO_CFG_RCGR,
	.cmd_rcgr = GCC_QPIC_IO_MACRO_CMD_RCGR,
};

static const struct bcr_regs_v2 nss_cc_port1_rx_regs = {
	.cfg_rcgr = NSS_CC_PORT1_RX_CFG_RCGR,
	.cmd_rcgr = NSS_CC_PORT1_RX_CMD_RCGR,
	.div_cdivr = NSS_CC_PORT1_RX_DIV_CDIVR,
};

static const struct bcr_regs_v2 nss_cc_port1_tx_regs = {
	.cfg_rcgr = NSS_CC_PORT1_TX_CFG_RCGR,
	.cmd_rcgr = NSS_CC_PORT1_TX_CMD_RCGR,
	.div_cdivr = NSS_CC_PORT1_TX_DIV_CDIVR,
};

static const struct bcr_regs_v2 nss_cc_port2_rx_regs = {
	.cfg_rcgr = NSS_CC_PORT2_RX_CFG_RCGR,
	.cmd_rcgr = NSS_CC_PORT2_RX_CMD_RCGR,
	.div_cdivr = NSS_CC_PORT2_RX_DIV_CDIVR,
};

static const struct bcr_regs_v2 nss_cc_port2_tx_regs = {
	.cfg_rcgr = NSS_CC_PORT2_TX_CFG_RCGR,
	.cmd_rcgr = NSS_CC_PORT2_TX_CMD_RCGR,
	.div_cdivr = NSS_CC_PORT2_TX_DIV_CDIVR,
};

static int calc_div_for_nss_port_clk(struct clk *clk, ulong rate,
		int *div, int *cdiv)
{
	int pclk_rate = clk_get_parent_rate(clk);

	if (pclk_rate == CLK_125_MHZ) {
		switch (rate) {
		case CLK_2_5_MHZ:
			*div = 9;
			*cdiv = 9;
			break;
		case CLK_25_MHZ:
			*div = 9;
			break;
		case CLK_125_MHZ:
			*div = 1;
			break;
		default:
			return -EINVAL;
		}
	} else if (pclk_rate == CLK_312_5_MHZ) {
		switch (rate) {
		case CLK_2_5_MHZ:
			break;
		case CLK_25_MHZ:
			break;
		case CLK_78_125_MHZ:
			*div = 7;
			break;
		case CLK_125_MHZ:
			*div = 4;
			break;
		case CLK_156_25_MHZ:
			*div = 3;
			break;
		case CLK_312_5_MHZ:
			*div = 1;
			break;
		default:
			return -EINVAL;
		}
	} else
		return -EINVAL;

	return 0;
}

int msm_set_parent(struct clk *clk, struct clk* parent)
{
	assert(clk);
	assert(parent);
	clk->dev->parent = parent->dev;
	dev_set_uclass_priv(parent->dev, parent);
	return 0;
}

ulong msm_get_rate(struct clk *clk)
{
	return (ulong)clk->rate;
}

ulong msm_set_rate(struct clk *clk, ulong rate)
{
	struct msm_clk_priv *priv = dev_get_priv(clk->dev);
	int ret, src, div = 0, cdiv = 0;

	switch (clk->id) {

	case GCC_BLSP1_UART0_APPS_CLK:
		/* UART: 115200 */
		clk_rcg_set_rate_mnd(priv->base, &uart1_regs, 0, 36, 15625,
				     SDCC1_SRC_SEL_GPLL0_OUT_MAIN);
		break;
	case GCC_BLSP1_UART1_APPS_CLK:
		/* UART: 115200 */
		clk_rcg_set_rate_mnd(priv->base, &uart2_regs, 0, 36, 15625,
				     SDCC1_SRC_SEL_GPLL0_OUT_MAIN);
		break;
	case GCC_BLSP1_UART2_APPS_CLK:
		/* UART: 115200 */
		clk_rcg_set_rate_mnd(priv->base, &uart3_regs, 0, 36, 15625,
				     SDCC1_SRC_SEL_GPLL0_OUT_MAIN);
		break;
	case GCC_SDCC1_APPS_CLK:
		/* SDCC1: 200MHz */
		clk_rcg_set_rate_mnd(priv->base, &sdc_regs, 6, 0, 0,
				     SDCC1_SRC_SEL_GPLL2_OUT_MAIN);
		break;
	case GCC_BLSP1_QUP1_SPI_APPS_CLK:
		/* QUP1 SPI APPS CLK: 50MHz */
		clk_rcg_set_rate_mnd(priv->base, &qup1_spi_regs, 16, 0, 0,
				     BLSP1_QUP_SPI_SRC_SEL_GPLL0_OUT_MAIN);
		break;
	case GCC_BLSP1_QUP2_SPI_APPS_CLK:
		/* QUP2 SPI APPS CLK: 50MHz */
		clk_rcg_set_rate_mnd(priv->base, &qup2_spi_regs, 16, 0, 0,
				     BLSP1_QUP_SPI_SRC_SEL_GPLL0_OUT_MAIN);
		break;
	case GCC_BLSP1_QUP3_SPI_APPS_CLK:
		/* QUP3 SPI APPS CLK: 50MHz */
		clk_rcg_set_rate_mnd(priv->base, &qup3_spi_regs, 16, 0, 0,
				     BLSP1_QUP_SPI_SRC_SEL_GPLL0_OUT_MAIN);
		break;
	case GCC_QDSS_AT_CLK:
		clk_rcg_set_rate_v2(priv->base, &gcc_qdss_at_regs, 9, 0,
				QDSS_SRC_SEL_GPLL4_OUT_MAIN);
		break;
	case GCC_PCNOC_BFDCD_CLK:
		clk_rcg_set_rate_v2(priv->base, &gcc_pcnoc_bfdcd_regs, 15, 0,
				PCCNOC_BFDCD_SRC_SEL_GPLL0_OUT_MAIN);
		break;
	case GCC_SYSTEM_NOC_BFDCD_CLK:
		clk_rcg_set_rate_v2(priv->base, &gcc_system_noc_bfdcd_regs,
				8, 0,
				GCC_SYSTEM_NOC_BFDCD_SRC_SEL_GPLL4_OUT_MAIN);
		break;
	case GCC_QPIC_IO_MACRO_CLK:
		src = GCC_QPIC_IO_MACRO_SRC_SEL_GPLL0_OUT_MAIN;
		cdiv = 0;
		switch (rate) {
		case IO_MACRO_CLK_24_MHZ:
			src = GCC_QPIC_IO_MACRO_SRC_SEL_XO_CLK;
			div = 0;
			break;
		case IO_MACRO_CLK_100_MHZ:
			div = 15;
			break;
		case IO_MACRO_CLK_200_MHZ:
			div = 7;
			break;
		case IO_MACRO_CLK_228_MHZ:
			div = 6;
			break;
		case IO_MACRO_CLK_266_MHZ:
			div = 5;
			break;
		case IO_MACRO_CLK_320_MHZ:
			div = 4;
			break;
		default:
			return -EINVAL;
		}
		clk_rcg_set_rate_v2(priv->base, &gcc_qpic_io_macro_regs,
				div, cdiv, src);
		break;

	/*
	 * NSS controlled clock
	 */
	case NSS_CC_CFG_CLK:
		clk_rcg_set_rate_v2(priv->base, &nss_cc_cfg_regs, 15, 0,
				NSS_CC_SRC_SEL_GCC_GPLL0_OUT_AUX);
		break;
	case NSS_CC_PPE_CLK:
		clk_rcg_set_rate_v2(priv->base, &nss_cc_ppe_regs,
				1, 0, NSS_CC_PPE_SRC_SEL_CMN_PLL_NSS_CLK_200M);
		break;
	case NSS_CC_PORT1_RX_CLK:
		ret = calc_div_for_nss_port_clk(clk, rate, &div, &cdiv);
		if (ret < 0)
			return ret;
		clk_rcg_set_rate_v2(priv->base, &nss_cc_port1_rx_regs,
				div, cdiv,
				NSS_CC_PORT_RX_SRC_SEL_UNIPHY_NSS_RX_CLK);
		break;
	case NSS_CC_PORT1_TX_CLK:
		ret = calc_div_for_nss_port_clk(clk, rate, &div, &cdiv);
		if (ret < 0)
			return ret;
		clk_rcg_set_rate_v2(priv->base, &nss_cc_port1_tx_regs,
				div, cdiv,
				NSS_CC_PORT_TX_SRC_SEL_UNIPHY_NSS_TX_CLK);
		break;
	case NSS_CC_PORT2_RX_CLK:
		ret = calc_div_for_nss_port_clk(clk, rate, &div, &cdiv);
		if (ret < 0)
			return ret;
		clk_rcg_set_rate_v2(priv->base, &nss_cc_port2_rx_regs,
				div, cdiv,
				NSS_CC_PORT_RX_SRC_SEL_UNIPHY_NSS_RX_CLK);
		break;
	case NSS_CC_PORT2_TX_CLK:
		ret = calc_div_for_nss_port_clk(clk, rate, &div, &cdiv);
		if (ret < 0)
			return ret;
		clk_rcg_set_rate_v2(priv->base, &nss_cc_port2_tx_regs,
				div, cdiv,
				NSS_CC_PORT_TX_SRC_SEL_UNIPHY_NSS_TX_CLK);
		break;

	case UNIPHY0_NSS_RX_CLK:
	case UNIPHY0_NSS_TX_CLK:
	case UNIPHY1_NSS_RX_CLK:
	case UNIPHY1_NSS_TX_CLK:
		if (rate == CLK_125_MHZ)
			clk->rate = CLK_125_MHZ;
		else if (rate == CLK_312_5_MHZ)
			clk->rate = CLK_312_5_MHZ;
		else
			ret = -EINVAL;
		break;
	default:
		ret = -EINVAL;
	}

	return 0;
}

int msm_enable(struct clk *clk)
{
	struct msm_clk_priv *priv = dev_get_priv(clk->dev);

	switch (clk->id) {
	case GCC_BLSP1_AHB_CLK:
		clk_enable_vote_clk(priv->base, &gcc_blsp1_ahb_clk);
		break;
	case GCC_BLSP1_QUP1_SPI_APPS_CLK:
		clk_enable_cbc(priv->base + BLSP1_QUP_SPI_APPS_CBCR(0));
		break;
	case GCC_BLSP1_QUP2_SPI_APPS_CLK:
		clk_enable_cbc(priv->base + BLSP1_QUP_SPI_APPS_CBCR(1));
		break;
	case GCC_BLSP1_QUP3_SPI_APPS_CLK:
		clk_enable_cbc(priv->base + BLSP1_QUP_SPI_APPS_CBCR(2));
		break;
	case GCC_SDCC1_APPS_CLK:
		clk_enable_cbc(priv->base + SDCC1_APPS_CBCR);
		break;
	case GCC_QDSS_AT_CLK:
		clk_enable_cbc(priv->base + GCC_QDSS_AT_CBCR);
		break;
	case GCC_NSSCFG_CLK:
		clk_enable_cbc(priv->base + GCC_NSSCFG_CBCR);
		break;
	case GCC_NSSNOC_ATB_CLK:
		clk_enable_cbc(priv->base + GCC_NSSNOC_ATB_CBCR);
		break;
	case GCC_NSSNOC_QOSGEN_REF_CLK:
		clk_enable_cbc(priv->base + GCC_NSSNOC_QOSGEN_REF_CBCR);
		break;
	case GCC_NSSNOC_TIMEOUT_REF_CLK:
		clk_enable_cbc(priv->base + GCC_NSSNOC_TIMEOUT_REF_CBCR);
		break;
	case GCC_NSSCC_CLK:
		clk_enable_cbc(priv->base + GCC_NSSCC_CBCR);
		break;
	case GCC_NSSNOC_NSSCC_CLK:
		clk_enable_cbc(priv->base + GCC_NSSNOC_NSSCC_CBCR);
		break;
	case GCC_IM_SLEEP_CLK:
		clk_enable_cbc(priv->base + GCC_IM_SLEEP_CBCR);
		break;
	case GCC_CMN_12GPLL_AHB_CLK:
		clk_enable_cbc(priv->base + GCC_CMN_12GPLL_AHB_CBCR);
		break;
	case GCC_CMN_12GPLL_SYS_CLK:
		clk_enable_cbc(priv->base + GCC_CMN_12GPLL_SYS_CBCR);
		break;
	case GCC_UNIPHY0_SYS_CLK:
		clk_enable_cbc(priv->base + GCC_UNIPHY0_SYS_CBCR);
		break;
	case GCC_UNIPHY0_AHB_CLK:
		clk_enable_cbc(priv->base + GCC_UNIPHY0_AHB_CBCR);
		break;
	case GCC_UNIPHY1_SYS_CLK:
		clk_enable_cbc(priv->base + GCC_UNIPHY1_SYS_CBCR);
		break;
	case GCC_UNIPHY1_AHB_CLK:
		clk_enable_cbc(priv->base + GCC_UNIPHY1_AHB_CBCR);
		break;
	case GCC_MDIO_MASTER_AHB_CLK:
		clk_enable_cbc(priv->base + GCC_MDIO_MASTER_AHB_CBCR);
		break;
	case GCC_NSSNOC_SNOC_CLK:
		clk_enable_cbc(priv->base + GCC_NSSNOC_SNOC_CBCR);
		break;
	case GCC_NSSNOC_SNOC_1_CLK:
		clk_enable_cbc(priv->base + GCC_NSSNOC_SNOC_1_CBCR);
		break;
	case GCC_MEM_NOC_SNOC_AXI_CLK:
		clk_enable_cbc(priv->base + GCC_MEM_NOC_SNOC_AXI_CBCR);
		break;
	case GCC_QPIC_IO_MACRO_CLK:
		clk_enable_cbc(priv->base + GCC_QPIC_IO_MACRO_CBCR);
		break;
	case GCC_SDCC1_AHB_CLK:
		clk_enable_cbc(priv->base + GCC_SDCC1_AHB_CBCR);
		break;

	/*
	 * NSS controlled clock
	 */
	case NSS_CC_NSS_CSR_CLK:
		clk_enable_cbc(priv->base + NSS_CC_NSS_CSR_CBCR);
		break;
	case NSS_CC_NSSNOC_NSS_CSR_CLK:
		clk_enable_cbc(priv->base + NSS_CC_NSSNOC_NSS_CSR_CBCR);
		break;
	case NSS_CC_PORT1_MAC_CLK:
		clk_enable_cbc(priv->base + NSS_CC_PORT1_MAC_CBCR);
		break;
	case NSS_CC_PORT2_MAC_CLK:
		clk_enable_cbc(priv->base + NSS_CC_PORT2_MAC_CBCR);
		break;
	case NSS_CC_PPE_SWITCH_IPE_CLK:
		clk_enable_cbc(priv->base + NSS_CC_PPE_SWITCH_IPE_CBCR);
		break;
	case NSS_CC_PPE_SWITCH_CLK:
		clk_enable_cbc(priv->base + NSS_CC_PPE_SWITCH_CBCR);
		break;
	case NSS_CC_PPE_SWITCH_CFG_CLK:
		clk_enable_cbc(priv->base + NSS_CC_PPE_SWITCH_CFG_CBCR);
		break;
	case NSS_CC_PPE_EDMA_CLK:
		clk_enable_cbc(priv->base + NSS_CC_PPE_EDMA_CBCR);
		break;
	case NSS_CC_PPE_EDMA_CFG_CLK:
		clk_enable_cbc(priv->base + NSS_CC_PPE_EDMA_CFG_CBCR);
		break;
	case NSS_CC_NSSNOC_PPE_CLK:
		clk_enable_cbc(priv->base + NSS_CC_NSSNOC_PPE_CBCR);
		break;
	case NSS_CC_NSSNOC_PPE_CFG_CLK:
		clk_enable_cbc(priv->base + NSS_CC_NSSNOC_PPE_CFG_CBCR);
		break;
	case NSS_CC_PPE_SWITCH_BTQ_CLK:
		clk_enable_cbc(priv->base + NSS_CC_PPE_SWITCH_BTQ_CBCR);
		break;
	case NSS_CC_PORT1_RX_CLK:
		clk_enable_cbc(priv->base + NSS_CC_PORT1_RX_CBCR);
		break;
	case NSS_CC_PORT1_TX_CLK:
		clk_enable_cbc(priv->base + NSS_CC_PORT1_TX_CBCR);
		break;
	case NSS_CC_PORT2_RX_CLK:
		clk_enable_cbc(priv->base + NSS_CC_PORT2_RX_CBCR);
		break;
	case NSS_CC_PORT2_TX_CLK:
		clk_enable_cbc(priv->base + NSS_CC_PORT2_TX_CBCR);
		break;
	case NSS_CC_UNIPHY_PORT1_RX_CLK:
		clk_enable_cbc(priv->base + NSS_CC_UNIPHY_PORT1_RX_CBCR);
		break;
	case NSS_CC_UNIPHY_PORT1_TX_CLK:
		clk_enable_cbc(priv->base + NSS_CC_UNIPHY_PORT1_TX_CBCR);
		break;
	case NSS_CC_UNIPHY_PORT2_RX_CLK:
		clk_enable_cbc(priv->base + NSS_CC_UNIPHY_PORT2_RX_CBCR);
		break;
	case NSS_CC_UNIPHY_PORT2_TX_CLK:
		clk_enable_cbc(priv->base + NSS_CC_UNIPHY_PORT2_TX_CBCR);
		break;

	case UNIPHY0_NSS_RX_CLK:
	case UNIPHY0_NSS_TX_CLK:
	case UNIPHY1_NSS_RX_CLK:
	case UNIPHY1_NSS_TX_CLK:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
