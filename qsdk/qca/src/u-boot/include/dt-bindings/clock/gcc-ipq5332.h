/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _DT_BINDINGS_CLK_GCC_IPQ5332_H
#define _DT_BINDINGS_CLK_GCC_IPQ5332_H

/* GCC controlled clock IDs */
#define GCC_BLSP1_UART0_APPS_CLK			0
#define GCC_BLSP1_UART1_APPS_CLK			1
#define GCC_BLSP1_UART2_APPS_CLK			2
#define GCC_BLSP1_AHB_CLK				3
#define GCC_SDCC1_APPS_CLK				4
#define GCC_SDCC1_AHB_CLK				5
#define GCC_BLSP1_QUP1_SPI_APPS_CLK			6
#define GCC_BLSP1_QUP2_SPI_APPS_CLK			7
#define GCC_BLSP1_QUP3_SPI_APPS_CLK			8
#define GCC_QDSS_AT_CLK					9
#define GCC_NSSCFG_CLK					10
#define GCC_NSSNOC_ATB_CLK				11
#define GCC_NSSNOC_QOSGEN_REF_CLK			12
#define GCC_NSSNOC_TIMEOUT_REF_CLK			13
#define GCC_PCNOC_BFDCD_CLK				14
#define GCC_SYSTEM_NOC_BFDCD_CLK			15
#define GCC_NSSCC_CLK					16
#define GCC_NSSNOC_NSSCC_CLK				17
#define GCC_IM_SLEEP_CLK				18
#define GCC_CMN_12GPLL_AHB_CLK				19
#define GCC_CMN_12GPLL_SYS_CLK				20
#define GCC_UNIPHY0_SYS_CLK				21
#define GCC_UNIPHY0_AHB_CLK				22
#define GCC_UNIPHY1_SYS_CLK				23
#define GCC_UNIPHY1_AHB_CLK				24
#define GCC_MDIO_MASTER_AHB_CLK				25
#define GCC_NSSNOC_SNOC_CLK				26
#define GCC_NSSNOC_SNOC_1_CLK				27
#define GCC_MEM_NOC_SNOC_AXI_CLK			28
#define GCC_QPIC_IO_MACRO_CLK				29

/* NSS controlled clock IDs */
#define NSS_CC_CFG_CLK					100
#define NSS_CC_PPE_CLK					101
#define NSS_CC_NSS_CSR_CLK				102
#define NSS_CC_NSSNOC_NSS_CSR_CLK			103
#define NSS_CC_PORT1_MAC_CLK				104
#define NSS_CC_PORT2_MAC_CLK				105
#define NSS_CC_PPE_SWITCH_IPE_CLK			106
#define NSS_CC_PPE_SWITCH_CLK				107
#define NSS_CC_PPE_SWITCH_CFG_CLK			108
#define NSS_CC_PPE_EDMA_CLK				109
#define NSS_CC_PPE_EDMA_CFG_CLK				110
#define NSS_CC_NSSNOC_PPE_CLK				111
#define NSS_CC_NSSNOC_PPE_CFG_CLK			112
#define NSS_CC_PPE_SWITCH_BTQ_CLK			113
#define NSS_CC_PORT1_RX_CLK				114
#define NSS_CC_PORT1_TX_CLK				115
#define NSS_CC_PORT2_RX_CLK				116
#define NSS_CC_PORT2_TX_CLK				117
#define NSS_CC_UNIPHY_PORT1_RX_CLK			118
#define NSS_CC_UNIPHY_PORT1_TX_CLK			119
#define NSS_CC_UNIPHY_PORT2_RX_CLK			120
#define NSS_CC_UNIPHY_PORT2_TX_CLK			121

#define UNIPHY0_NSS_RX_CLK				200
#define UNIPHY0_NSS_TX_CLK				201
#define UNIPHY1_NSS_RX_CLK				202
#define UNIPHY1_NSS_TX_CLK				203

#endif
