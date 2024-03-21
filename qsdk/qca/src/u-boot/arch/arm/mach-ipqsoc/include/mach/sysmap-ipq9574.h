/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2022 Sumit Garg <sumit.garg@linaro.org>
 *
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 *
 */

#ifndef _MACH_SYSMAP_IPQ9574_H
#define _MACH_SYSMAP_IPQ9574_H

#define SDCC1_SRC_SEL_GPLL2_OUT_MAIN	(2 << 8)
#define SDCC1_SRC_SEL_GPLL0_OUT_MAIN	(1 << 8)

/* BLSP UART clock register */
#define BLSP1_AHB_CBCR			0x01004
#define BLSP1_UART1_BCR			0x02028
#define APCS_CLOCK_BRANCH_ENA_VOTE	0x0B004

#define BLSP1_UART_BCR(id)		((id < 1) ? \
					(BLSP1_UART1_BCR):\
					(BLSP1_UART1_BCR + (0x1000 * id)))

#define BLSP1_UART_APPS_CMD_RCGR(id)	(BLSP1_UART_BCR(id) + 0x04)
#define BLSP1_UART_APPS_CFG_RCGR(id)	(BLSP1_UART_BCR(id) + 0x08)
#define BLSP1_UART_APPS_M(id)		(BLSP1_UART_BCR(id) + 0x0c)
#define BLSP1_UART_APPS_N(id)		(BLSP1_UART_BCR(id) + 0x10)
#define BLSP1_UART_APPS_D(id)		(BLSP1_UART_BCR(id) + 0x14)
#define BLSP1_UART_APPS_CBCR(id)	(BLSP1_UART_BCR(id) + 0x18)

/* Uart clock control registers */
#define BLSP1_UART2_BCR			(0x3028)
#define BLSP1_UART2_APPS_CBCR		(0x3040)
#define BLSP1_UART2_APPS_CMD_RCGR	(0x302C)
#define BLSP1_UART2_APPS_CFG_RCGR	(0x3030)
#define BLSP1_UART2_APPS_M		(0x3034)
#define BLSP1_UART2_APPS_N		(0x3038)
#define BLSP1_UART2_APPS_D		(0x303C)

/* SD controller clock control registers */
#define SDCC1_BCR			(0x33000)
#define SDCC1_APPS_CMD_RCGR		(0x33004)
#define SDCC1_APPS_CFG_RCGR		(0x33008)
#define SDCC1_APPS_M			(0x3300C)
#define SDCC1_APPS_N			(0x33010)
#define SDCC1_APPS_D			(0x33014)
#define SDCC1_APPS_CBCR			(0x3302C)
#define GCC_SDCC1_AHB_CBCR		(0x33034)

/* BLSP QUP SPI clock register */
#define BLSP1_QUP1_SPI_BCR		0x02000

#define BLSP1_QUP_SPI_BCR(id)		((id < 1) ? \
					(BLSP1_QUP1_SPI_BCR):\
					(BLSP1_QUP1_SPI_BCR + (0x1000 * id)))

#define BLSP1_QUP_SPI_APPS_CMD_RCGR(id)	(BLSP1_QUP_SPI_BCR(id) + 0x04)
#define BLSP1_QUP_SPI_APPS_CFG_RCGR(id)	(BLSP1_QUP_SPI_BCR(id) + 0x08)
#define BLSP1_QUP_SPI_APPS_M(id)	(BLSP1_QUP_SPI_BCR(id) + 0x0c)
#define BLSP1_QUP_SPI_APPS_N(id)	(BLSP1_QUP_SPI_BCR(id) + 0x10)
#define BLSP1_QUP_SPI_APPS_D(id)	(BLSP1_QUP_SPI_BCR(id) + 0x14)
#define BLSP1_QUP_SPI_APPS_CBCR(id)	(BLSP1_QUP_SPI_BCR(id) + 0x20)

#define BLSP1_QUP_SPI_SRC_SEL_XO			(0 << 8)
#define BLSP1_QUP_SPI_SRC_SEL_GPLL0_OUT_MAIN		(1 << 8)

/* GCC clock control registers */
#define GCC_NSSNOC_ATB_CBCR				(0x17014)
#define GCC_NSSNOC_QOSGEN_REF_CBCR			(0x1701C)
#define GCC_NSSNOC_TIMEOUT_REF_CBCR			(0x17020)
#define GCC_NSSNOC_MEMNOC_CBCR				(0x17024)
#define GCC_NSSNOC_SNOC_CBCR				(0x17028)
#define GCC_NSSCFG_CBCR					(0x1702C)
#define GCC_MDIO_AHB_CBCR				(0x17040)
#define GCC_UNIPHY0_SYS_CBCR				(0x17048)
#define GCC_UNIPHY0_AHB_CBCR				(0x1704C)
#define GCC_UNIPHY_SYS_CBCR(id)		((id < 1) ? \
					(GCC_UNIPHY0_SYS_CBCR):\
					(GCC_UNIPHY0_SYS_CBCR + (0x10 * id)))
#define GCC_UNIPHY_AHB_CBCR(id)		((id < 1) ? \
					(GCC_UNIPHY0_AHB_CBCR):\
					(GCC_UNIPHY0_AHB_CBCR + (0x10 * id)))
#define GCC_NSSNOC_SNOC_1_CBCR				(0x1707C)
#define GCC_NSSNOC_MEM_NOC_1_CBCR			(0x17084)
#define GCC_UNIPHY_SYS_CMD_RCGR				(0x17090)
#define GCC_UNIPHY_SYS_CFG_RCGR				(0x17094)
#define GCC_MEM_NOC_NSSNOC_CBCR				(0x19014)
#define GCC_MEM_NOC_SNOC_AXI_CBCR			(0x19018)
#define GCC_CMN_12GPLL_AHB_CBCR				(0x3A004)
#define GCC_CMN_12GPLL_SYS_CBCR				(0x3A008)

#define GCC_NSSNOC_MEMNOC_BFDCD_CMD_RCGR		(0x17004)
#define GCC_NSSNOC_MEMNOC_BFDCD_CFG_RCGR		(0x17008)
#define GCC_QDSS_AT_CMD_RCGR				(0x2D004)
#define GCC_QDSS_AT_CFG_RCGR				(0x2D008)
#define GCC_SYSTEM_NOC_BFDCD_CMD_RCGR			(0x2E004)
#define GCC_SYSTEM_NOC_BFDCD_CFG_RCGR			(0x2E008)
#define GCC_PCNOC_BFDCD_CMD_RCGR			(0x31004)
#define GCC_PCNOC_BFDCD_CFG_RCGR			(0x31008)

#define GCC_NSSNOC_MEMNOC_BFDCD_SRC_SEL_GPLL0_OUT_MAIN	(1 << 8)
#define GCC_QDSS_AT_SRC_SEL_GPLL0_OUT_MAIN		(1 << 8)
#define GCC_PCNOC_BFDCD_SRC_SEL_GPLL0_OUT_MAIN		(1 << 8)
#define GCC_SYSTEM_NOC_BFDCD_SRC_SEL_GPLL4_OUT_MAIN	(2 << 8)

/* NSS clock control registers */
#define NSS_CC_PORT1_RX_CBCR				(0x281A0)
#define NSS_CC_PORT1_TX_CBCR				(0x281A4)
#define NSS_CC_PORT_RX_CBCR(id)		((id <= 1) ? \
					(NSS_CC_PORT1_RX_CBCR):\
					(NSS_CC_PORT1_RX_CBCR + (0x8*(id-1))))
#define NSS_CC_PORT_TX_CBCR(id)		((id <= 1) ? \
					(NSS_CC_PORT1_TX_CBCR):\
					(NSS_CC_PORT1_TX_CBCR + (0x8*(id-1))))
#define NSS_CC_NSS_CSR_CBCR				(0x281D0)
#define NSS_CC_NSSNOC_NSS_CSR_CBCR			(0x281D4)
#define NSS_CC_PPE_SWITCH_IPE_CBCR			(0x2822C)
#define NSS_CC_PPE_SWITCH_CBCR				(0x28230)
#define NSS_CC_PPE_SWITCH_CFG_CBCR			(0x28234)
#define NSS_CC_PPE_EDMA_CBCR				(0x28238)
#define NSS_CC_PPE_EDMA_CFG_CBCR			(0x2823C)
#define NSS_CC_CRYPTO_PPE_CBCR				(0x28240)
#define NSS_CC_NSSNOC_PPE_CBCR				(0x28244)
#define NSS_CC_NSSNOC_PPE_CFG_CBCR			(0x28248)
#define NSS_CC_PORT1_MAC_CBCR				(0x2824C)
#define NSS_CC_PORT_MAC_CBCR(id)	((id <= 1) ? \
					(NSS_CC_PORT1_MAC_CBCR):\
					(NSS_CC_PORT1_MAC_CBCR + (0x4*(id-1))))
#define NSS_CC_PPE_SWITCH_BTQ_CBCR			(0x2827C)
#define NSS_CC_UNIPHY_PORT1_RX_CBCR			(0x28904)
#define NSS_CC_UNIPHY_PORT1_TX_CBCR			(0x28908)
#define NSS_CC_UNIPHY_PORT_RX_CBCR(id)	((id <= 1) ? \
					(NSS_CC_UNIPHY_PORT1_RX_CBCR):\
					(NSS_CC_UNIPHY_PORT1_RX_CBCR +\
					 (0x8 * (id-1))))
#define NSS_CC_UNIPHY_PORT_TX_CBCR(id)	((id <= 1) ? \
					(NSS_CC_UNIPHY_PORT1_TX_CBCR):\
					(NSS_CC_UNIPHY_PORT1_TX_CBCR +\
					 (0x8 * (id-1))))

#define NSS_CC_CFG_CMD_RCGR				(0x28104)
#define NSS_CC_CFG_CFG_RCGR				(0x28108)
#define NSS_CC_PORT1_RX_CMD_RCGR			(0x28110)
#define NSS_CC_PORT1_RX_CFG_RCGR			(0x28114)
#define NSS_CC_PORT1_RX_DIV_CDIVR			(0x28118)
#define NSS_CC_PORT1_TX_CMD_RCGR			(0x2811C)
#define NSS_CC_PORT1_TX_CFG_RCGR			(0x28120)
#define NSS_CC_PORT1_TX_DIV_CDIVR			(0x28124)
#define NSS_CC_PORT_RX_CMD_RCGR(id)	((id <= 1) ? \
					(NSS_CC_PORT1_RX_CMD_RCGR):\
					(NSS_CC_PORT1_RX_CMD_RCGR +\
					 (0x18*(id-1))))
#define NSS_CC_PORT_RX_CFG_RCGR(id)	((id <= 1) ? \
					(NSS_CC_PORT1_RX_CFG_RCGR):\
					(NSS_CC_PORT1_RX_CFG_RCGR +\
					 (0x18*(id-1))))
#define NSS_CC_PORT_RX_DIV_CDIVR(id)	((id <= 1) ? \
					(NSS_CC_PORT1_RX_DIV_CDIVR):\
					(NSS_CC_PORT1_RX_DIV_CDIVR +\
					 (0x18*(id-1))))
#define NSS_CC_PORT_TX_CMD_RCGR(id)	((id <= 1) ? \
					(NSS_CC_PORT1_TX_CMD_RCGR):\
					(NSS_CC_PORT1_TX_CMD_RCGR +\
					 (0x18*(id-1))))
#define NSS_CC_PORT_TX_CFG_RCGR(id)	((id <= 1) ? \
					(NSS_CC_PORT1_TX_CFG_RCGR):\
					(NSS_CC_PORT1_TX_CFG_RCGR +\
					 (0x18*(id-1))))
#define NSS_CC_PORT_TX_DIV_CDIVR(id)	((id <= 1) ? \
					(NSS_CC_PORT1_TX_DIV_CDIVR):\
					(NSS_CC_PORT1_TX_DIV_CDIVR +\
					 (0x18*(id-1))))
#define NSS_CC_PPE_CMD_RCGR				(0x28204)
#define NSS_CC_PPE_CFG_RCGR				(0x28208)

#define NSS_CC_CFG_SRC_SEL_GCC_GPLL0_OUT_AUX		(2 << 8)
#define NSS_CC_PPE_SRC_SEL_BIAS_PLL_UBI_NC_CLK		(1 << 8)
#define NSS_CC_PORT1_RX_SRC_SEL_UNIPHY0_NSS_RX_CLK	(2 << 8)
#define NSS_CC_PORT1_TX_SRC_SEL_UNIPHY0_NSS_TX_CLK	(3 << 8)
#define NSS_CC_PORT5_RX_SRC_SEL_UNIPHY0_NSS_RX_CLK	(2 << 8)
#define NSS_CC_PORT5_TX_SRC_SEL_UNIPHY0_NSS_TX_CLK	(3 << 8)
#define NSS_CC_PORT5_RX_SRC_SEL_UNIPHY1_NSS_RX_CLK	(4 << 8)
#define NSS_CC_PORT5_TX_SRC_SEL_UNIPHY1_NSS_TX_CLK	(5 << 8)
#define NSS_CC_PORT6_RX_SRC_SEL_UNIPHY2_NSS_RX_CLK	(2 << 8)
#define NSS_CC_PORT6_TX_SRC_SEL_UNIPHY2_NSS_TX_CLK	(3 << 8)

#define CLK_1_25_MHZ			(1250000UL)
#define CLK_2_5_MHZ			(2500000UL)
#define CLK_12_5_MHZ			(12500000UL)
#define CLK_25_MHZ			(25000000UL)
#define CLK_78_125_MHZ			(78125000UL)
#define CLK_125_MHZ			(125000000UL)
#define CLK_156_25_MHZ			(156250000UL)
#define CLK_312_5_MHZ			(312500000UL)

/*
 * QTI SPI NAND clock
 */
#define GCC_QPIC_IO_MACRO_CMD_RCGR	(0x32004)
#define GCC_QPIC_IO_MACRO_CFG_RCGR      (0x32008)
#define GCC_QPIC_IO_MACRO_CBCR          (0x3200C)

#define IO_MACRO_CLK_320_MHZ            (320000000)
#define IO_MACRO_CLK_266_MHZ            (266000000)
#define IO_MACRO_CLK_228_MHZ            (228000000)
#define IO_MACRO_CLK_200_MHZ            (200000000)
#define IO_MACRO_CLK_100_MHZ            (100000000)
#define IO_MACRO_CLK_24_MHZ		(24000000)

#define GCC_QPIC_IO_MACRO_SRC_SEL_XO_CLK		(0 << 8)
#define GCC_QPIC_IO_MACRO_SRC_SEL_GPLL0_OUT_MAIN	(1 << 8)

#endif
