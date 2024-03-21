/*
 * Copyright (c) 2016-2017, 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
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


#if defined(CONFIG_CPU_BIG_ENDIAN)
#include "hppe_servcode_reg_be.h"
#else
/**
 * @defgroup
 * @{
 */
#ifndef HPPE_SERVCODE_REG_H
#define HPPE_SERVCODE_REG_H

/*[table] SERVICE_TBL*/
#define SERVICE_TBL
#define SERVICE_TBL_ADDRESS 0x6000
#define SERVICE_TBL_NUM     256
#define SERVICE_TBL_INC     0x10
#define SERVICE_TBL_TYPE    REG_TYPE_RW
#define SERVICE_TBL_DEFAULT 0x0
	/*[field] BYPASS_BITMAP*/
	#define SERVICE_TBL_BYPASS_BITMAP
	#define SERVICE_TBL_BYPASS_BITMAP_OFFSET  0
	#define SERVICE_TBL_BYPASS_BITMAP_LEN     32
	#define SERVICE_TBL_BYPASS_BITMAP_DEFAULT 0x0
	/*[field] RX_COUNTING_EN*/
	#define SERVICE_TBL_RX_COUNTING_EN
	#define SERVICE_TBL_RX_COUNTING_EN_OFFSET  32
	#define SERVICE_TBL_RX_COUNTING_EN_LEN     1
	#define SERVICE_TBL_RX_COUNTING_EN_DEFAULT 0x0

struct service_tbl {
	a_uint32_t  bypass_bitmap:32;
	a_uint32_t  rx_counting_en:1;
	a_uint32_t  _reserved0:31;
};

union service_tbl_u {
	a_uint32_t val[2];
	struct service_tbl bf;
};

/*[table] IN_L2_SERVICE_TBL*/
#define IN_L2_SERVICE_TBL
#if defined(APPE)
#define IN_L2_SERVICE_TBL_ADDRESS 0x6000
#else
#define IN_L2_SERVICE_TBL_ADDRESS 0x4000
#endif
#define IN_L2_SERVICE_TBL_NUM     256
#define IN_L2_SERVICE_TBL_INC     0x10
#define IN_L2_SERVICE_TBL_TYPE    REG_TYPE_RW
#define IN_L2_SERVICE_TBL_DEFAULT 0x0
	/*[field] DST_PORT_ID_VALID*/
	#define IN_L2_SERVICE_TBL_DST_PORT_ID_VALID
	#define IN_L2_SERVICE_TBL_DST_PORT_ID_VALID_OFFSET  0
	#define IN_L2_SERVICE_TBL_DST_PORT_ID_VALID_LEN     1
	#define IN_L2_SERVICE_TBL_DST_PORT_ID_VALID_DEFAULT 0x0
	/*[field] DST_PORT_ID*/
	#define IN_L2_SERVICE_TBL_DST_PORT_ID
	#define IN_L2_SERVICE_TBL_DST_PORT_ID_OFFSET  1
	#define IN_L2_SERVICE_TBL_DST_PORT_ID_LEN     4
	#define IN_L2_SERVICE_TBL_DST_PORT_ID_DEFAULT 0x0
	/*[field] DIRECTION*/
	#define IN_L2_SERVICE_TBL_DIRECTION
	#define IN_L2_SERVICE_TBL_DIRECTION_OFFSET  5
	#define IN_L2_SERVICE_TBL_DIRECTION_LEN     1
	#define IN_L2_SERVICE_TBL_DIRECTION_DEFAULT 0x0
	/*[field] BYPASS_BITMAP*/
	#define IN_L2_SERVICE_TBL_BYPASS_BITMAP
	#define IN_L2_SERVICE_TBL_BYPASS_BITMAP_OFFSET  6
	#define IN_L2_SERVICE_TBL_BYPASS_BITMAP_LEN     24
	#define IN_L2_SERVICE_TBL_BYPASS_BITMAP_DEFAULT 0x0
	/*[field] RX_CNT_EN*/
	#define IN_L2_SERVICE_TBL_RX_CNT_EN
	#define IN_L2_SERVICE_TBL_RX_CNT_EN_OFFSET  30
	#define IN_L2_SERVICE_TBL_RX_CNT_EN_LEN     1
	#define IN_L2_SERVICE_TBL_RX_CNT_EN_DEFAULT 0x0
	/*[field] TX_CNT_EN*/
	#define IN_L2_SERVICE_TBL_TX_CNT_EN
	#define IN_L2_SERVICE_TBL_TX_CNT_EN_OFFSET  31
	#define IN_L2_SERVICE_TBL_TX_CNT_EN_LEN     1
	#define IN_L2_SERVICE_TBL_TX_CNT_EN_DEFAULT 0x0

struct in_l2_service_tbl {
	a_uint32_t  dst_port_id_valid:1;
	a_uint32_t  dst_port_id:4;
	a_uint32_t  direction:1;
	a_uint32_t  bypass_bitmap:24;
	a_uint32_t  rx_cnt_en:1;
	a_uint32_t  tx_cnt_en:1;
};

union in_l2_service_tbl_u {
	a_uint32_t val;
	struct in_l2_service_tbl bf;
};

/*[table] EG_SERVICE_TBL*/
#define EG_SERVICE_TBL
#if defined(APPE)
#define EG_SERVICE_TBL_ADDRESS 0x23000
#define EG_SERVICE_TBL_INC     0x10
#else
#define EG_SERVICE_TBL_ADDRESS 0xc000
#define EG_SERVICE_TBL_INC     0x8
#endif
#define EG_SERVICE_TBL_NUM     256
#define EG_SERVICE_TBL_TYPE    REG_TYPE_RW
#define EG_SERVICE_TBL_DEFAULT 0x0
	/*[field] FIELD_UPDATE_ACTION*/
	#define EG_SERVICE_TBL_FIELD_UPDATE_ACTION
	#define EG_SERVICE_TBL_FIELD_UPDATE_ACTION_OFFSET  0
	#define EG_SERVICE_TBL_FIELD_UPDATE_ACTION_LEN     32
	#define EG_SERVICE_TBL_FIELD_UPDATE_ACTION_DEFAULT 0x0
	/*[field] NEXT_SERVICE_CODE*/
	#define EG_SERVICE_TBL_NEXT_SERVICE_CODE
	#define EG_SERVICE_TBL_NEXT_SERVICE_CODE_OFFSET  32
	#define EG_SERVICE_TBL_NEXT_SERVICE_CODE_LEN     8
	#define EG_SERVICE_TBL_NEXT_SERVICE_CODE_DEFAULT 0x0
	/*[field] HW_SERVICES*/
	#define EG_SERVICE_TBL_HW_SERVICES
	#define EG_SERVICE_TBL_HW_SERVICES_OFFSET  40
	#define EG_SERVICE_TBL_HW_SERVICES_LEN     6
	#define EG_SERVICE_TBL_HW_SERVICES_DEFAULT 0x0
	/*[field] OFFSET_SEL*/
	#define EG_SERVICE_TBL_OFFSET_SEL
	#define EG_SERVICE_TBL_OFFSET_SEL_OFFSET  46
	#define EG_SERVICE_TBL_OFFSET_SEL_LEN     1
	#define EG_SERVICE_TBL_OFFSET_SEL_DEFAULT 0x0
	/*[field] TX_COUNTING_EN*/
	#define EG_SERVICE_TBL_TX_COUNTING_EN
	#define EG_SERVICE_TBL_TX_COUNTING_EN_OFFSET  47
	#define EG_SERVICE_TBL_TX_COUNTING_EN_LEN     1
	#define EG_SERVICE_TBL_TX_COUNTING_EN_DEFAULT 0x0
#if defined(MPPE)
	/*[field] IP_LENGTH_UPDATE*/
	#define EG_SERVICE_TBL_IP_LENGTH_UPDATE
	#define EG_SERVICE_TBL_IP_LENGTH_UPDATE_OFFSET  48
	#define EG_SERVICE_TBL_IP_LENGTH_UPDATE_LEN     1
	#define EG_SERVICE_TBL_IP_LENGTH_UPDATE_DEFAULT 0x0
	/*[field] ATH_HDR_INSERT_DIS*/
	#define EG_SERVICE_TBL_ATH_HDR_INSERT_DIS
	#define EG_SERVICE_TBL_ATH_HDR_INSERT_DIS_OFFSET  49
	#define EG_SERVICE_TBL_ATH_HDR_INSERT_DIS_LEN     1
	#define EG_SERVICE_TBL_ATH_HDR_INSERT_DIS_DEFAULT 0x0
	/*[field] ATH_HDR_TYPE*/
	#define EG_SERVICE_TBL_ATH_HDR_TYPE
	#define EG_SERVICE_TBL_ATH_HDR_TYPE_OFFSET  50
	#define EG_SERVICE_TBL_ATH_HDR_TYPE_LEN     3
	#define EG_SERVICE_TBL_ATH_HDR_TYPE_DEFAULT 0x0
	/*[field] ATH_FROM_CPU*/
	#define EG_SERVICE_TBL_ATH_FROM_CPU
	#define EG_SERVICE_TBL_ATH_FROM_CPU_OFFSET  53
	#define EG_SERVICE_TBL_ATH_FROM_CPU_LEN     1
	#define EG_SERVICE_TBL_ATH_FROM_CPU_DEFAULT 0x0
	/*[field] ATH_DISABLE_BIT*/
	#define EG_SERVICE_TBL_ATH_DISABLE_BIT
	#define EG_SERVICE_TBL_ATH_DISABLE_BIT_OFFSET  54
	#define EG_SERVICE_TBL_ATH_DISABLE_BIT_LEN     1
	#define EG_SERVICE_TBL_ATH_DISABLE_BIT_DEFAULT 0x0
	/*[field] ATH_PORT_BITMAP*/
	#define EG_SERVICE_TBL_ATH_PORT_BITMAP
	#define EG_SERVICE_TBL_ATH_PORT_BITMAP_OFFSET  55
	#define EG_SERVICE_TBL_ATH_PORT_BITMAP_LEN     7
	#define EG_SERVICE_TBL_ATH_PORT_BITMAP_DEFAULT 0x0
#endif

struct eg_service_tbl {
	a_uint32_t  field_update_action:32;
	a_uint32_t  next_service_code:8;
	a_uint32_t  hw_services:6;
	a_uint32_t  offset_sel:1;
	a_uint32_t  tx_counting_en:1;
#if defined(MPPE)
	a_uint32_t  ip_length_update:1;
	a_uint32_t  ath_hdr_insert_dis:1;
	a_uint32_t  ath_hdr_type:3;
	a_uint32_t  ath_from_cpu:1;
	a_uint32_t  ath_disable_bit:1;
	a_uint32_t  ath_port_bitmap:7;
	a_uint32_t  _reserved0:2;
#else
	a_uint32_t  _reserved0:16;
#endif
};

union eg_service_tbl_u {
	a_uint32_t val[2];
	struct eg_service_tbl bf;
};
#endif
#endif
