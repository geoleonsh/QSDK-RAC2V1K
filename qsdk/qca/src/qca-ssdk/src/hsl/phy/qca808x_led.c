/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
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

#include "sw.h"
#include "hsl_phy.h"
#include "ssdk_plat.h"
#include "qca808x_phy.h"
#include "qca808x_led.h"
#ifdef MHT
#include "ssdk_mht_pinctrl.h"
#include "mht_sec_ctrl.h"
#endif

static sw_error_t
_qca808x_phy_led_active_set(a_uint32_t dev_id, a_uint32_t phy_addr,
	led_ctrl_pattern_t *pattern)
{
	a_uint16_t phy_data = 0;

	if(pattern->map & BIT(LED_ACTIVE_HIGH))
	{
		phy_data |= QCA808X_PHY_MMD7_LED_POLARITY_MASK;
	}

	return hsl_phy_modify_mmd(dev_id, phy_addr, A_TRUE, QCA808X_PHY_MMD7_NUM,
		QCA808X_PHY_MMD7_LED_POLARITY_CTRL, QCA808X_PHY_MMD7_LED_POLARITY_MASK,
		phy_data);
}

static sw_error_t
_qca808x_phy_led_active_get(a_uint32_t dev_id, a_uint32_t phy_addr,
	led_ctrl_pattern_t *pattern)
{
	a_uint16_t phy_data = 0;

	phy_data = hsl_phy_mmd_reg_read(dev_id, phy_addr, A_TRUE, QCA808X_PHY_MMD7_NUM,
		QCA808X_PHY_MMD7_LED_POLARITY_CTRL);
	PHY_RTN_ON_READ_ERROR(phy_data);
	if(phy_data & QCA808X_PHY_MMD7_LED_POLARITY_MASK)
	{
		pattern->map |= BIT(LED_ACTIVE_HIGH);
	}

	return SW_OK;
}

static sw_error_t
_qca808x_phy_led_pattern_map_from_phy(a_uint32_t dev_id, a_uint32_t phy_addr,
	led_ctrl_pattern_t *pattern, a_uint16_t *phy_data)
{
	if (qca808x_phy_2500caps(dev_id, phy_addr) == A_TRUE)
	{
		if(*phy_data & QCA808X_PHY_LINK_2500M_LIGHT_EN)
		{
			pattern->map |= BIT(LINK_2500M_LIGHT_EN);
		}
	}
	if(*phy_data & QCA808X_PHY_LINK_1000M_LIGHT_EN)
	{
		pattern->map |= BIT(LINK_1000M_LIGHT_EN);
	}
	if(*phy_data & QCA808X_PHY_LINK_100M_LIGHT_EN)
	{
		pattern->map |= BIT(LINK_100M_LIGHT_EN);
	}
	if(*phy_data & QCA808X_PHY_LINK_10M_LIGHT_EN)
	{
		pattern->map |= BIT(LINK_10M_LIGHT_EN);
	}
	if (*phy_data & QCA808X_PHY_RX_TRAFFIC_BLINK_EN)
	{
		pattern->map |= BIT(RX_TRAFFIC_BLINK_EN);
	}
	if (*phy_data & QCA808X_PHY_TX_TRAFFIC_BLINK_EN)
	{
		pattern->map |= BIT(TX_TRAFFIC_BLINK_EN);
	}

	return SW_OK;
}

static sw_error_t
_qca808x_phy_led_pattern_map_to_phy(a_uint32_t dev_id, a_uint32_t phy_addr,
	led_ctrl_pattern_t *pattern, a_uint32_t *led_map)
{
	if (qca808x_phy_2500caps(dev_id, phy_addr) == A_TRUE)
	{
		if (pattern->map & BIT(LINK_2500M_LIGHT_EN))
		{
			*led_map |=  QCA808X_PHY_LINK_2500M_LIGHT_EN;
		}
	}
	if (pattern->map & BIT(LINK_1000M_LIGHT_EN))
	{
		*led_map |=  QCA808X_PHY_LINK_1000M_LIGHT_EN;
	}
	if (pattern->map & BIT(LINK_100M_LIGHT_EN))
	{
		*led_map |=  QCA808X_PHY_LINK_100M_LIGHT_EN;
	}
	if (pattern->map & BIT(LINK_10M_LIGHT_EN))
	{
		*led_map |=  QCA808X_PHY_LINK_10M_LIGHT_EN;
	}
	if (pattern->map & BIT(RX_TRAFFIC_BLINK_EN))
	{
		*led_map |=  QCA808X_PHY_RX_TRAFFIC_BLINK_EN;
	}
	if (pattern->map & BIT(TX_TRAFFIC_BLINK_EN))
	{
		*led_map |=  QCA808X_PHY_TX_TRAFFIC_BLINK_EN;
	}

	return SW_OK;
}

/******************************************************************************
*
* qca808x_phy_led_ctrl_pattern_set
*
*/
sw_error_t
qca808x_phy_led_ctrl_pattern_set(a_uint32_t dev_id, a_uint32_t phy_addr,
	led_ctrl_pattern_t *pattern)
{
	sw_error_t rv = SW_OK;
	a_uint32_t source_id = 0;

	if(LED_PATTERN_MAP_EN != pattern->mode)
	{
		SSDK_ERROR("led mode %d is not supported\n", pattern->mode);
		return SW_NOT_SUPPORTED;
	}
	for(source_id = QCA808X_PHY_LED_SOURCE0; source_id <= QCA808X_PHY_LED_SOURCE2;
		source_id++)
	{
		/*three source use the same pattern*/
		rv = qca808x_phy_led_ctrl_source_set (dev_id, phy_addr, source_id, pattern);
		PHY_RTN_ON_ERROR(rv);
	}

	return rv;
}

/******************************************************************************
*
* qca808x_phy_led_ctrl_pattern_get
*
*/
sw_error_t
qca808x_phy_led_ctrl_pattern_get(a_uint32_t dev_id, a_uint32_t phy_addr,
	led_ctrl_pattern_t *pattern)
{
	/*three source use the same pattern*/
	return qca808x_phy_led_ctrl_source_get(dev_id, phy_addr,
		QCA808X_PHY_LED_SOURCE0, pattern);
}

/******************************************************************************
*
* qca808x_phy_led_source_pattern_set
*
*/
#ifdef MHT
#define QCA8084_LED_FUNC(lend_func, mht_port_id, source_id) \
{                                                           \
    if(mht_port_id == SSDK_PHYSICAL_PORT1)                  \
        lend_func = MHT_PIN_FUNC_P0_LED_##source_id;        \
    else if(mht_port_id == SSDK_PHYSICAL_PORT2)             \
        lend_func = MHT_PIN_FUNC_P1_LED_##source_id;        \
    else if(mht_port_id == SSDK_PHYSICAL_PORT3)             \
        lend_func = MHT_PIN_FUNC_P2_LED_##source_id;        \
    else                                                    \
        lend_func = MHT_PIN_FUNC_P3_LED_##source_id;        \
}

static sw_error_t
qca8084_phy_led_ctrl_source_pin_cfg(a_uint32_t dev_id, a_uint32_t phy_addr,
	a_uint32_t source_id)
{
	sw_error_t rv = SW_OK;
	a_uint32_t mht_port_id = 0, led_start_pin = 0, led_pin = 0, led_func = 0;

	/*get the led pin and led func for mht port*/
	rv = qca_mht_port_id_get(dev_id, phy_addr, &mht_port_id);
	PHY_RTN_ON_ERROR(rv);
	if(source_id == QCA808X_PHY_LED_SOURCE0)
	{
		led_start_pin = 2;
		QCA8084_LED_FUNC(led_func, mht_port_id, 0)
	}
	else if(source_id == QCA808X_PHY_LED_SOURCE1)
	{
		led_start_pin = 16;
		QCA8084_LED_FUNC(led_func, mht_port_id, 1)
	}
	else
	{
		led_start_pin = 6;
		QCA8084_LED_FUNC(led_func, mht_port_id, 2)
	}
	led_pin = led_start_pin + (mht_port_id - SSDK_PHYSICAL_PORT1);
	SSDK_DEBUG("mht port:%d, led_pin:%d, led_func:%d", mht_port_id,
		led_pin, led_func);
	rv = mht_gpio_pin_mux_set(dev_id, led_pin, led_func);

	return rv;
}
#endif
sw_error_t
qca808x_phy_led_ctrl_source_set(a_uint32_t dev_id, a_uint32_t phy_addr,
	a_uint32_t source_id, led_ctrl_pattern_t *pattern)
{
	sw_error_t rv = SW_OK;
	a_uint32_t led_map = 0;
	a_uint16_t led_mmd_addr = 0;

	if(LED_PATTERN_MAP_EN != pattern->mode)
	{
		return SW_NOT_SUPPORTED;
	}

	rv = _qca808x_phy_led_active_set(dev_id,  phy_addr, pattern);
	PHY_RTN_ON_ERROR(rv);
	rv = _qca808x_phy_led_pattern_map_to_phy(dev_id, phy_addr, pattern,
		&led_map);
	PHY_RTN_ON_ERROR(rv);
	switch(source_id)
	{
		case QCA808X_PHY_LED_SOURCE0:
			led_mmd_addr = QCA808X_PHY_MMD7_LED0_CTRL;
			break;
		case QCA808X_PHY_LED_SOURCE1:
			led_mmd_addr = QCA808X_PHY_MMD7_LED1_CTRL;
			break;
		case QCA808X_PHY_LED_SOURCE2:
			led_mmd_addr = QCA808X_PHY_MMD7_LED2_CTRL;
			break;
		default:
			SSDK_ERROR("source %d is not support\n", source_id);
			break;
	}
	rv = hsl_phy_mmd_reg_write(dev_id, phy_addr, A_TRUE, QCA808X_PHY_MMD7_NUM,
		led_mmd_addr, led_map);
	PHY_RTN_ON_ERROR(rv);
#ifdef MHT
	if(qca808x_phy_id_check(dev_id, phy_addr, QCA8084_PHY))
	{
		rv = qca8084_phy_led_ctrl_source_pin_cfg(dev_id, phy_addr, source_id);
		PHY_RTN_ON_ERROR(rv);
	}
#endif
	return SW_OK;
}
/******************************************************************************
*
* qca808x_phy_led_source_pattern_get
*
*/
sw_error_t
qca808x_phy_led_ctrl_source_get(a_uint32_t dev_id, a_uint32_t phy_addr,
	a_uint32_t source_id, led_ctrl_pattern_t *pattern)
{
	sw_error_t rv = SW_OK;
	a_uint16_t phy_data = 0, led_mmd_addr = 0;

	pattern->map = 0;
	pattern->mode = LED_PATTERN_MAP_EN;
	rv = _qca808x_phy_led_active_get(dev_id, phy_addr, pattern);
	PHY_RTN_ON_ERROR(rv);
	switch(source_id)
	{
		case QCA808X_PHY_LED_SOURCE0:
			led_mmd_addr = QCA808X_PHY_MMD7_LED0_CTRL;
			break;
		case QCA808X_PHY_LED_SOURCE1:
			led_mmd_addr = QCA808X_PHY_MMD7_LED1_CTRL;
			break;
		case QCA808X_PHY_LED_SOURCE2:
			led_mmd_addr = QCA808X_PHY_MMD7_LED2_CTRL;
			break;
		default:
			SSDK_ERROR("source %d is not support\n", source_id);
			break;
	}
	phy_data = hsl_phy_mmd_reg_read(dev_id, phy_addr, A_TRUE, QCA808X_PHY_MMD7_NUM,
		led_mmd_addr);
	PHY_RTN_ON_READ_ERROR(phy_data);
	return _qca808x_phy_led_pattern_map_from_phy(dev_id, phy_addr, pattern,
		&phy_data);
}

void qca808x_phy_led_api_ops_init(hsl_phy_ops_t *qca808x_phy_led_api_ops)
{
	if (!qca808x_phy_led_api_ops) {
		return;
	}
	qca808x_phy_led_api_ops->phy_led_ctrl_pattern_get = qca808x_phy_led_ctrl_pattern_get;
	qca808x_phy_led_api_ops->phy_led_ctrl_pattern_set = qca808x_phy_led_ctrl_pattern_set;
	qca808x_phy_led_api_ops->phy_led_ctrl_source_set = qca808x_phy_led_ctrl_source_set;

	return;
}
