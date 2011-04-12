/*
 * arch/arm/mach-omap2/board-44xx-tablet-panel.c
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/leds-omap4430sdp-display.h>
#include <linux/platform_device.h>

#include <linux/i2c/twl.h>

#include <plat/display.h>
#include <plat/i2c.h>
#include <plat/toshiba-dsi-panel.h>

#include "board-44xx-tablet.h"
#include "mux.h"

#define DP_4430_GPIO_59         59
#define LED_SEC_DISP_GPIO	27
#define DSI2_GPIO_59		59

#define LED_PWM2ON		0x03
#define LED_PWM2OFF		0x04
#define LED_TOGGLE3		0x92


static void __init omap4_tablet_init_display_led(void)
{
	twl_i2c_write_u8(TWL_MODULE_PWM, 0xFF, LED_PWM2ON);
	twl_i2c_write_u8(TWL_MODULE_PWM, 0x7F, LED_PWM2OFF);
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x30, TWL6030_TOGGLE3);
}

static void omap4_tablet_set_primary_brightness(u8 brightness)
{
	if (brightness > 1) {
		if (brightness == 255)
			brightness = 0x7f;
		else
			brightness = (~(brightness/2)) & 0x7f;

		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x30, TWL6030_TOGGLE3);
		twl_i2c_write_u8(TWL_MODULE_PWM, brightness, LED_PWM2ON);
	} else if (brightness <= 1) {
		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x08, TWL6030_TOGGLE3);
		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x38, TWL6030_TOGGLE3);
	}
}

static void omap4_tablet_set_secondary_brightness(u8 brightness)
{
	if (brightness > 0)
		brightness = 1;

	gpio_set_value(LED_SEC_DISP_GPIO, brightness);
}

static struct omap4430_sdp_disp_led_platform_data sdp4430_disp_led_data = {
	.flags = LEDS_CTRL_AS_ONE_DISPLAY,
	.display_led_init = omap4_tablet_init_display_led,
	.primary_display_set = omap4_tablet_set_primary_brightness,
	.secondary_display_set = omap4_tablet_set_secondary_brightness,
};

static void __init omap_disp_led_init(void)
{
	/* Seconday backlight control */
	gpio_request(DSI2_GPIO_59, "dsi2_bl_gpio");
	gpio_direction_output(DSI2_GPIO_59, 0);

	if (sdp4430_disp_led_data.flags & LEDS_CTRL_AS_ONE_DISPLAY) {
		pr_info("%s: Configuring as one display LED\n", __func__);
		gpio_set_value(DSI2_GPIO_59, 1);
	}

	gpio_request(LED_SEC_DISP_GPIO, "dsi1_bl_gpio");
	gpio_direction_output(LED_SEC_DISP_GPIO, 1);
	mdelay(120);
	gpio_set_value(LED_SEC_DISP_GPIO, 0);

}

static struct platform_device omap4_tablet_disp_led = {
	.name	=	"display_led",
	.id	=	-1,
	.dev	= {
		.platform_data = &sdp4430_disp_led_data,
	},
};

static struct toshiba_dsi_panel_data blazetablet_dsi_panel = {
	.name	= "d2l",
	.reset_gpio	= 102,
	.use_ext_te	= false,
	.ext_te_gpio	= 101,
	.use_esd_check	= false,
	.set_backlight	= NULL,
};

static struct omap_dss_device blazetablet_lcd_device = {
	.name			= "lcd",
	.driver_name		= "d2l",
	.type			= OMAP_DISPLAY_TYPE_DSI,
	.data			= &blazetablet_dsi_panel,
	.phy.dsi		= {
		.clk_lane	= 1,
		.clk_pol	= 0,
		.data1_lane	= 2,
		.data1_pol	= 0,
		.data2_lane	= 3,
		.data2_pol	= 0,
		.data3_lane	= 4,
		.data3_pol	= 0,
		.data4_lane	= 5,
		.data4_pol	= 0,
		.div		= {
			.lck_div	= 1,	/* LCD */
			.pck_div	= 2,	/* PCD */
			.regm		= 394,	/* DSI_PLL_REGM */
			.regn		= 38,	/* DSI_PLL_REGN */
			.regm_dispc	= 6,	/* PLL_CLK1 (M4) */
			.regm_dsi	= 9,	/* PLL_CLK2 (M5) */
			.lp_clk_div	= 5,	/* LPDIV */
		},
	},
	.channel		= OMAP_DSS_CHANNEL_LCD,
};

/* PARADE DP501, DisplayPort chip */
static int sdp4430_panel_enable_displayport(struct omap_dss_device *dssdev)
{
	printk(KERN_ERR "sdp4430_panel_enable_displayport is called\n");
	gpio_request(DP_4430_GPIO_59, "DISPLAYPORT POWER DOWN");
	gpio_direction_output(DP_4430_GPIO_59, 0);
	mdelay(100);
	gpio_set_value(DP_4430_GPIO_59, 1);
	mdelay(100);
	return 0;
}

static void sdp4430_panel_disable_displayport(struct omap_dss_device *dssdev)
{
	printk(KERN_DEBUG "sdp4430_panel_disable_displayport is called\n");
	gpio_set_value(DP_4430_GPIO_59, 0);
}

static struct omap_dss_device sdp4430_displayport_device = {
	.name				= "DP501",
	.driver_name			= "displayport_panel",
	.type				= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines		= 24,
	.platform_enable		= sdp4430_panel_enable_displayport,
	.platform_disable		= sdp4430_panel_disable_displayport,
	.channel			= OMAP_DSS_CHANNEL_LCD2,
};

static struct omap_dss_device *blazetablet_dss_devices[] = {
	&blazetablet_lcd_device,
#ifdef CONFIG_PANEL_DP501
	&sdp4430_displayport_device,
#endif
};

static struct omap_dss_board_info blazetablet_dss_data = {
	.num_devices	=	ARRAY_SIZE(blazetablet_dss_devices),
	.devices	=	blazetablet_dss_devices,
	.default_device =	&blazetablet_lcd_device,
};

static struct i2c_board_info __initdata omap4xx_i2c_bus3_panel_info[] = {
	{
		I2C_BOARD_INFO("d2l_i2c_driver", 0x0f),
	},
	{
		I2C_BOARD_INFO("DP501_i2c_driver", 0x08),
	},
};

int __init omap4_tablet_panel_init(void)
{
	void __iomem *phymux_base = NULL;
	u32 val = 0xFFFFC000;

	/* TO DO: Need to clean this up and use the MUX
	framework */
	phymux_base = ioremap(0x4A100000, 0x1000);
	/* Turning on DSI PHY Mux*/
	__raw_writel(val, phymux_base + 0x618);

	/* Set mux to choose GPIO 101 for Taal 1 ext te line*/
	val = __raw_readl(phymux_base + 0x90);
	val = (val & 0xFFFFFFE0) | 0x11B;
	__raw_writel(val, phymux_base + 0x90);

	/* Set mux to choose GPIO 103 for Taal 2 ext te line*/
	val = __raw_readl(phymux_base + 0x94);
	val = (val & 0xFFFFFFE0) | 0x11B;
	__raw_writel(val, phymux_base + 0x94);

	iounmap(phymux_base);

	/* Panel D2L reset and backlight GPIO init */
	gpio_request(blazetablet_dsi_panel.reset_gpio, "dsi1_en_gpio");
	gpio_direction_output(blazetablet_dsi_panel.reset_gpio, 0);

	i2c_register_board_info(2, omap4xx_i2c_bus3_panel_info,
		ARRAY_SIZE(omap4xx_i2c_bus3_panel_info));

	omap_display_init(&blazetablet_dss_data);

	omap_disp_led_init();
	platform_device_register(&omap4_tablet_disp_led);

	return 0;
}


