/*
 * Board support file for OMAP4430 SDP.
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
#include <linux/i2c/twl.h>
#include <linux/gpio_keys.h>
#include <linux/regulator/machine.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/leds-omap4430sdp-display.h>
#include <linux/delay.h>


#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>
#include <plat/usb.h>
#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>
#include <plat/omap4-keypad.h>
#include <plat/syntm12xx.h>
#include <plat/mmc.h>
#include <plat/nokia-dsi-panel.h>

#include "board-blaze.h"
#include "mux.h"
#include "timer-gp.h"
#include "control.h"

#define ETH_KS8851_IRQ			34
#define ETH_KS8851_POWER_ON		48
#define ETH_KS8851_QUART		138
#define OMAP4SDP_MDM_PWR_EN_GPIO	157


#define LED_SEC_DISP_GPIO 27
#define DSI2_GPIO_59	59

#define LED_PWM2ON		0x03
#define LED_PWM2OFF		0x04
#define LED_TOGGLE3		0x92

static int sdp4430_keymap[] = {
	KEY(0, 0, KEY_E),
	KEY(0, 1, KEY_R),
	KEY(0, 2, KEY_T),
	KEY(0, 3, KEY_HOME),
	KEY(0, 4, KEY_F5),
	KEY(0, 5, KEY_UNKNOWN),
	KEY(0, 6, KEY_I),
	KEY(0, 7, KEY_LEFTSHIFT),

	KEY(1, 0, KEY_D),
	KEY(1, 1, KEY_F),
	KEY(1, 2, KEY_G),
	KEY(1, 3, KEY_SEND),
	KEY(1, 4, KEY_F6),
	KEY(1, 5, KEY_UNKNOWN),
	KEY(1, 6, KEY_K),
	KEY(1, 7, KEY_ENTER),

	KEY(2, 0, KEY_X),
	KEY(2, 1, KEY_C),
	KEY(2, 2, KEY_V),
	KEY(2, 3, KEY_END),
	KEY(2, 4, KEY_F7),
	KEY(2, 5, KEY_UNKNOWN),
	KEY(2, 6, KEY_DOT),
	KEY(2, 7, KEY_CAPSLOCK),

	KEY(3, 0, KEY_Z),
	KEY(3, 1, KEY_KPPLUS),
	KEY(3, 2, KEY_B),
	KEY(3, 3, KEY_F1),
	KEY(3, 4, KEY_F8),
	KEY(3, 5, KEY_UNKNOWN),
	KEY(3, 6, KEY_O),
	KEY(3, 7, KEY_SPACE),

	KEY(4, 0, KEY_W),
	KEY(4, 1, KEY_Y),
	KEY(4, 2, KEY_U),
	KEY(4, 3, KEY_F2),
	KEY(4, 4, KEY_VOLUMEUP),
	KEY(4, 5, KEY_UNKNOWN),
	KEY(4, 6, KEY_L),
	KEY(4, 7, KEY_LEFT),

	KEY(5, 0, KEY_S),
	KEY(5, 1, KEY_H),
	KEY(5, 2, KEY_J),
	KEY(5, 3, KEY_F3),
	KEY(5, 4, KEY_F9),
	KEY(5, 5, KEY_VOLUMEDOWN),
	KEY(5, 6, KEY_M),
	KEY(5, 7, KEY_RIGHT),

	KEY(6, 0, KEY_Q),
	KEY(6, 1, KEY_A),
	KEY(6, 2, KEY_N),
	KEY(6, 3, KEY_BACK),
	KEY(6, 4, KEY_BACKSPACE),
	KEY(6, 5, KEY_UNKNOWN),
	KEY(6, 6, KEY_P),
	KEY(6, 7, KEY_UP),

	KEY(7, 0, KEY_PROG1),
	KEY(7, 1, KEY_PROG2),
	KEY(7, 2, KEY_PROG3),
	KEY(7, 3, KEY_PROG4),
	KEY(7, 4, KEY_F4),
	KEY(7, 5, KEY_UNKNOWN),
	KEY(7, 6, KEY_OK),
	KEY(7, 7, KEY_DOWN),
};

static struct matrix_keymap_data sdp4430_keymap_data = {
	.keymap			= sdp4430_keymap,
	.keymap_size		= ARRAY_SIZE(sdp4430_keymap),
};

static struct omap4_keypad_platform_data sdp4430_keypad_data = {
	.keymap_data		= &sdp4430_keymap_data,
	.rows			= 8,
	.cols			= 8,
};

void keyboard_mux_init(void)
{
	omap_mux_init_signal("kpd_col0.kpd_col0",
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_col1.kpd_col1",
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_col2.kpd_col2",
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_col3.kpd_col3",
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_col4.kpd_col4",
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_col5.kpd_col5",
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
	omap_mux_init_signal("gpmc_a23.kpd_col7",
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
	omap_mux_init_signal("gpmc_a22.kpd_col6",
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_row0.kpd_row0",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
				OMAP_INPUT_EN);
	omap_mux_init_signal("kpd_row1.kpd_row1",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
				OMAP_INPUT_EN);
	omap_mux_init_signal("kpd_row2.kpd_row2",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
				OMAP_INPUT_EN);
	omap_mux_init_signal("kpd_row3.kpd_row3",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
				OMAP_INPUT_EN);
	omap_mux_init_signal("kpd_row4.kpd_row4",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
				OMAP_INPUT_EN);
	omap_mux_init_signal("kpd_row5.kpd_row5",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
				OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_a18.kpd_row6",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
				OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_a19.kpd_row7",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
				OMAP_INPUT_EN);
}

static struct gpio_led sdp4430_gpio_leds[] = {
	{
		.name	= "omap4:green:debug0",
		.gpio	= 61,
	},
	{
		.name	= "omap4:green:debug1",
		.gpio	= 30,
	},
	{
		.name	= "omap4:green:debug2",
		.gpio	= 7,
	},
	{
		.name	= "omap4:green:debug3",
		.gpio	= 8,
	},
	{
		.name	= "omap4:green:debug4",
		.gpio	= 50,
	},
	{
		.name	= "omap4:blue:user",
		.gpio	= 169,
	},
	{
		.name	= "omap4:red:user",
		.gpio	= 170,
	},
	{
		.name	= "omap4:green:user",
		.gpio	= 139,
	},

};

static struct gpio_led_platform_data sdp4430_led_data = {
	.leds	= sdp4430_gpio_leds,
	.num_leds	= ARRAY_SIZE(sdp4430_gpio_leds),
};

static struct led_pwm sdp4430_pwm_leds[] = {
	{
		.name		= "omap4:green:chrg",
		.pwm_id		= 1,
		.max_brightness	= 255,
		.pwm_period_ns	= 7812500,
	},
};

static struct led_pwm_platform_data sdp4430_pwm_data = {
	.num_leds	= ARRAY_SIZE(sdp4430_pwm_leds),
	.leds		= sdp4430_pwm_leds,
};

static struct platform_device sdp4430_leds_pwm = {
	.name	= "leds_pwm",
	.id	= -1,
	.dev	= {
		.platform_data = &sdp4430_pwm_data,
	},
};

static struct platform_device sdp4430_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data = &sdp4430_led_data,
	},
};
static struct spi_board_info sdp4430_spi_board_info[] __initdata = {
	{
		.modalias               = "ks8851",
		.bus_num                = 1,
		.chip_select            = 0,
		.max_speed_hz           = 24000000,
		.irq                    = ETH_KS8851_IRQ,
	},
};

static int omap_ethernet_init(void)
{
	int status;

	/* Request of GPIO lines */

	status = gpio_request(ETH_KS8851_POWER_ON, "eth_power");
	if (status) {
		pr_err("Cannot request GPIO %d\n", ETH_KS8851_POWER_ON);
		return status;
	}

	status = gpio_request(ETH_KS8851_QUART, "quart");
	if (status) {
		pr_err("Cannot request GPIO %d\n", ETH_KS8851_QUART);
		goto error1;
	}

	status = gpio_request(ETH_KS8851_IRQ, "eth_irq");
	if (status) {
		pr_err("Cannot request GPIO %d\n", ETH_KS8851_IRQ);
		goto error2;
	}

	/* Configuration of requested GPIO lines */

	status = gpio_direction_output(ETH_KS8851_POWER_ON, 1);
	if (status) {
		pr_err("Cannot set output GPIO %d\n", ETH_KS8851_IRQ);
		goto error3;
	}

	status = gpio_direction_output(ETH_KS8851_QUART, 1);
	if (status) {
		pr_err("Cannot set output GPIO %d\n", ETH_KS8851_QUART);
		goto error3;
	}

	status = gpio_direction_input(ETH_KS8851_IRQ);
	if (status) {
		pr_err("Cannot set input GPIO %d\n", ETH_KS8851_IRQ);
		goto error3;
	}

	return 0;

error3:
	gpio_free(ETH_KS8851_IRQ);
error2:
	gpio_free(ETH_KS8851_QUART);
error1:
	gpio_free(ETH_KS8851_POWER_ON);
	return status;
}

static void __init sdp4430_init_display_led(void)
{
	twl_i2c_write_u8(TWL_MODULE_PWM, 0xFF, LED_PWM2ON);
	twl_i2c_write_u8(TWL_MODULE_PWM, 0x7F, LED_PWM2OFF);
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x30, TWL6030_TOGGLE3);
}

static void sdp4430_set_primary_brightness(u8 brightness)
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

static void sdp4430_set_secondary_brightness(u8 brightness)
{
	if (brightness > 0)
		brightness = 1;

	gpio_set_value(LED_SEC_DISP_GPIO, brightness);
}

static struct omap4430_sdp_disp_led_platform_data sdp4430_disp_led_data = {
	.flags = LEDS_CTRL_AS_ONE_DISPLAY,
	.display_led_init = sdp4430_init_display_led,
	.primary_display_set = sdp4430_set_primary_brightness,
	.secondary_display_set = sdp4430_set_secondary_brightness,
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

static struct platform_device sdp4430_disp_led = {
	.name	=	"display_led",
	.id	=	-1,
	.dev	= {
		.platform_data = &sdp4430_disp_led_data,
	},
};

static struct nokia_dsi_panel_data dsi_panel = {
		.name	= "taal",
		.reset_gpio	= 102,
		.use_ext_te	= false,
		.ext_te_gpio	= 101,
		.use_esd_check	= false,
};

static struct nokia_dsi_panel_data dsi2_panel = {
		.name   = "taal2",
		.reset_gpio     = 104,
		.use_ext_te     = false,
		.ext_te_gpio    = 103,
		.use_esd_check  = false,
};

static struct omap_dss_device sdp4430_lcd_device = {
	.name			= "lcd",
	.driver_name		= "taal",
	.type			= OMAP_DISPLAY_TYPE_DSI,
	.data			= &dsi_panel,
	.phy.dsi		= {
		.clk_lane	= 1,
		.clk_pol	= 0,
		.data1_lane	= 2,
		.data1_pol	= 0,
		.data2_lane	= 3,
		.data2_pol	= 0,
		.div		= {
			.lck_div	= 1,
			.pck_div	= 5,
			.regm		= 150,
			.regn		= 17,
			.regm_dispc	= 4,
			.regm_dsi	= 4,
			.lp_clk_div	= 8,
		},
	},
	.channel		= OMAP_DSS_CHANNEL_LCD,
};

static struct omap_dss_device sdp4430_lcd2_device = {
	.name			= "lcd2",
	.driver_name		= "taal2",
	.type			= OMAP_DISPLAY_TYPE_DSI,
	.data			= &dsi2_panel,
	.phy.dsi		= {
		.clk_lane	= 1,
		.clk_pol	= 0,
		.data1_lane	= 2,
		.data1_pol	= 0,
		.data2_lane	= 3,
		.data2_pol	= 0,
		.div		= {
			.lck_div	= 1,
			.pck_div	= 5,
			.regm		= 150,
			.regn		= 17,
			.regm_dispc	= 4,
			.regm_dsi	= 4,
			.lp_clk_div	= 8,
		},
	},
	.channel		= OMAP_DSS_CHANNEL_LCD2,
};

static struct omap_dss_device *sdp4430_dss_devices[] = {
	&sdp4430_lcd_device,
	&sdp4430_lcd2_device,
};

static struct omap_dss_board_info sdp4430_dss_data = {
	.num_devices	=	ARRAY_SIZE(sdp4430_dss_devices),
	.devices	=	sdp4430_dss_devices,
	.default_device	=	&sdp4430_lcd_device,
};

static struct platform_device *sdp4430_devices[] __initdata = {
	&sdp4430_disp_led,
	&sdp4430_leds_gpio,
	&sdp4430_leds_pwm,
};

static void __init omap_4430sdp_init_irq(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
	gic_init_irq();
}

static const struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {
	.port_mode[0]	= EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1]	= EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[2]	= EHCI_HCD_OMAP_MODE_UNKNOWN,
	.phy_reset	= false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL,
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#if defined CONFIG_USB_MUSB_OTG
	.mode		= MUSB_OTG,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode		= MUSB_PERIPHERAL,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode           = MUSB_HOST,
#endif
	.power			= 100,
};

static struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_set_clock	= omap4430_phy_set_clk,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 2,
		.caps		=  MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable   = true,
		.ocr_mask	= MMC_VDD_29_30,
	},
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_wp	= -EINVAL,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply sdp4430_vaux_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "mmci-omap-hs.1",
	},
};
static struct regulator_consumer_supply sdp4430_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "mmci-omap-hs.0",
	},
};

static struct regulator_consumer_supply sdp4430_cam2_supply[] = {
	{
		.supply = "cam2pwr",
	},
};

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret)
			pr_err("Failed configuring MMC1 card detect\n");
		pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
						MMCDETECT_INTR_OFFSET;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}
	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed %s\n", __func__);
		return;
	}
	pdata = dev->platform_data;
	pdata->init =	omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

static struct regulator_init_data sdp4430_vaux1 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = sdp4430_vaux_supply,
};

static struct regulator_init_data sdp4430_vaux2 = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vaux3 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = sdp4430_cam2_supply,
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data sdp4430_vmmc = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = sdp4430_vmmc_supply,
};

static struct regulator_init_data sdp4430_vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vusim = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2900000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static void omap4_audio_conf(void)
{
	/* twl6040 naudint */
	omap_mux_init_signal("sys_nirq2.sys_nirq2", \
			     OMAP_PIN_INPUT_PULLUP);
}

static struct twl4030_codec_audio_data twl6040_audio = {
	/* single-step ramp for headset and handsfree */
	.left_step_hs	= 0x0f,
	.right_step_hs	= 0x0f,
	.left_step_hf	= 0x1d,
	.right_step_hf	= 0x1d,
};

static struct twl4030_codec_vibra_data twl6040_vibra = {
	.max_timeout	= 15000,
	.initial_vibrate = 0,
};

static struct twl4030_codec_data twl6040_codec = {
	.audio		= &twl6040_audio,
	.vibra		= &twl6040_vibra,
	.audpwron_gpio	= 127,
	.naudint_irq	= OMAP44XX_IRQ_SYS_2N,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
};

static struct twl4030_platform_data sdp4430_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &sdp4430_vmmc,
	.vpp		= &sdp4430_vpp,
	.vusim		= &sdp4430_vusim,
	.vana		= &sdp4430_vana,
	.vcxio		= &sdp4430_vcxio,
	.vdac		= &sdp4430_vdac,
	.vusb		= &sdp4430_vusb,
	.vaux1		= &sdp4430_vaux1,
	.vaux2		= &sdp4430_vaux2,
	.vaux3		= &sdp4430_vaux3,
	.usb		= &omap4_usbphy_data,

	/* children */
	.codec		= &twl6040_codec,
};

/* Begin Synaptic Touchscreen TM-01217 */

static char *tm12xx_idev_names[] = {
	"Synaptic TM12XX TouchPoint 1",
	"Synaptic TM12XX TouchPoint 2",
	"Synaptic TM12XX TouchPoint 3",
	"Synaptic TM12XX TouchPoint 4",
	"Synaptic TM12XX TouchPoint 5",
	"Synaptic TM12XX TouchPoint 6",
	NULL,
};

static u8 tm12xx_button_map[] = {
	KEY_F1,
	KEY_F2,
};

static struct tm12xx_ts_platform_data tm12xx_platform_data[] = {
	{ /* Primary Controller */
		.gpio_intr = 35,
		.idev_name = tm12xx_idev_names,
		.button_map = tm12xx_button_map,
		.num_buttons = ARRAY_SIZE(tm12xx_button_map),
		.repeat = 0,
		.swap_xy = 1,
	/* Android does not have touchscreen as wakeup source */
#if !defined(CONFIG_ANDROID)
		.suspend_state = SYNTM12XX_ON_ON_SUSPEND,
#else
		.suspend_state = SYNTM12XX_SLEEP_ON_SUSPEND,
#endif
	},
	{ /* Secondary Controller */
		.gpio_intr = 36,
		.idev_name = tm12xx_idev_names,
		.button_map = tm12xx_button_map,
		.num_buttons = ARRAY_SIZE(tm12xx_button_map),
		.repeat = 0,
		.swap_xy = 1,
	/* Android does not have touchscreen as wakeup source */
#if !defined(CONFIG_ANDROID)
		.suspend_state = SYNTM12XX_ON_ON_SUSPEND,
#else
		.suspend_state = SYNTM12XX_SLEEP_ON_SUSPEND,
#endif
	},
};

/* End Synaptic Touchscreen TM-01217 */

static struct i2c_board_info __initdata sdp4430_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &sdp4430_twldata,
	},
};

static struct i2c_board_info __initdata sdp4430_i2c_2_boardinfo[] = {
	{
		I2C_BOARD_INFO("tm12xx_ts_primary", 0x4b),
		.platform_data = &tm12xx_platform_data[0],
	},
};

static struct i2c_board_info __initdata sdp4430_i2c_3_boardinfo[] = {
	{
		I2C_BOARD_INFO("tm12xx_ts_secondary", 0x4b),
		.platform_data = &tm12xx_platform_data[1],
	},

};

static int __init omap4_i2c_init(void)
{
	/*
	 * Phoenix Audio IC needs I2C1 to
	 * start with 400 KHz or less
	 */
	omap_register_i2c_bus(1, 400, sdp4430_i2c_boardinfo,
			ARRAY_SIZE(sdp4430_i2c_boardinfo));
	omap_register_i2c_bus(2, 400, sdp4430_i2c_2_boardinfo,
			ARRAY_SIZE(sdp4430_i2c_2_boardinfo));
	omap_register_i2c_bus(3, 400, NULL, 0);
	omap_register_i2c_bus(4, 400, NULL, 0);

	return 0;
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	OMAP4_MUX(USBB2_ULPITLL_CLK, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static void __init omap4_display_init(void)
{
	void __iomem *phymux_base = NULL;
	u32 val = 0xFFFFC000;

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

	/* Panel Taal reset */
	gpio_request(dsi_panel.reset_gpio, "dsi1_en_gpio");
	gpio_direction_output(dsi_panel.reset_gpio, 0);

	gpio_request(dsi2_panel.reset_gpio, "dsi2_en_gpio");
	gpio_direction_output(dsi2_panel.reset_gpio, 0);
}

static void __init omap_4430sdp_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, package);

	omap4_audio_conf();
	omap4_i2c_init();
	blaze_sensor_init();
	i2c_register_board_info(3, sdp4430_i2c_3_boardinfo,
		ARRAY_SIZE(sdp4430_i2c_3_boardinfo));

	omap4_display_init();
	omap_disp_led_init();

	platform_add_devices(sdp4430_devices, ARRAY_SIZE(sdp4430_devices));
	omap_serial_init();
	omap4_twl6030_hsmmc_init(mmc);

	/* Power on the ULPI PHY */
	status = gpio_request(OMAP4SDP_MDM_PWR_EN_GPIO, "USBB1 PHY VMDM_3V3");
	if (status)
		pr_err("%s: Could not get USBB1 PHY GPIO\n", __func__);
	else
		gpio_direction_output(OMAP4SDP_MDM_PWR_EN_GPIO, 1);

	usb_ehci_init(&ehci_pdata);
	usb_musb_init(&musb_board_data);

	status = omap4_keypad_initialization(&sdp4430_keypad_data);
	if (status)
		pr_err("Keypad initialization failed: %d\n", status);

	status = omap_ethernet_init();
	if (status) {
		pr_err("Ethernet initialization failed: %d\n", status);
	} else {
		sdp4430_spi_board_info[0].irq = gpio_to_irq(ETH_KS8851_IRQ);
		spi_register_board_info(sdp4430_spi_board_info,
				ARRAY_SIZE(sdp4430_spi_board_info));
	}
	omap_display_init(&sdp4430_dss_data);
}

static void __init omap_4430sdp_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

MACHINE_START(OMAP_4430SDP, "OMAP4430")
	/* Maintainer: Santosh Shilimkar - Texas Instruments Inc */
	.boot_params	= 0x80000100,
	.map_io		= omap_4430sdp_map_io,
	.reserve	= omap_reserve,
	.init_irq	= omap_4430sdp_init_irq,
	.init_machine	= omap_4430sdp_init,
	.timer		= &omap_timer,
MACHINE_END
