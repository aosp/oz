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
#include <linux/input/sfh7741.h>
#include <linux/i2c/bma180.h>
#include <linux/qtouch_obp_ts.h>
#include <linux/i2c/cma3000.h>

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
#include <plat/mmc.h>
#include <plat/toshiba-dsi-panel.h>

#include "mux.h"
#include "timer-gp.h"
#include "control.h"

#define ETH_KS8851_IRQ			34
#define ETH_KS8851_POWER_ON		48
#define ETH_KS8851_QUART		138
#define OMAP4SDP_MDM_PWR_EN_GPIO	157

#define LED_SEC_DISP_GPIO 27
#define DSI2_GPIO_59	59
#define DP_4430_GPIO_59         59
#define OMAP4_TOUCH_IRQ_1		35
#define OMAP4_BMA180ACCEL_GPIO		178

#define LED_PWM2ON		0x03
#define LED_PWM2OFF		0x04
#define LED_TOGGLE3		0x92


/* GPIO_KEY for Tablet */
static struct gpio_keys_button tablet_gpio_keys_buttons[] = {
	[0] = {
		.code			= KEY_BACK,
		.gpio			= 43,
		.desc			= "SW1",
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 30,
	},
	[1] = {
		.code			= KEY_HOME,
		.gpio			= 46,
		.desc			= "SW2",
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 30,
	},
	[2] = {
		.code			= KEY_F1,
		.gpio			= 47,
		.desc			= "SW3",
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 30,
	},
};

static struct gpio_keys_platform_data tablet_gpio_keys = {
	.buttons		= tablet_gpio_keys_buttons,
	.nbuttons		= ARRAY_SIZE(tablet_gpio_keys_buttons),
	.rep			= 1,
};

static struct platform_device tablet_gpio_keys_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data	= &tablet_gpio_keys,
	},
};

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

/* BMA180 Accelerometer Start */
static struct bma180accel_platform_data bma180accel_platform_data = {
	.ctrl_reg0	= 0x11,
	.g_range	= BMA_GRANGE_2G,
	.bandwidth	= BMA_BW_10HZ,
	.mode		= BMA_MODE_LOW_NOISE,
	.bit_mode	= BMA_BITMODE_14BITS,
	.smp_skip	= 1,
	.def_poll_rate	= 200,
	.fuzz_x		= 25,
	.fuzz_y		= 25,
	.fuzz_z		= 25,
};

/* BMA180 Accelerometer End */

/* Atmel MXT224 TouchScreen Begin */
static struct qtm_touch_keyarray_cfg blaze_tablet_key_array_data[] = {
	{
		.ctrl = 0,
		.x_origin = 0,
		.y_origin = 0,
		.x_size = 0,
		.y_size = 0,
		.aks_cfg = 0,
		.burst_len = 0,
		.tch_det_thr = 0,
		.tch_det_int = 0,
		.rsvd1 = 0,
	},
};

static void blaze_tablet_touch_init(void)
{
	gpio_request(OMAP4_TOUCH_IRQ_1, "atmel touch irq");
	gpio_direction_input(OMAP4_TOUCH_IRQ_1);
	omap_mux_init_signal("gpmc_ad11.gpio_35",
			OMAP_PULL_ENA | OMAP_PULL_UP |
			OMAP_WAKEUP_EN | OMAP_MUX_MODE3 |
			OMAP_INPUT_EN | OMAP_PIN_OFF_INPUT_PULLUP);
}

static struct qtouch_ts_platform_data atmel_mxt224_ts_platform_data = {
	.irqflags	= (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_LOW),
	.flags		= (QTOUCH_USE_MULTITOUCH | QTOUCH_FLIP_Y),
	.abs_min_x	= 0,
	.abs_max_x	= 768,
	.abs_min_y	= 0,
	.abs_max_y	= 1024,
	.abs_min_p	= 0,
	.abs_max_p	= 255,
	.abs_min_w	= 0,
	.abs_max_w	= 15,
	.x_delta	= 1024,
	.y_delta	= 768,
	.nv_checksum	= 0xfaf5,
	.fuzz_x		= 0,
	.fuzz_y		= 0,
	.fuzz_p		= 2,
	.fuzz_w		= 2,
	.hw_reset	= NULL,
	.power_cfg	= {
		.idle_acq_int	= 0x58,
		.active_acq_int	= 0x58,
		.active_idle_to	= 0x32,
	},
	.acquire_cfg	= {
		.charge_time	= 0x0a,
		.atouch_drift	= 0x05,
		.touch_drift	= 0x14,
		.drift_susp	= 0x14,
		.touch_autocal	= 0x0a,
		.sync		= 0,
		.cal_suspend_time = 0x09,
		.cal_suspend_thresh = 0x23,
	},
	.multi_touch_cfg	= {
		.ctrl		= 0x83,
		.x_origin	= 0,
		.y_origin	= 0,
		.x_size		= 0x11,
		.y_size		= 0x0d,
		.aks_cfg	= 0,
		.burst_len	= 0x01,
		.tch_det_thr	= 0x20,
		.tch_det_int	= 0x2,
		.mov_hyst_init	= 0x0,
		.mov_hyst_next	= 0x0,
		.mov_filter	= 0x9,
		.num_touch	= 1,
		.orient		= 0x00,
		.mrg_timeout	= 0x01,
		.merge_hyst	= 0x0a,
		.merge_thresh	= 0x0a,
		.amp_hyst = 0x0a,
		 .x_res = 0x02ff,
		 .y_res = 0x03ff,
		 .x_low_clip = 0x00,
		 .x_high_clip = 0x00,
		 .y_low_clip = 0x00,
		 .y_high_clip = 0x00,
	},
	.key_array      = {
		.cfg		= blaze_tablet_key_array_data,
		.num_keys   = ARRAY_SIZE(blaze_tablet_key_array_data),
	},
	.grip_suppression_cfg = {
		.ctrl		= 0x00,
		.xlogrip	= 0x00,
		.xhigrip	= 0x00,
		.ylogrip	= 0x00,
		.yhigrip	= 0x00,
		.maxtchs	= 0x00,
		.reserve0   = 0x00,
		.szthr1	= 0x00,
		.szthr2	= 0x00,
		.shpthr1	= 0x00,
		.shpthr2	= 0x00,
	},
	.noise0_suppression_cfg = {
		.ctrl		= 0x07,
		.reserved	= 0x0000,
		.gcaf_upper_limit = 0x000a,
		.gcaf_lower_limit = 0xfff6,
		.gcaf_valid	= 0x04,
		.noise_thresh	= 0x08,
		.reserved1	= 0x00,
		.freq_hop_scale = 0x01,
		.burst_freq_0	= 0x0a,
		.burst_freq_1 = 0x0f,
		.burst_freq_2 = 0x14,
		.burst_freq_3 = 0x19,
		.burst_freq_4 = 0x1e,
		.num_of_gcaf_samples = 0x04,
	},
	.spt_cte_cfg = {
		.ctrl = 0x00,
		.command = 0x00,
		.mode = 0x01,
		.gcaf_idle_mode = 0x04,
		.gcaf_actv_mode = 0x08,
	},
};
/* End Atmel Touch screen */

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

	/* Panel D2L reset and backlight GPIO init */
	gpio_request(blazetablet_dsi_panel.reset_gpio, "dsi1_en_gpio");
	gpio_direction_output(blazetablet_dsi_panel.reset_gpio, 0);
}

static struct i2c_board_info __initdata tablet_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &sdp4430_twldata,
	},
};

static struct i2c_board_info __initdata tablet_i2c_2_boardinfo[] = {
	{
		I2C_BOARD_INFO("d2l_i2c_driver", 0x0f),
	},
	{
		I2C_BOARD_INFO("DP501_i2c_driver", 0x08),
	},
};

static struct i2c_board_info __initdata tablet_i2c_3_boardinfo[] = {
	{
		I2C_BOARD_INFO("tmp105", 0x48),
	},
};

static struct i2c_board_info __initdata tablet_i2c_4_boardinfo[] = {
	{
		I2C_BOARD_INFO(QTOUCH_TS_NAME, 0x4b),
		.platform_data = &atmel_mxt224_ts_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP4_TOUCH_IRQ_1),
	},
	{
		I2C_BOARD_INFO("bmp085", 0x77),
	},
	{
		I2C_BOARD_INFO("hmc5843", 0x1e),
	},
	{
		I2C_BOARD_INFO("bma180_accel", 0x40),
		.platform_data = &bma180accel_platform_data,
	},
};

static int __init tablet_i2c_init(void)
{
	/*
	 * Phoenix Audio IC needs I2C1 to
	 * start with 400 KHz or less
	 */
	omap_register_i2c_bus(1, 400, tablet_i2c_boardinfo,
			ARRAY_SIZE(tablet_i2c_boardinfo));
	omap_register_i2c_bus(2, 400, tablet_i2c_2_boardinfo,
			ARRAY_SIZE(tablet_i2c_2_boardinfo));
	omap_register_i2c_bus(3, 400, tablet_i2c_3_boardinfo,
			ARRAY_SIZE(tablet_i2c_3_boardinfo));
	omap_register_i2c_bus(4, 400, tablet_i2c_4_boardinfo,
			ARRAY_SIZE(tablet_i2c_4_boardinfo));
	return 0;
}

static struct platform_device *blazetablet_devices[] __initdata = {
	&sdp4430_disp_led,
	/* TODO. Review button LEDs functionality
	&sdp4430_leds_pwm, */
	&sdp4430_leds_gpio,
	&tablet_gpio_keys_device,
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

void omap_44xxtablet_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, package);

	tablet_i2c_init();
	omap4_display_init();
	omap_disp_led_init();
	blaze_tablet_touch_init();

	platform_add_devices(blazetablet_devices,
		ARRAY_SIZE(blazetablet_devices));

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

	status = omap_ethernet_init();
	if (status) {
		pr_err("Ethernet initialization failed: %d\n", status);
	} else {
		sdp4430_spi_board_info[0].irq = gpio_to_irq(ETH_KS8851_IRQ);
		spi_register_board_info(sdp4430_spi_board_info,
				ARRAY_SIZE(sdp4430_spi_board_info));
	}
	omap_display_init(&blazetablet_dss_data);
}

static void __init omap_4430sdp_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

MACHINE_START(OMAP_BLAZE, "OMAP4430")
	.boot_params	= 0x80000100,
	.map_io		= omap_4430sdp_map_io,
	.reserve	= omap_reserve,
	.init_irq	= omap_4430sdp_init_irq,
	.init_machine	= omap_44xxtablet_init,
	.timer		= &omap_timer,
MACHINE_END
