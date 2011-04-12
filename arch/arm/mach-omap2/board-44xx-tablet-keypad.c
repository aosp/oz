/*
 * arch/arm/mach-omap2/board-44xx-tablet-keypad.c
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

#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/platform_device.h>

#include "mux.h"

static struct gpio_led omap4_tablet_gpio_leds[] = {
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
		.name	= "blue",
		.default_trigger = "timer",
		.gpio	= 169,
	},
	{
		.name	= "red",
		.default_trigger = "timer",
		.gpio	= 170,
	},
	{
		.name	= "green",
		.default_trigger = "timer",
		.gpio	= 139,
	},
};

static struct gpio_led_platform_data omap4_tablet_led_data = {
	.leds	= omap4_tablet_gpio_leds,
	.num_leds = ARRAY_SIZE(omap4_tablet_gpio_leds),
};

static struct platform_device omap4_tablet_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data = &omap4_tablet_led_data,
	},
};

static struct gpio_keys_button omap4_tablet_gpio_keys_buttons[] = {
	[0] = {
		.code			= KEY_VOLUMEUP,
		.gpio			= 43,
		.desc			= "SW1",
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 5,
	},
	[1] = {
		.code			= KEY_HOME,
		.gpio			= 46,
		.desc			= "SW2",
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 5,
	},
	[2] = {
		.code			= KEY_VOLUMEDOWN,
		.gpio			= 47,
		.desc			= "SW3",
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 5,
		},
	};

static struct gpio_keys_platform_data omap4_tablet_gpio_keys = {
	.buttons		= omap4_tablet_gpio_keys_buttons,
	.nbuttons		= ARRAY_SIZE(omap4_tablet_gpio_keys_buttons),
	.rep			= 0,
};

static struct platform_device omap4_tablet_gpio_keys_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data	= &omap4_tablet_gpio_keys,
	},
};

static struct platform_device *omap4_tablet_keypad_devices[] __initdata = {
	&omap4_tablet_gpio_keys_device,
	&omap4_tablet_leds_gpio,
};

int __init omap4_tablet_keypad_init(void)
{

	omap_mux_init_signal("gpmc_a22.gpio_46",
			OMAP_PULL_ENA | OMAP_PULL_UP |
			OMAP_WAKEUP_EN | OMAP_MUX_MODE3 |
			OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_a23.gpio_47",
			OMAP_PULL_ENA | OMAP_PULL_UP |
			OMAP_WAKEUP_EN | OMAP_MUX_MODE3 |
			OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_a19.gpio_43",
			OMAP_PULL_ENA | OMAP_PULL_UP |
			OMAP_WAKEUP_EN | OMAP_MUX_MODE3 |
			OMAP_INPUT_EN);

	platform_add_devices(omap4_tablet_keypad_devices,
			ARRAY_SIZE(omap4_tablet_keypad_devices));
	return 0;
}

