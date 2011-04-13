/*
 * arch/arm/mach-omap2/board-blaze-keypad.c
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

#include <plat/omap4-keypad.h>

#include "board-blaze.h"
#include "mux.h"

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

static struct gpio_led blaze_gpio_leds[] = {
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

static struct gpio_led_platform_data blaze_led_data = {
	.leds	= blaze_gpio_leds,
	.num_leds = ARRAY_SIZE(blaze_gpio_leds),
};

static struct platform_device blaze_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data = &blaze_led_data,
	},
};

static struct platform_device blaze_keypad_led = {
	.name	=	"keypad_led",
	.id	=	-1,
	.dev	= {
		.platform_data = NULL,
	},
};

static struct platform_device *blaze_led_devices[] __initdata = {
	&blaze_leds_gpio,
	&blaze_keypad_led,
};

int __init blaze_keypad_init(void)
{
	int status = 0;

	status = omap4_keypad_initialization(&sdp4430_keypad_data);
	if (status)
		pr_err("Keypad initialization failed: %d\n", status);

	platform_add_devices(blaze_led_devices, ARRAY_SIZE(blaze_led_devices));

	return 0;
}
