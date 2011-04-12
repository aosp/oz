/*
 * arch/arm/mach-omap2/board-44xx-tablet-sensors.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <linux/input/sfh7741.h>
#include <linux/i2c/bma180.h>

#include <plat/i2c.h>

#include "board-44xx-tablet.h"
#include "mux.h"

#define OMAP4_BMA180ACCEL_GPIO		178

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

static struct i2c_board_info __initdata blaze_tablet_i2c_bus3_sensor_info[] = {
	{
		I2C_BOARD_INFO("tmp105", 0x48),
	},
};

static struct i2c_board_info __initdata blaze_tablet_i2c_bus4_sensor_info[] = {
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

int __init omap4_tablet_sensor_init(void)
{
	i2c_register_board_info(3, blaze_tablet_i2c_bus3_sensor_info,
		ARRAY_SIZE(blaze_tablet_i2c_bus3_sensor_info));
	i2c_register_board_info(4, blaze_tablet_i2c_bus4_sensor_info,
		ARRAY_SIZE(blaze_tablet_i2c_bus4_sensor_info));

	return 0;
}

