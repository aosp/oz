/*
 * OMAP4 OPP table definitions.
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *	Kevin Hilman
 *	Thara Gopinath
 * Copyright (C) 2010 Nokia Corporation.
 *      Eduardo Valentin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>

#include <plat/cpu.h>
#include <plat/voltage.h>

#include "omap_opp_data.h"

#if 1 /* XXX temp. work-around to enable SGX module */
#include <plat/io.h>
#include "cm2_44xx.h"
#endif

/* static struct clk *sgx_clk; */

static struct omap_opp_def __initdata omap44xx_opp_def_list[] = {
	/* MPU OPP1 - OPP50 */
	OPP_INITIALIZER("mpu", true, 300000000, OMAP4430_VDD_MPU_OPP50_UV),
	/* MPU OPP2 - OPP100 */
	OPP_INITIALIZER("mpu", true, 600000000, OMAP4430_VDD_MPU_OPP100_UV),
	/* MPU OPP3 - OPP-Turbo */
	OPP_INITIALIZER("mpu", false, 800000000, OMAP4430_VDD_MPU_OPPTURBO_UV),
	/* MPU OPP4 - OPP-SB */
	OPP_INITIALIZER("mpu", false, 1008000000, OMAP4430_VDD_MPU_OPPNITRO_UV),
	/* L3 OPP1 - OPP50 */
	OPP_INITIALIZER("l3_main_1", true, 100000000, OMAP4430_VDD_CORE_OPP50_UV),
	/* L3 OPP2 - OPP100, OPP-Turbo, OPP-SB */
	OPP_INITIALIZER("l3_main_1", true, 200000000, OMAP4430_VDD_CORE_OPP100_UV),
	/* TODO: add IVA, DSP, aess, fdif, gpu */

	/* gpu entries to go here */
};


/**
 * omap4_opp_init() - initialize omap4 opp table
 */
static int __init omap4_opp_init(void)
{
	int r = -ENODEV;
#if 0
	struct clk *gpu_fclk;
	struct device *dev;
#endif

	if (!cpu_is_omap44xx())
		return r;

	r = omap_init_opp_table(omap44xx_opp_def_list,
			ARRAY_SIZE(omap44xx_opp_def_list));

#if 0
	sgx_clk = clk_get(NULL, "dpll_per_m7x2_ck");
	gpu_fclk = clk_get(NULL, "gpu_fck");
	/* Set SGX parent to PER DPLL */
	clk_set_parent(gpu_fclk, sgx_clk);
	clk_put(gpu_fclk);

        dev = find_dev_ptr("gpu");
        if (dev)
                opp_populate_rate_fns(dev, omap4_sgx_set_rate,
                                omap4_sgx_get_rate);

#else
	{
	/* XXX temp. work-around to enable SGX module */
	u32* reg = OMAP4430_CM_GFX_GFX_CLKCTRL;
	u32 val;
	val = *reg & ~0x3;	/* clear bottom 2 bits */
	*reg = val | 2;	/* explicitly enable module */
	}
#endif

	return r;
}
device_initcall(omap4_opp_init);
