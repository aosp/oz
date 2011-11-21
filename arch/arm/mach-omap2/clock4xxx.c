/*
 * clock4xxx.c - OMAP4xxx-specific clock functions
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *
 * Contacts:
 * Mike Turquette <mturquette@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#undef DEBUG

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include "clock.h"
#include "cm.h"
#include "prm.h"
#include "prm-regbits-24xx.h"
#include "prm-regbits-44xx.h"
#include "cm-regbits-44xx.h"

#define MAX_DPLL_WAIT_TRIES	1000000

/*
 * PRCM bug WA: DPLL_ABE must be reconfigured before clocks initialize
 * when boot by warm reset
 */
void omap4_clk_prepare_for_dpll_relock(void)
{
	u32 reset_state, warm_state;
	u32 mask, bits, module, offset;
	int i = 0;

	/* Read the last reset state */
	reset_state = prm_read_mod_reg(OMAP4430_PRM_DEVICE_MOD,
						OMAP4_PRM_RSTST_OFFSET);

	/* Global Warm Reset is already cared to WA before warm resetting */
	warm_state = OMAP4430_MPU_WDT_RST_MASK | \
			OMAP4430_EXTERNAL_WARM_RST_MASK;

	/* don't care WA when not warm reset */
	if (!(reset_state & warm_state))
		return;

	/* Support only sys_clk is 38.4 MHz */
	module = OMAP4430_PRM_CKGEN_MOD;
	offset = OMAP4_CM_SYS_CLKSEL_OFFSET;

	if (prm_read_mod_reg(module, offset) != 0x7)
		return;

	module = OMAP4430_CM1_CKGEN_MOD;

	/* Reconfigure DPLL_MULT bits of CM_CLKSEL_DPLL_ABE */
	mask = OMAP4430_DPLL_MULT_MASK;
	bits = 0x2ee << OMAP4430_DPLL_MULT_SHIFT;
	offset = OMAP4_CM_CLKSEL_DPLL_ABE_OFFSET;
	cm_rmw_mod_reg_bits(mask, bits, module, offset);

	/* Lock the ABE DPLL */
	mask = bits = OMAP4430_DPLL_EN_MASK;
	offset = OMAP4_CM_CLKMODE_DPLL_ABE_OFFSET;
	cm_rmw_mod_reg_bits(mask, bits, module, offset);

	/* Wait until DPLL is locked */
	mask = OMAP4430_ST_DPLL_CLK_MASK;
	offset = OMAP4_CM_IDLEST_DPLL_ABE_OFFSET;

	while (((cm_read_mod_reg(module, offset) & mask) != 0x1) &&
				(i < MAX_DPLL_WAIT_TRIES)) {
		i++;
		udelay(1);
	}

	if (i >= MAX_DPLL_WAIT_TRIES)
		pr_err("Warm Reset WA: failed to lock the ABE DPLL\n");
	else
		pr_info("Warm Reset WA: succeeded to reconfigure the ABE DPLL\n");
}
/*
 * PRCM bug WA: DPLL_ABE must be enabled when performing a warm reset or
 * things go badly...
 */
void omap4_clk_prepare_for_reboot(void)
{
	struct clk *dpll_abe_ck;

	dpll_abe_ck = clk_get(NULL, "dpll_abe_ck");

	omap3_noncore_dpll_enable(dpll_abe_ck);
}
