/*
 * linux/drivers/video/omap2/dss/dsi.c
 *
 * Copyright (C) 2009 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DSS_SUBSYS_NAME "DSI"

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/seq_file.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <plat/board.h>
#include <plat/display.h>
#include <plat/clock.h>

#include "dss.h"

/*#define VERBOSE_IRQ*/
#define DSI_CATCH_MISSING_TE

struct dsi_reg { u16 idx; };

#define DSI_REG(idx)		((const struct dsi_reg) { idx })

/* DSI Protocol Engine */

#define DSI_REVISION			DSI_REG(0x0000)
#define DSI_SYSCONFIG			DSI_REG(0x0010)
#define DSI_SYSSTATUS			DSI_REG(0x0014)
#define DSI_IRQSTATUS			DSI_REG(0x0018)
#define DSI_IRQENABLE			DSI_REG(0x001C)
#define DSI_CTRL			DSI_REG(0x0040)
#define DSI_COMPLEXIO_CFG1		DSI_REG(0x0048)
#define DSI_COMPLEXIO_IRQ_STATUS	DSI_REG(0x004C)
#define DSI_COMPLEXIO_IRQ_ENABLE	DSI_REG(0x0050)
#define DSI_CLK_CTRL			DSI_REG(0x0054)
#define DSI_TIMING1			DSI_REG(0x0058)
#define DSI_TIMING2			DSI_REG(0x005C)
#define DSI_VM_TIMING1			DSI_REG(0x0060)
#define DSI_VM_TIMING2			DSI_REG(0x0064)
#define DSI_VM_TIMING3			DSI_REG(0x0068)
#define DSI_CLK_TIMING			DSI_REG(0x006C)
#define DSI_TX_FIFO_VC_SIZE		DSI_REG(0x0070)
#define DSI_RX_FIFO_VC_SIZE		DSI_REG(0x0074)
#define DSI_COMPLEXIO_CFG2		DSI_REG(0x0078)
#define DSI_RX_FIFO_VC_FULLNESS		DSI_REG(0x007C)
#define DSI_VM_TIMING4			DSI_REG(0x0080)
#define DSI_TX_FIFO_VC_EMPTINESS	DSI_REG(0x0084)
#define DSI_VM_TIMING5			DSI_REG(0x0088)
#define DSI_VM_TIMING6			DSI_REG(0x008C)
#define DSI_VM_TIMING7			DSI_REG(0x0090)
#define DSI_STOPCLK_TIMING		DSI_REG(0x0094)
#define DSI_VC_CTRL(n)			DSI_REG(0x0100 + (n * 0x20))
#define DSI_VC_TE(n)			DSI_REG(0x0104 + (n * 0x20))
#define DSI_VC_LONG_PACKET_HEADER(n)	DSI_REG(0x0108 + (n * 0x20))
#define DSI_VC_LONG_PACKET_PAYLOAD(n)	DSI_REG(0x010C + (n * 0x20))
#define DSI_VC_SHORT_PACKET_HEADER(n)	DSI_REG(0x0110 + (n * 0x20))
#define DSI_VC_IRQSTATUS(n)		DSI_REG(0x0118 + (n * 0x20))
#define DSI_VC_IRQENABLE(n)		DSI_REG(0x011C + (n * 0x20))

/* DSIPHY_SCP */

#define DSI_DSIPHY_CFG0			DSI_REG(0x200 + 0x0000)
#define DSI_DSIPHY_CFG1			DSI_REG(0x200 + 0x0004)
#define DSI_DSIPHY_CFG2			DSI_REG(0x200 + 0x0008)
#define DSI_DSIPHY_CFG5			DSI_REG(0x200 + 0x0014)

/* DSI Rev 3.0 Registers */
#define DSI_DSIPHY_CFG8			DSI_REG(0x200 + 0x0020)
#define DSI_DSIPHY_CFG12		DSI_REG(0x200 + 0x0030)
#define DSI_DSIPHY_CFG14		DSI_REG(0x200 + 0x0038)
#define DSI_TE_HSYNC_WIDTH(n)		DSI_REG(0xA0 + (0xC * n))
#define DSI_TE_VSYNC_WIDTH(n)		DSI_REG(0xA4 + (0xC * n))
#define DSI_TE_HSYNC_NUMBER(n)		DSI_REG(0xA8 + (0xC * n))

/* DSI_PLL_CTRL_SCP */

#define DSI_PLL_CONTROL			DSI_REG(0x300 + 0x0000)
#define DSI_PLL_STATUS			DSI_REG(0x300 + 0x0004)
#define DSI_PLL_GO			DSI_REG(0x300 + 0x0008)
#define DSI_PLL_CONFIGURATION1		DSI_REG(0x300 + 0x000C)
#define DSI_PLL_CONFIGURATION2		DSI_REG(0x300 + 0x0010)

#define REG_GET(no, idx, start, end) \
	FLD_GET(dsi_read_reg(no, idx), start, end)

#define REG_FLD_MOD(no, idx, val, start, end) \
	dsi_write_reg(no, idx, FLD_MOD(dsi_read_reg(no, idx), val, start, end))

/* Global interrupts */
#define DSI_IRQ_VC0		(1 << 0)
#define DSI_IRQ_VC1		(1 << 1)
#define DSI_IRQ_VC2		(1 << 2)
#define DSI_IRQ_VC3		(1 << 3)
#define DSI_IRQ_WAKEUP		(1 << 4)
#define DSI_IRQ_RESYNC		(1 << 5)
#define DSI_IRQ_PLL_LOCK	(1 << 7)
#define DSI_IRQ_PLL_UNLOCK	(1 << 8)
#define DSI_IRQ_PLL_RECALL	(1 << 9)
#define DSI_IRQ_COMPLEXIO_ERR	(1 << 10)
#define DSI_IRQ_HS_TX_TIMEOUT	(1 << 14)
#define DSI_IRQ_LP_RX_TIMEOUT	(1 << 15)
#define DSI_IRQ_TE_TRIGGER	(1 << 16)
#define DSI_IRQ_ACK_TRIGGER	(1 << 17)
#define DSI_IRQ_SYNC_LOST	(1 << 18)
#define DSI_IRQ_LDO_POWER_GOOD	(1 << 19)
#define DSI_IRQ_TA_TIMEOUT	(1 << 20)
#define DSI_IRQ_ERROR_MASK \
	(DSI_IRQ_HS_TX_TIMEOUT | DSI_IRQ_LP_RX_TIMEOUT | DSI_IRQ_SYNC_LOST | \
	DSI_IRQ_TA_TIMEOUT)
#define DSI_IRQ_CHANNEL_MASK	0xf

/* Virtual channel interrupts */
#define DSI_VC_IRQ_CS		(1 << 0)
#define DSI_VC_IRQ_ECC_CORR	(1 << 1)
#define DSI_VC_IRQ_PACKET_SENT	(1 << 2)
#define DSI_VC_IRQ_FIFO_TX_OVF	(1 << 3)
#define DSI_VC_IRQ_FIFO_RX_OVF	(1 << 4)
#define DSI_VC_IRQ_BTA		(1 << 5)
#define DSI_VC_IRQ_ECC_NO_CORR	(1 << 6)
#define DSI_VC_IRQ_FIFO_TX_UDF	(1 << 7)
#define DSI_VC_IRQ_PP_BUSY_CHANGE (1 << 8)
#define DSI_VC_IRQ_ERROR_MASK \
	(DSI_VC_IRQ_CS | DSI_VC_IRQ_ECC_CORR | DSI_VC_IRQ_FIFO_TX_OVF | \
	DSI_VC_IRQ_FIFO_RX_OVF | DSI_VC_IRQ_ECC_NO_CORR | \
	DSI_VC_IRQ_FIFO_TX_UDF)

/* ComplexIO interrupts */
#define DSI_CIO_IRQ_ERRSYNCESC1		(1 << 0)
#define DSI_CIO_IRQ_ERRSYNCESC2		(1 << 1)
#define DSI_CIO_IRQ_ERRSYNCESC3		(1 << 2)
#define DSI_CIO_IRQ_ERRESC1		(1 << 5)
#define DSI_CIO_IRQ_ERRESC2		(1 << 6)
#define DSI_CIO_IRQ_ERRESC3		(1 << 7)
#define DSI_CIO_IRQ_ERRCONTROL1		(1 << 10)
#define DSI_CIO_IRQ_ERRCONTROL2		(1 << 11)
#define DSI_CIO_IRQ_ERRCONTROL3		(1 << 12)
#define DSI_CIO_IRQ_STATEULPS1		(1 << 15)
#define DSI_CIO_IRQ_STATEULPS2		(1 << 16)
#define DSI_CIO_IRQ_STATEULPS3		(1 << 17)
#define DSI_CIO_IRQ_ERRCONTENTIONLP0_1	(1 << 20)
#define DSI_CIO_IRQ_ERRCONTENTIONLP1_1	(1 << 21)
#define DSI_CIO_IRQ_ERRCONTENTIONLP0_2	(1 << 22)
#define DSI_CIO_IRQ_ERRCONTENTIONLP1_2	(1 << 23)
#define DSI_CIO_IRQ_ERRCONTENTIONLP0_3	(1 << 24)
#define DSI_CIO_IRQ_ERRCONTENTIONLP1_3	(1 << 25)
#define DSI_CIO_IRQ_ULPSACTIVENOT_ALL0	(1 << 30)
#define DSI_CIO_IRQ_ULPSACTIVENOT_ALL1	(1 << 31)
#define DSI_CIO_IRQ_ERROR_MASK \
	(DSI_CIO_IRQ_ERRSYNCESC1 | DSI_CIO_IRQ_ERRSYNCESC2 | \
	 DSI_CIO_IRQ_ERRSYNCESC3 | DSI_CIO_IRQ_ERRESC1 | DSI_CIO_IRQ_ERRESC2 | \
	 DSI_CIO_IRQ_ERRESC3 | DSI_CIO_IRQ_ERRCONTROL1 | \
	 DSI_CIO_IRQ_ERRCONTROL2 | DSI_CIO_IRQ_ERRCONTROL3 | \
	 DSI_CIO_IRQ_ERRCONTENTIONLP0_1 | DSI_CIO_IRQ_ERRCONTENTIONLP1_1 | \
	 DSI_CIO_IRQ_ERRCONTENTIONLP0_2 | DSI_CIO_IRQ_ERRCONTENTIONLP1_2 | \
	 DSI_CIO_IRQ_ERRCONTENTIONLP0_3 | DSI_CIO_IRQ_ERRCONTENTIONLP1_3)

#define DSI_DT_DCS_SHORT_WRITE_0	0x05
#define DSI_DT_DCS_SHORT_WRITE_1	0x15
#define DSI_DT_DCS_READ			0x06
#define DSI_DT_SET_MAX_RET_PKG_SIZE	0x37
#define DSI_DT_NULL_PACKET		0x09
#define DSI_DT_DCS_LONG_WRITE		0x39

#define DSI_DT_RX_ACK_WITH_ERR		0x02
#define DSI_DT_RX_DCS_LONG_READ		0x1c
#define DSI_DT_RX_SHORT_READ_1		0x21
#define DSI_DT_RX_SHORT_READ_2		0x22

#define FINT_MAX (cpu_is_omap44xx() ? 2500000 : 2100000)
#define FINT_MIN (cpu_is_omap44xx() ? 750000 : 500000)
#define REGN_MAX (cpu_is_omap44xx() ? (1 << 8) : (1 << 7))
#define REGM_MAX (cpu_is_omap44xx() ? ((1 << 12) - 1) : ((1 << 11) - 1))
#define REGM_DISPC_MAX (cpu_is_omap44xx() ? (1 << 5) : (1 << 4))
#define REGM_DSI_MAX (cpu_is_omap44xx() ? (1 << 5) : (1 << 4))
#define LP_DIV_MAX ((1 << 13) - 1)

enum fifo_size {
	DSI_FIFO_SIZE_0		= 0,
	DSI_FIFO_SIZE_32	= 1,
	DSI_FIFO_SIZE_64	= 2,
	DSI_FIFO_SIZE_96	= 3,
	DSI_FIFO_SIZE_128	= 4,
};

enum dsi_vc_mode {
	DSI_VC_MODE_L4 = 0,
	DSI_VC_MODE_VP,
};

struct dsi_update_region {
	u16 x, y, w, h;
	struct omap_dss_device *device;
};

struct dsi_irq_stats {
	unsigned long last_reset;
	unsigned irq_count;
	unsigned dsi_irqs[32];
	unsigned vc_irqs[4][32];
	unsigned cio_irqs[32];
};

static struct dsi_struct {
	void __iomem	*base;

	struct dsi_clock_info current_cinfo;

	struct regulator *vdds_dsi_reg;

	struct {
		enum dsi_vc_mode mode;
		struct omap_dss_device *dssdev;
		enum fifo_size fifo_size;
		int dest_per;
	} vc[4];

	struct mutex lock;
	struct semaphore bus_lock;

	unsigned pll_locked;

	struct completion bta_completion;
	void (*bta_callback)(struct dsi_struct *ds);

	int update_channel;
	struct dsi_update_region update_region;

	bool te_enabled;

	struct workqueue_struct *workqueue;

	void (*framedone_callback)(int, void *);
	void *framedone_data;

	struct delayed_work framedone_timeout_work;

#ifdef DSI_CATCH_MISSING_TE
	struct timer_list te_timer;
#endif

	unsigned long cache_req_pck;
	unsigned long cache_clk_freq;
	struct dsi_clock_info cache_cinfo;

	u32		errors;
	spinlock_t	errors_lock;
#ifdef DEBUG
	ktime_t perf_setup_time;
	ktime_t perf_start_time;
#endif
	int debug_read;
	int debug_write;

#ifdef CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS
	spinlock_t irq_stats_lock;
	struct dsi_irq_stats irq_stats;
#endif
	bool enabled;

	struct omap_display_platform_data *pdata;
	struct platform_device *pdev;

	/* instance variables */
	int irq_vsync;
	int irq_dss;
	int irq_framedone;
	int channel;
	enum dss_clk_source clk_src_dispc;
	enum dss_clk_source clk_src_dsi;

	char *pdev_name;
	char *name;

} *dsis[OMAP_DSS_CHANNEL_LCD2 + 1], dsi[] = { {
	.irq_vsync	= DISPC_IRQ_VSYNC,
	.irq_dss	= OMAP44XX_IRQ_DSS_DSI1,
	.irq_framedone	= DISPC_IRQ_FRAMEDONE,
	.channel	= OMAP_DSS_CHANNEL_LCD,
	.clk_src_dispc	= DSS_SRC_PLL1_CLK1,
	.clk_src_dsi	= DSS_SRC_PLL1_CLK2,
	.pdev_name	= "dss_dsi1",
	.name		= "DSI1",
}, {
	.irq_vsync	= DISPC_IRQ_VSYNC2,
	.irq_dss	= OMAP44XX_IRQ_DSS_DSI2,
	.irq_framedone	= DISPC_IRQ_FRAMEDONE2,
	.channel	= OMAP_DSS_CHANNEL_LCD2,
	.clk_src_dispc	= DSS_SRC_PLL2_CLK1,
	.clk_src_dsi	= DSS_SRC_PLL2_CLK2,
	.pdev_name	= "dss_dsi2",
	.name		= "DSI2",
} };

#ifdef DEBUG
static unsigned int dsi_perf;
module_param_named(dsi_perf, dsi_perf, bool, 0644);
#endif

static inline void dsi_write_reg(struct dsi_struct *ds,
		const struct dsi_reg idx, u32 val)
{
	__raw_writel(val, ds->base + idx.idx);
}

static inline u32 dsi_read_reg(struct dsi_struct *ds,
		const struct dsi_reg idx)
{
	return __raw_readl(ds->base + idx.idx);
}

static inline struct dsi_struct *dsi4channel(enum omap_channel channel)
{
	BUG_ON(channel < 0 || channel >= ARRAY_SIZE(dsis) ||
						dsis[channel] == NULL);
	return dsis[channel];
}

static inline struct dsi_struct *dss2dsi(struct omap_dss_device *dssdev)
{
	return dsi4channel(dssdev->channel);
}

void dsi_save_context(void)
{
}

void dsi_restore_context(void)
{
}

void dsi_bus_lock(struct omap_dss_device *dssdev)
{
	struct dsi_struct *ds = dss2dsi(dssdev);
	down(&ds->bus_lock);
}
EXPORT_SYMBOL(dsi_bus_lock);

void dsi_bus_unlock(struct omap_dss_device *dssdev)
{
	struct dsi_struct *ds = dss2dsi(dssdev);
	up(&ds->bus_lock);
}
EXPORT_SYMBOL(dsi_bus_unlock);

static bool dsi_bus_is_locked(struct dsi_struct *ds)
{
	return ds->bus_lock.count == 0;
}

static inline int wait_for_bit_change(struct dsi_struct *ds,
		const struct dsi_reg idx, int bitnum,
		int value)
{
	int t = 100000;

	while (REG_GET(ds, idx, bitnum, bitnum) != value) {
		if (--t == 0)
			return !value;
	}

	return value;
}

#ifdef DEBUG
static void dsi_perf_mark_setup(struct dsi_struct *ds)
{
	ds->perf_setup_time = ktime_get();
}

static void dsi_perf_mark_start(struct dsi_struct *ds)
{
	ds->perf_start_time = ktime_get();
}

static void dsi_perf_show(struct dsi_struct *ds,
	const char *name)
{
	ktime_t t, setup_time, trans_time;
	u32 total_bytes;
	u32 setup_us, trans_us, total_us;

	if (!dsi_perf)
		return;

	t = ktime_get();

	setup_time = ktime_sub(ds->perf_start_time,
						ds->perf_setup_time);
	setup_us = (u32)ktime_to_us(setup_time);
	if (setup_us == 0)
		setup_us = 1;

	trans_time = ktime_sub(t, ds->perf_start_time);
	trans_us = (u32)ktime_to_us(trans_time);
	if (trans_us == 0)
		trans_us = 1;

	total_us = setup_us + trans_us;

	total_bytes = ds->update_region.w *
		ds->update_region.h *
		ds->update_region.device->ctrl.pixel_size / 8;

	printk(KERN_INFO "DSI(%s): %u us + %u us = %u us (%uHz), "
			"%u bytes, %u kbytes/sec\n",
			name,
			setup_us,
			trans_us,
			total_us,
			1000*1000 / total_us,
			total_bytes,
			total_bytes * 1000 / total_us);
}
#else
#define dsi_perf_mark_setup(x)
#define dsi_perf_mark_start(x)
#define dsi_perf_show(x, y)
#endif

static void print_irq_status(u32 status)
{
#ifndef VERBOSE_IRQ
	if ((status & ~DSI_IRQ_CHANNEL_MASK) == 0)
		return;
#endif
	printk(KERN_DEBUG "DSI IRQ: 0x%x: ", status);

#define PIS(x) \
	if (status & DSI_IRQ_##x) \
		printk(#x " ");
#ifdef VERBOSE_IRQ
	PIS(VC0);
	PIS(VC1);
	PIS(VC2);
	PIS(VC3);
#endif
	PIS(WAKEUP);
	PIS(RESYNC);
	PIS(PLL_LOCK);
	PIS(PLL_UNLOCK);
	PIS(PLL_RECALL);
	PIS(COMPLEXIO_ERR);
	PIS(HS_TX_TIMEOUT);
	PIS(LP_RX_TIMEOUT);
	PIS(TE_TRIGGER);
	PIS(ACK_TRIGGER);
	PIS(SYNC_LOST);
	PIS(LDO_POWER_GOOD);
	PIS(TA_TIMEOUT);
#undef PIS

	printk("\n");
}

static void print_irq_status_vc(int channel, u32 status)
{
#ifndef VERBOSE_IRQ
	if ((status & ~DSI_VC_IRQ_PACKET_SENT) == 0)
		return;
#endif
	printk(KERN_DEBUG "DSI VC(%d) IRQ 0x%x: ", channel, status);

#define PIS(x) \
	if (status & DSI_VC_IRQ_##x) \
		printk(#x " ");
	PIS(CS);
	PIS(ECC_CORR);
#ifdef VERBOSE_IRQ
	PIS(PACKET_SENT);
#endif
	PIS(FIFO_TX_OVF);
	PIS(FIFO_RX_OVF);
	PIS(BTA);
	PIS(ECC_NO_CORR);
	PIS(FIFO_TX_UDF);
	PIS(PP_BUSY_CHANGE);
#undef PIS
	printk("\n");
}

static void print_irq_status_cio(u32 status)
{
	printk(KERN_DEBUG "DSI CIO IRQ 0x%x: ", status);

#define PIS(x) \
	if (status & DSI_CIO_IRQ_##x) \
		printk(#x " ");
	PIS(ERRSYNCESC1);
	PIS(ERRSYNCESC2);
	PIS(ERRSYNCESC3);
	PIS(ERRESC1);
	PIS(ERRESC2);
	PIS(ERRESC3);
	PIS(ERRCONTROL1);
	PIS(ERRCONTROL2);
	PIS(ERRCONTROL3);
	PIS(STATEULPS1);
	PIS(STATEULPS2);
	PIS(STATEULPS3);
	PIS(ERRCONTENTIONLP0_1);
	PIS(ERRCONTENTIONLP1_1);
	PIS(ERRCONTENTIONLP0_2);
	PIS(ERRCONTENTIONLP1_2);
	PIS(ERRCONTENTIONLP0_3);
	PIS(ERRCONTENTIONLP1_3);
	PIS(ULPSACTIVENOT_ALL0);
	PIS(ULPSACTIVENOT_ALL1);
#undef PIS

	printk("\n");
}

static int debug_irq;

/* called from dss in OMAP3, in OMAP4 there is a dedicated
* interrupt line for DSI */
irqreturn_t dsi_irq_handler(int irq, void *arg)

{
	u32 irqstatus, vcstatus, ciostatus;
	int i;
	struct dsi_struct *ds = arg ? : dsi;

	irqstatus = dsi_read_reg(ds, DSI_IRQSTATUS);

#ifdef CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS
	spin_lock(&ds->irq_stats_lock);
	ds->irq_stats.irq_count++;
	dss_collect_irq_stats(irqstatus, ds->irq_stats.dsi_irqs);
#endif

	if (irqstatus & DSI_IRQ_ERROR_MASK) {
		DSSERR("DSI error, irqstatus %x\n", irqstatus);
		print_irq_status(irqstatus);
		spin_lock(&ds->errors_lock);
		ds->errors |= irqstatus & DSI_IRQ_ERROR_MASK;
		spin_unlock(&ds->errors_lock);
	} else if (debug_irq) {
		print_irq_status(irqstatus);
	}

#ifdef DSI_CATCH_MISSING_TE
	if (irqstatus & DSI_IRQ_TE_TRIGGER)
		del_timer(&ds->te_timer);
#endif

	for (i = 0; i < 4; ++i) {
		if ((irqstatus & (1<<i)) == 0)
			continue;

		vcstatus = dsi_read_reg(ds, DSI_VC_IRQSTATUS(i));

#ifdef CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS
		dss_collect_irq_stats(vcstatus, ds->irq_stats.vc_irqs[i]);
#endif

		if (vcstatus & DSI_VC_IRQ_BTA) {
			complete(&ds->bta_completion);

			if (ds->bta_callback)
				ds->bta_callback(ds);
		}

		if (vcstatus & DSI_VC_IRQ_ERROR_MASK) {
			DSSERR("DSI VC(%d) error, vc irqstatus %x\n",
					i, vcstatus);
			print_irq_status_vc(i, vcstatus);
		} else if (debug_irq) {
			print_irq_status_vc(i, vcstatus);
		}

		dsi_write_reg(ds, DSI_VC_IRQSTATUS(i), vcstatus);
		/* flush posted write */
		dsi_read_reg(ds, DSI_VC_IRQSTATUS(i));
	}

	if (irqstatus & DSI_IRQ_COMPLEXIO_ERR) {
		ciostatus = dsi_read_reg(ds, DSI_COMPLEXIO_IRQ_STATUS);

#ifdef CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS
		dss_collect_irq_stats(ciostatus, ds->irq_stats.cio_irqs);
#endif

		dsi_write_reg(ds, DSI_COMPLEXIO_IRQ_STATUS, ciostatus);
		/* flush posted write */
		dsi_read_reg(ds, DSI_COMPLEXIO_IRQ_STATUS);

		if (ciostatus & DSI_CIO_IRQ_ERROR_MASK) {
			DSSERR("DSI CIO error, cio irqstatus %x\n", ciostatus);
			print_irq_status_cio(ciostatus);
		} else if (debug_irq) {
			print_irq_status_cio(ciostatus);
		}
	}

	dsi_write_reg(ds, DSI_IRQSTATUS, irqstatus & ~DSI_IRQ_CHANNEL_MASK);
	/* flush posted write */
	dsi_read_reg(ds, DSI_IRQSTATUS);

#ifdef CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS
	spin_unlock(&ds->irq_stats_lock);
#endif
	return IRQ_HANDLED;
}

static void _dsi_initialize_irq(struct dsi_struct *ds)
{
	u32 l;
	int i;

	/* disable all interrupts */
	dsi_write_reg(ds, DSI_IRQENABLE, 0);
	for (i = 0; i < 4; ++i)
		dsi_write_reg(ds, DSI_VC_IRQENABLE(i), 0);
	dsi_write_reg(ds, DSI_COMPLEXIO_IRQ_ENABLE, 0);

	/* clear interrupt status */
	l = dsi_read_reg(ds, DSI_IRQSTATUS);
	dsi_write_reg(ds, DSI_IRQSTATUS, l & ~DSI_IRQ_CHANNEL_MASK);

	for (i = 0; i < 4; ++i) {
		l = dsi_read_reg(ds, DSI_VC_IRQSTATUS(i));
		dsi_write_reg(ds, DSI_VC_IRQSTATUS(i), l);
	}

	l = dsi_read_reg(ds, DSI_COMPLEXIO_IRQ_STATUS);
	dsi_write_reg(ds, DSI_COMPLEXIO_IRQ_STATUS, l);

	/* enable error irqs */
	l = DSI_IRQ_ERROR_MASK;
#ifdef DSI_CATCH_MISSING_TE
	l |= DSI_IRQ_TE_TRIGGER;
#endif
	dsi_write_reg(ds, DSI_IRQENABLE, l);

	l = DSI_VC_IRQ_ERROR_MASK;
	for (i = 0; i < 4; ++i)
		dsi_write_reg(ds, DSI_VC_IRQENABLE(i), l);

	l = DSI_CIO_IRQ_ERROR_MASK;
	dsi_write_reg(ds, DSI_COMPLEXIO_IRQ_ENABLE, l);
}

static u32 dsi_get_errors(struct dsi_struct *ds)
{
	unsigned long flags;
	u32 e;

	spin_lock_irqsave(&ds->errors_lock, flags);
	e = ds->errors;
	ds->errors = 0;
	spin_unlock_irqrestore(&ds->errors_lock, flags);
	return e;
}

static void dsi_vc_enable_bta_irq(struct dsi_struct *ds,
	int channel)
{
	u32 l;

	if (omap4_board_rev() == OMAP4_BLAZETABLET_BOARD)
		return;

	dsi_write_reg(ds, DSI_VC_IRQSTATUS(channel), DSI_VC_IRQ_BTA);

	l = dsi_read_reg(ds, DSI_VC_IRQENABLE(channel));
	l |= DSI_VC_IRQ_BTA;
	dsi_write_reg(ds, DSI_VC_IRQENABLE(channel), l);
}

static void dsi_vc_disable_bta_irq(struct dsi_struct *ds,
	int channel)
{
	u32 l;

	if (omap4_board_rev() == OMAP4_BLAZETABLET_BOARD)
		return;

	l = dsi_read_reg(ds, DSI_VC_IRQENABLE(channel));
	l &= ~DSI_VC_IRQ_BTA;
	dsi_write_reg(ds, DSI_VC_IRQENABLE(channel), l);
}

/* DSI func clock. this could also be DSI2_PLL_FCLK */
static inline void enable_clocks(bool enable)
{
	if (enable)
		dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);
	else
		dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
}

/* source clock for DSI PLL. this could also be PCLKFREE */
static inline void dsi_enable_pll_clock(struct dsi_struct *ds,
	bool enable)
{
	if (enable)
		dss_clk_enable(DSS_CLK_FCK2);
	else
		dss_clk_disable(DSS_CLK_FCK2);

	if (enable && ds->pll_locked) {
		if (wait_for_bit_change(ds, DSI_PLL_STATUS, 1, 1) != 1)
			DSSERR("cannot lock PLL when enabling clocks\n");
	}
}

#ifdef DEBUG
static void _dsi_print_reset_status(struct dsi_struct *ds)
{
	u32 l;

	if (!dss_debug)
		return;

	/* A dummy read using the SCP interface to any DSIPHY register is
	 * required after DSIPHY reset to complete the reset of the DSI complex
	 * I/O. */
	l = dsi_read_reg(ds, DSI_DSIPHY_CFG5);

	printk(KERN_DEBUG "DSI resets: ");

	l = dsi_read_reg(ds, DSI_PLL_STATUS);
	printk("PLL (%d) ", FLD_GET(l, 0, 0));

	l = dsi_read_reg(ds, DSI_COMPLEXIO_CFG1);
	printk("CIO (%d) ", FLD_GET(l, 29, 29));

	l = dsi_read_reg(ds, DSI_DSIPHY_CFG5);
	printk("PHY (%x, %d, %d, %d)\n",
			FLD_GET(l, 28, 26),
			FLD_GET(l, 29, 29),
			FLD_GET(l, 30, 30),
			FLD_GET(l, 31, 31));
}
#else
#define _dsi_print_reset_status(ds)
#endif

static inline int dsi_if_enable(struct dsi_struct *ds,
	bool enable)
{
	DSSDBG("dsi_if_enable(%d)\n", enable);

	enable = enable ? 1 : 0;
	REG_FLD_MOD(ds, DSI_CTRL, enable, 0, 0); /* IF_EN */

	if (wait_for_bit_change(ds, DSI_CTRL, 0, enable) != enable) {
			DSSERR("Failed to set dsi_if_enable to %d\n", enable);
			return -EIO;
	}

	return 0;
}

unsigned long dsi_get_pll_dispc_rate(enum omap_channel channel)
{
	struct dsi_struct *ds = dsi4channel(channel);
	return ds->current_cinfo.dsi_pll_dispc_fclk;
}

static unsigned long dsi_get_pll_dsi_rate(struct dsi_struct *ds)
{
	return ds->current_cinfo.dsi_pll_dsi_fclk;
}

static unsigned long dsi_get_txbyteclkhs(struct dsi_struct *ds)
{
	return ds->current_cinfo.clkin4ddr / 16;
}

static unsigned long dsi_fclk_rate(struct dsi_struct *ds)
{
	unsigned long r;

	if (dss_get_dsi_clk_source(ds->channel) == DSS_SRC_DSS1_ALWON_FCLK) {
		/* DSI FCLK source is DSS1_ALWON_FCK, which is dss1_fck */
		r = dss_clk_get_rate(DSS_CLK_FCK1);
	} else {
		/* DSI FCLK source is DSI2_PLL_FCLK */
		r = dsi_get_pll_dsi_rate(ds);
	}

	return r;
}

static int dsi_set_lp_clk_divisor(struct omap_dss_device *dssdev)
{
	unsigned long dsi_fclk;
	unsigned lp_clk_div;
	unsigned long lp_clk;
	struct dsi_struct *ds = dss2dsi(dssdev);

	lp_clk_div = dssdev->phy.dsi.div.lp_clk_div;

	if (lp_clk_div == 0 || lp_clk_div > LP_DIV_MAX)
		return -EINVAL;

	dsi_fclk = dsi_fclk_rate(ds);

	lp_clk = dsi_fclk / 2 / lp_clk_div;

	DSSDBG("LP_CLK_DIV %u, LP_CLK %lu\n", lp_clk_div, lp_clk);
	ds->current_cinfo.lp_clk = lp_clk;
	ds->current_cinfo.lp_clk_div = lp_clk_div;

	REG_FLD_MOD(ds, DSI_CLK_CTRL, lp_clk_div, 12, 0);   /* LP_CLK_DIVISOR */

	REG_FLD_MOD(ds, DSI_CLK_CTRL, dsi_fclk > 30000000 ? 1 : 0,
			21, 21);		/* LP_RX_SYNCHRO_ENABLE */

	return 0;
}


enum dsi_pll_power_state {
	DSI_PLL_POWER_OFF	= 0x0,
	DSI_PLL_POWER_ON_HSCLK	= 0x1,
	DSI_PLL_POWER_ON_ALL	= 0x2,
	DSI_PLL_POWER_ON_DIV	= 0x3,
};

static int dsi_pll_power(struct dsi_struct *ds,
	enum dsi_pll_power_state state)
{
	int t = 0;

	REG_FLD_MOD(ds, DSI_CLK_CTRL, state, 31, 30);	/* PLL_PWR_CMD */

	/* PLL_PWR_STATUS */
	while (FLD_GET(dsi_read_reg(ds, DSI_CLK_CTRL), 29, 28) != state) {
		if (++t > 1000) {
			DSSERR("Failed to set DSI PLL power mode to %d\n",
					state);
			return -ENODEV;
		}
		udelay(1);
	}

	return 0;
}

/* calculate clock rates using dividers in cinfo */
int dsi_calc_clock_rates(struct omap_dss_device *dssdev,
		struct dsi_clock_info *cinfo)
{
	if (cinfo->regn == 0 || cinfo->regn > REGN_MAX)
		return -EINVAL;

	if (cinfo->regm == 0 || cinfo->regm > REGM_MAX)
		return -EINVAL;

	if (cinfo->regm_dispc > REGM_DISPC_MAX)
		return -EINVAL;

	if (cinfo->regm_dsi > REGM_DSI_MAX)
		return -EINVAL;

	if (cinfo->use_dss2_fck) {
		cinfo->clkin = dss_clk_get_rate(DSS_CLK_FCK2);
		if (cpu_is_omap44xx())
			cinfo->clkin = 38400000;
		/* XXX it is unclear if highfreq should be used
		 * with DSS2_FCK source also */
		cinfo->highfreq = 0;
	} else {
		cinfo->clkin = dispc_pclk_rate(dssdev->manager->id);

		if (cinfo->clkin < 32000000)
			cinfo->highfreq = 0;
		else
			cinfo->highfreq = 1;
	}

	cinfo->fint = cinfo->clkin / (cinfo->regn * (cinfo->highfreq ? 2 : 1));

	if (cinfo->fint > FINT_MAX || cinfo->fint < FINT_MIN)
		return -EINVAL;

	cinfo->clkin4ddr = 2 * cinfo->regm * cinfo->fint;

	if (cinfo->clkin4ddr > 1800 * 1000 * 1000)
		return -EINVAL;

	if (cinfo->regm_dispc > 0)
		cinfo->dsi_pll_dispc_fclk =
			cinfo->clkin4ddr / cinfo->regm_dispc;
	else
		cinfo->dsi_pll_dispc_fclk = 0;

	if (cinfo->regm_dsi > 0)
		cinfo->dsi_pll_dsi_fclk = cinfo->clkin4ddr / cinfo->regm_dsi;
	else
		cinfo->dsi_pll_dsi_fclk = 0;

	return 0;
}

int dsi_pll_calc_clock_div_pck(enum omap_channel channel,
		bool is_tft, unsigned long req_pck,
		struct dsi_clock_info *dsi_cinfo,
		struct dispc_clock_info *dispc_cinfo)
{
	struct dsi_clock_info cur, best;
	struct dispc_clock_info best_dispc;
	int min_fck_per_pck;
	int match = 0;
	unsigned long dss_clk_fck2;
	struct dsi_struct *ds = dsi4channel(channel);

	dss_clk_fck2 = dss_clk_get_rate(DSS_CLK_FCK2);

	if (req_pck == ds->cache_req_pck &&
			ds->cache_cinfo.clkin == dss_clk_fck2) {
		DSSDBG("DSI clock info found from cache\n");
		*dsi_cinfo = ds->cache_cinfo;
		dispc_find_clk_divs(is_tft, req_pck,
				dsi_cinfo->dsi_pll_dispc_fclk, dispc_cinfo);
		return 0;
	}

	min_fck_per_pck = CONFIG_OMAP2_DSS_MIN_FCK_PER_PCK;

	if (min_fck_per_pck &&
		req_pck * min_fck_per_pck > DISPC_MAX_FCK) {
		DSSERR("Requested pixel clock not possible with the current "
				"OMAP2_DSS_MIN_FCK_PER_PCK setting. Turning "
				"the constraint off.\n");
		min_fck_per_pck = 0;
	}

	DSSDBG("dsi_pll_calc\n");

retry:
	memset(&best, 0, sizeof(best));
	memset(&best_dispc, 0, sizeof(best_dispc));

	memset(&cur, 0, sizeof(cur));
	cur.clkin = dss_clk_fck2;
	cur.use_dss2_fck = 1;
	cur.highfreq = 0;

	/* no highfreq: 0.75MHz < Fint = clkin / regn < 2.1MHz */
	/* highfreq: 0.75MHz < Fint = clkin / (2*regn) < 2.1MHz */
	/* To reduce PLL lock time, keep Fint high (around 2 MHz) */
	for (cur.regn = 1; cur.regn < REGN_MAX; ++cur.regn) {
		if (cur.highfreq == 0)
			cur.fint = cur.clkin / cur.regn;
		else
			cur.fint = cur.clkin / (2 * cur.regn);

		if (cur.fint > FINT_MAX || cur.fint < FINT_MIN)
			continue;

		/* DSIPHY(MHz) = (2 * regm / regn) * (clkin / (highfreq + 1)) */
		for (cur.regm = 1; cur.regm < REGM_MAX; ++cur.regm) {
			unsigned long a, b;

			a = 2 * cur.regm * (cur.clkin/1000);
			b = cur.regn * (cur.highfreq + 1);
			cur.clkin4ddr = a / b * 1000;

			if (cur.clkin4ddr > 1800 * 1000 * 1000)
				break;

			/* OMAP3:
			 * DSI1_PLL_FCLK(MHz) = DSIPHY(MHz) / regm3  < 173MHz
			 * OMAP4:
			 * PLLx_CLK1(MHz) = DSIPHY(MHz) / regm4 < 173MHz */
			for (cur.regm_dispc = 1;
					cur.regm_dispc < REGM_DISPC_MAX;
					++cur.regm_dispc) {
				struct dispc_clock_info cur_dispc;
				cur.dsi_pll_dispc_fclk =
					cur.clkin4ddr / cur.regm_dispc;

				/* this will narrow down the search a bit,
				 * but still give pixclocks below what was
				 * requested */
				if (cur.dsi_pll_dispc_fclk  < req_pck)
					break;

				if (cur.dsi_pll_dispc_fclk > DISPC_MAX_FCK)
					continue;

				if (min_fck_per_pck &&
					cur.dsi_pll_dispc_fclk <
						req_pck * min_fck_per_pck)
					continue;

				match = 1;

				dispc_find_clk_divs(is_tft, req_pck,
						cur.dsi_pll_dispc_fclk,
						&cur_dispc);

				if (abs(cur_dispc.pck - req_pck) <
						abs(best_dispc.pck - req_pck)) {
					best = cur;
					best_dispc = cur_dispc;

					if (cur_dispc.pck == req_pck)
						goto found;
				}
			}
		}
	}
found:
	if (!match) {
		if (min_fck_per_pck) {
			DSSERR("Could not find suitable clock settings.\n"
					"Turning FCK/PCK constraint off and"
					"trying again.\n");
			min_fck_per_pck = 0;
			goto retry;
		}

		DSSERR("Could not find suitable clock settings.\n");

		return -EINVAL;
	}

	/* OMAP3: DSI2_PLL_FCLK (regm4) is not used
	 * OMAP4: PLLx_CLK2 (regm5) is not used */
	best.regm_dsi = 0;
	best.dsi_pll_dsi_fclk = 0;

	if (dsi_cinfo)
		*dsi_cinfo = best;
	if (dispc_cinfo)
		*dispc_cinfo = best_dispc;

	ds->cache_req_pck = req_pck;
	ds->cache_clk_freq = 0;
	ds->cache_cinfo = best;

	return 0;
}

int dsi_pll_set_clock_div(enum omap_channel channel,
	struct dsi_clock_info *cinfo)
{
	int r = 0;
	u32 l;
	int f = 0;
	struct dsi_struct *ds = dsi4channel(channel);

	DSSDBGF();

	ds->current_cinfo.fint = cinfo->fint;
	ds->current_cinfo.clkin4ddr = cinfo->clkin4ddr;
	ds->current_cinfo.dsi_pll_dispc_fclk = cinfo->dsi_pll_dispc_fclk;
	ds->current_cinfo.dsi_pll_dsi_fclk = cinfo->dsi_pll_dsi_fclk;

	ds->current_cinfo.regn = cinfo->regn;
	ds->current_cinfo.regm = cinfo->regm;
	ds->current_cinfo.regm_dispc = cinfo->regm_dispc;
	ds->current_cinfo.regm_dsi = cinfo->regm_dsi;

	DSSDBG("DSI Fint %ld\n", cinfo->fint);

	DSSDBG("clkin (%s) rate %ld, highfreq %d\n",
			cinfo->use_dss2_fck ? "dss2_fck" : "pclkfree",
			cinfo->clkin,
			cinfo->highfreq);

	/* DSIPHY == CLKIN4DDR */
	DSSDBG("CLKIN4DDR = 2 * %d / %d * %lu / %d = %lu\n",
			cinfo->regm,
			cinfo->regn,
			cinfo->clkin,
			cinfo->highfreq + 1,
			cinfo->clkin4ddr);

	DSSDBG("Data rate on 1 DSI lane %ld Mbps\n",
			cinfo->clkin4ddr / 1000 / 1000 / 2);

	DSSDBG("Clock lane freq %ld Hz\n", cinfo->clkin4ddr / 4);

	DSSDBG("regm3(OMAP3) / regm4(OMAP4) = %d,"
		" dsi1_pll_fclk(OMAP3) / pllx_clk1(OMAP4) = %lu\n",
			cinfo->regm_dispc, cinfo->dsi_pll_dispc_fclk);
	DSSDBG("regm4(OMAP3) / regm5(OMAP4) = %d,"
		" dsi2_pll_fclk(OMAP3) / pllx_clk2(OMAP4) = %lu\n",
			cinfo->regm_dsi, cinfo->dsi_pll_dsi_fclk);

	/* DSI_PLL_AUTOMODE = manual */
	REG_FLD_MOD(ds, DSI_PLL_CONTROL, 0, 0, 0);

	l = dsi_read_reg(ds, DSI_PLL_CONFIGURATION1);
	l = FLD_MOD(l, 1, 0, 0);		/* DSI_PLL_STOPMODE */
	l = FLD_MOD(l, cinfo->regn - 1, (cpu_is_omap44xx()) ? 8 : 7,
			1);		/* DSI_PLL_REGN */
	l = FLD_MOD(l, cinfo->regm, (cpu_is_omap44xx()) ? 20 : 18,
			(cpu_is_omap44xx()) ? 9 : 8);	/* DSI_PLL_REGM */
	l = FLD_MOD(l, cinfo->regm_dispc > 0 ? cinfo->regm_dispc - 1 : 0,
			(cpu_is_omap44xx()) ? 25 : 22,
			(cpu_is_omap44xx()) ? 21 : 19);	/* DSI_CLOCK_DIV */
	l = FLD_MOD(l, cinfo->regm_dsi > 0 ? cinfo->regm_dsi - 1 : 0,
			(cpu_is_omap44xx()) ? 30 : 26,
			(cpu_is_omap44xx()) ? 26 : 23);	/* DSIPROTO_CLOCK_DIV */
	dsi_write_reg(ds, DSI_PLL_CONFIGURATION1, l);

	BUG_ON(cinfo->fint < FINT_MIN || cinfo->fint > FINT_MAX);
	if (!cpu_is_omap44xx()) {
		if (cinfo->fint < 1000000)
			f = 0x3;
		else if (cinfo->fint < 1250000)
			f = 0x4;
		else if (cinfo->fint < 1500000)
			f = 0x5;
		else if (cinfo->fint < 1750000)
			f = 0x6;
		else
			f = 0x7;
	}
	l = dsi_read_reg(ds, DSI_PLL_CONFIGURATION2);
	if (!cpu_is_omap44xx())
		l = FLD_MOD(l, f, 4, 1);	/* DSI_PLL_FREQSEL */
	l = FLD_MOD(l, cinfo->use_dss2_fck ? 0 : 1,
			11, 11);		/* DSI_PLL_CLKSEL */
	l = FLD_MOD(l, cinfo->highfreq,
			12, 12);		/* DSI_PLL_HIGHFREQ */
	l = FLD_MOD(l, 1, 13, 13);		/* DSI_PLL_REFEN */
	l = FLD_MOD(l, 0, 14, 14);		/* DSIPHY_CLKINEN */
	l = FLD_MOD(l, 1, 20, 20);		/* DSI_HSDIVBYPASS */
	if (cpu_is_omap44xx())
		l = FLD_MOD(l, 3, 22, 21);	/* DSI_REF_SEL */
	dsi_write_reg(ds, DSI_PLL_CONFIGURATION2, l);

	REG_FLD_MOD(ds, DSI_PLL_GO, 1, 0, 0);	/* DSI_PLL_GO */

	if (wait_for_bit_change(ds, DSI_PLL_GO, 0, 0) != 0) {
		DSSERR("dsi pll go bit not going down.\n");
		r = -EIO;
		goto err;
	}

	if (wait_for_bit_change(ds, DSI_PLL_STATUS, 1, 1) != 1) {
		DSSERR("cannot lock PLL\n");
		r = -EIO;
		goto err;
	}

	ds->pll_locked = 1;

	l = dsi_read_reg(ds, DSI_PLL_CONFIGURATION2);
	l = FLD_MOD(l, 0, 0, 0);	/* DSI_PLL_IDLE */
	l = FLD_MOD(l, 0, 5, 5);	/* DSI_PLL_PLLLPMODE */
	l = FLD_MOD(l, 0, 6, 6);	/* DSI_PLL_LOWCURRSTBY */
	if (!cpu_is_omap44xx())
		l = FLD_MOD(l, 0, 7, 7);	/* DSI_PLL_TIGHTPHASELOCK */
	l = FLD_MOD(l, 0, 8, 8);	/* DSI_PLL_DRIFTGUARDEN */
	l = FLD_MOD(l, 0, 10, 9);	/* DSI_PLL_LOCKSEL */
	l = FLD_MOD(l, 1, 13, 13);	/* DSI_PLL_REFEN */
	l = FLD_MOD(l, 1, 14, 14);	/* DSIPHY_CLKINEN */
	l = FLD_MOD(l, 0, 15, 15);	/* DSI_BYPASSEN */
	l = FLD_MOD(l, 1, 16, 16);	/* DSS_CLOCK_EN */
	l = FLD_MOD(l, 0, 17, 17);	/* DSS_CLOCK_PWDN */
	l = FLD_MOD(l, 1, 18, 18);	/* DSI_PROTO_CLOCK_EN */
	l = FLD_MOD(l, 0, 19, 19);	/* DSI_PROTO_CLOCK_PWDN */
	l = FLD_MOD(l, 0, 20, 20);	/* DSI_HSDIVBYPASS */
	if (cpu_is_omap44xx()) {
		l = FLD_MOD(l, 0, 25, 25);	/* M7_CLOCK_EN */
		l = FLD_MOD(l, 0, 26, 26);	/* M7_CLOCK_PWDN */
	}
	dsi_write_reg(ds, DSI_PLL_CONFIGURATION2, l);

	DSSDBG("PLL config done\n");
err:
	return r;
}

int dsi_pll_init(enum omap_channel channel, bool enable_hsclk,
		bool enable_hsdiv)
{
	int r = 0;
	enum dsi_pll_power_state pwstate;
	struct dsi_struct *ds = dsi4channel(channel);

	DSSDBG("PLL init\n");

	enable_clocks(1);
	dsi_enable_pll_clock(ds, 1);

	if (!cpu_is_omap44xx()) {
		r = regulator_enable(ds->vdds_dsi_reg);
		if (r)
			goto err0;
	}

	/* XXX PLL does not come out of reset without this... */
	dispc_pck_free_enable(1);

	if (wait_for_bit_change(ds, DSI_PLL_STATUS, 0, 1) != 1) {
		DSSERR("PLL not coming out of reset.\n");
		r = -ENODEV;
		dispc_pck_free_enable(0);
		goto err1;
	}

	/* XXX ... but if left on, we get problems when planes do not
	 * fill the whole display. No idea about this */
	dispc_pck_free_enable(0);

	if (enable_hsclk && enable_hsdiv)
		pwstate = DSI_PLL_POWER_ON_ALL;
	else if (enable_hsclk)
		pwstate = DSI_PLL_POWER_ON_HSCLK;
	else if (enable_hsdiv)
		pwstate = DSI_PLL_POWER_ON_DIV;
	else
		pwstate = DSI_PLL_POWER_OFF;

	r = dsi_pll_power(ds, pwstate);

	if (r)
		goto err1;

	DSSDBG("PLL init done\n");

	return 0;
err1:
	if (!cpu_is_omap44xx())
		regulator_disable(ds->vdds_dsi_reg);
err0:
	enable_clocks(0);
	dsi_enable_pll_clock(ds, 0);
	return r;
}

void dsi_pll_uninit(enum omap_channel channel)
{
	struct dsi_struct *ds = dsi4channel(channel);

	enable_clocks(0);
	dsi_enable_pll_clock(ds, 0);

	ds->pll_locked = 0;
	dsi_pll_power(ds, DSI_PLL_POWER_OFF);
	if (!cpu_is_omap44xx())
		regulator_disable(ds->vdds_dsi_reg);
	DSSDBG("PLL uninit done\n");
}

void dsi_dump_clocks(enum omap_channel channel, struct seq_file *s)
{
	int clksel;
	struct dsi_struct *ds = dsi4channel(channel);
	struct dsi_clock_info *cinfo = &ds->current_cinfo;

	if (!ds->enabled)
		return;

	enable_clocks(1);

	clksel = REG_GET(ds, DSI_PLL_CONFIGURATION2, 11, 11);

	seq_printf(s,	"- DSI PLL -\n");

	seq_printf(s,	"dsi pll source = %s\n",
			clksel == 0 ?
			"dss2_alwon_fclk" : "pclkfree");

	seq_printf(s,	"Fint\t\t%-16luregn %u\n", cinfo->fint, cinfo->regn);

	seq_printf(s,	"CLKIN4DDR\t%-16luregm %u\n",
			cinfo->clkin4ddr, cinfo->regm);

	seq_printf(s,	"dsi1_pll_fck(OMAP3) / pllx_clk1(OMAP4)"
			"\t%-16luregm3(OMAP3) / regm4(OMAP4) %u\t(%s)\n",
			cinfo->dsi_pll_dispc_fclk,
			cinfo->regm_dispc,
			dss_get_dispc_clk_source() == DSS_SRC_DSS1_ALWON_FCLK ?
			"off" : "on");

	seq_printf(s,	"dsi2_pll_fck(OMAP3) / pllx_clk2(OMAP4)"
			"\t%-16luregm4(OMAP3) / regm5(OMAP4)  %u\t(%s)\n",
			cinfo->dsi_pll_dsi_fclk,
			cinfo->regm_dsi,
			dss_get_dsi_clk_source(channel) ==
			DSS_SRC_DSS1_ALWON_FCLK ? "off" : "on");

	seq_printf(s,	"- DSI -\n");

	seq_printf(s,	"dsi fclk source = %s\n",
			dss_get_dsi_clk_source(channel) ==
			DSS_SRC_DSS1_ALWON_FCLK ?
			"dss1_alwon_fclk" :
			"dsi2_pll_fclk(OMAP3) / pllx_clk2(OMAP4)");

	seq_printf(s,	"DSI_FCLK\t%lu\n", dsi_fclk_rate(ds));

	seq_printf(s,	"DDR_CLK\t\t%lu\n",
			cinfo->clkin4ddr / 4);

	seq_printf(s,	"TxByteClkHS\t%lu\n", dsi_get_txbyteclkhs(ds));

	seq_printf(s,	"LP_CLK\t\t%lu\n", cinfo->lp_clk);

	seq_printf(s,	"VP_CLK\t\t%lu\n"
			"VP_PCLK\t\t%lu\n",
			dispc_lclk_rate(channel),
			dispc_pclk_rate(channel));

	enable_clocks(0);
}

#ifdef CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS
void dsi_dump_irqs(enum omap_channel channel, struct seq_file *s)
{
	unsigned long flags;
	struct dsi_irq_stats stats;
	struct dsi_struct *ds = dsi4channel(channel);

	if (!ds->enabled)
		return;

	spin_lock_irqsave(&ds->irq_stats_lock, flags);

	stats = ds->irq_stats;
	memset(&ds->irq_stats, 0, sizeof(ds->irq_stats));
	ds->irq_stats.last_reset = jiffies;

	spin_unlock_irqrestore(&ds->irq_stats_lock, flags);

	seq_printf(s, "period %u ms\n",
			jiffies_to_msecs(jiffies - stats.last_reset));

	seq_printf(s, "irqs %d\n", stats.irq_count);
#define PIS(x) \
	seq_printf(s, "%-20s %10d\n", #x, stats.dsi_irqs[ffs(DSI_IRQ_##x)-1]);

	seq_printf(s, "-- DSI interrupts --\n");
	PIS(VC0);
	PIS(VC1);
	PIS(VC2);
	PIS(VC3);
	PIS(WAKEUP);
	PIS(RESYNC);
	PIS(PLL_LOCK);
	PIS(PLL_UNLOCK);
	PIS(PLL_RECALL);
	PIS(COMPLEXIO_ERR);
	PIS(HS_TX_TIMEOUT);
	PIS(LP_RX_TIMEOUT);
	PIS(TE_TRIGGER);
	PIS(ACK_TRIGGER);
	PIS(SYNC_LOST);
	PIS(LDO_POWER_GOOD);
	PIS(TA_TIMEOUT);
#undef PIS

#define PIS(x) \
	seq_printf(s, "%-20s %10d %10d %10d %10d\n", #x, \
			stats.vc_irqs[0][ffs(DSI_VC_IRQ_##x)-1], \
			stats.vc_irqs[1][ffs(DSI_VC_IRQ_##x)-1], \
			stats.vc_irqs[2][ffs(DSI_VC_IRQ_##x)-1], \
			stats.vc_irqs[3][ffs(DSI_VC_IRQ_##x)-1]);

	seq_printf(s, "-- VC interrupts --\n");
	PIS(CS);
	PIS(ECC_CORR);
	PIS(PACKET_SENT);
	PIS(FIFO_TX_OVF);
	PIS(FIFO_RX_OVF);
	PIS(BTA);
	PIS(ECC_NO_CORR);
	PIS(FIFO_TX_UDF);
	PIS(PP_BUSY_CHANGE);
#undef PIS

#define PIS(x) \
	seq_printf(s, "%-20s %10d\n", #x, \
			stats.cio_irqs[ffs(DSI_CIO_IRQ_##x)-1]);

	seq_printf(s, "-- CIO interrupts --\n");
	PIS(ERRSYNCESC1);
	PIS(ERRSYNCESC2);
	PIS(ERRSYNCESC3);
	PIS(ERRESC1);
	PIS(ERRESC2);
	PIS(ERRESC3);
	PIS(ERRCONTROL1);
	PIS(ERRCONTROL2);
	PIS(ERRCONTROL3);
	PIS(STATEULPS1);
	PIS(STATEULPS2);
	PIS(STATEULPS3);
	PIS(ERRCONTENTIONLP0_1);
	PIS(ERRCONTENTIONLP1_1);
	PIS(ERRCONTENTIONLP0_2);
	PIS(ERRCONTENTIONLP1_2);
	PIS(ERRCONTENTIONLP0_3);
	PIS(ERRCONTENTIONLP1_3);
	PIS(ULPSACTIVENOT_ALL0);
	PIS(ULPSACTIVENOT_ALL1);
#undef PIS
}
#endif

void dsi_dump_regs(enum omap_channel channel, struct seq_file *s)
{
	struct dsi_struct *ds = dsi4channel(channel);

	if (!ds->enabled)
		return;

#define DUMPREG(ds, r) seq_printf(s, "%-35s %08x\n", #r, dsi_read_reg(ds, r))

	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

	DUMPREG(ds, DSI_REVISION);
	DUMPREG(ds, DSI_SYSCONFIG);
	DUMPREG(ds, DSI_SYSSTATUS);
	DUMPREG(ds, DSI_IRQSTATUS);
	DUMPREG(ds, DSI_IRQENABLE);
	DUMPREG(ds, DSI_CTRL);
	DUMPREG(ds, DSI_COMPLEXIO_CFG1);
	DUMPREG(ds, DSI_COMPLEXIO_IRQ_STATUS);
	DUMPREG(ds, DSI_COMPLEXIO_IRQ_ENABLE);
	DUMPREG(ds, DSI_CLK_CTRL);
	DUMPREG(ds, DSI_TIMING1);
	DUMPREG(ds, DSI_TIMING2);
	DUMPREG(ds, DSI_VM_TIMING1);
	DUMPREG(ds, DSI_VM_TIMING2);
	DUMPREG(ds, DSI_VM_TIMING3);
	DUMPREG(ds, DSI_CLK_TIMING);
	DUMPREG(ds, DSI_TX_FIFO_VC_SIZE);
	DUMPREG(ds, DSI_RX_FIFO_VC_SIZE);
	DUMPREG(ds, DSI_COMPLEXIO_CFG2);
	DUMPREG(ds, DSI_RX_FIFO_VC_FULLNESS);
	DUMPREG(ds, DSI_VM_TIMING4);
	DUMPREG(ds, DSI_TX_FIFO_VC_EMPTINESS);
	DUMPREG(ds, DSI_VM_TIMING5);
	DUMPREG(ds, DSI_VM_TIMING6);
	DUMPREG(ds, DSI_VM_TIMING7);
	DUMPREG(ds, DSI_STOPCLK_TIMING);

	DUMPREG(ds, DSI_VC_CTRL(0));
	DUMPREG(ds, DSI_VC_TE(0));
	DUMPREG(ds, DSI_VC_LONG_PACKET_HEADER(0));
	DUMPREG(ds, DSI_VC_LONG_PACKET_PAYLOAD(0));
	DUMPREG(ds, DSI_VC_SHORT_PACKET_HEADER(0));
	DUMPREG(ds, DSI_VC_IRQSTATUS(0));
	DUMPREG(ds, DSI_VC_IRQENABLE(0));

	DUMPREG(ds, DSI_VC_CTRL(1));
	DUMPREG(ds, DSI_VC_TE(1));
	DUMPREG(ds, DSI_VC_LONG_PACKET_HEADER(1));
	DUMPREG(ds, DSI_VC_LONG_PACKET_PAYLOAD(1));
	DUMPREG(ds, DSI_VC_SHORT_PACKET_HEADER(1));
	DUMPREG(ds, DSI_VC_IRQSTATUS(1));
	DUMPREG(ds, DSI_VC_IRQENABLE(1));

	DUMPREG(ds, DSI_VC_CTRL(2));
	DUMPREG(ds, DSI_VC_TE(2));
	DUMPREG(ds, DSI_VC_LONG_PACKET_HEADER(2));
	DUMPREG(ds, DSI_VC_LONG_PACKET_PAYLOAD(2));
	DUMPREG(ds, DSI_VC_SHORT_PACKET_HEADER(2));
	DUMPREG(ds, DSI_VC_IRQSTATUS(2));
	DUMPREG(ds, DSI_VC_IRQENABLE(2));

	DUMPREG(ds, DSI_VC_CTRL(3));
	DUMPREG(ds, DSI_VC_TE(3));
	DUMPREG(ds, DSI_VC_LONG_PACKET_HEADER(3));
	DUMPREG(ds, DSI_VC_LONG_PACKET_PAYLOAD(3));
	DUMPREG(ds, DSI_VC_SHORT_PACKET_HEADER(3));
	DUMPREG(ds, DSI_VC_IRQSTATUS(3));
	DUMPREG(ds, DSI_VC_IRQENABLE(3));

	DUMPREG(ds, DSI_DSIPHY_CFG0);
	DUMPREG(ds, DSI_DSIPHY_CFG1);
	DUMPREG(ds, DSI_DSIPHY_CFG2);
	DUMPREG(ds, DSI_DSIPHY_CFG5);

	DUMPREG(ds, DSI_PLL_CONTROL);
	DUMPREG(ds, DSI_PLL_STATUS);
	DUMPREG(ds, DSI_PLL_GO);
	DUMPREG(ds, DSI_PLL_CONFIGURATION1);
	DUMPREG(ds, DSI_PLL_CONFIGURATION2);

	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
#undef DUMPREG
}

enum dsi_complexio_power_state {
	DSI_COMPLEXIO_POWER_OFF		= 0x0,
	DSI_COMPLEXIO_POWER_ON		= 0x1,
	DSI_COMPLEXIO_POWER_ULPS	= 0x2,
};

static int dsi_complexio_power(struct dsi_struct *ds,
	enum dsi_complexio_power_state state)
{
	int t = 0;

	/* PWR_CMD */
	REG_FLD_MOD(ds, DSI_COMPLEXIO_CFG1, state, 28, 27);

	if (cpu_is_omap44xx())
		/*bit 30 has to be set to 1 to GO in omap4*/
		REG_FLD_MOD(ds, DSI_COMPLEXIO_CFG1, 1, 30, 30);

	/* PWR_STATUS */
	while (FLD_GET(dsi_read_reg(ds, DSI_COMPLEXIO_CFG1), 26, 25) != state) {
		if (++t > 1000) {
			DSSERR("failed to set complexio power state to "
					"%d\n", state);
			return -ENODEV;
		}
		udelay(1);
	}

	return 0;
}

static void dsi_complexio_config(struct omap_dss_device *dssdev)
{
	u32 r;
	struct dsi_struct *ds = dss2dsi(dssdev);

	int clk_lane   = dssdev->phy.dsi.clk_lane;
	int data1_lane = dssdev->phy.dsi.data1_lane;
	int data2_lane = dssdev->phy.dsi.data2_lane;
	int data3_lane = dssdev->phy.dsi.data3_lane;
	int data4_lane = dssdev->phy.dsi.data4_lane;
	int clk_pol    = dssdev->phy.dsi.clk_pol;
	int data1_pol  = dssdev->phy.dsi.data1_pol;
	int data2_pol  = dssdev->phy.dsi.data2_pol;
	int data3_pol  = dssdev->phy.dsi.data3_pol;
	int data4_pol  = dssdev->phy.dsi.data4_pol;

	r = dsi_read_reg(ds, DSI_COMPLEXIO_CFG1);
	r = FLD_MOD(r, clk_lane, 2, 0);
	r = FLD_MOD(r, clk_pol, 3, 3);
	r = FLD_MOD(r, data1_lane, 6, 4);
	r = FLD_MOD(r, data1_pol, 7, 7);
	r = FLD_MOD(r, data2_lane, 10, 8);
	r = FLD_MOD(r, data2_pol, 11, 11);

	if (omap4_board_rev() == OMAP4_BLAZETABLET_BOARD) {
		r = FLD_MOD(r, data3_lane, 14, 12);
		r = FLD_MOD(r, data3_pol, 15, 15);
		r = FLD_MOD(r, data4_lane, 18, 16);
		r = FLD_MOD(r, data4_pol, 19, 19);
	}

	dsi_write_reg(ds, DSI_COMPLEXIO_CFG1, r);

	/* The configuration of the DSI complex I/O (number of data lanes,
	   position, differential order) should not be changed while
	   DSS.DSI_CLK_CRTRL[20] LP_CLK_ENABLE bit is set to 1. In order for
	   the hardware to take into account a new configuration of the complex
	   I/O (done in DSS.DSI_COMPLEXIO_CFG1 register), it is recommended to
	   follow this sequence: First set the DSS.DSI_CTRL[0] IF_EN bit to 1,
	   then reset the DSS.DSI_CTRL[0] IF_EN to 0, then set
	   DSS.DSI_CLK_CTRL[20] LP_CLK_ENABLE to 1 and finally set again the
	   DSS.DSI_CTRL[0] IF_EN bit to 1. If the sequence is not followed, the
	   DSI complex I/O configuration is unknown. */

	/*
	REG_FLD_MOD(ds, DSI_CTRL, 1, 0, 0);
	REG_FLD_MOD(ds, DSI_CTRL, 0, 0, 0);
	REG_FLD_MOD(ds, DSI_CLK_CTRL, 1, 20, 20);
	REG_FLD_MOD(ds, DSI_CTRL, 1, 0, 0);
	*/
}

static inline unsigned ns2ddr(struct dsi_struct *ds, unsigned ns)
{
	/* convert time in ns to ddr ticks, rounding up */
	unsigned long ddr_clk = ds->current_cinfo.clkin4ddr / 4;
	return (ns * (ddr_clk / 1000 / 1000) + 999) / 1000;
}

static inline unsigned ddr2ns(struct dsi_struct *ds, unsigned ddr)
{
	unsigned long ddr_clk = ds->current_cinfo.clkin4ddr / 4;
	return ddr * 1000 * 1000 / (ddr_clk / 1000);
}

static void dsi_complexio_timings(struct dsi_struct *ds)
{
	u32 r;
	u32 ths_prepare, ths_prepare_ths_zero, ths_trail, ths_exit;
	u32 tlpx_half, tclk_trail, tclk_zero;
	u32 tclk_prepare;

	/* calculate timings */

	/* 1 * DDR_CLK = 2 * UI */

	/* min 40ns + 4*UI	max 85ns + 6*UI */
	ths_prepare = ns2ddr(ds, 70) + 2;

	/* min 145ns + 10*UI */
	ths_prepare_ths_zero = ns2ddr(ds, 175) + 2;

	/* min max(8*UI, 60ns+4*UI) */
	ths_trail = ns2ddr(ds, 60) + 5;

	/* min 100ns */
	ths_exit = ns2ddr(ds, 145);

	/* tlpx min 50n */
	tlpx_half = ns2ddr(ds, 25);

	/* min 60ns */
	tclk_trail = ns2ddr(ds, 60) + 2;

	/* min 38ns, max 95ns */
	tclk_prepare = ns2ddr(ds, 65);

	/* min tclk-prepare + tclk-zero = 300ns */
	tclk_zero = ns2ddr(ds, 260);

	DSSDBG("ths_prepare %u (%uns), ths_prepare_ths_zero %u (%uns)\n",
		ths_prepare, ddr2ns(ds, ths_prepare),
		ths_prepare_ths_zero, ddr2ns(ds, ths_prepare_ths_zero));
	DSSDBG("ths_trail %u (%uns), ths_exit %u (%uns)\n",
			ths_trail, ddr2ns(ds, ths_trail),
			ths_exit, ddr2ns(ds, ths_exit));

	DSSDBG("tlpx_half %u (%uns), tclk_trail %u (%uns), "
			"tclk_zero %u (%uns)\n",
			tlpx_half, ddr2ns(ds, tlpx_half),
			tclk_trail, ddr2ns(ds, tclk_trail),
			tclk_zero, ddr2ns(ds, tclk_zero));
	DSSDBG("tclk_prepare %u (%uns)\n",
			tclk_prepare, ddr2ns(ds, tclk_prepare));

	/* program timings */

	r = dsi_read_reg(ds, DSI_DSIPHY_CFG0);
	r = FLD_MOD(r, ths_prepare, 31, 24);
	r = FLD_MOD(r, ths_prepare_ths_zero, 23, 16);
	r = FLD_MOD(r, ths_trail, 15, 8);
	r = FLD_MOD(r, ths_exit, 7, 0);
	dsi_write_reg(ds, DSI_DSIPHY_CFG0, r);

	r = dsi_read_reg(ds, DSI_DSIPHY_CFG1);
	r = FLD_MOD(r, tlpx_half, 22, 16);
	r = FLD_MOD(r, tclk_trail, 15, 8);
	r = FLD_MOD(r, tclk_zero, 7, 0);
	dsi_write_reg(ds, DSI_DSIPHY_CFG1, r);

	r = dsi_read_reg(ds, DSI_DSIPHY_CFG2);
	r = FLD_MOD(r, tclk_prepare, 7, 0);
	dsi_write_reg(ds, DSI_DSIPHY_CFG2, r);
}


static int dsi_complexio_init(struct omap_dss_device *dssdev)
{
	int r = 0;
	struct dsi_struct *ds = dss2dsi(dssdev);

	DSSDBG("dsi_complexio_init\n");

	/* CIO_CLK_ICG, enable L3 clk to CIO */
	REG_FLD_MOD(ds, DSI_CLK_CTRL, 1, 14, 14);

	if (cpu_is_omap44xx()) {
		/* DDR_CLK_ALWAYS_ON */
		REG_FLD_MOD(ds, DSI_CLK_CTRL, 1, 13, 13);
		/* HS_AUTO_STOP_ENABLE */
		REG_FLD_MOD(ds, DSI_CLK_CTRL, 1, 18, 18);
	}
	/* A dummy read using the SCP interface to any DSIPHY register is
	 * required after DSIPHY reset to complete the reset of the DSI complex
	 * I/O. */
	dsi_read_reg(ds, DSI_DSIPHY_CFG5);

	if (wait_for_bit_change(ds, DSI_DSIPHY_CFG5, 30, 1) != 1) {
		DSSERR("ComplexIO PHY not coming out of reset.\n");
		r = -ENODEV;
		goto err;
	}

	dsi_complexio_config(dssdev);

	r = dsi_complexio_power(ds, DSI_COMPLEXIO_POWER_ON);

	if (r)
		goto err;

	if (wait_for_bit_change(ds, DSI_COMPLEXIO_CFG1, 29, 1) != 1) {
		DSSERR("ComplexIO not coming out of reset.\n");
		r = -ENODEV;
		goto err;
	}

	if (!cpu_is_omap44xx()) {
		if (wait_for_bit_change(ds, DSI_COMPLEXIO_CFG1, 21, 1) != 1) {
			DSSERR("ComplexIO LDO power down.\n");
			r = -ENODEV;
			goto err;
		}
	}
	dsi_complexio_timings(ds);

	/*
	   The configuration of the DSI complex I/O (number of data lanes,
	   position, differential order) should not be changed while
	   DSS.DSI_CLK_CRTRL[20] LP_CLK_ENABLE bit is set to 1. For the
	   hardware to recognize a new configuration of the complex I/O (done
	   in DSS.DSI_COMPLEXIO_CFG1 register), it is recommended to follow
	   this sequence: First set the DSS.DSI_CTRL[0] IF_EN bit to 1, next
	   reset the DSS.DSI_CTRL[0] IF_EN to 0, then set DSS.DSI_CLK_CTRL[20]
	   LP_CLK_ENABLE to 1, and finally, set again the DSS.DSI_CTRL[0] IF_EN
	   bit to 1. If the sequence is not followed, the DSi complex I/O
	   configuration is undetermined.
	   */
	dsi_if_enable(ds, 1);
	dsi_if_enable(ds, 0);
	REG_FLD_MOD(ds, DSI_CLK_CTRL, 1, 20, 20); /* LP_CLK_ENABLE */
	dsi_if_enable(ds, 1);
	dsi_if_enable(ds, 0);

	DSSDBG("CIO init done\n");
err:
	return r;
}

static void dsi_complexio_uninit(struct dsi_struct *ds)
{
	dsi_complexio_power(ds, DSI_COMPLEXIO_POWER_OFF);
}

static int _dsi_wait_reset(struct dsi_struct *ds)
{
	int t = 0;

	while (REG_GET(ds, DSI_SYSSTATUS, 0, 0) == 0) {
		if (++t > 5) {
			DSSERR("soft reset failed\n");
			return -ENODEV;
		}
		udelay(1);
	}

	return 0;
}

static int _dsi_reset(struct dsi_struct *ds)
{
	/* Soft reset */
	REG_FLD_MOD(ds, DSI_SYSCONFIG, 1, 1, 1);
	return _dsi_wait_reset(ds);
}

static void dsi_reset_tx_fifo(struct dsi_struct *ds, int channel)
{
	u32 mask;
	u32 l;

	/* set fifosize of the channel to 0, then return the old size */
	l = dsi_read_reg(ds, DSI_TX_FIFO_VC_SIZE);

	mask = FLD_MASK((8 * channel) + 7, (8 * channel) + 4);
	dsi_write_reg(ds, DSI_TX_FIFO_VC_SIZE, l & ~mask);

	dsi_write_reg(ds, DSI_TX_FIFO_VC_SIZE, l);
}

static void dsi_config_tx_fifo(struct dsi_struct *ds,
	enum fifo_size size1, enum fifo_size size2,
	enum fifo_size size3, enum fifo_size size4)
{
	u32 r = 0;
	int add = 0;
	int i;

	ds->vc[0].fifo_size = size1;
	ds->vc[1].fifo_size = size2;
	ds->vc[2].fifo_size = size3;
	ds->vc[3].fifo_size = size4;

	for (i = 0; i < 4; i++) {
		u8 v;
		int size = ds->vc[i].fifo_size;

		if (add + size > 4) {
			DSSERR("Illegal FIFO configuration\n");
			BUG();
		}

		v = FLD_VAL(add, 2, 0) | FLD_VAL(size, 7, 4);
		r |= v << (8 * i);
		/*DSSDBG("TX FIFO vc %d: size %d, add %d\n", i, size, add); */
		add += size;
	}

	dsi_write_reg(ds, DSI_TX_FIFO_VC_SIZE, r);
}

static void dsi_config_rx_fifo(struct dsi_struct *ds,
	enum fifo_size size1, enum fifo_size size2,
	enum fifo_size size3, enum fifo_size size4)
{
	u32 r = 0;
	int add = 0;
	int i;

	ds->vc[0].fifo_size = size1;
	ds->vc[1].fifo_size = size2;
	ds->vc[2].fifo_size = size3;
	ds->vc[3].fifo_size = size4;

	for (i = 0; i < 4; i++) {
		u8 v;
		int size = ds->vc[i].fifo_size;

		if (add + size > 4) {
			DSSERR("Illegal FIFO configuration\n");
			BUG();
		}

		v = FLD_VAL(add, 2, 0) | FLD_VAL(size, 7, 4);
		r |= v << (8 * i);
		/*DSSDBG("RX FIFO vc %d: size %d, add %d\n", i, size, add); */
		add += size;
	}

	dsi_write_reg(ds, DSI_RX_FIFO_VC_SIZE, r);
}

static int dsi_force_tx_stop_mode_io(struct dsi_struct *ds)
{
	u32 r;

	r = dsi_read_reg(ds, DSI_TIMING1);
	r = FLD_MOD(r, 1, 15, 15);	/* FORCE_TX_STOP_MODE_IO */
	dsi_write_reg(ds, DSI_TIMING1, r);

	if (wait_for_bit_change(ds, DSI_TIMING1, 15, 0) != 0) {
		DSSERR("TX_STOP bit not going down\n");
		return -EIO;
	}

	return 0;
}

static int dsi_vc_enable(struct dsi_struct *ds,
	int channel, bool enable)
{
	DSSDBG("dsi_vc_enable channel %d, enable %d\n",
			channel, enable);

	enable = enable ? 1 : 0;

	REG_FLD_MOD(ds, DSI_VC_CTRL(channel), enable, 0, 0);

	if (wait_for_bit_change(ds, DSI_VC_CTRL(channel),
			0, enable) != enable) {
			DSSERR("Failed to set dsi_vc_enable to %d\n", enable);
			return -EIO;
	}

	return 0;
}

static void dsi_vc_initial_config(struct dsi_struct *ds, int channel)
{
	u32 r;

	DSSDBGF("%d", channel);

	r = dsi_read_reg(ds, DSI_VC_CTRL(channel));

	if (FLD_GET(r, 15, 15)) /* VC_BUSY */
		DSSERR("VC(%d) busy when trying to configure it!\n",
				channel);

	r = FLD_MOD(r, 0, 1, 1); /* SOURCE, 0 = L4 */
	r = FLD_MOD(r, 0, 2, 2); /* BTA_SHORT_EN  */
	r = FLD_MOD(r, 0, 3, 3); /* BTA_LONG_EN */
	r = FLD_MOD(r, 0, 4, 4); /* MODE, 0 = command */
	r = FLD_MOD(r, 1, 7, 7); /* CS_TX_EN */
	r = FLD_MOD(r, 1, 8, 8); /* ECC_TX_EN */
	r = FLD_MOD(r, 0, 9, 9); /* MODE_SPEED, high speed on/off */
	if (cpu_is_omap44xx()) {
		r = FLD_MOD(r, 3, 11, 10);	/* OCP_WIDTH */
		r = FLD_MOD(r, 3, 19, 17);
		r = FLD_MOD(r, 1, 12, 12);	/*RGB565_ORDER*/
	}
	r = FLD_MOD(r, 4, 29, 27); /* DMA_RX_REQ_NB = no dma */
	if (!cpu_is_omap44xx())
		r = FLD_MOD(r, 4, 23, 21); /* DMA_TX_REQ_NB = no dma */

	dsi_write_reg(ds, DSI_VC_CTRL(channel), r);

	ds->vc[channel].mode = DSI_VC_MODE_L4;
}

static void dsi_vc_initial_config_vp(struct dsi_struct *ds, int channel)
{
	u32 r;

	DSSDBGF("%d", channel);

	r = dsi_read_reg(ds, DSI_VC_CTRL(channel));
	r = FLD_MOD(r, 1, 1, 1); /* SOURCE, 1 = video port */
	r = FLD_MOD(r, 0, 2, 2); /* BTA_SHORT_EN */
	r = FLD_MOD(r, 0, 3, 3); /* BTA_LONG_EN */
	r = FLD_MOD(r, 0, 4, 4); /* MODE, 0 = command */
	r = FLD_MOD(r, 1, 7, 7); /* CS_TX_EN */
	r = FLD_MOD(r, 1, 8, 8); /* ECC_TX_EN */
	r = FLD_MOD(r, 1, 9, 9); /* MODE_SPEED, high speed on/off */
	if (cpu_is_omap44xx())
		r = FLD_MOD(r, 3, 11, 10);	/* OCP_WIDTH */
	r = FLD_MOD(r, 1, 12, 12);	/*RGB565_ORDER*/
	r = FLD_MOD(r, 4, 29, 27); /* DMA_RX_REQ_NB = no dma */
	r = FLD_MOD(r, 4, 23, 21); /* DMA_TX_REQ_NB = no dma */
	r = FLD_MOD(r, 1, 30, 30);	/* DCS_CMD_ENABLE*/
	r = FLD_MOD(r, 0, 31, 31);	/* DCS_CMD_CODE*/
	dsi_write_reg(ds, DSI_VC_CTRL(channel), r);

	ds->vc[channel].mode = DSI_VC_MODE_VP;
}

static int dsi_vc_config_l4(struct dsi_struct *ds, int channel)
{
	if (ds->vc[channel].mode == DSI_VC_MODE_L4)
		return 0;

	DSSDBGF("%d", channel);

	dsi_vc_enable(ds, channel, 0);

	/* VC_BUSY */
	if (wait_for_bit_change(ds, DSI_VC_CTRL(channel), 15, 0) != 0) {
		DSSERR("vc(%d) busy when trying to config for L4\n", channel);
		return -EIO;
	}

	REG_FLD_MOD(ds, DSI_VC_CTRL(channel), 0, 1, 1); /* SOURCE, 0 = L4 */

	dsi_vc_enable(ds, channel, 1);

	ds->vc[channel].mode = DSI_VC_MODE_L4;

	return 0;
}

static int dsi_vc_config_vp(struct dsi_struct *ds, int channel)
{
	if (ds->vc[channel].mode == DSI_VC_MODE_VP)
		return 0;

	DSSDBGF("%d", channel);

	dsi_vc_enable(ds, channel, 0);

	/* VC_BUSY */
	if (wait_for_bit_change(ds, DSI_VC_CTRL(channel), 15, 0) != 0) {
		DSSERR("vc(%d) busy when trying to config for VP\n", channel);
		return -EIO;
	}

	/* SOURCE, 1 = video port */
	REG_FLD_MOD(ds, DSI_VC_CTRL(channel), 1, 1, 1);

	dsi_vc_enable(ds, channel, 1);

	ds->vc[channel].mode = DSI_VC_MODE_VP;

	return 0;
}


void omapdss_dsi_vc_enable_hs(struct omap_dss_device *dssdev,
	int channel, bool enable)
{
	struct dsi_struct *ds = dss2dsi(dssdev);

	DSSDBG("dsi_vc_enable_hs(%d, %d)\n", channel, enable);

	WARN_ON(!dsi_bus_is_locked(ds));

	dsi_vc_enable(ds, channel, 0);
	dsi_if_enable(ds, 0);

	REG_FLD_MOD(ds, DSI_VC_CTRL(channel), enable, 9, 9);

	dsi_vc_enable(ds, channel, 1);
	dsi_if_enable(ds, 1);

	dsi_force_tx_stop_mode_io(ds);
}
EXPORT_SYMBOL(omapdss_dsi_vc_enable_hs);

static void dsi_vc_flush_long_data(struct dsi_struct *ds,
		int channel)
{
	while (REG_GET(ds, DSI_VC_CTRL(channel), 20, 20)) {
		u32 val;
		val = dsi_read_reg(ds, DSI_VC_SHORT_PACKET_HEADER(channel));
		DSSDBG("\t\tb1 %#02x b2 %#02x b3 %#02x b4 %#02x\n",
				(val >> 0) & 0xff,
				(val >> 8) & 0xff,
				(val >> 16) & 0xff,
				(val >> 24) & 0xff);
	}
}

static void dsi_show_rx_ack_with_err(u16 err)
{
	DSSERR("\tACK with ERROR (%#x):\n", err);
	if (err & (1 << 0))
		DSSERR("\t\tSoT Error\n");
	if (err & (1 << 1))
		DSSERR("\t\tSoT Sync Error\n");
	if (err & (1 << 2))
		DSSERR("\t\tEoT Sync Error\n");
	if (err & (1 << 3))
		DSSERR("\t\tEscape Mode Entry Command Error\n");
	if (err & (1 << 4))
		DSSERR("\t\tLP Transmit Sync Error\n");
	if (err & (1 << 5))
		DSSERR("\t\tHS Receive Timeout Error\n");
	if (err & (1 << 6))
		DSSERR("\t\tFalse Control Error\n");
	if (err & (1 << 7))
		DSSERR("\t\t(reserved7)\n");
	if (err & (1 << 8))
		DSSERR("\t\tECC Error, single-bit (corrected)\n");
	if (err & (1 << 9))
		DSSERR("\t\tECC Error, multi-bit (not corrected)\n");
	if (err & (1 << 10))
		DSSERR("\t\tChecksum Error\n");
	if (err & (1 << 11))
		DSSERR("\t\tData type not recognized\n");
	if (err & (1 << 12))
		DSSERR("\t\tInvalid VC ID\n");
	if (err & (1 << 13))
		DSSERR("\t\tInvalid Transmission Length\n");
	if (err & (1 << 14))
		DSSERR("\t\t(reserved14)\n");
	if (err & (1 << 15))
		DSSERR("\t\tDSI Protocol Violation\n");
}

static u16 dsi_vc_flush_receive_data(struct dsi_struct *ds,
	int channel)
{
	/* RX_FIFO_NOT_EMPTY */
	while (REG_GET(ds, DSI_VC_CTRL(channel), 20, 20)) {
		u32 val;
		u8 dt;
		val = dsi_read_reg(ds, DSI_VC_SHORT_PACKET_HEADER(channel));
		DSSERR("\trawval %#08x\n", val);
		dt = FLD_GET(val, 5, 0);
		if (dt == DSI_DT_RX_ACK_WITH_ERR) {
			u16 err = FLD_GET(val, 23, 8);
			dsi_show_rx_ack_with_err(err);
		} else if (dt == DSI_DT_RX_SHORT_READ_1) {
			DSSERR("\tDCS short response, 1 byte: %#x\n",
					FLD_GET(val, 23, 8));
		} else if (dt == DSI_DT_RX_SHORT_READ_2) {
			DSSERR("\tDCS short response, 2 byte: %#x\n",
					FLD_GET(val, 23, 8));
		} else if (dt == DSI_DT_RX_DCS_LONG_READ) {
			DSSERR("\tDCS long response, len %d\n",
					FLD_GET(val, 23, 8));
			dsi_vc_flush_long_data(ds, channel);
		} else {
			DSSERR("\tunknown datatype 0x%02x\n", dt);
		}
	}
	return 0;
}

static int dsi_vc_send_bta(struct dsi_struct *ds, int channel)
{
	if (omap4_board_rev() == OMAP4_BLAZETABLET_BOARD)
		return 0;

	if (ds->debug_write || ds->debug_read)
		DSSDBG("dsi_vc_send_bta %d\n", channel);

	WARN_ON(!dsi_bus_is_locked(ds));

	if (REG_GET(ds, DSI_VC_CTRL(channel), 20, 20)) {
		/* RX_FIFO_NOT_EMPTY */
		DSSERR("rx fifo not empty when sending BTA, dumping data:\n");
		dsi_vc_flush_receive_data(ds, channel);
	}

	REG_FLD_MOD(ds, DSI_VC_CTRL(channel), 1, 6, 6); /* BTA_EN */

	return 0;
}

int dsi_vc_send_bta_sync(struct omap_dss_device *dssdev, int channel)
{
	struct dsi_struct *ds = dss2dsi(dssdev);
	int r = 0;
	u32 err;

	if (omap4_board_rev() == OMAP4_BLAZETABLET_BOARD)
		return 0;

	INIT_COMPLETION(ds->bta_completion);

	dsi_vc_enable_bta_irq(ds, channel);

	r = dsi_vc_send_bta(ds, channel);
	if (r)
		goto err;

	if (wait_for_completion_timeout(&ds->bta_completion,
				msecs_to_jiffies(500)) == 0) {
		DSSERR("Failed to receive BTA\n");
		r = -EIO;
		goto err;
	}

	err = dsi_get_errors(ds);
	if (err) {
		DSSERR("Error while sending BTA: %x\n", err);
		r = -EIO;
		goto err;
	}
err:
	dsi_vc_disable_bta_irq(ds, channel);

	return r;
}
EXPORT_SYMBOL(dsi_vc_send_bta_sync);

static inline void dsi_vc_write_long_header(struct dsi_struct *ds,
	int channel, u8 data_type,
	u16 len, u8 ecc)
{
	u32 val;
	u8 data_id;
	WARN_ON(!dsi_bus_is_locked(ds));

	data_id = data_type | ds->vc[channel].dest_per << 6;

	val = FLD_VAL(data_id, 7, 0) | FLD_VAL(len, 23, 8) |
		FLD_VAL(ecc, 31, 24);

	dsi_write_reg(ds, DSI_VC_LONG_PACKET_HEADER(channel), val);
}

static inline void dsi_vc_write_long_payload(struct dsi_struct *ds,
	int channel, u8 b1, u8 b2, u8 b3, u8 b4)
{
	u32 val;

	val = b4 << 24 | b3 << 16 | b2 << 8  | b1 << 0;

/*	DSSDBG("\twriting %02x, %02x, %02x, %02x (%#010x)\n",
			b1, b2, b3, b4, val); */

	dsi_write_reg(ds, DSI_VC_LONG_PACKET_PAYLOAD(channel), val);
}

int dsi_vc_send_long(struct omap_dss_device *dssdev, int channel,
	u8 data_type, u8 *data, u16 len,
		u8 ecc)
{
	/*u32 val; */
	int i;
	u8 *p;
	int r = 0;
	u8 b1, b2, b3, b4;

	struct dsi_struct *ds = dss2dsi(dssdev);

	if (ds->debug_write)
		DSSDBG("dsi_vc_send_long, %d bytes\n", len);

	/* len + header */
	if (ds->vc[channel].fifo_size * 32 * 4 < len + 4) {
		DSSERR("unable to send long packet: packet too long.\n");
		return -EINVAL;
	}

	dsi_vc_config_l4(ds, channel);

	dsi_vc_write_long_header(ds, channel, data_type, len, ecc);

	p = data;
	for (i = 0; i < len >> 2; i++) {
		if (ds->debug_write)
			DSSDBG("\tsending full packet %d\n", i);

		b1 = *p++;
		b2 = *p++;
		b3 = *p++;
		b4 = *p++;

		dsi_vc_write_long_payload(ds, channel, b1, b2, b3, b4);
	}

	i = len % 4;
	if (i) {
		b1 = 0; b2 = 0; b3 = 0;

		if (ds->debug_write)
			DSSDBG("\tsending remainder bytes %d\n", i);

		switch (i) {
		case 3:
			b1 = *p++;
			b2 = *p++;
			b3 = *p++;
			break;
		case 2:
			b1 = *p++;
			b2 = *p++;
			break;
		case 1:
			b1 = *p++;
			break;
		}

		dsi_vc_write_long_payload(ds, channel, b1, b2, b3, 0);
	}

	return r;
}
EXPORT_SYMBOL(dsi_vc_send_long);

static int dsi_vc_send_short(struct dsi_struct *ds,
	int channel, u8 data_type, u16 data, u8 ecc)
{
	u32 r;
	u8 data_id;

	WARN_ON(!dsi_bus_is_locked(ds));

	if (ds->debug_write)
		DSSDBG("dsi_vc_send_short(ch%d, dt %#x, b1 %#x, b2 %#x)\n",
				channel,
				data_type, data & 0xff, (data >> 8) & 0xff);

	dsi_vc_config_l4(ds, channel);

	if (FLD_GET(dsi_read_reg(ds, DSI_VC_CTRL(channel)), 16, 16)) {
		DSSERR("ERROR FIFO FULL, aborting transfer\n");
		return -EINVAL;
	}

	data_id = data_type | ds->vc[channel].dest_per << 6;

	r = (data_id << 0) | (data << 8) | (ecc << 24);

	dsi_write_reg(ds, DSI_VC_SHORT_PACKET_HEADER(channel), r);

	return 0;
}

int dsi_vc_send_null(struct omap_dss_device *dssdev, int channel)
{
	u8 nullpkg[] = {0, 0, 0, 0};
	return dsi_vc_send_long(dssdev, channel, DSI_DT_NULL_PACKET,
			nullpkg, 4, 0);
}
EXPORT_SYMBOL(dsi_vc_send_null);

int dsi_vc_dcs_write_nosync(struct omap_dss_device *dssdev, int channel,
	u8 *data, int len)
{
	struct dsi_struct *ds = dss2dsi(dssdev);

	int r;

	BUG_ON(len == 0);

	if (len == 1) {
		r = dsi_vc_send_short(ds, channel, DSI_DT_DCS_SHORT_WRITE_0,
				data[0], 0);
	} else if (len == 2) {
		r = dsi_vc_send_short(ds, channel, DSI_DT_DCS_SHORT_WRITE_1,
				data[0] | (data[1] << 8), 0);
	} else {
		/* 0x39 = DCS Long Write */
		r = dsi_vc_send_long(dssdev, channel, DSI_DT_DCS_LONG_WRITE,
				data, len, 0);
	}

	return r;
}
EXPORT_SYMBOL(dsi_vc_dcs_write_nosync);

int dsi_vc_dcs_write(struct omap_dss_device *dssdev, int channel,
	u8 *data, int len)
{
	struct dsi_struct *ds = dss2dsi(dssdev);

	int r;

	r = dsi_vc_dcs_write_nosync(dssdev, channel, data, len);
	if (r)
		goto err;

	r = dsi_vc_send_bta_sync(dssdev, channel);
	if (r)
		goto err;

	if (REG_GET(ds, DSI_VC_CTRL(channel), 20, 20)) {
		/* RX_FIFO_NOT_EMPTY */
		DSSERR("rx fifo not empty after write, dumping data:\n");
		dsi_vc_flush_receive_data(ds, channel);
		r = -EIO;
		goto err;
	}

	return 0;
err:
	DSSERR("dsi_vc_dcs_write(ch %d, cmd 0x%02x, len %d) failed\n",
			channel, data[0], len);
	return r;
}
EXPORT_SYMBOL(dsi_vc_dcs_write);

int dsi_vc_dcs_write_0(struct omap_dss_device *dssdev, int channel,
	u8 dcs_cmd)
{
	return dsi_vc_dcs_write(dssdev, channel, &dcs_cmd, 1);
}
EXPORT_SYMBOL(dsi_vc_dcs_write_0);

int dsi_vc_dcs_write_1(struct omap_dss_device *dssdev, int channel,
	u8 dcs_cmd, u8 param)
{
	u8 buf[2];
	buf[0] = dcs_cmd;
	buf[1] = param;
	return dsi_vc_dcs_write(dssdev, channel, buf, 2);
}
EXPORT_SYMBOL(dsi_vc_dcs_write_1);

int dsi_vc_dcs_read(struct omap_dss_device *dssdev, int channel,
	u8 dcs_cmd, u8 *buf, int buflen)
{
	struct dsi_struct *ds = dss2dsi(dssdev);
	u32 val;
	u8 dt;
	int r;

	if (ds->debug_read)
		DSSDBG("dsi_vc_dcs_read(ch%d, dcs_cmd %x)\n", channel, dcs_cmd);

	r = dsi_vc_send_short(ds, channel, DSI_DT_DCS_READ, dcs_cmd, 0);
	if (r)
		goto err;

	r = dsi_vc_send_bta_sync(dssdev, channel);
	if (r)
		goto err;

	/* RX_FIFO_NOT_EMPTY */
	if (REG_GET(ds, DSI_VC_CTRL(channel), 20, 20) == 0) {
		DSSERR("RX fifo empty when trying to read.\n");
		r = -EIO;
		goto err;
	}

	val = dsi_read_reg(ds, DSI_VC_SHORT_PACKET_HEADER(channel));
	if (ds->debug_read)
		DSSDBG("\theader: %08x\n", val);
	dt = FLD_GET(val, 5, 0);
	if (dt == DSI_DT_RX_ACK_WITH_ERR) {
		u16 err = FLD_GET(val, 23, 8);
		dsi_show_rx_ack_with_err(err);
		r = -EIO;
		goto err;

	} else if (dt == DSI_DT_RX_SHORT_READ_1) {
		u8 data = FLD_GET(val, 15, 8);
		if (ds->debug_read)
			DSSDBG("\tDCS short response, 1 byte: %02x\n", data);

		if (buflen < 1) {
			r = -EIO;
			goto err;
		}

		buf[0] = data;

		return 1;
	} else if (dt == DSI_DT_RX_SHORT_READ_2) {
		u16 data = FLD_GET(val, 23, 8);
		if (ds->debug_read)
			DSSDBG("\tDCS short response, 2 byte: %04x\n", data);

		if (buflen < 2) {
			r = -EIO;
			goto err;
		}

		buf[0] = data & 0xff;
		buf[1] = (data >> 8) & 0xff;

		return 2;
	} else if (dt == DSI_DT_RX_DCS_LONG_READ) {
		int w;
		int len = FLD_GET(val, 23, 8);
		if (ds->debug_read)
			DSSDBG("\tDCS long response, len %d\n", len);

		if (len > buflen) {
			r = -EIO;
			goto err;
		}

		/* two byte checksum ends the packet, not included in len */
		for (w = 0; w < len + 2;) {
			int b;
			val = dsi_read_reg(ds,
					DSI_VC_SHORT_PACKET_HEADER(channel));
			if (ds->debug_read)
				DSSDBG("\t\t%02x %02x %02x %02x\n",
						(val >> 0) & 0xff,
						(val >> 8) & 0xff,
						(val >> 16) & 0xff,
						(val >> 24) & 0xff);

			for (b = 0; b < 4; ++b) {
				if (w < len)
					buf[w] = (val >> (b * 8)) & 0xff;
				/* we discard the 2 byte checksum */
				++w;
			}
		}

		return len;
	} else {
		DSSERR("\tunknown datatype 0x%02x\n", dt);
		r = -EIO;
		goto err;
	}

	BUG();
err:
	DSSERR("dsi_vc_dcs_read(ch %d, cmd 0x%02x) failed\n",
			channel, dcs_cmd);
	return r;

}
EXPORT_SYMBOL(dsi_vc_dcs_read);

int dsi_vc_dcs_read_1(struct omap_dss_device *dssdev, int channel,
	u8 dcs_cmd, u8 *data)
{
	int r;

	r = dsi_vc_dcs_read(dssdev, channel, dcs_cmd, data, 1);

	if (r < 0)
		return r;

	if (r != 1)
		return -EIO;

	return 0;
}
EXPORT_SYMBOL(dsi_vc_dcs_read_1);

int dsi_vc_dcs_read_2(struct omap_dss_device *dssdev, int channel,
	u8 dcs_cmd, u8 *data1, u8 *data2)
{
	u8 buf[2];
	int r;

	r = dsi_vc_dcs_read(dssdev, channel, dcs_cmd, buf, 2);

	if (r < 0)
		return r;

	if (r != 2)
		return -EIO;

	*data1 = buf[0];
	*data2 = buf[1];

	return 0;
}
EXPORT_SYMBOL(dsi_vc_dcs_read_2);

int dsi_vc_set_max_rx_packet_size(struct omap_dss_device *dssdev,
	int channel, u16 len)
{
	struct dsi_struct *ds = dss2dsi(dssdev);
	return dsi_vc_send_short(ds, channel, DSI_DT_SET_MAX_RET_PKG_SIZE,
			len, 0);
}
EXPORT_SYMBOL(dsi_vc_set_max_rx_packet_size);

static void dsi_set_lp_rx_timeout(struct dsi_struct *ds, unsigned ticks,
							bool x4, bool x16)
{
	unsigned long fck;
	unsigned long total_ticks;
	u32 r;

	BUG_ON(ticks > 0x1fff);

	/* ticks in DSI_FCK */
	fck = dsi_fclk_rate(ds);

	r = dsi_read_reg(ds, DSI_TIMING2);
	r = FLD_MOD(r, cpu_is_omap44xx() ? 0 : 1,
			15, 15);		/* LP_RX_TO */
	r = FLD_MOD(r, x16 ? 1 : 0, 14, 14);	/* LP_RX_TO_X16 */
	r = FLD_MOD(r, x4 ? 1 : 0, 13, 13);	/* LP_RX_TO_X4 */
	r = FLD_MOD(r, ticks, 12, 0);		/* LP_RX_COUNTER */
	dsi_write_reg(ds, DSI_TIMING2, r);

	total_ticks = ticks * (x16 ? 16 : 1) * (x4 ? 4 : 1);

	DSSDBG("LP_RX_TO %lu ticks (%#x%s%s) = %lu ns\n",
			total_ticks,
			ticks, x4 ? " x4" : "", x16 ? " x16" : "",
			(total_ticks * 1000) / (fck / 1000 / 1000));
}

static void dsi_set_ta_timeout(struct dsi_struct *ds, unsigned ticks,
							bool x8, bool x16)
{
	unsigned long fck;
	unsigned long total_ticks;
	u32 r;

	BUG_ON(ticks > 0x1fff);

	/* ticks in DSI_FCK */
	fck = dsi_fclk_rate(ds);

	r = dsi_read_reg(ds, DSI_TIMING1);
	r = FLD_MOD(r, cpu_is_omap44xx() ? 0 : 1,
			31, 31);		/* TA_TO */
	r = FLD_MOD(r, x16 ? 1 : 0, 30, 30);	/* TA_TO_X16 */
	r = FLD_MOD(r, x8 ? 1 : 0, 29, 29);	/* TA_TO_X8 */
	r = FLD_MOD(r, ticks, 28, 16);		/* TA_TO_COUNTER */
	dsi_write_reg(ds, DSI_TIMING1, r);

	total_ticks = ticks * (x16 ? 16 : 1) * (x8 ? 8 : 1);

	DSSDBG("TA_TO %lu ticks (%#x%s%s) = %lu ns\n",
			total_ticks,
			ticks, x8 ? " x8" : "", x16 ? " x16" : "",
			(total_ticks * 1000) / (fck / 1000 / 1000));
}

static void dsi_set_stop_state_counter(struct dsi_struct *ds, unsigned ticks,
							bool x4, bool x16)
{
	unsigned long fck;
	unsigned long total_ticks;
	u32 r;

	BUG_ON(ticks > 0x1fff);

	/* ticks in DSI_FCK */
	fck = dsi_fclk_rate(ds);

	r = dsi_read_reg(ds, DSI_TIMING1);
	r = FLD_MOD(r, cpu_is_omap44xx() ? 0 : 1,
			15, 15);		/* FORCE_TX_STOP_MODE_IO */
	r = FLD_MOD(r, x16 ? 1 : 0, 14, 14);	/* STOP_STATE_X16_IO */
	r = FLD_MOD(r, x4 ? 1 : 0, 13, 13);	/* STOP_STATE_X4_IO */
	r = FLD_MOD(r, ticks, 12, 0);		/* STOP_STATE_COUNTER_IO */
	dsi_write_reg(ds, DSI_TIMING1, r);

	total_ticks = ticks * (x16 ? 16 : 1) * (x4 ? 4 : 1);

	DSSDBG("STOP_STATE_COUNTER %lu ticks (%#x%s%s) = %lu ns\n",
			total_ticks,
			ticks, x4 ? " x4" : "", x16 ? " x16" : "",
			(total_ticks * 1000) / (fck / 1000 / 1000));
}

static void dsi_set_hs_tx_timeout(struct dsi_struct *ds, unsigned ticks,
							bool x4, bool x16)
{
	unsigned long fck;
	unsigned long total_ticks;
	u32 r;

	BUG_ON(ticks > 0x1fff);

	/* ticks in TxByteClkHS */
	fck = dsi_get_txbyteclkhs(ds);

	r = dsi_read_reg(ds, DSI_TIMING2);
	r = FLD_MOD(r, cpu_is_omap44xx() ? 0 : 1,
			31, 31);		/* HS_TX_TO */
	r = FLD_MOD(r, x16 ? 1 : 0, 30, 30);	/* HS_TX_TO_X16 */
	r = FLD_MOD(r, x4 ? 1 : 0, 29, 29);	/* HS_TX_TO_X8 (4 really) */
	r = FLD_MOD(r, ticks, 28, 16);		/* HS_TX_TO_COUNTER */
	dsi_write_reg(ds, DSI_TIMING2, r);

	total_ticks = ticks * (x16 ? 16 : 1) * (x4 ? 4 : 1);

	DSSDBG("HS_TX_TO %lu ticks (%#x%s%s) = %lu ns\n",
			total_ticks,
			ticks, x4 ? " x4" : "", x16 ? " x16" : "",
			(total_ticks * 1000) / (fck / 1000 / 1000));
}
static int dsi_proto_config(struct omap_dss_device *dssdev)
{
	u32 r;
	int buswidth = 0;
	struct dsi_struct *ds = dss2dsi(dssdev);

	dsi_config_tx_fifo(ds, DSI_FIFO_SIZE_32,
			DSI_FIFO_SIZE_32,
			DSI_FIFO_SIZE_32,
			DSI_FIFO_SIZE_32);

	dsi_config_rx_fifo(ds, DSI_FIFO_SIZE_32,
			DSI_FIFO_SIZE_32,
			DSI_FIFO_SIZE_32,
			DSI_FIFO_SIZE_32);

	/* XXX what values for the timeouts? */
	dsi_set_stop_state_counter(ds, 0x1000, false, false);
	dsi_set_ta_timeout(ds, 0x1fff, true, true);
	dsi_set_lp_rx_timeout(ds, 0x1fff, true, true);
	dsi_set_hs_tx_timeout(ds, 0x1fff, true, true);

	switch (dssdev->ctrl.pixel_size) {
	case 16:
		buswidth = 0;
		break;
	case 18:
		buswidth = 1;
		break;
	case 24:
		buswidth = 2;
		break;
	default:
		BUG();
	}

	r = dsi_read_reg(ds, DSI_CTRL);
	r = FLD_MOD(r, (cpu_is_omap44xx()) ? 0 : 1,
			1, 1);		/* CS_RX_EN */
	r = FLD_MOD(r, (cpu_is_omap44xx()) ? 0 : 1,
			2, 2);		/* ECC_RX_EN */
	r = FLD_MOD(r, 1, 3, 3);	/* TX_FIFO_ARBITRATION */
	r = FLD_MOD(r, 1, 4, 4);	/* VP_CLK_RATIO, always 1, see errata*/
	r = FLD_MOD(r, buswidth, 7, 6); /* VP_DATA_BUS_WIDTH */
	r = FLD_MOD(r, 0, 8, 8);	/* VP_CLK_POL */
	r = FLD_MOD(r, (cpu_is_omap44xx()) ? 1 : 0,
			9, 9);		/* VP_DE_POL */
	r = FLD_MOD(r, (cpu_is_omap44xx()) ? 1 : 0,
			11, 11);	/* VP_VSYNC_POL */
	r = FLD_MOD(r, 2, 13, 12);	/* LINE_BUFFER, 2 lines */
	r = FLD_MOD(r, 1, 14, 14);	/* TRIGGER_RESET_MODE */
	r = FLD_MOD(r, 1, 19, 19);	/* EOT_ENABLE */
	if (cpu_is_omap34xx()) {
		r = FLD_MOD(r, 1, 24, 24);	/* DCS_CMD_ENABLE */
		r = FLD_MOD(r, 0, 25, 25);	/* DCS_CMD_CODE */
	}
	dsi_write_reg(ds, DSI_CTRL, r);

	dsi_vc_initial_config(ds, 0);
	if (cpu_is_omap44xx())
		dsi_vc_initial_config_vp(ds, 1);
	else
		dsi_vc_initial_config(ds, 1);
	dsi_vc_initial_config(ds, 2);
	dsi_vc_initial_config(ds, 3);

	/* In Present OMAP4 configuration, 2 VC's send data
	* to the same peripheral */
	for (r = 0; r < ARRAY_SIZE(ds->vc); r++)
		ds->vc[r].dest_per = cpu_is_omap44xx() ? 0 : r;

	return 0;
}

static void dsi_proto_timings(struct omap_dss_device *dssdev)
{
	unsigned tlpx, tclk_zero, tclk_prepare, tclk_trail;
	unsigned tclk_pre, tclk_post;
	unsigned ths_prepare, ths_prepare_ths_zero, ths_zero;
	unsigned ths_trail, ths_exit;
	unsigned ddr_clk_pre, ddr_clk_post;
	unsigned enter_hs_mode_lat, exit_hs_mode_lat;
	unsigned ths_eot;
	u32 r;
	struct dsi_struct *ds = dss2dsi(dssdev);

	r = dsi_read_reg(ds, DSI_DSIPHY_CFG0);
	ths_prepare = FLD_GET(r, 31, 24);
	ths_prepare_ths_zero = FLD_GET(r, 23, 16);
	ths_zero = ths_prepare_ths_zero - ths_prepare;
	ths_trail = FLD_GET(r, 15, 8);
	ths_exit = FLD_GET(r, 7, 0);

	r = dsi_read_reg(ds, DSI_DSIPHY_CFG1);
	tlpx = FLD_GET(r, 22, 16) * 2;
	tclk_trail = FLD_GET(r, 15, 8);
	tclk_zero = FLD_GET(r, 7, 0);

	r = dsi_read_reg(ds, DSI_DSIPHY_CFG2);
	tclk_prepare = FLD_GET(r, 7, 0);

	/* min 8*UI */
	tclk_pre = 20;
	/* min 60ns + 52*UI */
	tclk_post = ns2ddr(ds, 60) + 26;

	/* ths_eot is 2 for 2 datalanes and 4 for 1 datalane */
	if (dssdev->phy.dsi.data1_lane != 0 &&
			dssdev->phy.dsi.data2_lane != 0)
		ths_eot = 2;
	else
		ths_eot = 4;

	ddr_clk_pre = DIV_ROUND_UP(tclk_pre + tlpx + tclk_zero + tclk_prepare,
			4);
	ddr_clk_post = DIV_ROUND_UP(tclk_post + ths_trail, 4) + ths_eot;

	BUG_ON(ddr_clk_pre == 0 || ddr_clk_pre > 255);
	BUG_ON(ddr_clk_post == 0 || ddr_clk_post > 255);

	r = dsi_read_reg(ds, DSI_CLK_TIMING);
	r = FLD_MOD(r, ddr_clk_pre, 15, 8);
	r = FLD_MOD(r, ddr_clk_post, 7, 0);
	dsi_write_reg(ds, DSI_CLK_TIMING, r);

	DSSDBG("ddr_clk_pre %u, ddr_clk_post %u\n",
			ddr_clk_pre,
			ddr_clk_post);

	enter_hs_mode_lat = 1 + DIV_ROUND_UP(tlpx, 4) +
		DIV_ROUND_UP(ths_prepare, 4) +
		DIV_ROUND_UP(ths_zero + 3, 4);

	exit_hs_mode_lat = DIV_ROUND_UP(ths_trail + ths_exit, 4) + 1 + ths_eot;

	r = FLD_VAL(enter_hs_mode_lat, 31, 16) |
		FLD_VAL(exit_hs_mode_lat, 15, 0);
	dsi_write_reg(ds, DSI_VM_TIMING7, r);

	DSSDBG("enter_hs_mode_lat %u, exit_hs_mode_lat %u\n",
			enter_hs_mode_lat, exit_hs_mode_lat);
}


#define DSI_DECL_VARS \
	int __dsi_cb = 0; u32 __dsi_cv = 0;

#define DSI_FLUSH(ds, ch) \
	if (__dsi_cb > 0) { \
		/*DSSDBG("sending long packet %#010x\n", __dsi_cv);*/ \
		dsi_write_reg(ds, DSI_VC_LONG_PACKET_PAYLOAD(ch), __dsi_cv); \
		__dsi_cb = __dsi_cv = 0; \
	}

#define DSI_PUSH(ds, ch, data) \
	do { \
		__dsi_cv |= (data) << (__dsi_cb * 8); \
		/*DSSDBG("cv = %#010x, cb = %d\n", __dsi_cv, __dsi_cb);*/ \
		if (++__dsi_cb > 3) \
			DSI_FLUSH(ds, ch); \
	} while (0)

static int dsi_update_screen_l4(struct omap_dss_device *dssdev,
			int x, int y, int w, int h)
{
	/* Note: supports only 24bit colors in 32bit container */
	int first = 1;
	int fifo_stalls = 0;
	int max_dsi_packet_size;
	int max_data_per_packet;
	int max_pixels_per_packet;
	int pixels_left;
	int bytespp = dssdev->ctrl.pixel_size / 8;
	int scr_width;
	u32 __iomem *data;
	int start_offset;
	int horiz_inc;
	int current_x;
	struct omap_overlay *ovl;
	struct dsi_struct *ds = dss2dsi(dssdev);

	debug_irq = 0;

	DSSDBG("dsi_update_screen_l4 (%d,%d %dx%d)\n",
			x, y, w, h);

	ovl = dssdev->manager->overlays[0];

	if (ovl->info.color_mode != OMAP_DSS_COLOR_RGB24U)
		return -EINVAL;

	if (dssdev->ctrl.pixel_size != 24)
		return -EINVAL;

	scr_width = ovl->info.screen_width;
	data = ovl->info.vaddr;

	start_offset = scr_width * y + x;
	horiz_inc = scr_width - w;
	current_x = x;

	/* We need header(4) + DCSCMD(1) + pixels(numpix*bytespp) bytes
	 * in fifo */

	/* When using CPU, max long packet size is TX buffer size */
	max_dsi_packet_size = ds->vc[0].fifo_size * 32 * 4;

	/* we seem to get better perf if we divide the tx fifo to half,
	   and while the other half is being sent, we fill the other half
	   max_dsi_packet_size /= 2; */

	max_data_per_packet = max_dsi_packet_size - 4 - 1;

	max_pixels_per_packet = max_data_per_packet / bytespp;

	DSSDBG("max_pixels_per_packet %d\n", max_pixels_per_packet);

	pixels_left = w * h;

	DSSDBG("total pixels %d\n", pixels_left);

	data += start_offset;

	while (pixels_left > 0) {
		/* 0x2c = write_memory_start */
		/* 0x3c = write_memory_continue */
		u8 dcs_cmd = first ? 0x2c : 0x3c;
		int pixels;
		DSI_DECL_VARS;
		first = 0;

#if 1
		/* using fifo not empty */
		/* TX_FIFO_NOT_EMPTY */
		while (FLD_GET(dsi_read_reg(ds, DSI_VC_CTRL(0)), 5, 5)) {
			fifo_stalls++;
			if (fifo_stalls > 0xfffff) {
				DSSERR("fifo stalls overflow, pixels left %d\n",
						pixels_left);
				dsi_if_enable(ds, 0);
				return -EIO;
			}
			udelay(1);
		}
#elif 1
		/* using fifo emptiness */
		while ((REG_GET(ds, DSI_TX_FIFO_VC_EMPTINESS, 7, 0)+1)*4 <
				max_dsi_packet_size) {
			fifo_stalls++;
			if (fifo_stalls > 0xfffff) {
				DSSERR("fifo stalls overflow, pixels left %d\n",
						pixels_left);
				dsi_if_enable(ds, 0);
				return -EIO;
			}
		}
#else
		while ((REG_GET(ds, DSI_TX_FIFO_VC_EMPTINESS, 7, 0)+1)*4 == 0) {
			fifo_stalls++;
			if (fifo_stalls > 0xfffff) {
				DSSERR("fifo stalls overflow, pixels left %d\n",
						pixels_left);
				dsi_if_enable(ds, 0);
				return -EIO;
			}
		}
#endif
		pixels = min(max_pixels_per_packet, pixels_left);

		pixels_left -= pixels;

		dsi_vc_write_long_header(ds, 0, DSI_DT_DCS_LONG_WRITE,
				1 + pixels * bytespp, 0);

		DSI_PUSH(ds, 0, dcs_cmd);

		while (pixels-- > 0) {
			u32 pix = __raw_readl(data++);

			DSI_PUSH(ds, 0, (pix >> 16) & 0xff);
			DSI_PUSH(ds, 0, (pix >> 8) & 0xff);
			DSI_PUSH(ds, 0, (pix >> 0) & 0xff);

			current_x++;
			if (current_x == x+w) {
				current_x = x;
				data += horiz_inc;
			}
		}

		DSI_FLUSH(ds, 0);
	}

	return 0;
}

static void dsi_update_screen_dispc(struct omap_dss_device *dssdev,
		u16 x, u16 y, u16 w, u16 h)
{
	unsigned bytespp;
	unsigned bytespl;
	unsigned bytespf;
	unsigned total_len;
	unsigned packet_payload;
	unsigned packet_len;
	u32 l;
	int r;
	struct dsi_struct *ds = dss2dsi(dssdev);
	const unsigned channel = ds->update_channel;
	/* line buffer is 1024 x 24bits */
	/* XXX: for some reason using full buffer size causes considerable TX
	 * slowdown with update sizes that fill the whole buffer */
	const unsigned line_buf_size = 1023 * 3;

	DSSDBG("dsi_update_screen_dispc(%d,%d %dx%d)\n",
			x, y, w, h);

	dsi_vc_config_vp(ds, channel);

	bytespp	= dssdev->ctrl.pixel_size / 8;
	bytespl = w * bytespp;
	bytespf = bytespl * h;

	/* NOTE: packet_payload has to be equal to N * bytespl, where N is
	 * number of lines in a packet.  See errata about VP_CLK_RATIO */

	if (bytespf < line_buf_size)
		packet_payload = bytespf;
	else
		packet_payload = (line_buf_size) / bytespl * bytespl;

	packet_len = packet_payload + 1;	/* 1 byte for DCS cmd */
	total_len = (bytespf / packet_payload) * packet_len;

	if (bytespf % packet_payload)
		total_len += (bytespf % packet_payload) + 1;

	l = FLD_VAL(total_len, 23, 0); /* TE_SIZE */
	dsi_write_reg(ds, DSI_VC_TE(channel), l);

	dsi_vc_write_long_header(ds, channel, DSI_DT_DCS_LONG_WRITE,
			packet_len, 0);

	dsi_write_reg(ds, DSI_TE_HSYNC_WIDTH(channel), 0x00000100);
	dsi_write_reg(ds, DSI_TE_VSYNC_WIDTH(channel), 0x0000FF00);
	dsi_write_reg(ds, DSI_TE_HSYNC_NUMBER(channel), 0x0000006E);

	if (ds->te_enabled)
		l = FLD_MOD(l, 1, 30, 30); /* TE_EN */
	else
		l = FLD_MOD(l, 1, 31, 31); /* TE_START */
	dsi_write_reg(ds, DSI_VC_TE(channel), l);

	/* We put SIDLEMODE to no-idle for the duration of the transfer,
	 * because DSS interrupts are not capable of waking up the CPU and the
	 * framedone interrupt could be delayed for quite a long time. I think
	 * the same goes for any DSS interrupts, but for some reason I have not
	 * seen the problem anywhere else than here.
	 */
	if (!cpu_is_omap44xx())
		dispc_disable_sidle();

	dsi_perf_mark_start(ds);

	r = queue_delayed_work(ds->workqueue,
			&ds->framedone_timeout_work, msecs_to_jiffies(250));
	BUG_ON(r == 0);

	dss_start_update(dssdev);

	if (ds->te_enabled) {
		/* disable LP_RX_TO, so that we can receive TE.  Time to wait
		 * for TE is longer than the timer allows */
		REG_FLD_MOD(ds, DSI_TIMING2, 0, 15, 15); /* LP_RX_TO */

		dsi_vc_send_bta(ds, cpu_is_omap44xx() ? 0 : channel);

#ifdef DSI_CATCH_MISSING_TE
		mod_timer(&ds->te_timer, jiffies + msecs_to_jiffies(250));
#endif
	}
}

static void dsi_videomode_update_screen_dispc(struct omap_dss_device *dssdev,
		u16 x, u16 y, u16 w, u16 h)
{
	udelay(1000);
	dss_start_update(dssdev);
}

#ifdef DSI_CATCH_MISSING_TE
static void dsi_te_timeout(unsigned long arg)
{
	struct dsi_struct *ds = (struct dsi_struct *) arg;
	DSSERR("%s TE not received for 250ms\n", ds->name);
}
#endif

static void dsi_handle_framedone(struct dsi_struct *ds, int error)
{
	const int channel = cpu_is_omap44xx() ? 0 : ds->update_channel;
	struct omap_dss_device *device = ds->vc[channel].dssdev;

	__cancel_delayed_work(&ds->framedone_timeout_work);

	dsi_vc_disable_bta_irq(ds, channel);

	/* SIDLEMODE back to smart-idle */
	dispc_enable_sidle();

	ds->bta_callback = NULL;

	if (error == -ETIMEDOUT)
		/* Ensures recovery of DISPC after a failed lcd_enable*/
		device->manager->disable(device->manager);

	if (ds->te_enabled) {
		/* enable LP_RX_TO again after the TE */
		REG_FLD_MOD(ds, DSI_TIMING2, cpu_is_omap44xx() ? 0 : 1, 15, 15);
	}

	/* RX_FIFO_NOT_EMPTY */
	if (REG_GET(ds, DSI_VC_CTRL(channel), 20, 20)) {
		DSSERR("Received error during frame transfer:\n");
		dsi_vc_flush_receive_data(ds, channel);
		if (!error)
			error = -EIO;
	}

	ds->framedone_callback(error, ds->framedone_data);

	if (!error)
		dsi_perf_show(ds, "DISPC");
}

static void dsi_framedone_timeout_work_callback(struct work_struct *work)
{
	struct dsi_struct *ds = container_of(work, struct dsi_struct,
						framedone_timeout_work.work);

	/* XXX While extremely unlikely, we could get FRAMEDONE interrupt after
	 * 250ms which would conflict with this timeout work. What should be
	 * done is first cancel the transfer on the HW, and then cancel the
	 * possibly scheduled framedone work. However, cancelling the transfer
	 * on the HW is buggy, and would probably require resetting the whole
	 * DSI */

	DSSERR("%s Framedone not received for 250ms!\n", ds->name);

	dsi_handle_framedone(ds, -ETIMEDOUT);
}

static void dsi_framedone_bta_callback(struct dsi_struct *ds)
{
	dsi_handle_framedone(ds, 0);

#ifdef CONFIG_OMAP2_DSS_FAKE_VSYNC
	dispc_fake_irq(ds->irq_vsync);
#endif
}

static void dsi_framedone_irq_callback(void *data, u32 mask)
{
	struct dsi_struct *ds = data;
	const int channel = cpu_is_omap44xx() ? 0 : ds->update_channel;
	int r;

	/* Note: We get FRAMEDONE when DISPC has finished sending pixels and
	 * turns itself off. However, DSI still has the pixels in its buffers,
	 * and is sending the data.
	 */

	if (ds->te_enabled) {
		/* enable LP_RX_TO again after the TE */
		REG_FLD_MOD(ds, DSI_TIMING2, cpu_is_omap44xx() ? 0 : 1, 15, 15);
	}

	/* Send BTA after the frame. We need this for the TE to work, as TE
	 * trigger is only sent for BTAs without preceding packet. Thus we need
	 * to BTA after the pixel packets so that next BTA will cause TE
	 * trigger.
	 *
	 * This is not needed when TE is not in use, but we do it anyway to
	 * make sure that the transfer has been completed. It would be more
	 * optimal, but more complex, to wait only just before starting next
	 * transfer.
	 *
	 * Also, as there's no interrupt telling when the transfer has been
	 * done and the channel could be reconfigured, the only way is to
	 * busyloop until TE_SIZE is zero. With BTA we can do this
	 * asynchronously.
	 * */

	ds->bta_callback = dsi_framedone_bta_callback;

	barrier();

	dsi_vc_enable_bta_irq(ds, channel);

	r = dsi_vc_send_bta(ds, channel);
	if (r) {
		DSSERR("BTA after framedone failed\n");
		dsi_handle_framedone(ds, -EIO);
	}
}

int omap_dsi_prepare_update(struct omap_dss_device *dssdev,
				    u16 *x, u16 *y, u16 *w, u16 *h,
				    bool enlarge_update_area)
{
	u16 dw, dh;
	struct dsi_struct *ds = dss2dsi(dssdev);

	dssdev->driver->get_resolution(dssdev, &dw, &dh);

	if  (*x > dw || *y > dh)
		return -EINVAL;

	if (*x + *w > dw)
		return -EINVAL;

	if (*y + *h > dh)
		return -EINVAL;

	if (*w == 1)
		return -EINVAL;

	if (*w == 0 || *h == 0)
		return -EINVAL;

	dsi_perf_mark_setup(ds);

	if (dssdev->manager->caps & OMAP_DSS_OVL_MGR_CAP_DISPC) {
		dss_setup_partial_planes(dssdev, x, y, w, h,
				enlarge_update_area);
		dispc_set_lcd_size(dssdev->manager->id, *w, *h);
	}

	return 0;
}
EXPORT_SYMBOL(omap_dsi_prepare_update);

int omap_dsi_update(struct omap_dss_device *dssdev,
		int channel, u16 x, u16 y, u16 w, u16 h,
		void (*callback)(int, void *), void *data)
{
	struct dsi_struct *ds = dss2dsi(dssdev);

	dss_hybrid_update_defined(dssdev, &x, &y, &w, &h);

	ds->update_channel = channel;

	/* OMAP DSS cannot send updates of odd widths.
	 * omap_dsi_prepare_update() makes the widths even, but add a BUG_ON
	 * here to make sure we catch erroneous updates. Otherwise we'll only
	 * see rather obscure HW error happening, as DSS halts. */
	BUG_ON(x % 2 == 1);

	if (dssdev->manager) {
		if (dssdev->manager->caps & OMAP_DSS_OVL_MGR_CAP_DISPC) {
			ds->framedone_callback = callback;
			ds->framedone_data = data;

			ds->update_region.x = x;
			ds->update_region.y = y;
			ds->update_region.w = w;
			ds->update_region.h = h;
			ds->update_region.device = dssdev;

			if (omap4_board_rev() == OMAP4_BLAZETABLET_BOARD)
				dsi_videomode_update_screen_dispc(dssdev,
					x, y, w, h);
			else
				dsi_update_screen_dispc(dssdev, x, y, w, h);
		} else {
			int r;

			r = dsi_update_screen_l4(dssdev, x, y, w, h);
			if (r)
				return r;

			dsi_perf_show(ds, "L4");
			callback(0, data);
		}
	}

	return 0;
}
EXPORT_SYMBOL(omap_dsi_update);

/* Display funcs */

static int dsi_display_init_dispc(struct omap_dss_device *dssdev)
{
	int r;
	struct dsi_struct *ds = dss2dsi(dssdev);

	if (omap4_board_rev() == OMAP4_BLAZETABLET_BOARD)
		ds->irq_framedone = DISPC_IRQ_VSYNC;

	r = omap_dispc_register_isr(dsi_framedone_irq_callback, ds,
			ds->irq_framedone);
	if (r) {
		DSSERR("can't get FRAMEDONE irq\n");
		return r;
	}

	dispc_set_lcd_display_type(dssdev->manager->id,
			OMAP_DSS_LCD_DISPLAY_TFT);

	dispc_set_parallel_interface_mode(dssdev->manager->id,
			OMAP_DSS_PARALLELMODE_DSI);
	dispc_enable_fifohandcheck(dssdev->manager->id, 1);

	dispc_set_tft_data_lines(dssdev->manager->id, dssdev->ctrl.pixel_size);
	if (omap4_board_rev() == OMAP4_BLAZETABLET_BOARD) {
		/* Just set the values on panel-d2l,
		no need to hardcode them here anymore. */
		dispc_set_lcd_timings(dssdev->channel, &dssdev->panel.timings);
	} else {
		{
			struct omap_video_timings timings = {
				.hsw		= 1,
				.hfp		= 1,
				.hbp		= 1,
				.vsw		= 1,
				.vfp		= 0,
				.vbp		= 0,
			};
			dispc_set_lcd_timings(dssdev->channel, &timings);

		}
	}

	return 0;
}

static void dsi_display_uninit_dispc(struct omap_dss_device *dssdev)
{
	struct dsi_struct *ds = dss2dsi(dssdev);

	omap_dispc_unregister_isr(dsi_framedone_irq_callback, ds,
			ds->irq_framedone);
}

static int dsi_configure_dsi_clocks(struct omap_dss_device *dssdev)
{
	struct dsi_clock_info cinfo;
	int r;

	/* we always use DSS2_FCK as input clock */
	cinfo.use_dss2_fck = true;
	cinfo.regn  = dssdev->phy.dsi.div.regn;
	cinfo.regm  = dssdev->phy.dsi.div.regm;
	cinfo.regm_dispc = dssdev->phy.dsi.div.regm_dispc;
	cinfo.regm_dsi = dssdev->phy.dsi.div.regm_dsi;
	r = dsi_calc_clock_rates(dssdev, &cinfo);
	if (r) {
		DSSERR("Failed to calc dsi clocks\n");
		return r;
	}

	r = dsi_pll_set_clock_div(dssdev->channel, &cinfo);
	if (r) {
		DSSERR("Failed to set dsi clocks\n");
		return r;
	}

	return 0;
}

static int dsi_configure_dispc_clocks(struct omap_dss_device *dssdev)
{
	struct dispc_clock_info dispc_cinfo;
	int r;
	unsigned long long fck;

	fck = dsi_get_pll_dispc_rate(dssdev->channel);

	dispc_cinfo.lck_div = dssdev->phy.dsi.div.lck_div;
	dispc_cinfo.pck_div = dssdev->phy.dsi.div.pck_div;

	r = dispc_calc_clock_rates(fck, &dispc_cinfo);
	if (r) {
		DSSERR("Failed to calc dispc clocks\n");
		return r;
	}

	r = dispc_set_clock_div(dssdev->manager->id, &dispc_cinfo);
	if (r) {
		DSSERR("Failed to set dispc clocks\n");
		return r;
	}

	return 0;
}

static int dsi_display_init_dsi(struct omap_dss_device *dssdev)
{
	int r;
	struct dsi_struct *ds = dss2dsi(dssdev);

	/* The SCPClk is required for PLL and complexio registers on OMAP4 */
	if (cpu_is_omap44xx())
		REG_FLD_MOD(ds, DSI_CLK_CTRL, 1, 14, 14);

	_dsi_print_reset_status(ds);

	r = dsi_pll_init(dssdev->channel, true, true);
	if (r)
		goto err0;

	r = dsi_configure_dsi_clocks(dssdev);
	if (r)
		goto err1;

	dsi_wait_pll_dispc_active(dssdev->channel);
	dss_select_dispc_clk_source(ds->clk_src_dispc);
	dsi_wait_pll_dsi_active(dssdev->channel);
	dss_select_dsi_clk_source(dssdev->channel, ds->clk_src_dsi);
	if (cpu_is_omap44xx())
		dss_select_lcd_clk_source(dssdev->channel, ds->clk_src_dispc);

	DSSDBG("PLL OK\n");

	r = dsi_configure_dispc_clocks(dssdev);
	if (r)
		goto err2;

	r = dsi_complexio_init(dssdev);
	if (r)
		goto err2;

	_dsi_print_reset_status(ds);

	dsi_proto_timings(dssdev);
	dsi_set_lp_clk_divisor(dssdev);

	if (1)
		_dsi_print_reset_status(ds);

	r = dsi_proto_config(dssdev);
	if (r)
		goto err3;

	/* enable interface */
	dsi_vc_enable(ds, 0, 1);
	dsi_vc_enable(ds, 1, 1);
	dsi_vc_enable(ds, 2, 1);
	dsi_vc_enable(ds, 3, 1);
	dsi_if_enable(ds, 1);
	dsi_force_tx_stop_mode_io(ds);

	return 0;
err3:
	dsi_complexio_uninit(ds);
err2:
	dss_select_dispc_clk_source(DSS_SRC_DSS1_ALWON_FCLK);
	dss_select_dsi_clk_source(dssdev->channel, DSS_SRC_DSS1_ALWON_FCLK);
	if (cpu_is_omap44xx())
		dss_select_lcd_clk_source(dssdev->channel,
						DSS_SRC_DSS1_ALWON_FCLK);
err1:
	dsi_pll_uninit(dssdev->channel);
err0:
	return r;
}

static void dsi_display_uninit_dsi(struct omap_dss_device *dssdev)
{
	struct dsi_struct *ds = dss2dsi(dssdev);

	/* disable interface */
	dsi_if_enable(ds, 0);
	dsi_vc_enable(ds, 0, 0);
	dsi_vc_enable(ds, 1, 0);
	dsi_vc_enable(ds, 2, 0);
	dsi_vc_enable(ds, 3, 0);

	dss_select_dispc_clk_source(DSS_SRC_DSS1_ALWON_FCLK);
	dss_select_dsi_clk_source(dssdev->channel, DSS_SRC_DSS1_ALWON_FCLK);
	if (cpu_is_omap44xx())
		dss_select_lcd_clk_source(dssdev->channel,
						DSS_SRC_DSS1_ALWON_FCLK);
	dsi_complexio_uninit(ds);
	dsi_pll_uninit(dssdev->channel);
}

static int dsi_core_init(struct dsi_struct *ds)
{
	/* Autoidle */
	REG_FLD_MOD(ds, DSI_SYSCONFIG, 1, 0, 0);

	/* ENWAKEUP */
	REG_FLD_MOD(ds, DSI_SYSCONFIG, 1, 2, 2);

	/* SIDLEMODE smart-idle */
	REG_FLD_MOD(ds, DSI_SYSCONFIG, 2, 4, 3);

	_dsi_initialize_irq(ds);

	return 0;
}

int omapdss_dsi_display_enable(struct omap_dss_device *dssdev)
{
	int r = 0;
	struct dsi_struct *ds = dss2dsi(dssdev);

	DSSDBG("dsi_display_enable\n");

	WARN_ON(!dsi_bus_is_locked(ds));

	mutex_lock(&ds->lock);

	r = omap_dss_start_device(dssdev);
	if (r) {
		DSSERR("failed to start device\n");
		goto err0;
	}

	enable_clocks(1);
	dsi_enable_pll_clock(ds, 1);

	r = _dsi_reset(ds);
	if (r)
		goto err1;

	dsi_core_init(ds);

	r = dsi_display_init_dispc(dssdev);
	if (r)
		goto err1;

	r = dsi_display_init_dsi(dssdev);
	if (r)
		goto err2;

	ds->enabled = true;
	mutex_unlock(&ds->lock);

	return 0;

err2:
	dsi_display_uninit_dispc(dssdev);
err1:
	enable_clocks(0);
	dsi_enable_pll_clock(ds, 0);
	omap_dss_stop_device(dssdev);
err0:
	mutex_unlock(&ds->lock);
	DSSDBG("dsi_display_enable FAILED\n");
	return r;
}
EXPORT_SYMBOL(omapdss_dsi_display_enable);

void omapdss_dsi_display_disable(struct omap_dss_device *dssdev)
{
	struct dsi_struct *ds = dss2dsi(dssdev);

	DSSDBG("dsi_display_disable\n");

	WARN_ON(!dsi_bus_is_locked(ds));

	mutex_lock(&ds->lock);

	dsi_display_uninit_dispc(dssdev);

	dsi_display_uninit_dsi(dssdev);

	enable_clocks(0);
	dsi_enable_pll_clock(ds, 0);

	omap_dss_stop_device(dssdev);

	ds->enabled = false;

	mutex_unlock(&ds->lock);
}
EXPORT_SYMBOL(omapdss_dsi_display_disable);

int omapdss_dsi_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	struct dsi_struct *ds = dss2dsi(dssdev);
	ds->te_enabled = enable;
	return 0;
}
EXPORT_SYMBOL(omapdss_dsi_enable_te);

void dsi_get_overlay_fifo_thresholds(enum omap_plane plane,
		u32 fifo_size, enum omap_burst_size *burst_size,
		u32 *fifo_low, u32 *fifo_high)
{
	unsigned burst_size_bytes;

	*burst_size = OMAP_DSS_BURST_16x32;
	burst_size_bytes = 16 * 32 / 8;

	*fifo_high = fifo_size - burst_size_bytes;
	*fifo_low = fifo_size - burst_size_bytes * 2;
}

int dsi_init_display(struct omap_dss_device *dssdev)
{
	struct dsi_struct *ds = dss2dsi(dssdev);
	DSSDBG("DSI init\n");

	/* XXX these should be figured out dynamically */
	dssdev->caps = OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE |
		OMAP_DSS_DISPLAY_CAP_TEAR_ELIM;

	ds->vc[0].dssdev = dssdev;
	ds->vc[1].dssdev = dssdev;

	return 0;
}

void dsi_wait_pll_dispc_active(enum omap_channel channel)
{
	struct dsi_struct *ds = dsi4channel(channel);
	if (wait_for_bit_change(ds, DSI_PLL_STATUS, 7, 1) != 1)
		DSSERR("DSI1 PLL clock not active\n");
}

void dsi_wait_pll_dsi_active(enum omap_channel channel)
{
	struct dsi_struct *ds = dsi4channel(channel);
	if (wait_for_bit_change(ds, DSI_PLL_STATUS, 8, 1) != 1)
		DSSERR("DSI2 PLL clock not active\n");
}

void dsi_set_stop_mode(struct omap_dss_device *dssdev, bool stop)
{
	/* set Stop state (LP-11). FORCE_TX_STOP_MODE_IO = stop */
	struct dsi_struct *ds = dss2dsi(dssdev);
	REG_FLD_MOD(ds, DSI_TIMING1, stop, 15, 15);

}
EXPORT_SYMBOL(dsi_set_stop_mode);

/* set extra register hardcoding values for now */
void dsi_videomode_panel_preinit(struct omap_dss_device *dssdev)
{
	struct dsi_struct *ds = dss2dsi(dssdev);
	/* enable DSI interrupts */
	/*REG_FLD_MOD(ds, DSI_IRQENABLE, 0xFFFFFFFF, 31, 0);
	REG_FLD_MOD(ds, DSI_COMPLEXIO_IRQ_STATUS, 0xFFFFFFFF, 31, 0);
	REG_FLD_MOD(ds, DSI_VC_IRQENABLE(0), 0xFFFFFFFF, 31, 0);
	*/

	/* DSI_CTRL */
	REG_FLD_MOD(ds, DSI_CTRL, 1, 15, 15);		/* VP_VSYNC_START */
	REG_FLD_MOD(ds, DSI_CTRL, 1, 17, 17);		/* VP_HSYNC_START */
	REG_FLD_MOD(ds, DSI_CTRL, 1, 21, 21);		/* HFP BLANKING */
	REG_FLD_MOD(ds, DSI_CTRL, 1, 22, 22);		/* HBP BLANKING */
	REG_FLD_MOD(ds, DSI_CTRL, 1, 23, 23);		/* HSA BLANKING */

	/* Configure timings */
	/* HSA=1, HFP=211, HBP=26 */
	REG_FLD_MOD(ds, DSI_VM_TIMING1, 0x010D301A, 31, 0);
	/* WINDOW_SIZE=4, VSA=8, VFP=15, VBP=15 */
	REG_FLD_MOD(ds, DSI_VM_TIMING2, 0x04080F0F, 31, 0);
	/* TL(31:16)=1008, VACT(15:0)=768 */
	REG_FLD_MOD(ds, DSI_VM_TIMING3, 0x03F00300, 31, 0);
	/* HSA_HS_INTERLEAVING(23:16)=72,
	HFP_HS_INTERLEAVING(15:8)=114,
	HBP_HS_INTERLEAVING(7:0)=150 */
	REG_FLD_MOD(ds, DSI_VM_TIMING4, 0x00487296, 31, 0);
	/* VC3_FIFO_EMPTINESS, VC2_FIFO_EMPTINESS,
	VC1_FIFO_EMPTINESS, VC0_FIFO_EMPTINESS*/
	REG_FLD_MOD(ds, DSI_VM_TIMING5, 0x0082DF3B, 31, 0);
	/* HSA_LP_INTERLEAVING(23:16)=103, HFP_HS_INTERLEAVING(15:8)=49,
	HBP_HS_INTERLEAVING(7:0)=209 */
	REG_FLD_MOD(ds, DSI_VM_TIMING6, 0x7A6731D1, 31, 0);
	/* BL_HS_INTERLEAVING(23:16)=14, BL_LP_INTERLEAVING(15:0)=19 */
	REG_FLD_MOD(ds, DSI_VM_TIMING7, 0x000E0013, 31, 0);
	/* set TA_TO_COUNTER accordignly to kozio value */
	REG_FLD_MOD(ds, DSI_TIMING1, 0xFFFF7FFF, 31, 0);

	/* set TA_TO_COUNTER accordignly to kozio value */
	REG_FLD_MOD(ds, DSI_TIMING2, 0xFFFF7FFF, 31, 0);

	/* Added because there were some differences with reference dump */
	/*REG_FLD_MOD(ds, DSI_CLK_TIMING, 0x0000161B, 31, 0);
	REG_FLD_MOD(ds, DSI_TX_FIFO_VC_SIZE, 0x00004040, 31, 0);
	REG_FLD_MOD(ds, DSI_RX_FIFO_VC_SIZE, 0x00001010, 31, 0);
	REG_FLD_MOD(ds, DSI_COMPLEXIO_CFG2, 0x00010000, 31, 0);*/

	/*REG_FLD_MOD(ds, DSI_CLK_TIMING, 0x00001810, 31, 0);
	REG_FLD_MOD(ds, DSI_TX_FIFO_VC_SIZE, 0x04042220, 31, 0);
	REG_FLD_MOD(ds, DSI_RX_FIFO_VC_SIZE, 0x04042220, 31, 0);
	REG_FLD_MOD(ds, DSI_COMPLEXIO_CFG2, 0x00010000, 31, 0);
	*/

	/* set VC0: */
	REG_FLD_MOD(ds, DSI_VC_CTRL(0) , 0x20800780, 31, 0);

	/* set VC1: */
	REG_FLD_MOD(ds, DSI_VC_CTRL(1) , 0x20800D80, 31, 0);


	/* disable if interface */
	dsi_if_enable(ds, 0);
	mdelay(5);

	/* enable vc1 */
	dsi_vc_enable(ds, 1, 1);
	mdelay(5);

	/* enable vc0 */
	dsi_vc_enable(ds, 0, 1);
	mdelay(5);

	/* enable if interface */
	dsi_if_enable(ds, 1);
	mdelay(5);

	/* Send null packet to start DDR clock */
	dsi_write_reg(ds, DSI_VC_SHORT_PACKET_HEADER(0), 0);
	mdelay(1);

}

/* set extra register hardcoding values for now (according to kozio) */
void dsi_videomode_panel_postinit(struct omap_dss_device *dssdev)
{
	u32 header;
	u16 word_count;
	u8 data_type;
	u8 ecc;
	u8 vc_id;
	u16 pixels_per_line = 1024;
	struct dsi_struct *ds = dss2dsi(dssdev);

	/* disable if interface */
	dsi_if_enable(ds, 0);
	mdelay(5);

	/* disable vc1 */
	dsi_vc_enable(ds, 1, 0);
	mdelay(5);

	/* disable vc0 */
	dsi_vc_enable(ds, 0, 0);
	mdelay(5);

	/* config vc0 for video mode */
	REG_FLD_MOD(ds, DSI_VC_CTRL(0) , 0x20800790, 31, 0);

	/* Fill VC header to 24-bit RGB888 (0x3E) */
	vc_id = 0;
	data_type = 0x3E;
	word_count = pixels_per_line * 3;
	ecc = 0;

	/* form the DSI video mode packet header */
	header = ((ecc & 0xff) << 24) | ((word_count & 0xffff) << 8) |
			((vc_id & 0x03) << 6) | (data_type & 0x3f);
	/* write the header to DSI register */
	dsi_write_reg(ds, DSI_VC_LONG_PACKET_HEADER(0), header);
	mdelay(5);

	/* enable vc0 */
	dsi_vc_enable(ds, 0, 1);
	mdelay(5);

	/* enable if interface */
	dsi_if_enable(ds, 1);
	mdelay(5);

	/* unset Stop state (LP-11) */
	dsi_set_stop_mode(dssdev, 0);
	mdelay(5);

}

int dsi_init(struct platform_device *pdev)
{
	u32 rev;
	int r;
	struct resource *dsi_mem;

	struct dsi_struct *ds;

	/* find DSI device */
	for (r = 0; r < ARRAY_SIZE(dsi); r++) {
		if (!strcmp(dsi[r].pdev_name, pdev->name))
			break;
	}
	if (r == ARRAY_SIZE(dsi))
		return -EINVAL;
	ds = dsi + r;

	ds->pdata = pdev->dev.platform_data;
	ds->pdev = pdev;

	dsis[ds->channel] = ds;

	spin_lock_init(&ds->errors_lock);
	ds->errors = 0;

#ifdef CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS
	spin_lock_init(&ds->irq_stats_lock);
	ds->irq_stats.last_reset = jiffies;
#endif

	init_completion(&ds->bta_completion);

	mutex_init(&ds->lock);
	sema_init(&ds->bus_lock, 1);

	ds->workqueue = create_singlethread_workqueue(ds->name);
	if (ds->workqueue == NULL)
		return -ENOMEM;

	INIT_DELAYED_WORK_DEFERRABLE(&ds->framedone_timeout_work,
			dsi_framedone_timeout_work_callback);

	if (cpu_is_omap44xx()) {
		r = request_irq(ds->irq_dss, dsi_irq_handler, 0,
			ds->name, ds);
		if (r)
			goto err2;
	}
#ifdef DSI_CATCH_MISSING_TE
	init_timer(&ds->te_timer);
	ds->te_timer.function = dsi_te_timeout;
	ds->te_timer.data = (unsigned long) ds;
#endif
	dsi_mem = platform_get_resource(pdev, IORESOURCE_MEM,
		cpu_is_omap44xx() ? 1 : 0);
	ds->base = ioremap(dsi_mem->start, resource_size(dsi_mem));
	if (!ds->base) {
		DSSERR("can't ioremap %s\n", ds->name);
		r = -ENOMEM;
		goto err1;
	}

	if (!cpu_is_omap44xx()) {
		ds->vdds_dsi_reg = dss_get_vdds_dsi();
		if (IS_ERR(ds->vdds_dsi_reg)) {
			DSSERR("can't get VDDS_%s regulator\n", ds->name);
			r = PTR_ERR(ds->vdds_dsi_reg);
			goto err2;
		}
	}

	enable_clocks(1);

	rev = dsi_read_reg(ds, DSI_REVISION);
	printk(KERN_INFO "OMAP %s rev %d.%d\n", ds->name,
	       FLD_GET(rev, 7, 4), FLD_GET(rev, 3, 0));

	enable_clocks(0);

	return 0;
err2:
	iounmap(ds->base);
	if (cpu_is_omap44xx())
		free_irq(ds->irq_dss, ds);
err1:
	destroy_workqueue(ds->workqueue);
	return r;
}

void dsi_exit(struct platform_device *pdev)
{
	struct omap_dss_device *dssdev = to_dss_device(&pdev->dev);
	struct dsi_struct *ds = dss2dsi(dssdev);

	dsis[ds->channel] = NULL;

	if (cpu_is_omap44xx())
		free_irq(ds->irq_dss, ds);
	iounmap(ds->base);

	destroy_workqueue(ds->workqueue);

	DSSDBG("%s dsi_exit\n", ds->name);
}
