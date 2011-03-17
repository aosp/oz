/*
 * linux/arch/arm/mach-omap2/usb-musb.c
 *
 * This file will contain the board specific details for the
 * MENTOR USB OTG controller on OMAP3430
 *
 * Copyright (C) 2007-2008 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Author: Vikram Pandita
 *
 * Generalization by:
 * Felipe Balbi <felipe.balbi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/usb/android_composite.h>

#include <linux/usb/musb.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/am35xx.h>
#include <plat/usb.h>
#include "control.h"

#define CONTROL_DEV_CONF                0x300
#define PHY_PD				(1 << 0)

#ifdef CONFIG_ARCH_OMAP4
#define DIE_ID_REG_BASE         (L4_44XX_PHYS + 0x2000)
#define DIE_ID_REG_OFFSET               0x200
#else
#define DIE_ID_REG_BASE         (L4_WK_34XX_PHYS + 0xA000)
#define DIE_ID_REG_OFFSET               0x218
#endif /* CONFIG_ARCH_OMAP4 */

#if defined(CONFIG_USB_MUSB_OMAP2PLUS) || defined (CONFIG_USB_MUSB_AM35X)

static void am35x_musb_reset(void)
{
	u32	regval;

	/* Reset the musb interface */
	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);

	regval |= AM35XX_USBOTGSS_SW_RST;
	omap_ctrl_writel(regval, AM35XX_CONTROL_IP_SW_RESET);

	regval &= ~AM35XX_USBOTGSS_SW_RST;
	omap_ctrl_writel(regval, AM35XX_CONTROL_IP_SW_RESET);

	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
}

static void am35x_musb_phy_power(u8 on)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(100);
	u32 devconf2;

	if (on) {
		/*
		 * Start the on-chip PHY and its PLL.
		 */
		devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

		devconf2 &= ~(CONF2_RESET | CONF2_PHYPWRDN | CONF2_OTGPWRDN);
		devconf2 |= CONF2_PHY_PLLON;

		omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);

		pr_info(KERN_INFO "Waiting for PHY clock good...\n");
		while (!(omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2)
				& CONF2_PHYCLKGD)) {
			cpu_relax();

			if (time_after(jiffies, timeout)) {
				pr_err(KERN_ERR "musb PHY clock good timed out\n");
				break;
			}
		}
	} else {
		/*
		 * Power down the on-chip PHY.
		 */
		devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

		devconf2 &= ~CONF2_PHY_PLLON;
		devconf2 |=  CONF2_PHYPWRDN | CONF2_OTGPWRDN;
		omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);
	}
}

static void am35x_musb_clear_irq(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval |= AM35XX_USBOTGSS_INT_CLR;
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static void am35x_musb_set_mode(u8 musb_mode)
{
	u32 devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	devconf2 &= ~CONF2_OTGMODE;
	switch (musb_mode) {
#ifdef	CONFIG_USB_MUSB_HDRC_HCD
	case MUSB_HOST:		/* Force VBUS valid, ID = 0 */
		devconf2 |= CONF2_FORCE_HOST;
		break;
#endif
#ifdef	CONFIG_USB_GADGET_MUSB_HDRC
	case MUSB_PERIPHERAL:	/* Force VBUS valid, ID = 1 */
		devconf2 |= CONF2_FORCE_DEVICE;
		break;
#endif
#ifdef	CONFIG_USB_MUSB_OTG
	case MUSB_OTG:		/* Don't override the VBUS/ID comparators */
		devconf2 |= CONF2_NO_OVERRIDE;
		break;
#endif
	default:
		pr_info(KERN_INFO "Unsupported mode %u\n", musb_mode);
	}

	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);
}

static struct resource musb_resources[] = {
	[0] = { /* start and end set dynamically */
		.flags	= IORESOURCE_MEM,
	},
	[1] = {	/* general IRQ */
		.start	= INT_243X_HS_USB_MC,
		.flags	= IORESOURCE_IRQ,
		.name	= "mc",
	},
	[2] = {	/* DMA IRQ */
		.start	= INT_243X_HS_USB_DMA,
		.flags	= IORESOURCE_IRQ,
		.name	= "dma",
	},
};

static struct musb_hdrc_config musb_config = {
	.multipoint	= 1,
	.dyn_fifo	= 1,
	.num_eps	= 16,
	.ram_bits	= 12,
};

#ifdef CONFIG_ANDROID
#define MAX_USB_SERIAL_NUM		17
#define OMAP_VENDOR_ID			0x0451
#define OMAP_UMS_PRODUCT_ID		0xD100
#define OMAP_ADB_PRODUCT_ID		0xD101
#define OMAP_UMS_ADB_PRODUCT_ID		0xD102
#define OMAP_RNDIS_PRODUCT_ID		0xD103
#define OMAP_RNDIS_ADB_PRODUCT_ID	0xD104
#define OMAP_ACM_PRODUCT_ID		0xD105
#define OMAP_ACM_ADB_PRODUCT_ID		0xD106
#define OMAP_ACM_UMS_ADB_PRODUCT_ID	0xD107
#define OMAP_MTP_PRODUCT_ID		0xD108
#define OMAP_MTP_ADB_PRODUCT_ID		0xD109
#define OMAP_MTP_UMS_ADB_PRODUCT_ID	0xD10A

static char device_serial[MAX_USB_SERIAL_NUM];

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_adb[] = {
	"adb",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_acm[] = {
	"acm",
};

static char *usb_functions_acm_adb[] = {
	"acm",
	"adb",
};

static char *usb_functions_acm_ums_adb[] = {
	"acm",
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_mtp[] = {
	"mtp",
};

static char *usb_functions_mtp_adb[] = {
	"mtp",
	"adb",
};
static char *usb_functions_mtp_ums_adb[] = {
	"mtp",
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
#ifdef CONFIG_USB_ANDROID_MTP
	"mtp",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id     = OMAP_UMS_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_ums),
		.functions      = usb_functions_ums,
	},
	{
		.product_id     = OMAP_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_adb),
		.functions      = usb_functions_adb,
	},
	{
		.product_id     = OMAP_UMS_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_ums_adb),
		.functions      = usb_functions_ums_adb,
	},
	{
		.product_id     = OMAP_RNDIS_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_rndis),
		.functions      = usb_functions_rndis,
	},
	{
		.product_id     = OMAP_RNDIS_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_rndis_adb),
		.functions      = usb_functions_rndis_adb,
	},
	{
		.product_id     = OMAP_ACM_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_acm),
		.functions      = usb_functions_acm,
	},
	{
		.product_id     = OMAP_ACM_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_acm_adb),
		.functions      = usb_functions_acm_adb,
	},
	{
		.product_id     = OMAP_ACM_UMS_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_acm_ums_adb),
		.functions      = usb_functions_acm_ums_adb,
	},
	{
		.product_id     = OMAP_MTP_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp),
		.functions      = usb_functions_mtp,
	},
	{
		.product_id     = OMAP_MTP_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp_adb),
		.functions      = usb_functions_mtp_adb,
	},
	{
		.product_id     = OMAP_MTP_UMS_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp_ums_adb),
		.functions      = usb_functions_mtp_ums_adb,
        },
};

/* standard android USB platform data */
static struct android_usb_platform_data andusb_plat = {
	.vendor_id		= OMAP_VENDOR_ID,
	.product_id		= OMAP_UMS_PRODUCT_ID,
	.manufacturer_name	= "Texas Instruments Inc.",
	.product_name		= "OMAP-3/4",
	.serial_number		= device_serial,
	.num_products		= ARRAY_SIZE(usb_products),
	.products		= usb_products,
	.num_functions		= ARRAY_SIZE(usb_functions_all),
	.functions		= usb_functions_all,
};

static struct platform_device androidusb_device = {
	.name		= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data  = &andusb_plat,
	},
};

#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
static struct usb_mass_storage_platform_data usbms_plat = {
	.vendor		= "Texas Instruments Inc.",
	.product	= "OMAP4",
	.release	= 1,
	.nluns		= 1,
};

static struct platform_device usb_mass_storage_device = {
	.name		= "usb_mass_storage",
	.id		= -1,
	.dev		= {
		.platform_data = &usbms_plat,
	},
};
#endif

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= OMAP_VENDOR_ID,
	.vendorDescr	= "Texas Instruments Inc.",
	};

static struct platform_device rndis_device = {
	.name		= "rndis",
	.id		= -1,
	.dev		= {
		.platform_data = &rndis_pdata,
	},
};
#endif

static void usb_gadget_init(void)
{
	unsigned int val[4] = { 0 };
	unsigned int reg;
#ifdef CONFIG_USB_ANDROID_RNDIS
	int i;
	char *src;
#endif
	reg = DIE_ID_REG_BASE + DIE_ID_REG_OFFSET;

	if (cpu_is_omap44xx()) {
		val[0] = omap_readl(reg);
		val[1] = omap_readl(reg + 0x8);
		val[2] = omap_readl(reg + 0xC);
		val[3] = omap_readl(reg + 0x10);
	} else if (cpu_is_omap34xx()) {
		val[0] = omap_readl(reg);
		val[1] = omap_readl(reg + 0x4);
		val[2] = omap_readl(reg + 0x8);
		val[3] = omap_readl(reg + 0xC);
	}

	snprintf(device_serial, MAX_USB_SERIAL_NUM, "%08X%08X%08X%08X",
					val[3], val[2], val[1], val[0]);

#ifdef CONFIG_USB_ANDROID_RNDIS
	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	src = device_serial;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}

	platform_device_register(&rndis_device);
#endif

#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	platform_device_register(&usb_mass_storage_device);
#endif
	platform_device_register(&androidusb_device);
}

#else

static void usb_gadget_init(void)
{
}

#endif /* CONFIG_ANDROID */

static struct musb_hdrc_platform_data musb_plat = {
#ifdef CONFIG_USB_MUSB_OTG
	.mode		= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode		= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode		= MUSB_PERIPHERAL,
#endif
	/* .clock is set dynamically */
	.config		= &musb_config,

	/* REVISIT charge pump on TWL4030 can supply up to
	 * 100 mA ... but this value is board-specific, like
	 * "mode", and should be passed to usb_musb_init().
	 */
	.power		= 50,			/* up to 100 mA */
};

static u64 musb_dmamask = DMA_BIT_MASK(32);

static struct platform_device musb_device = {
	.name		= "musb-omap2430",
	.id		= -1,
	.dev = {
		.dma_mask		= &musb_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &musb_plat,
	},
	.num_resources	= ARRAY_SIZE(musb_resources),
	.resource	= musb_resources,
};

void __init usb_musb_init(struct omap_musb_board_data *board_data)
{
	if (cpu_is_omap243x()) {
		musb_resources[0].start = OMAP243X_HS_BASE;
	} else if (cpu_is_omap3517() || cpu_is_omap3505()) {
		musb_device.name = "musb-am35x";
		musb_resources[0].start = AM35XX_IPSS_USBOTGSS_BASE;
		musb_resources[1].start = INT_35XX_USBOTG_IRQ;
		board_data->set_phy_power = am35x_musb_phy_power;
		board_data->clear_irq = am35x_musb_clear_irq;
		board_data->set_mode = am35x_musb_set_mode;
		board_data->reset = am35x_musb_reset;
	} else if (cpu_is_omap34xx()) {
		musb_resources[0].start = OMAP34XX_HSUSB_OTG_BASE;
	} else if (cpu_is_omap44xx()) {
		musb_resources[0].start = OMAP44XX_HSUSB_OTG_BASE;
		musb_resources[1].start = OMAP44XX_IRQ_HS_USB_MC_N;
		musb_resources[2].start = OMAP44XX_IRQ_HS_USB_DMA_N;
	}
	musb_resources[0].end = musb_resources[0].start + SZ_4K - 1;

	/*
	 * REVISIT: This line can be removed once all the platforms using
	 * musb_core.c have been converted to use use clkdev.
	 */
	musb_plat.clock = "ick";
	musb_plat.board_data = board_data;
	musb_plat.power = board_data->power >> 1;
	musb_plat.mode = board_data->mode;
	musb_plat.extvbus = board_data->extvbus;

	if (platform_device_register(&musb_device) < 0)
		printk(KERN_ERR "Unable to register HS-USB (MUSB) device\n");

	usb_gadget_init();
}

#else
void __init usb_musb_init(struct omap_musb_board_data *board_data)
{
}
#endif /* CONFIG_USB_MUSB_SOC */
