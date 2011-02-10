/*
 * Toshiba TC358765 bridge chip panel support
 *
 * Copyright (C) Texas Instruments  Corporation
 * Author: Jerry Alexander <x0135174@ti.com>
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <plat/display.h>

#include <plat/toshiba-dsi-panel.h>

#include "panel-d2l.h"

/* DSI Virtual channel. Hardcoded for now. */
#define TCH 1

#define DRIVER_NAME "d2l_i2c_driver"
#define DEVICE_NAME "d2l_i2c_driver"

//define this if you want debug print messages
//#define DEB

#ifdef DEB
	#define PRINT_DEBUG(...) printk(__VA_ARGS__)
#else
	#define PRINT_DEBUG(...)
#endif

static int __devinit d2l_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit d2l_i2c_remove(struct i2c_client *client);

/******************************
******* STRUCTURES ************
******************************/

/* i2c device id table */
static const struct i2c_device_id d2l_i2c_idtable[] = {
	{ DEVICE_NAME, 0 },
	{ },
};

/* i2c_driver structure */
static struct i2c_driver d2l_i2c_driver = {
	.probe		= d2l_i2c_probe,
	.remove		= __exit_p(d2l_i2c_remove),
	.id_table	= d2l_i2c_idtable,
	.driver = {
		.name = DRIVER_NAME
	},
};

static int d2l_update(struct omap_dss_device *dssdev,
				u16 x, u16 y, u16 w, u16 h);

struct panel_regulator {
	struct regulator *regulator;
	const char *name;
	int min_uV;
	int max_uV;
};

static void free_regulators(struct panel_regulator *regulators, int n)
{
	int i;

	for (i = 0; i < n; i++) {
		/* disable/put in reverse order */
		regulator_disable(regulators[n - i - 1].regulator);
		regulator_put(regulators[n - i - 1].regulator);
	}
}

static int init_regulators(struct omap_dss_device *dssdev,
			struct panel_regulator *regulators, int n)
{
	int r, i, v;

	for (i = 0; i < n; i++) {
		struct regulator *reg;

		reg = regulator_get(&dssdev->dev, regulators[i].name);
		if (IS_ERR(reg)) {
			dev_err(&dssdev->dev, "failed to get regulator %s\n",
				regulators[i].name);
			r = PTR_ERR(reg);
			goto err;
		}

		/* FIXME: better handling of fixed vs. variable regulators */
		v = regulator_get_voltage(reg);
		if (v < regulators[i].min_uV || v > regulators[i].max_uV) {
			r = regulator_set_voltage(reg, regulators[i].min_uV,
						regulators[i].max_uV);
			if (r) {
				dev_err(&dssdev->dev,
					"failed to set regulator %s voltage\n",
					regulators[i].name);
				regulator_put(reg);
				goto err;
			}
		}

		r = regulator_enable(reg);
		if (r) {
			dev_err(&dssdev->dev, "failed to enable regulator %s\n",
				regulators[i].name);
			regulator_put(reg);
			goto err;
		}

		regulators[i].regulator = reg;
	}

	return 0;

err:
	free_regulators(regulators, i);

	return r;
}

/**
 * struct panel_config - panel configuration
 * @name: panel name
 * @type: panel type
 * @timings: panel resolution
 * @sleep: various panel specific delays, passed to msleep() if non-zero
 * @reset_sequence: reset sequence timings, passed to udelay() if non-zero
 * @regulators: array of panel regulators
 * @num_regulators: number of regulators in the array
 */
struct panel_config {
	const char *name;
	int type;

	struct omap_video_timings timings;

	struct {
		unsigned int sleep_in;
		unsigned int sleep_out;
		unsigned int hw_reset;
		unsigned int enable_te;
	} sleep;

	struct {
		unsigned int high;
		unsigned int low;
	} reset_sequence;

	struct panel_regulator *regulators;
	int num_regulators;
};

enum {
	PANEL_D2L,
};

static struct panel_config panel_configs[] = {
	{
		.name		= "d2l",
		.type		= PANEL_D2L,
		.timings	= {
			.x_res		= 1024,
			.y_res		= 768,
		        .pixel_clock    = 65183,
		        .hfp            = 282,
		        .hsw            = 6,
		        .hbp            = 32,
		        .vfp            = 15,
		        .vsw            = 8,
		        .vbp            = 15,
		},
	},
};


/* device private data structure */
struct d2l_data {
	struct mutex lock;

        //struct backlight_device *bldev;
        struct omap_dss_device *dssdev;
        bool enabled;
        u8 rotate;
        bool mirror;
        bool use_dsi_bl;
        unsigned long   hw_guard_end;   /* next value of jiffies when we can
                                         * issue the next sleep in/out command
                                         */
        unsigned long   hw_guard_wait;  /* max guard time in jiffies */

	atomic_t do_update;
	struct {
		u16 x;
		u16 y;
		u16 w;
		u16 h;
	} update_region;

        bool cabc_broken;
        unsigned cabc_mode;

	bool force_update;
	struct panel_config *panel_config;
};

struct d2l_i2c {
	struct i2c_client    *client;
	struct mutex xfer_lock;
} *sd1;


static inline struct toshiba_dsi_panel_data
*get_panel_data(const struct omap_dss_device *dssdev)
{
	return (struct toshiba_dsi_panel_data *) dssdev->data;
}

/*************************************
**** Interface utility functions *****
*************************************/
static int d2l_read_block(int reg, u8 *data, int len)
{
	unsigned char wb[2];
	struct i2c_msg msg[2];
	int r;
	mutex_lock(&sd1->xfer_lock);
	wb[0] = (reg & 0xff00) >> 8;
	wb[1] = reg & 0xff;
	msg[0].addr = sd1->client->addr;
	msg[0].len = 2;
	msg[0].flags = 0;
	msg[0].buf = wb;
	msg[1].addr = sd1->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	r = i2c_transfer(sd1->client->adapter, msg, 2);
	mutex_unlock(&sd1->xfer_lock);

	if (r == 2)
		return len;

	return r;
}

static int d2l_i2c_read(int reg)
{
	int r;
	u8 data[4];
	data[1] = data[2] = data[3] = data[0] = 0;

	r = d2l_read_block(reg, data, 4);
	return (int)data[3] <<24 | ((int)(data[2]) << 16) | ((int)(data[1]) << 8) | ((int)(data[0]) );
}

int d2l_write_register(u16 reg, u32 value) {

	int ret;
	DSI_long_write_packet dsi_buf;
	enum omap_dsi_index lcd_ix = DSI1;

	dsi_buf.data_type = 0x29; //generic long write
	dsi_buf.ECC = 0; //no error correcting code
	dsi_buf.WC = 6; //pay load byte count

	// Configure register and value
        dsi_buf.Data_buf[0] = (reg >> 0) & 0xFF;
        dsi_buf.Data_buf[1] = (reg >> 8) & 0xFF;
        dsi_buf.Data_buf[2] = (value >> 0) & 0xFF;
        dsi_buf.Data_buf[3] = (value >> 8) & 0xFF;
        dsi_buf.Data_buf[4] = (value >> 16) & 0xFF;
        dsi_buf.Data_buf[5] = (value >> 24) & 0xFF;

	// Send long packet to DSI
        ret = dsi_vc_send_long(lcd_ix, TCH, dsi_buf.data_type, (u8 *)dsi_buf.Data_buf, 6, 0);

        mdelay(100);

#define PRINT_ERR(r) printk("%s: D2L write register failure (reg=%s)\n",__FUNCTION__, #r)
	if(ret !=  0)
		PRINT_ERR(reg);
#undef PRINT_ERR

	return ret;
}

/****************************
********* DEBUG *************
****************************/
void d2l_dump_regs(void)
{
#define DUMPREG(r) printk("%-35s : %08x\n", #r, d2l_i2c_read(r))

	printk(KERN_ALERT "I2C Toshiba Registers\n");

	printk(KERN_ALERT "DSI D-PHY Layer Registers\n");
	DUMPREG( D0W_DPHYCONTTX);
	DUMPREG( CLW_DPHYCONTRX);
	DUMPREG( D0W_DPHYCONTRX);
	DUMPREG( D1W_DPHYCONTRX);
	DUMPREG( D2W_DPHYCONTRX);
	DUMPREG( D3W_DPHYCONTRX);
	DUMPREG( COM_DPHYCONTRX);
	DUMPREG( CLW_CNTRL);
	DUMPREG( D0W_CNTRL);
	DUMPREG( D1W_CNTRL);
	DUMPREG( D2W_CNTRL);
	DUMPREG( D3W_CNTRL);
	DUMPREG( DFTMODE_CNTRL);

	printk(KERN_ALERT "DSI PPI Layer Registers\n");
	DUMPREG( PPI_STARTPPI);
	DUMPREG( PPI_BUSYPPI);
	DUMPREG( PPI_LINEINITCNT);
	DUMPREG( PPI_LPTXTIMECNT);
	DUMPREG( PPI_LANEENABLE);
	DUMPREG( PPI_TX_RX_TA);
	DUMPREG( PPI_CLS_ATMR);
	DUMPREG( PPI_D0S_ATMR);
	DUMPREG( PPI_D1S_ATMR);
	DUMPREG( PPI_D2S_ATMR);
	DUMPREG( PPI_D3S_ATMR);
	DUMPREG( PPI_D0S_CLRSIPOCOUNT);
	DUMPREG( PPI_D1S_CLRSIPOCOUNT);
	DUMPREG( PPI_D2S_CLRSIPOCOUNT);
	DUMPREG( PPI_D3S_CLRSIPOCOUNT);
	DUMPREG( CLS_PRE);
	DUMPREG( D0S_PRE);
	DUMPREG( D1S_PRE);
	DUMPREG( D2S_PRE);
	DUMPREG( D3S_PRE);
	DUMPREG( CLS_PREP);
	DUMPREG( D0S_PREP);
	DUMPREG( D1S_PREP);
	DUMPREG( D2S_PREP);
	DUMPREG( D3S_PREP);
	DUMPREG( CLS_ZERO);
	DUMPREG( D0S_ZERO);
	DUMPREG( D1S_ZERO);
	DUMPREG( D2S_ZERO);
	DUMPREG( D3S_ZERO);
	DUMPREG( PPI_CLRFLG);
	DUMPREG( PPI_CLRSIPO);
	DUMPREG( PPI_HSTimeout);
	DUMPREG( PPI_HSTimeoutEnable);

	printk(KERN_ALERT "DSI Protocol Layer Registers\n");
	//DUMPREG( DSI_STARTDSI);//write only
	DUMPREG( DSI_BUSYDSI);
	DUMPREG( DSI_LANEENABLE);
	DUMPREG( DSI_LANESTATUS0);
	DUMPREG( DSI_LANESTATUS1);
	DUMPREG( DSI_INTSTATUS);
	DUMPREG( DSI_INTMASK);
	DUMPREG( DSI_INTCLR);
	DUMPREG( DSI_LPTXTO);;

	printk(KERN_ALERT "DSI General Registers\n");
	DUMPREG( DSIERRCNT);

	printk(KERN_ALERT "DSI Application Layer Registers\n");
	DUMPREG( APLCTRL);
	DUMPREG( RDPKTLN);

	printk(KERN_ALERT "Video Path Registers\n");
	DUMPREG( VPCTRL);
	DUMPREG( HTIM1);
	DUMPREG( HTIM2);
	DUMPREG( VTIM1);
	DUMPREG( VTIM2);
	DUMPREG( VFUEN);

	printk(KERN_ALERT "LVDS Registers\n");
	DUMPREG( LVMX0003);
	DUMPREG( LVMX0407);
	DUMPREG( LVMX0811);
	DUMPREG( LVMX1215);
	DUMPREG( LVMX1619);
	DUMPREG( LVMX2023);
	DUMPREG( LVMX2427);
	DUMPREG( LVCFG);
	DUMPREG( LVPHY0);
	DUMPREG( LVPHY1);

	printk(KERN_ALERT "System Registers\n");
	DUMPREG( SYSSTAT);
	DUMPREG( SYSRST);

	printk(KERN_ALERT "GPIO Registers\n");
	DUMPREG( GPIOC);
	DUMPREG( GPIOO);
	DUMPREG( GPIOI);

	printk(KERN_ALERT "Chip Revision Registers\n");
	DUMPREG( IDREG);

	printk(KERN_ALERT "Debug Registers\n");
	DUMPREG( DEBUG00);
	DUMPREG( DEBUG01);
#undef DUMPREG
}
EXPORT_SYMBOL(d2l_dump_regs);

/***********************
*** DUMMY FUNCTIONS ****
***********************/

static int d2l_rotate(struct omap_dss_device *dssdev, u8 rotate)
{
        return 0;
}

static u8 d2l_get_rotate(struct omap_dss_device *dssdev)
{
	return 0;
}

static int d2l_mirror(struct omap_dss_device *dssdev, bool enable)
{
        return 0;
}

static bool d2l_get_mirror(struct omap_dss_device *dssdev)
{
	return 0;

}

static int d2l_dcs_read1(enum omap_dsi_index lcd_ix, u8 dcs_cmd, u8 *data)
{
        int ret;
        u8 buf[1];

        ret = dsi_vc_dcs_read(lcd_ix, TCH, dcs_cmd, buf, 1);
        if (ret < 0)
                return ret;
        *data = buf[0];
        return 0;
}

static int d2l_dcs_write1(enum omap_dsi_index lcd_ix, u8 dcs_cmd, u8 param)
{
        u8 buf[2];
        buf[0] = dcs_cmd;
        buf[1] = param;
        return dsi_vc_dcs_write(lcd_ix, TCH, buf, 2);
}

static int d2l_get_id(enum omap_dsi_index lcd_ix,
        u8 *id1, u8 *id2, u8 *id3)
{
        int ret;
	PRINT_DEBUG("%s:%s\n",__FILE__,__FUNCTION__);
        ret = d2l_dcs_read1(lcd_ix, DCS_GET_ID1, id1);
        if (ret) return ret;
        ret = d2l_dcs_read1(lcd_ix, DCS_GET_ID2, id2);
        if (ret) return ret;
        ret = d2l_dcs_read1(lcd_ix, DCS_GET_ID3, id3);
        if (ret) return ret;
        return 0;
}

static void d2l_get_timings(struct omap_dss_device *dssdev,
                        struct omap_video_timings *timings)
{
	PRINT_DEBUG("%s:%s\n",__FILE__,__FUNCTION__);
        *timings = dssdev->panel.timings;
}

static void d2l_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	dssdev->panel.timings.x_res = timings->x_res;
	dssdev->panel.timings.y_res = timings->y_res;
	dssdev->panel.timings.pixel_clock = timings->pixel_clock;
	dssdev->panel.timings.hsw = timings->hsw;
	dssdev->panel.timings.hfp = timings->hfp;
	dssdev->panel.timings.hbp = timings->hbp;
	dssdev->panel.timings.vsw = timings->vsw;
	dssdev->panel.timings.vfp = timings->vfp;
	dssdev->panel.timings.vbp = timings->vbp;
}

static int d2l_check_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	if (timings->x_res != 1024 || timings->y_res != 768)
		return -EINVAL;

	return 0;
}

static void d2l_get_resolution(struct omap_dss_device *dssdev,
                u16 *xres, u16 *yres)
{
        struct d2l_data *d2d = dev_get_drvdata(&dssdev->dev);

        if (d2d->rotate == 0 || d2d->rotate == 2) {
                *xres = dssdev->panel.timings.x_res;
                *yres = dssdev->panel.timings.y_res;
        } else {
                *yres = dssdev->panel.timings.x_res;
                *xres = dssdev->panel.timings.y_res;
        }
	PRINT_DEBUG("%s:%s xres %d yres %d\n",__FILE__,__FUNCTION__,*xres,*yres);
}

static ssize_t d2l_num_errors_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        struct omap_dss_device *dssdev = to_dss_device(dev);
        struct d2l_data *d2d = dev_get_drvdata(&dssdev->dev);
        u8 errors;
        int ret;
        enum omap_dsi_index lcd_ix;

        lcd_ix = DSI1;

	PRINT_DEBUG("%s:%s\n",__FILE__,__FUNCTION__);

	mutex_lock(&d2d->lock);

        if (d2d->enabled) {
                dsi_bus_lock(lcd_ix);
                ret = d2l_dcs_read1(lcd_ix, DCS_READ_NUM_ERRORS, &errors);
                dsi_bus_unlock(lcd_ix);
        } else {
                ret = -ENODEV;
        }

	mutex_lock(&d2d->lock);

        if (ret)
                return ret;

        return snprintf(buf, PAGE_SIZE, "%d\n", errors);
}

static ssize_t d2l_hw_revision_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        struct omap_dss_device *dssdev = to_dss_device(dev);
        struct d2l_data *d2d = dev_get_drvdata(&dssdev->dev);
        u8 id1, id2, id3;
        int r;
        enum omap_dsi_index lcd_ix;

        lcd_ix =  DSI1;
	PRINT_DEBUG("%s:%s\n",__FILE__,__FUNCTION__);

	mutex_lock(&d2d->lock);

        if (d2d->enabled) {
                dsi_bus_lock(lcd_ix);
                r = d2l_get_id(lcd_ix, &id1, &id2, &id3);
                dsi_bus_unlock(lcd_ix);
        } else {
                r = -ENODEV;
        }

	mutex_lock(&d2d->lock);

        if (r) return r;

        return snprintf(buf, PAGE_SIZE, "%02x.%02x.%02x\n", id1, id2, id3);
}

static const char *cabc_modes[] = {
        "off",          /* used also always when CABC is not supported */
        "ui",
        "still-image",
        "moving-image",
};

static ssize_t show_cabc_mode(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
        struct omap_dss_device *dssdev = to_dss_device(dev);
        struct d2l_data *d2d = dev_get_drvdata(&dssdev->dev);
        const char *mode_str;
        int mode;
        int len;

        mode = d2d->cabc_mode;

	PRINT_DEBUG("%s:%s\n",__FILE__,__FUNCTION__);
        mode_str = "unknown";
        if (mode >= 0 && mode < ARRAY_SIZE(cabc_modes))
                mode_str = cabc_modes[mode];
        len = snprintf(buf, PAGE_SIZE, "%s\n", mode_str);

        return len < PAGE_SIZE - 1 ? len : PAGE_SIZE - 1;
}

static ssize_t store_cabc_mode(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
        struct omap_dss_device *dssdev = to_dss_device(dev);
        struct d2l_data *d2d = dev_get_drvdata(&dssdev->dev);
        int i;
        enum omap_dsi_index lcd_ix;

        lcd_ix = DSI1;
	PRINT_DEBUG("%s:%s\n",__FILE__,__FUNCTION__);
        for (i = 0; i < ARRAY_SIZE(cabc_modes); i++) {
                if (sysfs_streq(cabc_modes[i], buf))
                        break;
        }
        if (i == ARRAY_SIZE(cabc_modes))
                return -EINVAL;

	mutex_lock(&d2d->lock);

        if (d2d->enabled) {
                dsi_bus_lock(lcd_ix);
                if (!d2d->cabc_broken)
                        d2l_dcs_write1(lcd_ix, DCS_WRITE_CABC, i);
                dsi_bus_unlock(lcd_ix);
        }

        d2d->cabc_mode = i;

	mutex_unlock(&d2d->lock);

        return count;
}

static ssize_t show_cabc_available_modes(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
        int len;
        int i;

	PRINT_DEBUG("%s:%s\n",__FILE__,__FUNCTION__);
        for (i = 0, len = 0;
             len < PAGE_SIZE && i < ARRAY_SIZE(cabc_modes); i++)
                len += snprintf(&buf[len], PAGE_SIZE - len, "%s%s%s",
                        i ? " " : "", cabc_modes[i],
                        i == ARRAY_SIZE(cabc_modes) - 1 ? "\n" : "");

        return len < PAGE_SIZE ? len : PAGE_SIZE - 1;
}

static DEVICE_ATTR(num_dsi_errors, S_IRUGO, d2l_num_errors_show, NULL);
static DEVICE_ATTR(hw_revision, S_IRUGO, d2l_hw_revision_show, NULL);
static DEVICE_ATTR(cabc_mode, S_IRUGO | S_IWUSR,
                show_cabc_mode, store_cabc_mode);
static DEVICE_ATTR(cabc_available_modes, S_IRUGO,
                show_cabc_available_modes, NULL);

static struct attribute *d2l_attrs[] = {
	&dev_attr_num_dsi_errors.attr,
        &dev_attr_hw_revision.attr,
        &dev_attr_cabc_mode.attr,
        &dev_attr_cabc_available_modes.attr,
        NULL,
};

static struct attribute_group d2l_attr_group = {
        .attrs = d2l_attrs,
};


static int d2l_enable_te(struct omap_dss_device *dssdev, bool enable)
{
        return 0;
}

/***************************************
******* WORKING FUNCTIONS **************
***************************************/
static int d2l_hw_reset(struct omap_dss_device *dssdev)
{
	struct d2l_data *d2d = dev_get_drvdata(&dssdev->dev);
	struct toshiba_dsi_panel_data *panel_data = get_panel_data(dssdev);

	if (panel_data->reset_gpio == -1)
		return -1;

	gpio_set_value(panel_data->reset_gpio, 1);
	if (d2d->panel_config->reset_sequence.high)
		udelay(d2d->panel_config->reset_sequence.high);
	/* reset the panel */
	gpio_set_value(panel_data->reset_gpio, 0);
	/* assert reset */
	if (d2d->panel_config->reset_sequence.low)
		udelay(d2d->panel_config->reset_sequence.low);
	gpio_set_value(panel_data->reset_gpio, 1);
	/* wait after releasing reset */
	if (d2d->panel_config->sleep.hw_reset)
		msleep(d2d->panel_config->sleep.hw_reset);

	return 0;
}

static int d2l_probe(struct omap_dss_device *dssdev)
{
	struct d2l_data *d2d;
	int ret = 0;
	int i;

	struct toshiba_dsi_panel_data *panel_data = get_panel_data(dssdev);
	struct panel_config *panel_config = NULL;


	dev_dbg(&dssdev->dev, "d2l_probe\n");

	if (!panel_data || !panel_data->name) {
		ret = -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(panel_configs); i++) {
		if (strcmp(panel_data->name, panel_configs[i].name) == 0) {
			panel_config = &panel_configs[i];
			break;
		}
	}

	if (!panel_config) {
		ret = -EINVAL;
		//goto err;
	}

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = panel_config->timings;
        dssdev->ctrl.pixel_size = 24;
	dssdev->panel.acbi = 0;
        dssdev->panel.acb = 40;

	d2d = kzalloc(sizeof(*d2d), GFP_KERNEL);
        if (!d2d) {
                ret = -ENOMEM;
                return ret;
        }

        d2d->dssdev = dssdev;

	d2d->panel_config = panel_config;

	mutex_init(&d2d->lock);

	atomic_set(&d2d->do_update, 0);

	ret = init_regulators(dssdev, panel_config->regulators,
			panel_config->num_regulators);
	if (ret)
		goto err_reg;

	dev_set_drvdata(&dssdev->dev, d2d);

	if (cpu_is_omap44xx())
		d2d->force_update = true;

	ret = sysfs_create_group(&dssdev->dev.kobj, &d2l_attr_group);
	if (ret)
		dev_err(&dssdev->dev, "failed to create sysfs files\n");

	// do I need an err and kfree(d2d)
	return ret;

err_reg:
	kfree(d2d);

	return 0;
}

static void d2l_remove(struct omap_dss_device *dssdev)
{
	struct d2l_data *d2d = dev_get_drvdata(&dssdev->dev);
	kfree(d2d);
}

static int d2l_power_on(struct omap_dss_device *dssdev)
{
        struct d2l_data *d2d = dev_get_drvdata(&dssdev->dev);
        int ret=0;

	if(d2d->enabled != 1) {

		//set Stop state (LP-11)
		dsi_set_stop_mode(1);

		msleep(5);

		ret = omapdss_dsi_display_enable(dssdev);
		if (ret) {
			dev_err(&dssdev->dev, "failed to enable DSI\n");
			//goto err0;
		}

		/* reset d2l bridge */
		d2l_hw_reset(dssdev);

		msleep(10);

		//do extra job to match kozio registers
		dsi_videomode_panel_preinit();

		/* Need to wait a certain time - Toshiba Bridge Constraint */
		msleep(1000);

#ifdef DEB
		d2l_dump_regs();
#endif

		//configure D2L chip DSI-RX configuration registers
		d2l_write_register(PPI_TX_RX_TA, 0x00000004); //this register setting is required only if host wishes to perform DSI read transactions.
		d2l_write_register(PPI_LPTXTIMECNT, 0x00000004); //SYSLPTX Timing Generation Counter
		d2l_write_register(PPI_D0S_CLRSIPOCOUNT, 0x00000003); //D*S_CLRSIPOCOUNT = [(THS-SETTLE + THS-ZERO) / HS_byte_clock_period ]
		d2l_write_register(PPI_D1S_CLRSIPOCOUNT, 0x00000003);
		d2l_write_register(PPI_D2S_CLRSIPOCOUNT, 0x00000003);
		d2l_write_register(PPI_D3S_CLRSIPOCOUNT, 0x00000003);
		d2l_write_register(DSI_LANEENABLE, 0x0000001F); //SpeedLaneSel == HS4L
		d2l_write_register(PPI_LANEENABLE, 0x0000001F); //SpeedLaneSel == HS4L
		d2l_write_register(PPI_STARTPPI, 0x00000001); //Changed to 1
		d2l_write_register(DSI_STARTDSI, 0x00000001); //Changed to 1

		//configure D2L on-chip PLL.
		d2l_write_register(LVPHY1, 0x00000000);
		d2l_write_register(LVPHY0, 0x00044006); //set frequency range allowed and clock/data lanes.

		//configure D2L chip LCD Controller configuration registers
		d2l_write_register(VPCTRL, 0x00F00110);
		d2l_write_register(HTIM1, 0x00200006);
		d2l_write_register(HTIM2, 0x011A0400);
		d2l_write_register(VTIM1, 0x000F0008);
		d2l_write_register(VTIM2, 0x000F0300);
		d2l_write_register(LVCFG, 0x00000001);

		//Issue a soft reset to LCD Controller for a clean start
		d2l_write_register(SYSRST, 0x00000004);
		d2l_write_register(VFUEN, 0x00000001);

#ifdef DEB
		d2l_dump_regs();
#endif

		dsi_videomode_panel_postinit();

		d2d->enabled = 1;
	}

        return ret;
}

static void d2l_power_off(struct omap_dss_device *dssdev)
{
	msleep(10);

	omapdss_dsi_display_disable(dssdev);
}

static int d2l_start(struct omap_dss_device *dssdev)
{
	struct d2l_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;
	enum omap_dsi_index lcd_ix;

	lcd_ix = DSI1;

	mutex_lock(&d2d->lock);

	if (d2d->force_update) {
		//d2d->te_enabled = 1;
		//d2l_prepare_workqueue(dssdev);
	}

	dsi_bus_lock(lcd_ix);

	r = d2l_power_on(dssdev);

	dsi_bus_unlock(lcd_ix);

	if (r) {
		dev_dbg(&dssdev->dev, "enable failed\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		//if (panel_data->use_esd_check)
		//	queue_delayed_work(d2d->esd_wq, &d2d->esd_work,
		//			D2L_ESD_CHECK_PERIOD);
	}

	mutex_unlock(&d2d->lock);
	return r;
}

static void d2l_stop(struct omap_dss_device *dssdev)
{
	struct d2l_data *d2d = dev_get_drvdata(&dssdev->dev);
	enum omap_dsi_index lcd_ix;

	lcd_ix = DSI1;

	if (d2d->force_update) {
		dsi_bus_lock(lcd_ix);
		//_d2l_enable_te(dssdev, 0);
		dsi_bus_unlock(lcd_ix);
		//destroy_workqueue(td->te_wq);
	}

	mutex_lock(&d2d->lock);

	//if (d2d->force_update)
		//d2d->te_enabled = 0;

	//cancel_delayed_work(&d2d->esd_work);

	dsi_bus_lock(lcd_ix);

	d2l_power_off(dssdev);

	dsi_bus_unlock(lcd_ix);

	mutex_unlock(&d2d->lock);
}

static void d2l_disable(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE ||
	    dssdev->state == OMAP_DSS_DISPLAY_TRANSITION)
		d2l_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int d2l_enable(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	return d2l_start(dssdev);
}

static void d2l_framedone_cb(int err, void *data)
{
	struct omap_dss_device *dssdev = data;
        enum omap_dsi_index lcd_ix;

        lcd_ix = DSI1;

	printk(KERN_ALERT "d2l_framedone_cb\n");

	dev_dbg(&dssdev->dev, "framedone, err %d\n", err);
	dsi_bus_unlock(lcd_ix);

	d2l_update(dssdev, 0, 0, 1024, 768);
}

static int d2l_update(struct omap_dss_device *dssdev,
				    u16 x, u16 y, u16 w, u16 h)
{
	struct d2l_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r;
	enum omap_dsi_index lcd_ix;

	lcd_ix = DSI1;

	printk(KERN_ALERT "d2l_update\n");

	dev_dbg(&dssdev->dev, "update %d, %d, %d x %d\n", x, y, w, h);

	printk(KERN_ALERT "d2l_update... before mutex_lock\n");
	mutex_lock(&d2d->lock);
	printk(KERN_ALERT "d2l_update... before dsi_bus_lock\n");
	dsi_bus_lock(lcd_ix);
	printk(KERN_ALERT "d2l_update... after locks\n");

	if (!d2d->enabled) {
		r = 0;
		goto err;
	}

	r = omap_dsi_prepare_update(dssdev, &x, &y, &w, &h, true);
	if (r)
		goto err;

	//r = d2l_set_update_window(lcd_ix, x, y, w, h);
	//if (r)
	//	goto err;

	//if (d2d->te_enabled && panel_data->use_ext_te) {
	//	d2d->update_region.x = x;
	//	d2d->update_region.y = y;
	//	d2d->update_region.w = w;
	//	d2d->update_region.h = h;
	//	barrier();
	//	schedule_delayed_work(&td->te_timeout_work,
	//			msecs_to_jiffies(250));
	//	atomic_set(&td->do_update, 1);
	//} else {
		/* We use VC(0) for VideoPort Data and VC(1) for commands */
		if (cpu_is_omap44xx())
			r = omap_dsi_update(dssdev, 0, x, y, w, h,
				d2l_framedone_cb, dssdev);
		else
			r = omap_dsi_update(dssdev, TCH, x, y, w, h,
				d2l_framedone_cb, dssdev);
		if (r)
			goto err;
	//}

	dsi_bus_unlock(lcd_ix);
	/* note: no bus_unlock here. unlock is in framedone_cb */
	mutex_unlock(&d2d->lock);
	return 0;
err:
	dsi_bus_unlock(lcd_ix);
	mutex_unlock(&d2d->lock);
	return r;
}

static int d2l_sync(struct omap_dss_device *dssdev)
{
	struct d2l_data *d2d = dev_get_drvdata(&dssdev->dev);
	enum omap_dsi_index lcd_ix;

	lcd_ix = DSI1;

	dev_dbg(&dssdev->dev, "sync\n");

	mutex_lock(&d2d->lock);
	dsi_bus_lock(lcd_ix);
	dsi_bus_unlock(lcd_ix);
	mutex_unlock(&d2d->lock);

	dev_dbg(&dssdev->dev, "sync done\n");

	return 0;
}

static int d2l_set_update_mode(struct omap_dss_device *dssdev,
		enum omap_dss_update_mode mode)
{

	struct d2l_data *d2d = dev_get_drvdata(&dssdev->dev);

	if (d2d->force_update) {
		if (mode != OMAP_DSS_UPDATE_AUTO)
			return -EINVAL;
	} else {
		if (mode != OMAP_DSS_UPDATE_MANUAL)
			return -EINVAL;
	}
	return 0;
}

static enum omap_dss_update_mode d2l_get_update_mode(
		struct omap_dss_device *dssdev)
{
	struct d2l_data *d2d = dev_get_drvdata(&dssdev->dev);

	if (d2d->force_update)
		return OMAP_DSS_UPDATE_AUTO;
	else
		return OMAP_DSS_UPDATE_MANUAL;
}

#ifdef CONFIG_PM
static int d2l_resume(struct omap_dss_device *dssdev)
{
	struct d2l_data *td = dev_get_drvdata(&dssdev->dev);
	int r = 0;
/*
	dev_dbg(&dssdev->dev, "resume\n");

	mutex_lock(&td->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED) {
		r = -EINVAL;
		goto err;
	}

	r = d2l_start(dssdev);
err:
	mutex_unlock(&td->lock);
*/
	return r;
}

static int d2l_suspend(struct omap_dss_device *dssdev)
{
	struct d2l_data *td = dev_get_drvdata(&dssdev->dev);
	int r = 0;
/*
	dev_dbg(&dssdev->dev, "suspend\n");

	mutex_lock(&td->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		r = -EINVAL;
		goto err;
	}

	d2l_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
err:
	mutex_unlock(&td->lock);
*/
	return r;
}
#endif

static struct omap_dss_driver d2l_driver = {
	.probe		= d2l_probe,
	.remove		= d2l_remove,

	.enable		= d2l_enable,
	.disable	= d2l_disable,
#ifdef CONFIG_PM
	.suspend		= d2l_suspend,
	.resume			= d2l_resume,
#endif
//        .setup_update   = d2l_setup_update, //dummy

	.set_update_mode = d2l_set_update_mode,
	.get_update_mode = d2l_get_update_mode,

	.update		= d2l_update,
	.sync		= d2l_sync,

	.get_resolution	= d2l_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

        .enable_te      = d2l_enable_te,   //dummy
//        .wait_for_te    = d2l_wait_te,     //dummy

        .set_rotate     = d2l_rotate,     //dummy
        .get_rotate     = d2l_get_rotate, //dummy

        .set_mirror     = d2l_mirror,	//dummy
        .get_mirror     = d2l_get_mirror, //dummy

	.get_timings	= d2l_get_timings,
	.set_timings	= d2l_set_timings,
	.check_timings	= d2l_check_timings,

	.driver         = {
		.name   = "d2l",
		.owner  = THIS_MODULE,
	},
};

/* driver probe function (this is called when a device name on the device id table matches one of those set on board file for the given i2c bus) */
static int __devinit d2l_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	sd1 = kzalloc(sizeof(struct d2l_i2c), GFP_KERNEL);
	if (sd1 == NULL)
		return -ENOMEM;

	/* store i2c_client pointer on private data structure */
	sd1->client = client;

	/* store private data structure pointer on i2c_client structure */
	i2c_set_clientdata(client, sd1);

	/* init mutex */
	mutex_init(&sd1->xfer_lock);

	return 0;
}

/* driver remove function */
static int __devexit d2l_i2c_remove(struct i2c_client *client)
{
	struct d2l_i2c *sd1 = i2c_get_clientdata(client);

	/* remove client data */
	i2c_set_clientdata(client, NULL);

	/* free private data memory */
	kfree(sd1);

	return 0;
}

static int __init d2l_init(void)
{
	omap_dss_register_driver(&d2l_driver);;
	i2c_add_driver(&d2l_i2c_driver);
        return 0;
}

static void __exit d2l_exit(void)
{
	omap_dss_unregister_driver(&d2l_driver);
	i2c_del_driver(&d2l_i2c_driver);
}

module_init(d2l_init);
module_exit(d2l_exit);

MODULE_AUTHOR("Jerry Alexander <x0135174@ti.com>");
MODULE_DESCRIPTION("D2l Driver");
MODULE_LICENSE("GPL");
