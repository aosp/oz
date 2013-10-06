/*
 * Copyright (C) 2013 Texas Instruments Inc
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

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/i2c/lvds-serlink.h>
#include <linux/i2c/lvds-de-serlink.h>

#include <video/omapdss.h>
#include <video/omap-panel-lg101.h>

static const struct omap_video_timings lg101_default_timings = {
	/* 1280 x 800 @ 60 Hz Reduced blanking VESA CVT 0.31M3-R */
	.x_res		= 1280,
	.y_res		= 800,

	.pixel_clock	= 67333,

	.hfp		= 32,
	.hsw		= 48,
	.hbp		= 80,

	.vfp		= 4,
	.vsw		= 3,
	.vbp		= 7,

	.vsync_level	= OMAPDSS_SIG_ACTIVE_LOW,
	.hsync_level	= OMAPDSS_SIG_ACTIVE_LOW,
	.data_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
	.de_level	= OMAPDSS_SIG_ACTIVE_HIGH,
	.sync_pclk_edge = OMAPDSS_DRIVE_SIG_OPPOSITE_EDGES,
};

struct panel_drv_data {
	struct omap_dss_device dssdev;
	struct omap_dss_device *in;

	int data_lines;
	struct omap_video_timings videomode;

	int p_gpio;
	int dith;
	struct i2c_client *ser_i2c_client;
	struct i2c_client *deser_i2c_client;
	struct mutex lock;
};

#define to_panel_data(p) container_of(p, struct panel_drv_data, dssdev)

static int send_i2c_cmd(struct i2c_client *client, int cmd, void *arg)
{
	int status = 0;
	if (client && client->driver && client->driver->command)
		status = client->driver->command(client, cmd, arg);

	return status;
}
static int lg101_power_on(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;
	u8 data;

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	dev_dbg(dssdev->dev, "lg101_power_on");

	gpio_set_value_cansleep(ddata->p_gpio, 0); /* S0-0 S1-1 S2-0 A1 -> B2*/

	in->ops.dpi->set_data_lines(in, ddata->data_lines);
	in->ops.dpi->set_timings(in, &ddata->videomode);

	r = in->ops.dpi->enable(in);
	if (r)
		goto err0;

	r = send_i2c_cmd(ddata->ser_i2c_client, SER_RESET, NULL);
	if (r < 0) {
		dev_err(dssdev->dev, "failed to reset serializer ...");
		goto err0;
	}
	dev_dbg(dssdev->dev, "Serializer Reset done...");

	r = send_i2c_cmd(ddata->ser_i2c_client, SER_CONFIGURE, NULL);
	if (r < 0) {
		dev_err(dssdev->dev, "configure serializer failed ...");
		goto err0;
	}
	dev_dbg(dssdev->dev, "Serializer configuration done...");

	r = send_i2c_cmd(ddata->ser_i2c_client, SER_VALIDATE_PCLK, NULL);
	if (r < 0) {
		dev_err(dssdev->dev, "PCLK not present ...");
		goto err0;
	}
	dev_dbg(dssdev->dev, "PCLK detection done...");

	r = send_i2c_cmd(ddata->ser_i2c_client, SER_GET_I2C_ADDR, NULL);
	if ((r < 0) || (r != ddata->ser_i2c_client->addr)) {
		dev_err(dssdev->dev, "couldn't read i2c serializer address ...");
		goto err0;
	}
	dev_dbg(dssdev->dev, "Serializer i2c addr %x ...", r);

	r = send_i2c_cmd(ddata->ser_i2c_client, SER_VALIDATE_LINK, NULL);
	if (r != 1) {
		dev_err(dssdev->dev, "link not preset ...");
		goto err0;
	}
	dev_dbg(dssdev->dev, "Link detected ...");

	r = send_i2c_cmd(ddata->ser_i2c_client, SER_GET_DES_I2C_ADDR,
								(void *)&data);
	if ((r < 0) || (data != ddata->deser_i2c_client->addr)) {
		dev_err(dssdev->dev, "couldn't read i2c deserializer address ...");
		goto err0;
	}
	dev_dbg(dssdev->dev, "Deserilizer i2c addr %x", data);

	r = send_i2c_cmd(ddata->ser_i2c_client, SER_REG_DUMP,
							NULL);
	if (r < 0) {
		dev_err(dssdev->dev, "couldn't read serializer sts ...");
		goto err0;
	}
	dev_dbg(dssdev->dev, "Serializer status dump");

	/* --------- Deserializer -------------- */
	r = send_i2c_cmd(ddata->deser_i2c_client, DSER_RESET, NULL);
	if (r < 0) {
		dev_err(dssdev->dev, "failed to reset de serializer ...");
		goto err0;
	}
	dev_dbg(dssdev->dev, "DeSerializer Reset done...");

	r = send_i2c_cmd(ddata->deser_i2c_client, DSER_EN_BC, NULL);
	if (r < 0) {
		dev_err(dssdev->dev, "failed to enable back channel ...");
		goto err0;
	}
	dev_dbg(dssdev->dev, "DeSerializer back channel enabled ...");

	r = send_i2c_cmd(ddata->deser_i2c_client, DSER_GET_I2C_ADDR, NULL);
	if ((r < 0) || (r != ddata->deser_i2c_client->addr)) {
		dev_err(dssdev->dev, "couldn't read i2c de serializer address ...");
		goto err0;
	}
	dev_dbg(dssdev->dev, "Deserializer i2c addr %x ...", r);

	r = send_i2c_cmd(ddata->deser_i2c_client, DSER_GET_SER_I2C_ADDR,
								(void *)&data);
	if ((r < 0) || (data != ddata->ser_i2c_client->addr)) {
		dev_err(dssdev->dev, "failed to get serializer id ...");
		goto err0;
	}
	dev_dbg(dssdev->dev, "Serilizer i2c addr %x\n", data);

	r = send_i2c_cmd(ddata->deser_i2c_client, DSER_CONFIGURE,
								NULL);
	if (r < 0) {
		dev_err(dssdev->dev, "failed to configure deserializer...");
		goto err0;
	}
	dev_dbg(dssdev->dev, "Deserilizer configuration done ...\n");

	r = send_i2c_cmd(ddata->deser_i2c_client, DSER_REG_DUMP,
							NULL);
	if (r < 0) {
		dev_err(dssdev->dev, "couldn't read deserializer sts ...");
		goto err0;
	}
	dev_dbg(dssdev->dev, "Deserializer status dump");

	return 0;
err0:
	return r;
}

static void lg101_power_off(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;
	dev_dbg(dssdev->dev, "lg101_power_off");

	gpio_set_value_cansleep(ddata->p_gpio, 1); /* S0-1 S1-0 S2-0 A1 -> B1*/

	in->ops.dpi->disable(in);
}

static int lg101_connect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	if (omapdss_device_is_connected(dssdev))
		return 0;

	r = in->ops.dpi->connect(in, dssdev);
	if (r)
		return r;

	return 0;
}

static void lg101_disconnect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (!omapdss_device_is_connected(dssdev))
		return;

	in->ops.dpi->disconnect(in, dssdev);
}

static int lg101_enable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	int r;

	mutex_lock(&ddata->lock);

	r = lg101_power_on(dssdev);
	if (r == 0)
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	mutex_unlock(&ddata->lock);

	return r;
}

static void lg101_disable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	mutex_lock(&ddata->lock);

	lg101_power_off(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	mutex_unlock(&ddata->lock);
}

static void lg101_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	mutex_lock(&ddata->lock);
	dssdev->panel.timings = *timings;
	in->ops.dpi->set_timings(in, timings);
	mutex_unlock(&ddata->lock);
}

static void lg101_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	mutex_lock(&ddata->lock);
	*timings = dssdev->panel.timings;
	mutex_unlock(&ddata->lock);
}

static int lg101_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	mutex_lock(&ddata->lock);
	r = in->ops.dpi->check_timings(in, timings);
	mutex_unlock(&ddata->lock);

	return r;
}

static struct omap_dss_driver panel_dpi_ops = {
	.connect	= lg101_connect,
	.disconnect	= lg101_disconnect,

	.enable		= lg101_enable,
	.disable	= lg101_disable,

	.set_timings	= lg101_set_timings,
	.get_timings	= lg101_get_timings,
	.check_timings	= lg101_check_timings,

	.get_resolution	= omapdss_default_get_resolution,
};

static int lg101_probe_of(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct device_node *node = pdev->dev.of_node;
	struct omap_dss_device *in;
	struct device_node *src_node;
	int r, datalines, gpio, gpio_count;

	src_node = of_parse_phandle(node, "video-source", 0);
	if (!src_node) {
		dev_err(&pdev->dev, "failed to parse video source\n");
		return -ENODEV;
	}

	in = omap_dss_find_output_by_node(src_node);
	if (in == NULL) {
		dev_err(&pdev->dev, "failed to find video source\n");
		return -EPROBE_DEFER;
	}

	ddata->in = in;

	gpio_count = of_gpio_count(node);
	/* DRA-7 board has 1 selection gpio */
	if (gpio_count != 1) {
		dev_err(&pdev->dev, "wrong number of GPIOs %d\n", gpio_count);
		return -EINVAL;
	}
	dev_dbg(&pdev->dev, "gpio count %d\n", gpio_count);

	/* Get the GPIO for GPMC vs VID3 selection */
	gpio = of_get_gpio(node, 0);
	if (gpio_is_valid(gpio)) {
		ddata->p_gpio = gpio;
	} else {
		dev_err(&pdev->dev, "fail to parse vid3 gpio %d\n", gpio);
		return -EPROBE_DEFER;
	}
	dev_dbg(&pdev->dev, "gpio number %d\n", gpio);

	r = of_property_read_u32(node, "data-lines", &datalines);
	if (r) {
		dev_err(&pdev->dev, "failed to parse datalines");
		return r;
	}
	ddata->data_lines = datalines;

	/* Get I2c node of serializer */
	node = of_parse_phandle(pdev->dev.of_node, "serializer", 0);
	if (node)
		ddata->ser_i2c_client = of_find_i2c_device_by_node(node);
	else {
		dev_err(&pdev->dev, "failed to get serializer node");
		return -EPROBE_DEFER;
	}
	dev_err(&pdev->dev, "Serial I2C ID %x", ddata->ser_i2c_client->addr);

	/* Get I2c node of dserializer */
	node = of_parse_phandle(pdev->dev.of_node, "deserializer", 0);
	if (node)
		ddata->deser_i2c_client = of_find_i2c_device_by_node(node);
	else {
		dev_err(&pdev->dev, "failed to get deserializer node");
		return -EPROBE_DEFER;
	}
	dev_err(&pdev->dev, "DeSerial I2C ID %x",
					ddata->deser_i2c_client->addr);

	return 0;
}

static int lg101_probe(struct platform_device *pdev)
{
	struct omap_dss_device *dssdev;
	struct panel_drv_data *ddata;
	int r;

	dev_err(&pdev->dev, "lg101_probe probe\n");

	ddata = devm_kzalloc(&pdev->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	platform_set_drvdata(pdev, ddata);

	r = lg101_probe_of(pdev);
	if (r)
		return r;

	mutex_init(&ddata->lock);

	ddata->videomode = lg101_default_timings;

	if (ddata->data_lines == 24)
		ddata->dith = 0;
	else if (ddata->data_lines == 18)
		ddata->dith = 1;
	else
		return -EINVAL;

	r = gpio_request_one(ddata->p_gpio, GPIOF_OUT_INIT_LOW, "vid3_sel");
	if (r)
		return r;

	dssdev = &ddata->dssdev;
	dssdev->dev = &pdev->dev;
	dssdev->driver = &panel_dpi_ops;
	dssdev->type = OMAP_DISPLAY_TYPE_DPI;
	dssdev->owner = THIS_MODULE;
	dssdev->panel.timings = ddata->videomode;
	dssdev->phy.dpi.data_lines = ddata->data_lines;

	r = omapdss_register_display(dssdev);
	if (r) {
		dev_err(&pdev->dev, "Failed to register panel\n");
		goto err_reg;
	}

	dev_info(&pdev->dev, "lg101_probe probe sucessful..\n");

	return 0;
err_reg:
	omap_dss_put_device(ddata->in);
	return r;
}

static int __exit lg101_remove(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	struct omap_dss_device *in = ddata->in;

	mutex_lock(&ddata->lock);

	gpio_free(ddata->p_gpio);

	omapdss_unregister_display(dssdev);

	lg101_disable(dssdev);

	omap_dss_put_device(in);

	mutex_unlock(&ddata->lock);

	kfree(ddata);

	return 0;
}
static const struct of_device_id lg101_of_match[] = {
	{
		.compatible = "ti,lg101",
	},
	{},
};

MODULE_DEVICE_TABLE(of, lg101_of_match);

static struct platform_driver lg101_driver = {
	.probe		= lg101_probe,
	.remove		= __exit_p(lg101_remove),

	.driver         = {
		.name   = "lg101",
		.owner  = THIS_MODULE,
		.of_match_table = lg101_of_match,
	},
};

module_platform_driver(lg101_driver);
MODULE_LICENSE("GPL");
