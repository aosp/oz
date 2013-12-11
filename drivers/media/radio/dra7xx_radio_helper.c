/*
 * TI DRA7xx Radio Helper Sub-driver
 *
 * Copyright (C) 2013-2014 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Ravikumar Kattekola <rk@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
#include <linux/of.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>

static int dra7xx_radio_subdev_reparent_set_clock(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct clk *mclk, *parent_clk;
	const char *parent_name;
	unsigned long requested_rate, set_rate;
	unsigned int clk_freq;
	int ret;

	parent_name = of_get_property(node, "fck_parent", NULL);
	if (!parent_name)
		return 0;

	mclk = clk_get(&pdev->dev, "fck");
	if (IS_ERR(mclk)) {
		dev_err(&pdev->dev, "failed to get fck\n");
		return PTR_ERR(mclk);
	}

	parent_clk = clk_get(NULL, parent_name);
	if (IS_ERR(parent_clk)) {
		dev_err(&pdev->dev, "failed to get new parent clock parent\n");
		ret = PTR_ERR(parent_clk);
		goto err1;
	}

	ret = clk_set_parent(mclk, parent_clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to reparent fck\n");
		goto err2;
	}

	ret = of_property_read_u32(node, "clock-frequency", &clk_freq);
	if (ret) {
		dev_warn(&pdev->dev, "no clock-frequency specified\n");
		goto err2;
	}

	requested_rate = (unsigned long) clk_freq;
	ret = clk_set_rate(parent_clk, requested_rate);
	if (ret) {
		dev_err(&pdev->dev, "failed to set fck rate\n");
		goto err2;
	}

	set_rate = clk_get_rate(parent_clk);
	if (set_rate != requested_rate) {
		dev_warn(&pdev->dev,
			 "%s: Could not get requested rate %lu using %lu.\n",
				 __func__, requested_rate, set_rate);
	}

err2:
	clk_put(parent_clk);
err1:
	clk_put(mclk);
	return ret;
}

static int dra7xx_radio_subdev_probe(struct platform_device *pdev)
{
	int ret;

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	ret = dra7xx_radio_subdev_reparent_set_clock(pdev);
	if (ret)
		dev_err(&pdev->dev, "failed to reparent and set clock\n");

	pr_info("%s: Probe done\n", pdev->name);
	return 0;
}

static int dra7xx_radio_subdev_remove(struct platform_device *pdev)
{
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	return 0;
}

static const struct of_device_id dra7xx_radio_subdev_of_match[] = {
	{ .compatible = "ti,dra7xx_radio_subdev", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, dra7xx_radio_subdev_of_match);

static struct platform_driver dra7xx_radio_subdev_driver = {
	.driver	= {
		.of_match_table = of_match_ptr(dra7xx_radio_subdev_of_match),
		.name	= "dra7xx_radio_subdev",
		.owner	= THIS_MODULE,
	},
	.probe = dra7xx_radio_subdev_probe,
	.remove = dra7xx_radio_subdev_remove,
};

module_platform_driver(dra7xx_radio_subdev_driver);

MODULE_DESCRIPTION("DRA7xx Radio Helper Sub-driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:dra7xx_radio_helper");
MODULE_AUTHOR("Texas Instrument Inc.");
