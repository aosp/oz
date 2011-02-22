/*
 * twl6040-vibra.c - TWL6040 Vibrator driver
 *
 * Author:      Jorge Eduardo Candelaria <jorge.candelaria@ti.com>
 *
 * Copyright:   (C) 2011 Texas Instruments, Inc.
 *
 * Based on twl4030-vibra.c by Henrik Saari <henrik.saari@nokia.com>
 *				Felipe Balbi <felipe.balbi@nokia.com>
 *				Jari Vanhala <ext-javi.vanhala@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c/twl.h>
#include <linux/mfd/twl6040-codec.h>

struct vibra_info {
	struct device		*dev;
	struct input_dev	*input_dev;
	struct workqueue_struct	*workqueue;
	struct work_struct	play_work;

	bool			enabled;
	bool			coexist;

	struct twl6040		*twl6040;
};

static void vibra_enable(struct vibra_info *info)
{
	struct twl6040 *twl6040 = info->twl6040;
	u8 mask = TWL6040_VIBENAL | TWL6040_VIBCTRLLP;

	twl6040_set_bits(twl6040, TWL6040_REG_VIBCTLL, mask);
	twl6040_set_bits(twl6040, TWL6040_REG_VIBCTLR, mask);

	info->enabled = true;
}

static void vibra_disable(struct vibra_info *info)
{
	struct twl6040 *twl6040 = info->twl6040;
	u8 mask = TWL6040_VIBENAL | TWL6040_VIBCTRLLP;

	twl6040_clear_bits(twl6040, TWL6040_REG_VIBCTLL, mask);
	twl6040_clear_bits(twl6040, TWL6040_REG_VIBCTLR, mask);

	info->enabled = false;
}

static void vibra_play_work(struct work_struct *work)
{
	struct vibra_info *info = container_of(work,
				struct vibra_info, play_work);
	struct twl6040 *twl6040 = info->twl6040;

	if (!info->enabled)
		vibra_enable(info);

	/* TODO: Set direction and speed to vibra */
	twl6040_reg_write(twl6040, TWL6040_REG_VIBDATL, 0x32);
	twl6040_reg_write(twl6040, TWL6040_REG_VIBDATR, 0x32);
}

static int vibra_play(struct input_dev *input, void *data,
		      struct ff_effect *effect)
{
	struct vibra_info *info = input_get_drvdata(input);
	int ret;

	ret = queue_work(info->workqueue, &info->play_work);
	if (!ret)
		dev_err(&input->dev, "work is already on queue\n");

	return ret;
}

static int twl6040_vibra_open(struct input_dev *input)
{
	struct vibra_info *info = input_get_drvdata(input);

	info->workqueue = create_singlethread_workqueue("vibra");
	if (info->workqueue == 	NULL) {
		dev_err(&input->dev, "couldn't create workqueue\n");
		return -ENOMEM;
	}

	return 0;
}

static void twl6040_vibra_close(struct input_dev *input)
{
	struct vibra_info *info = input_get_drvdata(input);

	cancel_work_sync(&info->play_work);
	INIT_WORK(&info->play_work, vibra_play_work);
	destroy_workqueue(info->workqueue);
	info->workqueue = NULL;

	if (info->enabled)
		vibra_disable(info);
}

#if CONFIG_PM
static int twl6040_vibra_suspend(struct device *dev)
{
	struct vibra_info *info = dev_get_drvdata(dev);

	twl6040_disable(info->twl6040);

	return 0;
}

static int twl6040_vibra_resume(struct device *dev)
{
	struct vibra_info *info = dev_get_drvdata(dev);

	twl6040_enable(info->twl6040);

	return 0;
}
#else
#define twl6040_vibra_suspend NULL
#define twl6040_vibra_resume  NULL
#endif

static const struct dev_pm_ops twl6040_vibra_pm_ops = {
	.suspend = twl6040_vibra_suspend,
	.resume = twl6040_vibra_resume,
};

static int __devinit twl6040_vibra_probe(struct platform_device *pdev)
{
	struct twl4030_codec_vibra_data *pdata = pdev->dev.platform_data;
	struct vibra_info *info;
	int ret;

	if (!pdata) {
		dev_err(&pdev->dev, "platform_data not available\n");
		return -EINVAL;
	}

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "couldn't allocate memory \n");
		return -ENOMEM;
	}

	info->dev = &pdev->dev;
	info->coexist = pdata->coexist;
	info->twl6040 = dev_get_drvdata(pdev->dev.parent);
	INIT_WORK(&info->play_work, vibra_play_work);

	info->input_dev = input_allocate_device();
	if (info->input_dev == NULL) {
		dev_err(&pdev->dev, "couldn't allocate input device\n");
		ret = -ENOMEM;
		goto err_kzalloc;
	}

	input_set_drvdata(info->input_dev, info);

	info->input_dev->name = "twl6040:vibrator";
	info->input_dev->id.version = 1;
	info->input_dev->dev.parent = pdev->dev.parent;
	info->input_dev->open = twl6040_vibra_open;
	info->input_dev->close = twl6040_vibra_close;
	__set_bit(FF_RUMBLE, info->input_dev->ffbit);

	ret = input_ff_create_memless(info->input_dev, NULL, vibra_play);
	if (ret < 0) {
		dev_err(&pdev->dev, "couldn't register vibrator to FF\n");
		goto err_ialloc;
	}

	ret = input_register_device(info->input_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "couldn't register input device\n");
		goto err_iff;
	}

	platform_set_drvdata(pdev, info);

	twl6040_enable(info->twl6040);

	return 0;

err_iff:
	input_ff_destroy(info->input_dev);
err_ialloc:
	input_free_device(info->input_dev);
err_kzalloc:
	kfree(info);
	return ret;
}

static int __devexit twl6040_vibra_remove(struct platform_device *pdev)
{
	struct vibra_info *info = platform_get_drvdata(pdev);

	twl6040_disable(info->twl6040);
	input_unregister_device(info->input_dev);
	kfree(info);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver twl6040_vibra_driver = {
	.probe		= twl6040_vibra_probe,
	.remove		= __devexit_p(twl6040_vibra_remove),
	.driver		= {
		.name	= "twl6040-vibra",
		.owner 	= THIS_MODULE,
		.pm	= &twl6040_vibra_pm_ops,
	},
};

static int __init twl6040_vibra_init(void)
{
	return platform_driver_register(&twl6040_vibra_driver);
}
module_init(twl6040_vibra_init);

static void __exit twl6040_vibra_exit(void)
{
	platform_driver_unregister(&twl6040_vibra_driver);
}
module_exit(twl6040_vibra_exit);

MODULE_DESCRIPTION("TWL6040 Vibra driver");
MODULE_AUTHOR("Jorge Eduardo Candelaria");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:twl6040-vibra");
