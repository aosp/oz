/*
FIXME: Add copyright info here

This support LP101WX2 touch screen controller

Right now this is the only controller from LG-Philips
the driver supports
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/of_i2c.h>
#include <linux/regmap.h>

#define TLC_NAME		"tlc59108"
#define TLC_I2C_ADDR		0x40

#define TLC59108_MODE1		0x00
#define TLC59108_PWM2		0x04
#define TLC59108_LEDOUT0	0x0c
#define TLC59108_LEDOUT1	0x0d
#define IDLE_MASK	0x01
#define IDLE_OFF	0x0

/* polling mode set comment this line to enable interrupt mode */

/* #define TS_POLL */

#define MAX_TOUCH_POINTS	10 /* max touch points handled by the TSC */
#define TP_READ_SIZE		3 /* co-ordinates size in bytes */
#define DEFAULT_READ_LEN	2 /* corresponds to the ID bit-field */



static int ldc3001_probe(struct i2c_client *client,
					const struct i2c_device_id *idev_id);
static int ldc3001_remove(struct i2c_client *client);
static int tlc_wakeup_ldc(struct i2c_client *client);

struct tlc_data {
	struct	device *dev;
	struct  regmap *regmap;
};

struct touch_point {
	u8 touch_id;
	u8 state;
	int x_pos;
	int y_pos;
};

/* The platform data for the LGPhilips LDC3001 TSC */
struct ldc3001_platform_data {
	const u8 *config;
	size_t config_length;

	unsigned int x_line;
	unsigned int y_line;
	unsigned int x_size;
	unsigned int y_size;
	unsigned int blen;
	unsigned int threshold;
	unsigned int voltage;
	unsigned char orient;
	unsigned long irqflags;
};

/* Each client has this additional data */
struct ldc3001_data {
	struct i2c_client *client;
	struct i2c_client *tlc_client;
	struct input_dev *input_dev;
	char phys[64];		/* device physical location */
	const struct ldc3001_platform_data *pdata;
	/* FIXME: Not required for now */
	/* struct mxt_object *object_table;
	struct mxt_info info; */
	unsigned int irq;
	unsigned int max_x;
	unsigned int max_y;
	struct task_struct *task;
	u8 message[MAX_TOUCH_POINTS * TP_READ_SIZE + DEFAULT_READ_LEN];
	struct touch_point tps[MAX_TOUCH_POINTS];
	u8 cur_index;
	u16 tp_fields;
	/* Cached parameters from object table */
	/* FIXME: Remove stuff thats not required
	u8 T6_reportid;
	u8 T9_reportid_min;
	u8 T9_reportid_max;
	*/
	int p_gpio;
};

static const struct i2c_device_id ldc3001_i2c_ts_id[] = {
	{"ldc3001", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ldc3001_i2c_ts_id);

static ssize_t ldc3001_fw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf) {

	return scnprintf(buf, PAGE_SIZE, "1.04\n");

}
static DEVICE_ATTR(fw_version, S_IRUGO, ldc3001_fw_version_show, NULL);
/* static DEVICE_ATTR(object, S_IRUGO, mxt_object_show, NULL); */

static struct attribute *mxt_attrs[] = {
	&dev_attr_fw_version.attr,
	/* &dev_attr_object.attr, */
	NULL
};

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
};






#ifdef CONFIG_PM_SLEEP

static int ldc3001_i2c_ts_suspend(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
	return 0;
}

static int ldc3001_i2c_ts_resume(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);

	return 0;
}

static SIMPLE_DEV_PM_OPS(ldc3001_dev_pm_ops,
			 ldc3001_i2c_ts_suspend, ldc3001_i2c_ts_resume);


#endif

static int dump_touch_data(u8 *message, int len)
{

	print_hex_dump(KERN_DEBUG, " \t", DUMP_PREFIX_NONE, 16, 1,
			message, len, false);
}


static u16 get_touch_fields(struct ldc3001_data *data)
{
	u8 *message = data->message;
	/*FIXME: remove magic numbers */
	data->tp_fields = data->message[0] & 0x00ff;
	data->tp_fields |= ((message[1] & 0x30)  << 4);

	pr_err("Bit field is %u\n", data->tp_fields);
	return data->tp_fields;
}

/*
 * index	- index to the read message
 * touch_idx	- touch_idx tracking id
 */

u16  get_tpos(struct ldc3001_data *data, u8 index, u8 touch_idx)
{
	u16 x, y;
	u8 *message = &data->message[index*3 + 2];

	x	  = (((message[0] >> 4) & 0x0f) << 8);
	pr_err("x-hi %x\n", x);
	data->tps[index].x_pos = (x | message[1]);
	pr_err("x %x\n", data->tps[index].x_pos);

	y	  = (message[0] & 0x0f) << 8;
	pr_err("y-hi %x\n", y);
	data->tps[index].y_pos = (y | message[2]);
	pr_err("y %x\n", data->tps[index].y_pos);

	data->tps[index].touch_id = touch_idx;
}


static int ldc3001_write_reg(struct i2c_client *client, u16 reg, u16 len,
		const void *val)
{
	u8 *buf;
	size_t count;
	int ret;

	count = len + 2;
	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	memcpy(&buf[2], val, len);

	ret = i2c_master_send(client, buf, count);
	if (ret == count) {
		ret = 0;
	} else {
		if (ret >= 0)
			ret = -EIO;
		dev_err(&client->dev, "%s: i2c send failed (%d)\n",
			__func__, ret);
	}

	kfree(buf);
	return ret;
}




static int ldc3001_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];
	int ret;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	ret = i2c_transfer(client->adapter, xfer, 2);
	if (ret == 2) {
		ret = 0;
	} else {
		if (ret >= 0)
			ret = -EIO;
		dev_err(&client->dev, "%s: i2c transfer failed (%d)\n",
			__func__, ret);
	}

	return ret;
}


static int ldc3001_ts_read_data(struct ldc3001_data *data, int len)
{
	int error;
	u8 msg;
	struct i2c_client *client = data->client;

	error = ldc3001_read_reg(client, 0xfa, len, &data->message);
	if (error)
		dev_err(&client->dev, "i2c_read failed %x\n", error);

	return error;
}

static int ldc3001_ts_read_len(struct ldc3001_data *data)
{
	int error;
	u8 len = 0;

	struct i2c_client *client = data->client;

	/*FIXME: remove magic numbers */
	error = ldc3001_read_reg(client, 0xf5, 1, &len);
	if (error)
		dev_err(&client->dev, "i2c_read failed %x\n", error);
	else
		dev_dbg(&client->dev, "length is %x\n", len);

	return len;
}



static void ldc3001_input_touchevent(struct ldc3001_data *data)
{
	u16 x, y, touch_index = 0;
	u8 i, msg_index = 0;
	struct input_dev *input_dev = data->input_dev;
	u8 *message = &data->message[0];


	/* get valid touch ids */
	/* FIXME: remove all magic numbers */
	touch_index = message[0] & 0x00ff;
	touch_index |= ((message[1] & 0x30) << 4);

	/* loop through all tps to process changes */
	for (i = 0; i < MAX_TOUCH_POINTS; i++) {

		/* tracking id has a touch event */
		if (touch_index & (1 << i)) {
			message = &data->message[msg_index*3 + 2];

			/* decode the x and y co-ordinates */
			x = (((message[0] >> 4) & 0x0f) << 8);
			data->tps[i].x_pos = (x | message[1]);

			y = (message[0] & 0x0f) << 8;
			data->tps[i].y_pos = (y | message[2]);

			/* update state and move to the next message */
			data->tps[i].state = 1;
			msg_index++;

			/* report the touch event */
			input_mt_slot(input_dev, i);
			input_mt_report_slot_state(input_dev,
					MT_TOOL_FINGER, data->tps[i].state);
			input_report_abs(input_dev, ABS_MT_POSITION_X,
					data->tps[i].x_pos);
			input_report_abs(input_dev, ABS_MT_POSITION_Y,
					data->tps[i].y_pos);

			pr_debug("x_pos = %d for %d\n", data->tps[i].x_pos, i);
			pr_debug("y_pos = %d for %d\n", data->tps[i].y_pos, i);

		} else if (data->tps[i].state == 1) {
			/* FIXME: remove all magic numbers */
			/* this is a pen release event */
			data->tps[i].state = 0;
			input_mt_slot(input_dev, i);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER,
							data->tps[i].state);
			pr_debug("pen release event for %d\n", i);
		}
	}

	pr_debug("Reported %d events to userspace\n", msg_index);

	/* send sync event */
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(input_dev);
}


#ifdef TS_POLL
static int ldc3001_ts_interrupt(void *dev_id)
#else /* INTERRPUT */
static irqreturn_t ldc3001_ts_interrupt(int irq, void *dev_id)
#endif
{
	struct ldc3001_data *data = (struct ldc3001_data *)dev_id;
	int read_len;
#ifdef TS_POLL
	do {
		u16 touch_index = 0, i = 0, j = 0;

#else
		disable_irq_nosync(irq);
#endif
		/* Read and print the len value
		   Read i2c messages for the size */
		read_len = ldc3001_ts_read_len(data);

		/* reset touchid fields */
		data->message[0] = 0x0;
		data->message[1] = 0x0;

		if (read_len > 2) {
			/* fill the message array with data read from tsc */
			ldc3001_ts_read_data(data, read_len);
		}
		ldc3001_input_touchevent(data);
#ifdef TS_POLL
		schedule();
		mdelay(50);
	} while (!kthread_should_stop());
#else
	enable_irq(irq);
	return IRQ_HANDLED;
#endif
}


static const struct of_device_id ldc3001_dt_ids[] = {
	{ .compatible = "lgphilips,ldc3001"},
	{ /* sentinel */ }
};


static struct i2c_driver ldc3001_i2c_ts_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ldc3001",
		.pm	= &ldc3001_dev_pm_ops,
		.of_match_table = ldc3001_dt_ids,
	},
	.probe		= ldc3001_probe,
	.remove		= ldc3001_remove,
	.id_table	= ldc3001_i2c_ts_id,
};

static int tlc_wakeup_ldc(struct i2c_client *client)
{
	struct tlc_data *data = dev_get_drvdata(&client->dev);
	struct regmap *map = data->regmap;

	regmap_write(map, TLC59108_LEDOUT1, 0x00);
	regmap_write(map, TLC59108_LEDOUT1, 0x10);
	udelay(1000);
	/* FIXME: return error */
	return 0;
}

/* TLCRemove later --START */
static int tlc_init(struct i2c_client *client)
{
	struct tlc_data *data = dev_get_drvdata(&client->dev);
	struct regmap *map = data->regmap;

	/* init the TLC chip */
	regmap_write(map, TLC59108_MODE1, 0x01);

	/*
	 * set LED1(AVDD) to ON state(default), enable LED2 in PWM mode, enable
	 * LED0 to OFF state
	 */
	regmap_write(map, TLC59108_LEDOUT0, 0x21);

	/* set LED2 PWM to full freq */
	regmap_write(map, TLC59108_PWM2, 0xff);

	/* set LED4(UPDN) and LED6(MODE3) to OFF state */
	regmap_write(map, TLC59108_LEDOUT1, 0x11);

	return 0;
}

static int tlc_uninit(struct i2c_client *client)
{
	struct tlc_data *data = dev_get_drvdata(&client->dev);
	struct regmap *map = data->regmap;

	/* clear TLC chip regs */
	regmap_write(map, TLC59108_PWM2, 0x0);
	regmap_write(map, TLC59108_LEDOUT0, 0x0);
	regmap_write(map, TLC59108_LEDOUT1, 0x0);

	regmap_write(map, TLC59108_MODE1, 0x0);

	return 0;
}

static struct i2c_board_info tlc_i2c_board_info = {
	I2C_BOARD_INFO(TLC_NAME, TLC_I2C_ADDR),
};


static struct regmap_config tlc59108_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int tlc59108_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int r;
	struct regmap *regmap;
	struct tlc_data *data;
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	unsigned int val;
	int gpio;

	pr_err("%s\n", __func__);
	regmap = devm_regmap_init_i2c(client, &tlc59108_regmap_config);
	if (IS_ERR(regmap)) {
		r = PTR_ERR(regmap);
		dev_err(&client->dev, "Failed to init regmap: %d\n", r);
		return r;
	}

	data = devm_kzalloc(dev, sizeof(struct tlc_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dev_set_drvdata(dev, data);
	data->dev = dev;
	data->regmap = regmap;

	/* msleep(20); */

	/* Try to read a TLC register to verify if i2c works */
	r = regmap_read(data->regmap, TLC59108_MODE1, &val);
	if (r < 0) {
		dev_err(dev, "Failed to set MODE1: %d\n", r);
		return r;
	}

	dev_info(dev, "Successfully initialized %s\n", TLC_NAME);
	pr_err("%s done...\n", __func__);

	return 0;
}

static int tlc59108_i2c_remove(struct i2c_client *client)
{
	struct tlc_data *data = dev_get_drvdata(&client->dev);

	i2c_del_driver(&ldc3001_i2c_ts_driver);

	return 0;
}

static const struct i2c_device_id tlc59108_id[] = {
	{ TLC_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tlc59108_id);

static const struct of_device_id tlc59108_of_match[] = {
	{ .compatible = "ti,tlc59108", },
	{ },
};
MODULE_DEVICE_TABLE(of, tlc59108_of_match);

static struct i2c_driver tlc59108_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= TLC_NAME,
		.of_match_table = tlc59108_of_match,
	},
	.id_table	= tlc59108_id,
	.probe		= tlc59108_i2c_probe,
	.remove		= tlc59108_i2c_remove,
};

/* TLCRemove later --END */

static int ldc3001_init(struct i2c_client *i2c_client)
{
	/* read firmware version here */
	int error;
	u8 val;

	struct ldc3001_data *ldc_data = i2c_get_clientdata(i2c_client);

	if (ldc_data) {
		struct i2c_client *tlc_client = ldc_data->tlc_client;
		error  = 0;
		error = tlc_wakeup_ldc(tlc_client);
		if (error) {
			pr_err("TLC I2C wakeup error\n");
			goto end;
		}
		/* FIXME: remove magic numbers */
		error = ldc3001_read_reg(i2c_client, 0xd0, 1, &val);
		if (error) {
			dev_err(&i2c_client->dev, "i2c_read failed %x\n",
									error);
			goto end;
		}
		dev_info(&i2c_client->dev, "ldc3001 version number is %x\n",
									val);

		/* FIXME: remove magic numbers */
		/* read idle mode */
		error = ldc3001_read_reg(i2c_client, 0x17, 1, &val);
		dev_info(&i2c_client->dev, "ldc3001 idle mode is is %x\n", val);

		/* FIXME: remove magic numbers */
		/* disable idle mode */
		val = ((val & ~IDLE_MASK) | IDLE_OFF);
		error = ldc3001_write_reg(i2c_client, 0x11, 1, &val);

		/* FIXME: remove magic numbers */
		/* re-dead idle mode */
		error = ldc3001_read_reg(i2c_client, 0x17, 1, &val);
		dev_info(&i2c_client->dev, "new ldc3001 idle mode is is %x\n",
									val);

	} else
		dev_err(&i2c_client->dev, "invalid driver data\n");
end:
	return error;
}

static int ldc3001_probe(struct i2c_client *client,
					const struct i2c_device_id *idev_id)
{
	struct device_node *node = client->dev.of_node;
	struct device_node *tlc;
	struct ldc3001_platform_data *pdata;
	struct ldc3001_data *data;
	struct input_dev *input_dev;
	const struct of_device_id *match;
	struct i2c_client *tlc_client;
	unsigned int num_mt_slots;
	int error;
	int gpio;

	/* - enable the GPIO to keep the TSC in power on condition
	 *   this will be moved to suspend and resume later
	 * - create the i2c client
	 * - create the input device
	 * - setup multitouch
	 * - request IRQ
	 *
	 **/
	dev_dbg(&client->dev, "%s\n", __func__);
	match = of_match_device(of_match_ptr(ldc3001_dt_ids), &client->dev);
	if (match) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct ldc3001_platform_data), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
		/* FIXME: add this function back later
		   error = ldc3001_of_get_platform_info(client, pdata);
		   if (error)
		   return -EINVAL;
		 */
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			dev_err(&client->dev, "Platform data not populated\n");
			return -EINVAL;
		}
	}

	data = devm_kzalloc(&client->dev,
			sizeof(struct ldc3001_data), GFP_KERNEL);
	input_dev = input_allocate_device();

	/*FIXME: need to free which ever is not NULL */
	if (!data || !input_dev) {
		dev_err(&client->dev, "failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	/*FIXME: HACK to avoid changes to android FS */
	input_dev->name = "Atmel maXTouch Touchscreen";
	/* input_dev->name = "LDC 3001 based TouchScreen Controller";*/

	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
			client->adapter->nr, client->addr);

	/* get the irq line. for j6 it is a gpio line */

	data->p_gpio = of_get_gpio(node, 0);

	if (gpio_is_valid(data->p_gpio)) {
		error  = devm_gpio_request_one(&client->dev, data->p_gpio,
						GPIOF_IN, "ldc3001_p_gpio");
		if (error) {
			dev_err(&client->dev, "Failed to request irq GPIO %d\n",
				data->p_gpio);
			return error;
		}
	}


	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	data->client = client;
	data->input_dev = input_dev;
	data->pdata = pdata;

	/*FIXME: tlc need to be a pwm driver. */
#if 1
	tlc = of_parse_phandle(node, "tlc", 0);
	if (!tlc) {
		dev_err(&client->dev, "could not find tlc device\n");
		return -EINVAL;
	}

	tlc_client = of_find_i2c_device_by_node(tlc);
	if (!tlc_client) {
		dev_err(&client->dev, "failed to find tlc i2c client device\n");
		return -EINVAL;
	}

	data->tlc_client = tlc_client;
#endif

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	/* init the controller here. Maybe read firmware version */
	ldc3001_init(client);

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			0, /* data->max_x */ 1280, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			0, /* data->max_y */ 800, 0, 0);
	/*input_set_abs_params(input_dev, ABS_PRESSURE,
			0, 255, 0, 0);*/

	/* For multi touch */
	/*
	num_mt_slots = data->T9_reportid_max - data->T9_reportid_min + 1;
	error = input_mt_init_slots(input_dev, num_mt_slots, 0);
	*/
	num_mt_slots = 10;
	error = input_mt_init_slots(input_dev, 10, 0);
	if (error)
		goto err_free_object;
	/*input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			0, MXT_MAX_AREA, 0, 0);*/
	/* input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			0, 0xff, 0, 0);*/
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			0, /* data->max_x */ 1280, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			0, /* data->max_y */ 800, 0, 0);

#ifdef TS_POLL

	data->task = kthread_run(ldc3001_ts_interrupt, (void *)data,
							"ts_poll_thread");
#else


	client->irq = gpio_to_irq(data->p_gpio);
	if (error < 0) {
		dev_err(&client->dev, "gpio_to_irq(): %d\n", error);
		return error;
	}

	data->irq = client->irq;
	pr_debug("irq registered is %x\n", client->irq);

	error = devm_request_threaded_irq(&client->dev, data->irq,
			NULL, ldc3001_ts_interrupt,
			IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
			/* IRQF_ONESHOT, */
			dev_name(&client->dev), data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_object;
	}
#endif

	error = input_register_device(input_dev);
	if (error)
		goto err_free_object;

	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);

	return 0;
	/*FIXME: need to fix error handling */
err_free_object:
err_free_mem:
	devm_kfree(&client->dev, pdata);

	return error;
}

static int ldc3001_remove(struct i2c_client *client)
{
	struct ldc3001_data *data = i2c_get_clientdata(client);

	dev_err(&client->dev, "%s\n", __func__);
	i2c_del_driver(&ldc3001_i2c_ts_driver);
	return 0;
}


static int __init ldc_module_init(void)
{

	pr_err("%s\n", __func__);
	i2c_register_driver(THIS_MODULE, &ldc3001_i2c_ts_driver);
	return 0;
}

static void __exit ldc_module_exit(void)
{
	pr_err("%s\n", __func__);
	i2c_del_driver(&ldc3001_i2c_ts_driver);
}

module_init(ldc_module_init);
module_exit(ldc_module_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("LDC3001 I2C Touchscreen driver");
MODULE_LICENSE("GPL");
