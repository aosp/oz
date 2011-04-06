/*
 * mpu3050.c
 * MPU-3050 Gyroscope driver
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Dan Murphy <Dmurphy@ti.com>
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
 *
 * Derived work from mpu3050_gyro.c from Jorge Bustamante <jbustamante@ti.com>
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/i2c/mpu3050.h>
#include <linux/gpio.h>

#define MPU3050_DEBUG 1

#define DEVICE_NAME "mpu3050_gyro"
#define DRIVER_NAME "mpu3050_gyro"
#define INPUT_WORKQUEUE_SIZE 1

#define MPU3050_WHO_AM_I		0x00
#define MPU3050_PRODUCT_ID		0x01
#define MPU3050_X_OFFS_USRH		0x0C
#define MPU3050_X_OFFS_USRL		0x0D
#define MPU3050_Y_OFFS_USRH		0x0E
#define MPU3050_Y_OFFS_USRL		0x0F
#define MPU3050_Z_OFFS_USRH		0x10
#define MPU3050_Z_OFFS_USRL		0x11
#define MPU3050_IME_SLV_ADDR		0x14
#define MPU3050_SMPLRT_DIV		0x15
#define MPU3050_DLPF_FS_SYNC		0x16
#define MPU3050_INT_CFG			0x17
#define MPU3050_ACCEL_BUS_LVL		0x18
#define MPU3050_INT_STATUS		0x1A
#define MPU3050_TEMP_OUT_H		0x1B
#define MPU3050_TEMP_OUT_L		0x1C
#define MPU3050_GYRO_XOUT_H		0x1D
#define MPU3050_GYRO_XOUT_L		0x1E
#define MPU3050_GYRO_YOUT_H		0x1F
#define MPU3050_GYRO_YOUT_L		0x20
#define MPU3050_GYRO_ZOUT_H		0x21
#define MPU3050_GYRO_ZOUT_L		0x22
#define MPU3050_FIFO_COUNTH		0x3A
#define MPU3050_FIFO_COUNTL		0x3B
#define MPU3050_FIFO_R			0x3C
#define MPU3050_USER_CTRL		0x3D
#define MPU3050_PWR_MGMT		0x3E

struct mpu3050_gyro_data {
	struct mpu3050gyro_platform_data *pdata;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *wq;
	struct work_struct isr_work;
	struct mutex mutex;

	int buffer_queues;
	int enable;
	int def_poll_rate;
};

static uint32_t gyro_debug;
module_param_named(mpu3050_debug, gyro_debug, uint, 0664);

#ifdef MPU3050_DEBUG
struct mpu3050_reg {
	const char *name;
	uint8_t reg;
	int writeable;
} mpu3050_regs[] = {
	{ "CHIP_ID",		MPU3050_WHO_AM_I, 0 },
	{ "PRODUCT_ID",		MPU3050_PRODUCT_ID, 0 },
	{ "X_OFFSET_HI",	MPU3050_X_OFFS_USRH, 1},
	{ "X_OFFSET_LOW",	MPU3050_X_OFFS_USRL, 1},
	{ "Y_OFFSET_HI",	MPU3050_Y_OFFS_USRH, 1},
	{ "Y_OFFSET_LOW",	MPU3050_Y_OFFS_USRL, 1},
	{ "Z_OFFSET_HI",	MPU3050_Z_OFFS_USRH, 1},
	{ "Z_OFFSET_LOW",	MPU3050_Z_OFFS_USRL, 1},
	{ "SLAVE_I2C_ADDR",	MPU3050_IME_SLV_ADDR, 1},
	{ "SAMPLE_DIVISOR",	MPU3050_SMPLRT_DIV, 1},
	{ "DLPF_FS_SYNC",	MPU3050_DLPF_FS_SYNC, 1},
	{ "INT_CFG",		MPU3050_INT_CFG, 1},
	{ "ACCEL_BUS_LVL",	MPU3050_ACCEL_BUS_LVL, 1},
	{ "INT_STATUS",		MPU3050_INT_STATUS, 0},
	{ "TEMP_HIGH",		MPU3050_INT_STATUS, 0},
	{ "TEMP_LOW",		MPU3050_INT_STATUS, 0},
	{ "XOUT_HIGH",		MPU3050_GYRO_XOUT_H, 0},
	{ "XOUT_LOW",		MPU3050_GYRO_XOUT_L, 0},
	{ "YOUT_HIGH",		MPU3050_GYRO_YOUT_H, 0},
	{ "YOUT_LOW",		MPU3050_GYRO_YOUT_L, 0},
	{ "ZOUT_HIGH",		MPU3050_GYRO_ZOUT_H, 0},
	{ "ZOUT_LOW",		MPU3050_GYRO_ZOUT_L, 0},
	{ "FIFO_CNT_H",		MPU3050_FIFO_COUNTH, 0},
	{ "FIFO_CNT_L",		MPU3050_FIFO_COUNTL, 0},
	{ "FIFO_R",		MPU3050_FIFO_R, 0},
	{ "USR_CTRL",		MPU3050_USER_CTRL, 0},
	{ "PWR_MGMT",		MPU3050_PWR_MGMT, 0},

};
#endif

static int mpu3050_write(struct mpu3050_gyro_data *data,
				u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(data->client, reg, val);
	if (ret < 0)
		dev_err(&data->client->dev,
			"i2c_smbus_write_byte_data failed\n");
	return ret;
}

static int mpu3050_read(struct mpu3050_gyro_data *data, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(data->client, reg);
	if (ret < 0)
		dev_err(&data->client->dev,
			"i2c_smbus_read_byte_data failed\n");
	return ret;
}

/* TO DO clean this function up.  Maybe change the I2C call
to do I2c transfers with auto increment so you can read 6
bytes in one transfer it will reduce time on the I2c bus
and make this function a lot cleaner */
static int mpu3050_data_ready(struct mpu3050_gyro_data *data)
{
	uint8_t data_val_h, data_val_l;
	short int x = 0;
	short int y = 0;
	short int z = 0;
	short int temp = 0;

	data_val_h = mpu3050_read(data, MPU3050_TEMP_OUT_H);
	data_val_l = mpu3050_read(data, MPU3050_TEMP_OUT_L);
	temp = ((data_val_h << 8) | data_val_l);
	if (gyro_debug)
		pr_info("%s: temp low 0x%X temp high 0x%X\n",
		__func__, data_val_l, data_val_h);


	data_val_h = mpu3050_read(data, MPU3050_GYRO_XOUT_H);
	data_val_l = mpu3050_read(data, MPU3050_GYRO_XOUT_L);
	if (gyro_debug)
		pr_info("%s: X low 0x%X X high 0x%X\n",
		__func__, data_val_l, data_val_h);
	x = ((data_val_h << 8) | data_val_l);

	data_val_h = mpu3050_read(data, MPU3050_GYRO_YOUT_H);
	data_val_l = mpu3050_read(data, MPU3050_GYRO_YOUT_L);
	if (gyro_debug)
		pr_info("%s: Y low 0x%X Y high 0x%X\n",
		__func__, data_val_l, data_val_h);

	y = ((data_val_h << 8) | data_val_l);

	data_val_h = mpu3050_read(data, MPU3050_GYRO_ZOUT_H);
	data_val_l = mpu3050_read(data, MPU3050_GYRO_ZOUT_L);
	if (gyro_debug)
		pr_info("%s: Z low 0x%X Z high 0x%X\n",
		__func__, data_val_l, data_val_h);

	z = ((data_val_h << 8) | data_val_l);
	if (gyro_debug)
		pr_info("%s: X: 0x%X Y: 0x%X Z: 0x%X\n",
			__func__, x, y, z);
	input_report_rel(data->input_dev, REL_RX, x);
	input_report_rel(data->input_dev, REL_RY, y);
	input_report_rel(data->input_dev, REL_RZ, z);
	input_sync(data->input_dev);

	return 0;
}

/* TODO: fix this function since it is failing some times,
	probably because reads on H and L parts are not atomic. */
static int mpu3050_device_autocalibrate(struct mpu3050_gyro_data *data)
{
#if 0
	uint8_t data_val_h, data_val_l;
	short int data_val;

	/* read the values on a stable position (no movement) */
	data_val_h = mpu3050_read(data, MPU3050_GYRO_XOUT_H);
	data_val_l = mpu3050_read(data, MPU3050_GYRO_XOUT_L);
	/* get the 2's complement of data_val and
	leave the corresponding values on data_val_h and data_val_l */
	data_val = (short int) ((data_val_h << 8) | data_val_l);
	data_val = -data_val;
	data_val_h = (data_val >> 8);
	data_val_l = (data_val & 0x00FF);
	/* write the value to the corresponding offset register */
	mpu3050_write(data, MPU3050_X_OFFS_USRH, data_val_h);
	mpu3050_write(data, MPU3050_X_OFFS_USRL, data_val_l);

	/* read the values on a stable position (no movement) */
	data_val_h = mpu3050_read(data, MPU3050_GYRO_YOUT_H);
	data_val_l = mpu3050_read(data, MPU3050_GYRO_YOUT_L);
	/* get the 2's complement of data_val and leave the
	corresponding values on data_val_h and data_val_l */
	data_val = (short int) ((data_val_h << 8) | data_val_l);
	data_val = -data_val;
	data_val_h = (data_val >> 8);
	data_val_l = (data_val & 0x00FF);
	/* write the value to the corresponding offset register */
	mpu3050_write(data, MPU3050_Y_OFFS_USRH, data_val_h);
	mpu3050_write(data, MPU3050_Y_OFFS_USRL, data_val_l);

	/* read the values on a stable position (no movement) */
	data_val_h = mpu3050_read(data, MPU3050_GYRO_ZOUT_H);
	data_val_l = mpu3050_read(data, MPU3050_GYRO_ZOUT_L);
	/* get the 2's complement of data_val and leave the
	corresponding values on data_val_h and data_val_l */
	data_val = (short int) ((data_val_h << 8) | data_val_l);
	data_val = -data_val;
	data_val_h = (data_val >> 8);
	data_val_l = (data_val & 0x00FF);
	/* write the value to the corresponding offset register */
	mpu3050_write(data, MPU3050_Z_OFFS_USRH, data_val_h);
	mpu3050_write(data, MPU3050_Z_OFFS_USRL, data_val_l);
#endif
	return 0;
}

static irqreturn_t mpu3050_thread_irq(int irq, void *dev_data)
{
	uint8_t reg_val;
	struct mpu3050_gyro_data *data = (struct mpu3050_gyro_data *) dev_data;

	reg_val = mpu3050_read(data, MPU3050_INT_STATUS);

	if (reg_val & MPU3050_INT_STATUS_MPU_RDY) {
#ifdef MPU3050_DEBUG
		printk(KERN_ALERT "%s: interrupt: MPU ready.", __func__);
#endif
	} else if (reg_val & MPU3050_INT_STATUS_RAW_DATA_RDY) {
		if (data->wq != NULL) {
			int r;
			if (data->buffer_queues < INPUT_WORKQUEUE_SIZE) {
				data->buffer_queues++;
				r = queue_work(data->wq, &data->isr_work);
				if (!r) {
					dev_err(&data->client->dev,
						"Unable to create workqueue\n");
					return IRQ_NONE;
				}
			}
		}
	} else {
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static int mpu3050_device_isr_work(void *work)
{
	struct mpu3050_gyro_data *data = container_of((struct work_struct *)work,
				struct mpu3050_gyro_data, isr_work);

	mpu3050_data_ready(data);

	data->buffer_queues--;

	return IRQ_HANDLED;
}

/* TO DO: Need to fix auto calibrate for the Gyro */
static int mpu3050_device_hw_init(struct mpu3050_gyro_data *data)
{
	int ret = 0;
	uint8_t reg_val;

	mpu3050_write(data, MPU3050_PWR_MGMT,
		(MPU3050_PWR_MGMT_RESET | MPU3050_PWR_MGMT_CLK_STOP_RESET));
	mpu3050_write(data, MPU3050_PWR_MGMT, 0x00);

	mpu3050_write(data, MPU3050_WHO_AM_I, data->client->addr);
	mpu3050_write(data, MPU3050_IME_SLV_ADDR, data->pdata->slave_i2c_addr);
	mpu3050_write(data, MPU3050_SMPLRT_DIV,	data->pdata->sample_rate_div);
	mpu3050_write(data, MPU3050_DLPF_FS_SYNC, data->pdata->dlpf_fs_sync);
	mpu3050_write(data, MPU3050_INT_CFG, data->pdata->interrupt_cfg);
	mpu3050_write(data, MPU3050_ACCEL_BUS_LVL, 0x00);

	msleep(1);

	/* auto calibrate (dont move the IC while this function is called) */
	mpu3050_device_autocalibrate(data);

	reg_val = mpu3050_read(data, MPU3050_INT_STATUS);

	return ret;
}

static ssize_t mpu3050_show_attr_enable(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mpu3050_gyro_data *data = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", data->enable);
}

/* TO DO: This function will change based on the if you are doing
an interrupt or doing a work queue */
static ssize_t mpu3050_store_attr_enable(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mpu3050_gyro_data *data = platform_get_drvdata(pdev);
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 0, &val);
	if (error)
		return error;

	if (val == 0)
		data->enable = val;
	else
		data->enable = 0x01;

	if (data->enable) {
		queue_work(data->wq, &data->isr_work);
		enable_irq(data->client->irq);
	} else {
		disable_irq_nosync(data->client->irq);
	}
	return count;
}

static ssize_t mpu3050_show_attr_delay(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mpu3050_gyro_data *data = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", data->def_poll_rate);
}

/* TO DO: Need to fix this to have the IC perform and report
its measurements as close to the requested time as possible */
static ssize_t mpu3050_store_attr_delay(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mpu3050_gyro_data *data = platform_get_drvdata(pdev);
	unsigned long interval;
	int error;

	error = strict_strtoul(buf, 0, &interval);
	if (error)
		return error;


	if (interval <= 0 || interval > 200)
		return -EINVAL;

	data->def_poll_rate = interval;
#if 0
	cancel_delayed_work_sync(&data->worklogic);
	schedule_delayed_work(&data->worklogic, 0);
#endif
	return count;

}

#ifdef MPU3050_DEBUG
static ssize_t mpu3050_registers_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mpu3050_gyro_data *data = platform_get_drvdata(pdev);
	unsigned i, n, reg_count;
	uint8_t value;

	reg_count = sizeof(mpu3050_regs) / sizeof(mpu3050_regs[0]);
	for (i = 0, n = 0; i < reg_count; i++) {
		value = mpu3050_read(data, mpu3050_regs[i].reg);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "%-20s = 0x%02X\n",
			       mpu3050_regs[i].name,
			       value);
	}

	return n;
}

static ssize_t mpu3050_registers_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mpu3050_gyro_data *data = platform_get_drvdata(pdev);
	unsigned i, reg_count, value;
	int error = 0;
	char name[30];

	if (count >= 30) {
		pr_err("%s:input too long\n", __func__);
		return -1;
	}

	if (sscanf(buf, "%s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -1;
	}

	reg_count = sizeof(mpu3050_regs) / sizeof(mpu3050_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, mpu3050_regs[i].name)) {
			if (mpu3050_regs[i].writeable) {
				error = mpu3050_write(data,
					mpu3050_regs[i].reg,
					value);
				if (error) {
					pr_err("%s:Failed to write %s\n",
						__func__, name);
					return -1;
				}
			} else {
				pr_err("%s:Register %s is not writeable\n",
						__func__, name);
					return -1;
			}
			return count;
		}
	}

	pr_err("%s:no such register %s\n", __func__, name);
	return -1;
}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		mpu3050_registers_show, mpu3050_registers_store);
#endif

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
		mpu3050_show_attr_enable, mpu3050_store_attr_enable);

static DEVICE_ATTR(delay, S_IWUSR | S_IRUGO,
		mpu3050_show_attr_delay, mpu3050_store_attr_delay);

static struct attribute *mpu3050_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
#ifdef MPU3050_DEBUG
	&dev_attr_registers.attr,
#endif
	NULL
};

static const struct attribute_group mpu3050_attr_group = {
	.attrs = mpu3050_attrs,
};

/* TO DO: Implement a polling mechanism when the IRQ method is not desired.
Also find out if the gyro will reliably support a polling mechanism */
static int __devinit mpu3050_driver_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct mpu3050gyro_platform_data *pdata = client->dev.platform_data;
	struct mpu3050_gyro_data *data;
	int ret = 0;

	pr_info("%s: Enter\n", __func__);

	if (pdata == NULL) {
		pr_err("%s: Platform data not found\n", __func__);
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: need I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}

	data = kzalloc(sizeof(struct mpu3050_gyro_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	INIT_WORK(&data->isr_work, (void *)mpu3050_device_isr_work);

	data->wq = create_singlethread_workqueue("mpu3050");
	if (!data->wq) {
		ret = -ENOMEM;
		goto error;
	}

	data->pdata = pdata;
	data->client = client;
	data->enable = 0;
	i2c_set_clientdata(client, data);

	data->input_dev = input_allocate_device();
	if (!data->input_dev) {
		ret = -ENOMEM;
		dev_err(&data->client->dev,
			"Failed to allocate input device\n");
		goto error;
	}

	data->input_dev->name = "mpu3050";
	data->input_dev->id.bustype = BUS_I2C;

	data->input_dev->dev.parent = &data->client->dev;
	input_set_drvdata(data->input_dev, data);

	input_set_capability(data->input_dev, EV_REL, REL_RX);
	input_set_capability(data->input_dev, EV_REL, REL_RY);
	input_set_capability(data->input_dev, EV_REL, REL_RZ);


	ret = input_register_device(data->input_dev);
	if (ret) {
		dev_err(&data->client->dev,
			"Unable to register input device\n");
		goto error;
	}

	mutex_init(&data->mutex);
	if (data->client->irq) {
		ret = request_threaded_irq(data->client->irq, NULL,
					mpu3050_thread_irq,
					data->pdata->irq_flags,
					data->client->name, data);
		if (ret < 0) {
			dev_err(&data->client->dev,
				"request_threaded_irq failed\n");
			goto error;
		}
		disable_irq_nosync(data->client->irq);
	}

	data->buffer_queues = 0;

	mpu3050_device_hw_init(data);
	if (ret)
		goto error;

	ret = sysfs_create_group(&client->dev.kobj, &mpu3050_attr_group);
	if (ret)
		goto error;

	return 0;

error:
	mutex_destroy(&data->mutex);
	kfree(data);
	return ret;
}

static int __devexit mpu3050_driver_remove(struct i2c_client *client)
{
	struct mpu3050_gyro_data *data = i2c_get_clientdata(client);
	int ret;

	sysfs_remove_group(&client->dev.kobj, &mpu3050_attr_group);
	ret = mpu3050_device_deinit(data);
	i2c_set_clientdata(client, NULL);
	kfree(data);

	return ret;
}

#ifdef CONFIG_PM
/* TO DO: Need to figure out how to sleep the device and
instrument power on/off calls that can be called from suspend/resume
and from the enable sysfs entry */
static int mpu3050_driver_suspend(struct i2c_client *client,
		pm_message_t mesg)
{
	/* TODO */
	return 0;
}

static int mpu3050_driver_resume(struct i2c_client *client)
{
	/* TODO */
	return 0;
}
#endif


static const struct i2c_device_id mpu3050_idtable[] = {
	{ DEVICE_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, mpu3050_idtable);

static struct i2c_driver mpu3050_driver = {
	.probe		= mpu3050_driver_probe,
	.remove		= mpu3050_driver_remove,
	.id_table	= mpu3050_idtable,
#ifdef CONFIG_PM
	.suspend	= mpu3050_driver_suspend,
	.resume		= mpu3050_driver_resume,
#endif
	.driver = {
		.name = DRIVER_NAME
	},
};

static int __init mpu3050_driver_init(void)
{
	return i2c_add_driver(&mpu3050_driver);
}

static void __exit mpu3050_driver_exit(void)
{
	i2c_del_driver(&mpu3050_driver);
}

module_init(mpu3050_driver_init);
module_exit(mpu3050_driver_exit);

MODULE_DESCRIPTION("MPU-3050 Gyroscope Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dan Murphy <DMurphy@ti.com>");
MODULE_AUTHOR("Jorge Bustamante <jbustamante@ti.com>");
