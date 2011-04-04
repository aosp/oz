/*
 * bma180.c
 * BMA-180 Accelerometer driver
 *
 * Copyright (C) 2010 Texas Instruments
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
 * Derived work from bma180_accl.c from Jorge Bustamante <jbustamante@ti.com>
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/i2c/bma180.h>
#include <linux/gpio.h>

#define DEVICE_NAME "bma180"
#define DRIVER_NAME "bma180_accel"
#define INPUT_WORKQUEUE_SIZE 1

#define BMA180_OFFSET_Z			0x3A
#define BMA180_OFFSET_Y			0x39
#define BMA180_OFFSET_X			0x38
#define BMA180_OFFSET_T			0x37
#define BMA180_OFFSET_LSB2		0x36
#define BMA180_OFFSET_LSB1		0x35
#define BMA180_GAIN_Z			0x34
#define BMA180_GAIN_Y			0x33
#define BMA180_GAIN_X			0x32
#define BMA180_GAIN_T			0x31
#define BMA180_TCO_Z			0x30
#define BMA180_TCO_Y			0x2F
#define BMA180_TCO_X			0x2E
#define BMA180_CD2			0x2D
#define BMA180_CD1			0x2C
#define BMA180_SLOPE_TH			0x2B
#define BMA180_HIGH_TH			0x2A
#define BMA180_LOW_TH			0x29
#define BMA180_TAPSENS_TH		0x28
#define BMA180_HIGH_DUR			0x27
#define BMA180_LOW_DUR			0x26
#define BMA180_HIGH_LOW_INFO		0x25
#define BMA180_SLOPE_TAPSENS_INFO	0x24
#define BMA180_HY			0x23
#define BMA180_CTRL_REG4		0x22
#define BMA180_CTRL_REG3		0x21
#define BMA180_BW_TCS			0x20
#define BMA180_RESET			0x10
#define BMA180_CTRL_REG2		0x0F
#define BMA180_CTRL_REG1		0x0E
#define BMA180_CTRL_REG0		0x0D
#define BMA180_STATUS_REG4		0x0C
#define BMA180_STATUS_REG3		0x0B
#define BMA180_STATUS_REG2		0x0A
#define BMA180_STATUS_REG1		0x09
#define BMA180_TEMP			0x08
#define BMA180_ACC_Z_MSB		0x07
#define BMA180_ACC_Z_LSB		0x06
#define BMA180_ACC_Y_MSB		0x05
#define BMA180_ACC_Y_LSB		0x04
#define BMA180_ACC_X_MSB		0x03
#define BMA180_ACC_X_LSB		0x02
#define BMA180_VERSION			0x01
#define BMA180_CHIP_ID			0x00


struct bma180_accel_data {
	struct bma180accel_platform_data *pdata;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *wq;
	struct delayed_work worklogic;
	struct mutex mutex;

	bool interrupts_enabled;

	uint8_t method;
	uint32_t def_poll_rate;
	uint16_t fuzz_x;
	uint16_t fuzz_y;
	uint16_t fuzz_z;
	uint16_t bandwidth;
	uint8_t g_range;
	uint8_t mode;
	uint8_t bit_mode;
	uint8_t smp_skip;
	uint16_t LSB_per_half_range;

	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temperature;
};

static uint32_t accl_debug;
module_param_named(bma180_debug, accl_debug, uint, 0664);

static int g_range_table[7] = {
	1000,
	1500,
	2000,
	3000,
	4000,
	8000,
	16000,
};
#ifdef BMA180_DEBUG
struct bma180_reg {
	const char *name;
	uint8_t reg;
	int writeable;
} bma180_regs[] = {
	{ "CHIP_ID",       	BMA180_CHIP_ID, 0 },
	{ "VERSION",       	BMA180_VERSION, 0 },
	{ "X_LSB",        	BMA180_ACC_X_LSB, 0 },
	{ "X_MSB",       	BMA180_ACC_X_MSB, 0 },
	{ "Y_LSB",       	BMA180_ACC_Y_LSB, 0 },
	{ "Y_MSB",       	BMA180_ACC_Y_MSB, 0 },
	{ "Z_LSB",       	BMA180_ACC_Z_LSB, 0 },
	{ "Z_MSB",       	BMA180_ACC_Z_MSB, 0 },
	{ "TEMP",       	BMA180_TEMP, 0 },
	{ "STATUS1",       	BMA180_STATUS_REG1, 0 },
	{ "STATUS2",       	BMA180_STATUS_REG2, 0 },
	{ "STATUS3",       	BMA180_STATUS_REG3, 0 },
	{ "STATUS4",       	BMA180_STATUS_REG4, 0 },
	{ "CTRL0",       	BMA180_CTRL_REG0, 1 },
	{ "CTRL1",       	BMA180_CTRL_REG1, 1 },
	{ "CTRL2",       	BMA180_CTRL_REG2, 1 },
	{ "RESET",       	BMA180_RESET, 1 },
	{ "BW_TCS",       	BMA180_BW_TCS, 1 },
	{ "CTRL3",       	BMA180_CTRL_REG3, 1 },
	{ "CTRL4",       	BMA180_CTRL_REG4, 1 },
	{ "HY",       		BMA180_HY, 1 },
	{ "TAP_INFO",       	BMA180_SLOPE_TAPSENS_INFO, 1 },
	{ "HI_LOW_INFO",       	BMA180_HIGH_LOW_INFO, 1 },
	{ "LOW_DUR",       	BMA180_LOW_DUR, 1 },
	{ "HIGH_DUR",       	BMA180_HIGH_DUR, 1 },
	{ "TAP_THRESH",       	BMA180_TAPSENS_TH, 1 },
	{ "LOW_THRESH",       	BMA180_LOW_TH, 1 },
	{ "HIGH_THRESH",       	BMA180_HIGH_TH, 1 },
	{ "SLOPE_THRESH",       BMA180_SLOPE_TH, 1 },
	{ "CD1",       		BMA180_CD1, 1 },
	{ "CD2",       		BMA180_CD2, 1 },
	{ "TCO_X",       	BMA180_TCO_X, 1 },
	{ "TCO_Y",       	BMA180_TCO_Y, 1 },
	{ "TCO_Z",       	BMA180_TCO_Z, 1 },
	{ "GAIN_T",       	BMA180_GAIN_T, 1 },
	{ "GAIN_X",       	BMA180_GAIN_X, 1 },
	{ "GAIN_Y",       	BMA180_GAIN_Y, 1 },
	{ "GAIN_Z",       	BMA180_GAIN_Z, 1 },
	{ "OFFSET_LSB1",       	BMA180_OFFSET_LSB1, 1 },
	{ "OFFSET_LSB2",       	BMA180_OFFSET_LSB2, 1 },
	{ "OFFSET_T",       	BMA180_OFFSET_T, 1 },
	{ "OFFSET_X",       	BMA180_OFFSET_X, 1 },
	{ "OFFSET_Y",       	BMA180_OFFSET_Y, 1 },
	{ "OFFSET_Z",       	BMA180_OFFSET_Z, 1 },
};
#endif

static int bma180_write(struct bma180_accel_data *data, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(data->client, reg, val);
	if (ret < 0)
		dev_err(&data->client->dev,
			"i2c_smbus_write_byte_data failed\n");
	return ret;
}

static int bma180_read(struct bma180_accel_data *data, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(data->client, reg);
	if (ret < 0)
		dev_err(&data->client->dev,
			"i2c_smbus_read_byte_data failed\n");
	return ret;
}

static irqreturn_t bma180_accel_thread_irq(int irq, void *dev_data)
{
	struct bma180_accel_data *data = (struct bma180_accel_data *) dev_data;

	if (!data->interrupts_enabled)
		return IRQ_NONE;

	if (data->wq != NULL)
		queue_delayed_work(data->wq, &data->worklogic, 0);

	else
		return IRQ_NONE;

	return IRQ_HANDLED;
}

static int bma180_accel_data_ready(struct bma180_accel_data *data)
{
	uint8_t data_val_h, data_val_l;

	/* get temperature data and save it on platform data struct */
	data_val_l = bma180_read(data, BMA180_TEMP);
	data->temperature = (short int) data_val_l;

	/* get accel x data and save it on platform data struct */
	data_val_l = bma180_read(data, BMA180_ACC_X_LSB);
	data_val_h = bma180_read(data, BMA180_ACC_X_MSB);
	data->accel_x = (short int) ((data_val_h << 8) | data_val_l);
	data->accel_x = (data->accel_x >> 2);

	/* get accel y data and save it on platform data struct */
	data_val_l = bma180_read(data, BMA180_ACC_Y_LSB);
	data_val_h = bma180_read(data, BMA180_ACC_Y_MSB);
	data->accel_y = (short int) ((data_val_h << 8) | data_val_l);
	data->accel_y = (data->accel_y >> 2);

	/* get accel z data and save it on platform data struct */
	data_val_l = bma180_read(data, BMA180_ACC_Z_LSB);
	data_val_h = bma180_read(data, BMA180_ACC_Z_MSB);
	data->accel_z = (short int) ((data_val_h << 8) | data_val_l);
	data->accel_z = (data->accel_z >> 2);

	/* send input report */
	input_report_abs(data->input_dev, ABS_X,
		((int)data->accel_x)*g_range_table[data->g_range]/data->LSB_per_half_range);
	input_report_abs(data->input_dev, ABS_Y,
		((int)data->accel_y)*g_range_table[data->g_range]/data->LSB_per_half_range);
	input_report_abs(data->input_dev, ABS_Z,
		((int)-data->accel_z)*g_range_table[data->g_range]/data->LSB_per_half_range);
	input_sync(data->input_dev);

	return 0;
}

void bma180_accel_device_worklogic(struct work_struct *work)
{
	struct bma180_accel_data *data = container_of((struct work_struct *)work,
				struct bma180_accel_data, worklogic.work);

	bma180_accel_data_ready(data);

	if (data->method == BMA_METHOD_POLLING)
		queue_delayed_work(data->wq, &data->worklogic,
			msecs_to_jiffies(data->def_poll_rate));

}

static ssize_t bma180_show_attr_enable(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bma180_accel_data *data = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", data->mode);
}

static ssize_t bma180_store_attr_enable(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bma180_accel_data *data = platform_get_drvdata(pdev);
	unsigned long val;
	int error, enable;

	error = strict_strtoul(buf, 0, &val);
	if (error)
		return error;

	if (val == 0)
		enable = val;
	else
		enable = data->def_poll_rate;

	cancel_delayed_work_sync(&data->worklogic);

	if (enable)
		schedule_delayed_work(&data->worklogic, 0);

	return count;
}

static ssize_t bma180_show_attr_delay(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bma180_accel_data *data = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", data->def_poll_rate);
}

static ssize_t bma180_store_attr_delay(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bma180_accel_data *data = platform_get_drvdata(pdev);
	unsigned long interval;
	int error;

	error = strict_strtoul(buf, 0, &interval);
	if (error)
		return error;

	if (interval <= 0 || interval > 200)
		return -EINVAL;

	cancel_delayed_work_sync(&data->worklogic);
	data->def_poll_rate = interval;

	schedule_delayed_work(&data->worklogic, 0);

	return count;

}
#ifdef BMA180_DEBUG
static ssize_t bma180_registers_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bma180_accel_data *data = platform_get_drvdata(pdev);
	unsigned i, n, reg_count;
	uint8_t value;

	reg_count = sizeof(bma180_regs) / sizeof(bma180_regs[0]);
	for (i = 0, n = 0; i < reg_count; i++) {
		value = bma180_read(data, bma180_regs[i].reg);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "%-20s = 0x%02X\n",
			       bma180_regs[i].name,
			       value);
	}

	return n;
}

static ssize_t bma180_registers_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bma180_accel_data *data = platform_get_drvdata(pdev);
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

	reg_count = sizeof(bma180_regs) / sizeof(bma180_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, bma180_regs[i].name)) {
			if (bma180_regs[i].writeable) {
				error = bma180_write(data,
					bma180_regs[i].reg,
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
		bma180_registers_show, bma180_registers_store);
#endif
static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
		bma180_show_attr_enable, bma180_store_attr_enable);

static DEVICE_ATTR(delay, S_IWUSR | S_IRUGO,
		bma180_show_attr_delay, bma180_store_attr_delay);

static struct attribute *bma180_accel_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
#ifdef BMA180_DEBUG
	&dev_attr_registers.attr,
#endif
	NULL
};


static const struct attribute_group bma180_accel_attr_group = {
	.attrs = bma180_accel_attrs,
};

int bma180_accel_device_hw_reset(struct bma180_accel_data *data)
{
	/* write 0xB6 to this register to do a soft-reset */
	bma180_write(data, BMA180_RESET, 0xB6);

	return 0;
}

int bma180_accel_device_hw_reset_int(struct bma180_accel_data *data)
{
	uint8_t reg_val;

	/* read old value */
	reg_val = bma180_read(data, BMA180_CTRL_REG0);

	/* set bit 6 (reset_int) */
	reg_val |= 0x40;

	/* write new value */
	bma180_write(data, BMA180_CTRL_REG0, reg_val);

	return 0;
}

int bma180_accel_device_hw_set_grange(struct bma180_accel_data *data,
					int grange)
{
	uint8_t reg_val;

	/* read old value */
	reg_val = bma180_read(data, BMA180_OFFSET_LSB1);

	/* set bits 3:1 (grange) */
	reg_val = (reg_val & 0xF1) | ((grange << 1) & 0x0E);

	/* write new value */
	bma180_write(data, BMA180_OFFSET_LSB1, reg_val);

	return 0;
}

int bma180_accel_device_hw_set_smp_skip(struct bma180_accel_data *data,
					int smp_skip)
{
	uint8_t reg_val;

	/* read old value */
	reg_val = bma180_read(data, BMA180_OFFSET_LSB1);

	/* set bit 0 (smp_skip) */
	reg_val = (reg_val & 0xFE) | ((smp_skip << 0) & 0x01);

	/* write new value */
	bma180_write(data, BMA180_OFFSET_LSB1, reg_val);

	return 0;
}

int bma180_accel_device_hw_set_bandwidth(struct bma180_accel_data *data,
					int bandwidth)
{
	uint8_t reg_val;
	uint8_t int_val;

	/* save int register */
	int_val = bma180_read(data, BMA180_CTRL_REG3);

	/* clear int register */
	bma180_write(data, BMA180_CTRL_REG3, 0x00);

	/* read old value */
	reg_val = bma180_read(data, BMA180_BW_TCS);

	/* set bits 7:4 (bandwidth) */
	reg_val = (reg_val & 0x0F) | ((bandwidth << 4) & 0xF0);

	/* write new value */
	bma180_write(data, BMA180_BW_TCS, reg_val);
	msleep(10);
	bma180_write(data, BMA180_CTRL_REG3, int_val);

	return 0;
}

int bma180_accel_device_hw_set_mode(struct bma180_accel_data *data, int mode)
{
	uint8_t reg_val;

	/* read old value */
	reg_val = bma180_read(data, BMA180_TCO_Z);

	/* set bits 1:0 (mode) */
	reg_val = (reg_val & 0xFC) | ((mode << 0) & 0x03);

	/* write new value */
	bma180_write(data, BMA180_TCO_Z, reg_val);

	return 0;
}


int bma180_accel_device_hw_set_12bits(struct bma180_accel_data *data, int mode)
{
	uint8_t reg_val;

	/* read old value */
	reg_val = bma180_read(data, BMA180_OFFSET_T);

	/* set bit 0 (readout_12bit) */
	reg_val = (reg_val & 0xFE) | ((mode << 0) & 0x01);

	/* write new value */
	bma180_write(data, BMA180_OFFSET_T, reg_val);

	return 0;
}

int bma180_accel_device_hw_init(struct bma180_accel_data *data)
{
	int ret = 0;

	bma180_accel_device_hw_reset(data);
	msleep(1);
	bma180_write(data, BMA180_CTRL_REG0, 0x11);
	bma180_write(data, BMA180_CTRL_REG3, 0x00);
	bma180_write(data, BMA180_HIGH_LOW_INFO, 0x00);
	bma180_write(data, BMA180_SLOPE_TAPSENS_INFO, 0x00);
	bma180_write(data, BMA180_GAIN_Y, 0xA9);
	bma180_write(data, BMA180_HIGH_DUR, 0x00);
	bma180_accel_device_hw_set_grange(data, data->g_range);
	bma180_accel_device_hw_set_bandwidth(data, data->bandwidth);
	bma180_accel_device_hw_set_mode(data, data->mode);
	bma180_accel_device_hw_set_smp_skip(data, data->smp_skip);
	bma180_accel_device_hw_set_12bits(data, data->bit_mode);
	bma180_write(data, BMA180_CTRL_REG3, 0x02);

	/* enable interrupt handling */
	data->interrupts_enabled = 1;

	/* reset status */
	bma180_accel_device_hw_reset_int(data);

	return ret;
}

/* init device function */
int bma180_accel_device_init(struct bma180_accel_data *data)
{
	int ret = 0;

	/* init work queue for interrupt service routine */
	INIT_DELAYED_WORK(&data->worklogic, bma180_accel_device_worklogic);

	/* init variables */
	data->method = data->pdata->method;
	data->g_range = data->pdata->g_range;
	data->bandwidth = data->pdata->bandwidth;
	data->mode = data->pdata->mode;
	data->bit_mode = data->pdata->bit_mode;
	data->smp_skip = data->pdata->smp_skip;
	data->def_poll_rate = data->pdata->def_poll_rate;
	data->fuzz_x = data->pdata->fuzz_x;
	data->fuzz_y = data->pdata->fuzz_y;
	data->fuzz_z = data->pdata->fuzz_z;

	/* set LSB per half range variable */
	if (data->bit_mode == BMA_BITMODE_14BITS)
		data->LSB_per_half_range = 8192;
	else
		data->LSB_per_half_range = 2048;


	/* set interrupt service routine */
	if (data->method == BMA_METHOD_INTERRUPTS) {
		mutex_init(&data->mutex);
		if (data->client->irq) {
			ret = request_threaded_irq(gpio_to_irq(data->client->irq), NULL,
						bma180_accel_thread_irq,
						IRQF_TRIGGER_RISING | IRQF_ONESHOT,
						data->client->name, data);
			if (ret < 0) {
				dev_err(&data->client->dev,
					"request_threaded_irq failed\n");
				mutex_destroy(&data->mutex);
				goto error;
			}
		}
		mutex_destroy(&data->mutex);
	}

	bma180_accel_device_hw_init(data);

error:
	if (data->input_dev != NULL)
		input_free_device(data->input_dev);
	return ret;
}

int bma180_accel_device_deinit(struct bma180_accel_data *data)
{
	int ret = 0;

	if (data->client->irq)
		free_irq(data->client->irq, data);

	cancel_delayed_work_sync(&data->worklogic);

	if (data->input_dev != NULL)
		input_free_device(data->input_dev);

	return ret;
}

static int __devinit bma180_accel_driver_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct bma180accel_platform_data *pdata = client->dev.platform_data;
	struct bma180_accel_data *data;
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

	/* alloc memory for data structure */
	data = kzalloc(sizeof(struct bma180_accel_data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto error;
	}

	/* create worker thread for isr routine */
	data->wq = create_freezeable_workqueue("bma180");
	if (!data->wq) {
		ret = -ENOMEM;
		goto error_1;
	}

	data->pdata = pdata;
	data->client = client;
	i2c_set_clientdata(client, data);

	data->input_dev = input_allocate_device();
	if (data->input_dev == NULL) {
		ret = -ENOMEM;
		dev_err(&data->client->dev,
			"Failed to allocate input device\n");
		goto error;
	}

	data->input_dev->name = "bma180";
	data->input_dev->id.bustype = BUS_I2C;

	__set_bit(EV_ABS, data->input_dev->evbit);
	input_set_abs_params(data->input_dev, ABS_X,
				-g_range_table[data->g_range],
				g_range_table[data->g_range], data->fuzz_x, 0);
	input_set_abs_params(data->input_dev, ABS_Y,
				-g_range_table[data->g_range],
				g_range_table[data->g_range], data->fuzz_y, 0);
	input_set_abs_params(data->input_dev, ABS_Z,
				-g_range_table[data->g_range],
				g_range_table[data->g_range], data->fuzz_z, 0);

	data->input_dev->dev.parent = &data->client->dev;
	input_set_drvdata(data->input_dev, data);

	ret = input_register_device(data->input_dev);
	if (ret) {
		dev_err(&data->client->dev,
			"Unable to register input device\n");
	}

	ret = bma180_accel_device_init(data);
	if (ret)
		goto error_1;

	ret = sysfs_create_group(&client->dev.kobj, &bma180_accel_attr_group);
	if (ret)
		goto error_2;

	return 0;

error_2:
	bma180_accel_device_deinit(data);
error_1:
	if (data)
		kfree(data);
error:
	return ret;
}

static int __devexit bma180_accel_driver_remove(struct i2c_client *client)
{
	int ret;

	/* get private data structure pointer from i2c_client */
	struct bma180_accel_data *data = i2c_get_clientdata(client);

	/* remove sysfs group */
	sysfs_remove_group(&client->dev.kobj, &bma180_accel_attr_group);

	/* deinit device function */
	ret = bma180_accel_device_deinit(data);
	i2c_set_clientdata(client, NULL);
	flush_workqueue(data->wq);
	destroy_workqueue(data->wq);
	kfree(data);

	return ret;
}


#ifdef CONFIG_PM
static int bma180_accel_driver_suspend(struct i2c_client *client,
				pm_message_t mesg)
{
	/* TODO */
	return 0;
}

static int bma180_accel_driver_resume(struct i2c_client *client)
{
	/* TODO */
	return 0;
}
#endif

static const struct i2c_device_id bma180_accel_idtable[] = {
	{ DRIVER_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, bma180_accel_idtable);

static struct i2c_driver bma180_accel_driver = {
	.probe		= bma180_accel_driver_probe,
	.remove		= bma180_accel_driver_remove,
	.id_table	= bma180_accel_idtable,
#ifdef CONFIG_PM
	.suspend	= bma180_accel_driver_suspend,
	.resume		= bma180_accel_driver_resume,
#endif
	.driver = {
		.name = DRIVER_NAME
	},
};

static int __init bma180_accel_driver_init(void)
{
	return i2c_add_driver(&bma180_accel_driver);
}

static void __exit bma180_accel_driver_exit(void)
{
	i2c_del_driver(&bma180_accel_driver);
}

module_init(bma180_accel_driver_init);
module_exit(bma180_accel_driver_exit);

MODULE_DESCRIPTION("BMA-180 Accelerometer Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dan Murphy <DMurphy@ti.com>");
