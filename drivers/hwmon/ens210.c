/*
 * ams ENS210 relative humidity and temperature sensor driver
 *
 * Copyright (C) 2018 Richard Lai <richard@richardman.com>
 *
 * Datasheet available at: http://ams.com/eng/Products/Environmental-Sensors/Relative-Humidity-and-Temperature-Sensors/ENS210
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/page.h>
#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>


#define ENS210_REG_PART_ID     0x00
#define ENS210_REG_UID         0x04
#define ENS210_REG_SYS_CTRL    0x10
#define ENS210_REG_SYS_STAT    0x11
#define ENS210_REG_SENS_RUN    0x21
#define ENS210_REG_SENS_START  0x22
#define ENS210_REG_SENS_STOP   0x23
#define ENS210_REG_SENS_STAT   0x24
#define ENS210_REG_T_VAL       0x30
#define ENS210_REG_H_VAL       0x33

#define ENS210_SYS_CTRL_LOW_POWER_DISABLE  0x00
#define ENS210_SYS_CTRL_LOW_POWER_ENABLE   0x01
#define ENS210_SYS_CTRL_RESET              0x80

#define ENS210_SYS_ACTIVE_STATE_STANDBY    0x00
#define ENS210_SYS_ACTIVE_STATE_ACTIVE     0x01

#define ENS210_SENS_RUN_T_SINGLE_SHOT      0x00
#define ENS210_SENS_RUN_T_CONTINUOUS       0x01
#define ENS210_SENS_RUN_H_SINGLE_SHOT      0x00
#define ENS210_SENS_RUN_H_CONTINUOUS       0x02

#define ENS210_SENS_START_T                0x01
#define ENS210_SENS_START_H                0x02

#define ENS210_SENS_STOP_T                 0x01
#define ENS210_SENS_STOP_H                 0x02

#define ENS210_SENS_STAT_T_IDLE            0x00
#define ENS210_SENS_STAT_T_ACTIVE          0x01
#define ENS210_SENS_STAT_H_IDLE            0x00
#define ENS210_SENS_STAT_H_ACTIVE          0x02
#define ENS210_SENS_STAT_H_ACTIVE          0x02

#define ENS210_VAL_DATA(val)	           (0x7ffff & (val))
#define ENS210_VAL_VALID_MASK              0x8000
#define ENS210_VAL_CRC                     0x02

#define ENS210_CONVERSION_TIME_T_RH_MAX    130 /* ms */
#define ENS210_CONVERSION_TIME_T_MAX       110 /* ms */

#define ENS210_BOOTING_US		1200
#define ENS210_BOOTING_US_MAX		1600

#define ENS210_VAL_LENGTH          3

/* CRC macros */
#define ENS210_CRC7_WIDTH		7
#define ENS210_CRC7_POLY			0x89
#define ENS210_CRC7_IVEC			0x7F
#define ENS210_DATA7WIDTH		17
#define ENS210_CRC7_SHIFT		17
/* 0b 1 1111 1111 1111 1111 */
#define ENS210_DATA7MASK		((1 << ENS210_DATA7WIDTH) - 1)
/* 0b 1 0000 0000 0000*/
#define ENS210_DATA7MSB			(1 << (ENS210_DATA7WIDTH - 1))

#define ENS210_MASK_DATA		0x1FFFF
#define ENS210_BOOTING_US		1200
#define ENS210_BOOTING_US_MAX		1600    /* Here Max booting time is inclusive of latencies */
#define ENS210_VALID_BIT_MASK		0x10000
#define ENS210_RAW_DATA_MASK		0xFFFF
#define ENS210_CHIP_PARTID		0x0210

enum ens210_chips {
	ens210,
};

struct ens210_data {
	struct i2c_client *client;
	struct mutex data_lock; /* lock for updating driver data */
	bool valid;
	unsigned long last_update;
	int temperature;
	int humidity;
};

static int ens210_init(struct i2c_client *client)
{
	//struct ens210_data *data = i2c_get_clientdata(client);
	int ret;
	u8 buff[I2C_SMBUS_BLOCK_MAX];

	/* Configure low power mode */
	ret = i2c_smbus_write_byte_data(client, ENS210_REG_SYS_CTRL, ENS210_SYS_CTRL_RESET);
	if (ret)
		goto out;

	usleep_range(ENS210_BOOTING_US, ENS210_BOOTING_US_MAX);
	
	/* Disable Low Power Mode */
	ret = i2c_smbus_write_byte_data(client, ENS210_REG_SYS_CTRL, ENS210_SYS_CTRL_LOW_POWER_DISABLE);
	if (ret)
		goto out;

	usleep_range(ENS210_BOOTING_US, ENS210_BOOTING_US_MAX);

	/* Read and check PART_ID = 0x0210 */
	ret = i2c_smbus_read_word_data(client, ENS210_REG_PART_ID);
	if (ret < 0)
		goto out;

	if (ret != ENS210_CHIP_PARTID) {
		dev_err(&client->dev, "Invalid ENS210 sensor\n");
		goto out;
	}

	/* Read UID */
	ret = i2c_smbus_read_i2c_block_data(client, ENS210_REG_UID, 8, &buff[0]);
	if (ret != 8)
		goto out;

	//pr_debug("%s : UID_WORD: 0x%016LX", __func__, data->uid);

	/* Enable Low Power Mode */
	ret = i2c_smbus_write_byte_data(client, ENS210_REG_SYS_CTRL, ENS210_SYS_CTRL_LOW_POWER_ENABLE);
	if (ret)
		goto out;

	usleep_range(ENS210_BOOTING_US, ENS210_BOOTING_US_MAX);

out:

	return ret;
}

static uint32_t ens210_compute_crc7(uint32_t val)
{
	/* Setup polynomial */
	uint32_t pol = ENS210_CRC7_POLY;
	/* Align polynomial with data */
	/* Loop variable (indicates which bit to test, start with highest) */
	uint32_t bit = ENS210_DATA7MSB;
	pol = pol << (ENS210_DATA7WIDTH - ENS210_CRC7_WIDTH - 1);
	/* Make room for CRC value */
	val = val << ENS210_CRC7_WIDTH;
	bit = bit << ENS210_CRC7_WIDTH;
	pol = pol << ENS210_CRC7_WIDTH;
	/* Insert initial vector */
	val |= ENS210_CRC7_IVEC;
	/* Apply division until all bits done */
	while (bit & (ENS210_DATA7MASK << ENS210_CRC7_WIDTH)) {
		if (bit & val)
			val ^= pol;
		bit >>= 1;
		pol >>= 1;
	}
	return val;
}

bool ens210_raw_is_crc_ok(uint32_t raw)
{
	uint32_t crc, data;
	bool ret;

	crc  = (raw >> ENS210_CRC7_SHIFT) & 0x7F;
	data = raw & ENS210_MASK_DATA;
	ret = ens210_compute_crc7(data) == crc;
	if(ret < 0) {
		pr_err("%s : CRC failed\n", __func__);
	}
	return ret;
}

static int ens210_extract_data(uint8_t *rawbuf, int *data)
{
	int ret = 0;
	uint32_t raw;

	raw = (rawbuf[2] << 16)|(rawbuf[1] << 8) | (rawbuf[0]);

	if (!ens210_raw_is_crc_ok(raw)) 
		return -EINVAL;

	if (!(raw & ENS210_VALID_BIT_MASK)) {
		pr_err("%s : Raw data is Invalid raw values\n", __func__);
		return -EINVAL;
	}

	*data = raw & ENS210_RAW_DATA_MASK;

	return ret;
}

static int ens210_update_measurements(struct device *dev)
{
	struct ens210_data *data = dev_get_drvdata(dev);
	//struct i2c_client *client = data->client;
	unsigned char buf[ENS210_VAL_LENGTH];
	int ret = 0;

	mutex_lock(&data->data_lock);

	if (ret < 0)
		goto out;

	msleep(ENS210_CONVERSION_TIME_T_RH_MAX);

	/* TODO: Read Measurement - ENS210_REG_T_VAL*/

	ens210_extract_data(&buf[0], &data->temperature);
	
	/* TODO: Read Measurement - ENS210_REG_H_VAL*/

	ens210_extract_data(&buf[0], &data->humidity);

	data->last_update = jiffies;

out:
	mutex_unlock(&data->data_lock);

	return ret < 0 ? ret : 0;
}

static ssize_t ens210_show_temperature(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ens210_data *data = dev_get_drvdata(dev);
	int ret = ens210_update_measurements(dev);

	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", data->temperature);
}

static ssize_t ens210_show_humidity(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct ens210_data *data = dev_get_drvdata(dev);
	int ret = ens210_update_measurements(dev);

	if (ret < 0)
		return ret;

	return sprintf(buf, "%u\n", data->humidity);
}

/* sysfs attributes */
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, ens210_show_temperature, NULL, 0);
static SENSOR_DEVICE_ATTR(humidity1_input, S_IRUGO, ens210_show_humidity,
			  NULL, 0);

static struct attribute *ens210_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_humidity1_input.dev_attr.attr,
	NULL
};

ATTRIBUTE_GROUPS(ens210);

static int ens210_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ens210_data *data;
	struct device *hwmon_dev;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "adapter does not support true I2C\n");
		return -ENODEV;
	}

	if (ens210_init(client))
		return -ENODEV;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;

	mutex_init(&data->data_lock);

	hwmon_dev = devm_hwmon_device_register_with_groups(dev,
							   client->name,
							   data,
							   ens210_groups);

	if (IS_ERR(hwmon_dev))
		dev_dbg(dev, "unable to register hwmon device\n");

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

/* device ID table */
static const struct i2c_device_id ens210_id[] = {
	{"ens210", ens210},
	{}
};

MODULE_DEVICE_TABLE(i2c, ens210_id);

static struct i2c_driver ens210_i2c_driver = {
	.driver.name = "ens210",
	.probe       = ens210_probe,
	.id_table    = ens210_id,
};

module_i2c_driver(ens210_i2c_driver);

MODULE_AUTHOR("Richard Lai <richard.lai@ams.com");
MODULE_DESCRIPTION("ams ENS210 Relative Humidity and Temperature Sensor");
MODULE_LICENSE("GPL");
