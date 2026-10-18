/*
 * Goodix GT9xx touchscreen driver
 *
 * Copyright (C) 2016 - 2017 Goodix. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "gt9xx.h"

int gtp_int_gpio = -1;
int gtp_rst_gpio = -1;
struct i2c_client *i2c_connect_client = NULL;
static struct workqueue_struct *goodix_wq;
static struct workqueue_struct *gtp_esd_check_workqueue;
static struct delayed_work gtp_esd_check_work;
static struct proc_dir_entry *gt91xx_config_proc;
static u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH] = {0x80, 0x47};

static const u16 touch_key_array[] = GTP_KEY_TAB;

/* Forward declarations */
static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int goodix_ts_remove(struct i2c_client *client);
static int gtp_power_switch(struct i2c_client *client, int on);
static int gtp_i2c_test(struct i2c_client *client);
static void goodix_ts_work_func(struct work_struct *work);
static void gtp_pen_up(void);
static ssize_t gt91xx_config_read_proc(struct file *file, char __user *page,
				       size_t size, loff_t *ppos);
static ssize_t gt91xx_config_write_proc(struct file *filp,
					const char __user *buffer,
					size_t count, loff_t *off);
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id);
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer);
static int gtp_fb_notifier_callback(struct notifier_block *noti,
				    unsigned long event, void *data);
static void gtp_esd_check_func(struct work_struct *work);

static const struct file_operations config_proc_ops = {
	.owner = THIS_MODULE,
	.read = gt91xx_config_read_proc,
	.write = gt91xx_config_write_proc,
};

static const struct of_device_id goodix_match_table[] = {
	{ .compatible = "goodix,gt9xx", },
	{ }
};
MODULE_DEVICE_TABLE(of, goodix_match_table);

static const struct i2c_device_id goodix_ts_id[] = {
	{ GTP_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, goodix_ts_id);

static struct i2c_driver goodix_ts_driver = {
	.probe		= goodix_ts_probe,
	.remove		= goodix_ts_remove,
	.id_table	= goodix_ts_id,
	.driver = {
		.name	= GTP_I2C_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = goodix_match_table,
	},
};

/* 1. gtp_i2c_read */
int gtp_i2c_read(struct i2c_client *client, u8 *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret = -EIO;
	int retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr = client->addr;
	msgs[0].len = 2;
	msgs[0].buf = buf;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].len = len - 2;
	msgs[1].buf = &buf[2];

	while (1) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		if (++retries > 2) {
			printk("<<-GTP-ERROR->> I2C Read: 0x%04X, %d bytes failed, errcode: %d! Process reset.\n",
				(buf[0] << 8) | buf[1], len - 2, ret);
			gtp_reset_guitar(client, 10);
			return ret;
		}
	}
	return ret;
}

/* 2. gtp_reset_guitar */
void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
	u8 buf[3];

	printk("<<-GTP-INFO->> Guitar reset\n");
	gpio_direction_output(gtp_rst_gpio, 0);
	msleep(ms);
	gpio_direction_output(gtp_int_gpio, client->addr == 0x14);
	msleep(2);
	gpio_direction_output(gtp_rst_gpio, 1);
	msleep(6);
	gpio_direction_input(gtp_rst_gpio);
	gtp_int_sync(50);
	buf[0] = 0x80;
	buf[1] = 0x41;
	buf[2] = 0xAA;
	gtp_i2c_write_no_rst(client, buf, 3);
}

/* 3. gtp_i2c_write */
int gtp_i2c_write(struct i2c_client *client, u8 *buf, int len)
{
	struct i2c_msg msg;
	int ret = -EIO;
	int retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr = client->addr;
	msg.len = len;
	msg.buf = buf;

	while (1) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		if (++retries > 4) {
			printk("<<-GTP-ERROR->> I2C Write: 0x%04X, %d bytes failed, errcode: %d! Process reset.\n",
				(buf[0] << 8) | buf[1], len - 2, ret);
			gtp_reset_guitar(client, 10);
			return ret;
		}
	}
	return ret;
}

/* 4. gtp_i2c_read_dbl_check */
s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
	u8 confirm_buf1[len + 2];
	u8 confirm_buf2[len + 2];
	u8 retries = 0;

	memset(confirm_buf1, 0, len + 2);
	memset(confirm_buf2, 0, len + 2);

	while (retries <= 2) {
		retries++;
		memset(confirm_buf1, 0xaa, len + 2);
		confirm_buf1[0] = (addr >> 8) & 0xff;
		confirm_buf1[1] = addr & 0xff;
		gtp_i2c_read(client, confirm_buf1, len + 2);

		memset(confirm_buf2, 0xab, len + 2);
		confirm_buf2[0] = (addr >> 8) & 0xff;
		confirm_buf2[1] = addr & 0xff;
		gtp_i2c_read(client, confirm_buf2, len + 2);

		if (!memcmp(confirm_buf1, confirm_buf2, len + 2)) {
			memcpy(rxbuf, &confirm_buf2[2], len);
			return 1;
		}
	}
	printk("<<-GTP-ERROR->> I2C read 0x%04X, %d bytes, double check failed!\n", addr, len);
	return 0;
}

/* 5. gtp_send_cfg */
s32 gtp_send_cfg(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	int ret = 0;
	unsigned int retry;

	if (ts->pnl_init_error) {
		printk("<<-GTP-INFO->> Error occured in init_panel, no config sent\n");
		return 0;
	}

	printk("<<-GTP-INFO->> Driver send config.\n");
	retry = 1;
	do {
		ret = gtp_i2c_write(client, config, GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
		if (ret > 0)
			break;
	} while (retry++ < 5);

	printk("=============cjc function = %s,line = %d=======\n", "gtp_send_cfg", 397);
	return ret;
}

/* 6. gtp_irq_disable */
void gtp_irq_disable(struct goodix_ts_data *ts)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&ts->irq_lock.rlock, flags);
	if (!ts->irq_is_disable) {
		ts->irq_is_disable = 1;
		disable_irq_nosync(ts->client->irq);
	}
	raw_spin_unlock_irqrestore(&ts->irq_lock.rlock, flags);
}

/* 7. gtp_irq_enable */
void gtp_irq_enable(struct goodix_ts_data *ts)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&ts->irq_lock.rlock, flags);
	if (ts->irq_is_disable) {
		enable_irq(ts->client->irq);
		ts->irq_is_disable = 0;
	}
	raw_spin_unlock_irqrestore(&ts->irq_lock.rlock, flags);
}

/* 8. gtp_int_sync */
void gtp_int_sync(s32 ms)
{
	gpio_direction_output(gtp_int_gpio, 0);
	msleep(ms);
	gpio_direction_input(gtp_int_gpio);
}

/* 9. gtp_read_version */
int gtp_read_version(struct i2c_client *client, u16 *version)
{
	u8 buf[8];
	int ret;

	buf[0] = 0x81;
	buf[1] = 0x40;
	ret = gtp_i2c_read(client, buf, sizeof(buf));
	if (ret < 0) {
		printk("<<-GTP-ERROR->> GTP read version failed\n");
	} else {
		if (version)
			*version = (buf[7] << 8) | buf[6];
		if (buf[5])
			printk("<<-GTP-INFO->> IC Version: %c%c%c%c_%02x%02x\n",
				buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
		else
			printk("<<-GTP-INFO->> IC Version: %c%c%c_%02x%02x\n",
				buf[2], buf[3], buf[4], buf[7], buf[6]);
	}
	return ret;
}

/* 10. gtp_parse_dt_cfg */
int gtp_parse_dt_cfg(struct device *dev, u8 *cfg, int *cfg_len, u8 sid)
{
	struct property *prop;
	char propname[20];

	snprintf(propname, 18, "goodix,cfg-group%d", sid);
	prop = of_find_property(dev->of_node, propname, cfg_len);
	if (!prop || !prop->value)
		return -1;
	if (*cfg_len && *cfg_len <= GTP_CONFIG_MAX_LENGTH) {
		memcpy(cfg, prop->value, *cfg_len);
		return 0;
	}
	return -1;
}

/* 11. gtp_i2c_read_no_rst */
int gtp_i2c_read_no_rst(struct i2c_client *client, u8 *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret = -EIO;
	int retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr = client->addr;
	msgs[0].len = 2;
	msgs[0].buf = buf;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].len = len - 2;
	msgs[1].buf = &buf[2];

	while (1) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		if (++retries > 4) {
			printk("<<-GTP-ERROR->> I2C Read: 0x%04X, %d bytes failed, errcode: %d!\n",
				(buf[0] << 8) | buf[1], len - 2, ret);
			return ret;
		}
	}
	return ret;
}

/* 12. gtp_i2c_write_no_rst */
int gtp_i2c_write_no_rst(struct i2c_client *client, u8 *buf, int len)
{
	struct i2c_msg msg;
	int ret = -EIO;
	int retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr = client->addr;
	msg.len = len;
	msg.buf = buf;

	while (1) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		if (++retries > 4) {
			printk("<<-GTP-ERROR->> I2C Write: 0x%04X, %d bytes failed, errcode: %d!\n",
				(buf[0] << 8) | buf[1], len - 2, ret);
			return ret;
		}
	}
	return ret;
}

/* 13. gtp_esd_switch */
void gtp_esd_switch(struct i2c_client *client, int on)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	spin_lock(&ts->esd_lock);
	if (on == 1) {
		if (!ts->esd_running) {
			ts->esd_running = 1;
			spin_unlock(&ts->esd_lock);
			printk("<<-GTP-INFO->> Esd started\n");
			queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
			return;
		}
	} else if (ts->esd_running) {
		ts->esd_running = 0;
		spin_unlock(&ts->esd_lock);
		printk("<<-GTP-INFO->> Esd cancelled\n");
		cancel_delayed_work_sync(&gtp_esd_check_work);
		return;
	}
	spin_unlock(&ts->esd_lock);
}

/* 14. goodix_ts_probe */
static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct goodix_ts_data *ts;
	u16 version;
	u8 sensor_id = 0;
	u8 opr_buf[2] = {0};
	u8 cfg_info_group0[] = CTP_CFG_GROUP0;
	u8 cfg_info_group1[] = CTP_CFG_GROUP1;
	u8 cfg_info_group2[] = CTP_CFG_GROUP2;
	u8 cfg_info_group3[] = CTP_CFG_GROUP3;
	u8 cfg_info_group4[] = CTP_CFG_GROUP4;
	u8 cfg_info_group5[] = CTP_CFG_GROUP5;
	u8 *send_cfg_buf[] = {cfg_info_group0, cfg_info_group1, cfg_info_group2,
			cfg_info_group3, cfg_info_group4, cfg_info_group5};
	u8 cfg_info_len[] = { CFG_GROUP_LEN(cfg_info_group0),
			      CFG_GROUP_LEN(cfg_info_group1),
			      CFG_GROUP_LEN(cfg_info_group2),
			      CFG_GROUP_LEN(cfg_info_group3),
			      CFG_GROUP_LEN(cfg_info_group4),
			      CFG_GROUP_LEN(cfg_info_group5)};
	int i;
	int check_sum;
	u8 orig_cfg_ver;
	unsigned int trigger_types[] = {IRQF_TRIGGER_RISING, IRQF_TRIGGER_FALLING};

	printk("<<-GTP-INFO->> GTP Driver Version: %s\n", GTP_DRIVER_VERSION);
	printk("<<-GTP-INFO->> GTP I2C Address: 0x%02x\n", client->addr);

	i2c_connect_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("<<-GTP-ERROR->> I2C check functionality failed.\n");
		return -ENODEV;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		printk("<<-GTP-ERROR->> Alloc GFP_KERNEL memory failed.\n");
		return -ENOMEM;
	}

	if (client->dev.of_node) {
		gtp_rst_gpio = of_get_named_gpio_flags(client->dev.of_node, "goodix,reset-gpio", 0, NULL);
		if (gtp_rst_gpio < 0)
			printk("invalid reset-gpio in dt: %d", gtp_rst_gpio);
		gtp_int_gpio = of_get_named_gpio_flags(client->dev.of_node, "goodix,irq-gpio", 0, NULL);
		if (gtp_int_gpio < 0)
			printk("invalid irq-gpio in dt: %d", gtp_int_gpio);
	}

	if (gtp_power_switch(client, 1)) {
		printk("<<-GTP-ERROR->> GTP power on failed.\n");
		kfree(ts);
		return -EINVAL;
	}

	if ((gtp_int_gpio | gtp_rst_gpio) < 0) {
		printk("gtp_rst_gpio = %d, gtp_int_gpio = %d\n", gtp_rst_gpio, gtp_int_gpio);
		printk("rst or int is err\n");
		kfree(ts);
		return -EINVAL;
	}

	if (gtp_i2c_test(client) < 0) {
		printk("<<-GTP-ERROR->> I2C communication ERROR!\n");
		kfree(ts);
		return -EINVAL;
	}

	INIT_WORK(&ts->work, goodix_ts_work_func);
	ts->client = client;
	ts->clk_tick_cnt = 500;
	spin_lock_init(&ts->esd_lock);
	spin_lock_init(&ts->irq_lock);
	i2c_set_clientdata(client, ts);
	ts->rawdiff_mode = 0;

	ret = gpio_request(gtp_int_gpio, "GTP INT IRQ");
	if (ret < 0) {
		printk("<<-GTP-ERROR->> Failed to request GPIO:%d, ERRNO:%d\n", gtp_int_gpio, ret);
	} else {
		gpio_direction_input(gtp_int_gpio);
		client->irq = gpio_to_irq(gtp_int_gpio);
	}

	ret = gpio_request(gtp_rst_gpio, "GTP RST PORT");
	if (ret < 0) {
		printk("<<-GTP-ERROR->> Failed to request GPIO:%d, ERRNO:%d\n", gtp_rst_gpio, ret);
		ret = -ENODEV;
	}

	gpio_direction_input(gtp_rst_gpio);
	gtp_reset_guitar(ts->client, 20);
	if (ret < 0) {
		printk("gtp_reset_guitar is error\n");
		gpio_free(gtp_rst_gpio);
		gpio_free(gtp_int_gpio);
		kfree(ts);
		return ret;
	}

	if (gtp_i2c_test(client) < 0)
		printk("<<-GTP-ERROR->> I2C communication ERROR!\n");

	if (gtp_read_version(client, &version) < 0)
		printk("<<-GTP-ERROR->> Read version failed.\n");

	/* gtp_init_panel (inlined) */
	printk("=============cjc function = %s,line = %d=======\n", "gtp_init_panel", 1449);
	if (gtp_i2c_read_dbl_check(ts->client, 0x41e4, opr_buf, 1) == 1 && opr_buf[0] != 0xBE) {
		ts->fw_error = 1;
		printk("<<-GTP-ERROR->> Firmware error, no config sent!\n");
		goto pnl_init_error_out;
	}

	if (gtp_i2c_read_dbl_check(ts->client, 0x814A, &sensor_id, 1) != 1) {
		printk("<<-GTP-ERROR->> Failed to get sensor_id, No config sent!\n");
		ts->pnl_init_error = 1;
		goto pnl_init_error_out;
	}

	if (sensor_id >= 6) {
		printk("<<-GTP-ERROR->> Invalid sensor_id(0x%02X), No Config Sent!\n", sensor_id);
		ts->pnl_init_error = 1;
		goto pnl_init_error_out;
	}

	printk("<<-GTP-INFO->> Sensor_ID: %d\n", sensor_id);
	printk("===lwf2_config===Get config data from header file.\n");
	ts->cfg_len = cfg_info_len[sensor_id];
	memset(&config[GTP_ADDR_LENGTH + ts->cfg_len], 0, GTP_CONFIG_MAX_LENGTH - ts->cfg_len);
	memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id], ts->cfg_len);
	printk("<<-GTP-INFO->> Config group%d used,length: %d\n", sensor_id, ts->cfg_len);

	if (ts->cfg_len <= 185) {
		printk("<<-GTP-ERROR->> Config Group%d is INVALID CONFIG GROUP(Len: %d)! NO Config Sent! You need to check you header file CFG_GROUP section!\n",
			sensor_id, ts->cfg_len);
		ts->pnl_init_error = 1;
		goto pnl_init_error_out;
	}

	if (gtp_i2c_read_dbl_check(ts->client, 0x8047, opr_buf, 1) != 1) {
		printk("<<-GTP-ERROR->> Failed to get ic config version!No config sent!\n");
		goto pnl_init_error_out;
	}

	orig_cfg_ver = config[GTP_ADDR_LENGTH];
	if (opr_buf[0] <= 0x59 && opr_buf[0] > config[GTP_ADDR_LENGTH]) {
		config[GTP_ADDR_LENGTH] = 0;
	}

	check_sum = 0;
	if (ts->cfg_len >= 3) {
		if (ts->cfg_len != 3) {
			check_sum = config[GTP_ADDR_LENGTH] + 176;
			if (ts->cfg_len >= 5) {
				for (i = 4; i < ts->cfg_len; i++)
					check_sum += config[GTP_ADDR_LENGTH + i];
			}
		}
	}
	config[GTP_ADDR_LENGTH + ts->cfg_len] = -check_sum;

	if (ts->abs_x_max == 0 && ts->abs_y_max == 0) {
		ts->abs_x_max = (config[GTP_ADDR_LENGTH + 2] << 8) | config[GTP_ADDR_LENGTH + 1];
		ts->abs_y_max = (config[GTP_ADDR_LENGTH + 4] << 8) | config[GTP_ADDR_LENGTH + 3];
		ts->int_trigger_type = config[GTP_ADDR_LENGTH + 6] & 0x03;
	}

	if (gtp_send_cfg(ts->client) < 0)
		printk("<<-GTP-ERROR->> Send config error.\n");

	config[GTP_ADDR_LENGTH] = orig_cfg_ver;
	check_sum = 0;
	if (ts->cfg_len >= 3) {
		if (ts->cfg_len != 3) {
			for (i = 3; i < ts->cfg_len; i++)
				check_sum += config[GTP_ADDR_LENGTH + i];
			check_sum += orig_cfg_ver;
		}
	}
	config[GTP_ADDR_LENGTH + ts->cfg_len] = -check_sum;

	printk("<<-GTP-INFO->> X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x\n",
		ts->abs_x_max, ts->abs_y_max, ts->int_trigger_type);
	msleep(10);
	goto create_proc;

pnl_init_error_out:
	printk("<<-GTP-ERROR->> GTP init panel failed.\n");
	ts->abs_x_max = 1200;
	ts->abs_y_max = 1920;
	ts->int_trigger_type = 0;

create_proc:
	gt91xx_config_proc = proc_create(GT91XX_CONFIG_PROC_FILE, 0666, NULL, &config_proc_ops);
	if (gt91xx_config_proc)
		printk("<<-GTP-INFO->> create proc entry %s success\n", GT91XX_CONFIG_PROC_FILE);
	else
		printk("<<-GTP-ERROR->> create_proc_entry %s failed\n\n", GT91XX_CONFIG_PROC_FILE);

	gtp_esd_switch(ts->client, 1);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev) {
		ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
		input_mt_init_slots(ts->input_dev, 16, 0);

		for (i = 0; i < 7; i++)
			input_set_capability(ts->input_dev, EV_KEY, touch_key_array[i]);

		input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
		input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
		input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
		input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
		input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
		input_set_abs_params(ts->input_dev, ABS_MT_TOOL_TYPE, 0, 2, 0, 0);
		input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 1023, 0, 0);

		ts->input_dev->name = "goodix-ts";
		ts->input_dev->phys = "input/ts";
		ts->input_dev->id.bustype = BUS_I2C;
		ts->input_dev->id.vendor = 0xDEAD;
		ts->input_dev->id.product = 0xBEEF;
		ts->input_dev->id.version = 10427;

		if (!input_register_device(ts->input_dev)) {
			printk("<<-GTP-INFO->> Request input device for pen/stylus.\n");
			ts->pen_dev = input_allocate_device();
			if (ts->pen_dev) {
				ts->pen_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
				set_bit(BTN_TOUCH, ts->pen_dev->keybit);
				set_bit(BTN_TOOL_PEN, ts->pen_dev->keybit);
				input_set_abs_params(ts->pen_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
				input_set_abs_params(ts->pen_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
				input_set_abs_params(ts->pen_dev, ABS_PRESSURE, 0, 255, 0, 0);
				input_set_abs_params(ts->pen_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
				input_set_abs_params(ts->pen_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
				ts->pen_dev->name = "goodix-pen";
				ts->pen_dev->id.bustype = BUS_I2C;
				if (input_register_device(ts->pen_dev))
					printk("<<-GTP-ERROR->> Register %s input device failed\n", ts->pen_dev->name);
			} else {
				printk("<<-GTP-ERROR->> Failed to allocate input device for pen/stylus.\n");
			}
			goto req_irq;
		}
		printk("<<-GTP-ERROR->> Register %s input device failed\n", ts->input_dev->name);
	} else {
		printk("<<-GTP-ERROR->> Failed to allocate input device.\n");
	}
	printk("<<-GTP-ERROR->> GTP request input dev failed\n");

req_irq:
	ret = request_threaded_irq(client->irq, NULL, goodix_ts_irq_handler,
				   trigger_types[ts->int_trigger_type], client->name, ts);
	if (ret) {
		printk("<<-GTP-ERROR->> Request IRQ failed!ERRNO:%d.\n", ret);
		gpio_direction_input(gtp_int_gpio);
		gpio_free(gtp_int_gpio);
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = goodix_ts_timer_handler;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		printk("<<-GTP-INFO->> GTP works in polling mode.\n");
	} else {
		gtp_irq_disable(ts);
		ts->use_irq = 1;
		printk("<<-GTP-INFO->> GTP works in interrupt mode.\n");
	}

	if (ts->use_irq)
		gtp_irq_enable(ts);

	ts->fb_notif.notifier_call = gtp_fb_notifier_callback;
	fb_register_client(&ts->fb_notif);

	return 0;
}

/* 15. goodix_ts_remove */
static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	fb_unregister_client(&ts->fb_notif);
	if (gtp_esd_check_workqueue)
		destroy_workqueue(gtp_esd_check_workqueue);

	if (ts) {
		if (ts->use_irq) {
			gpio_direction_input(gtp_int_gpio);
			gpio_free(gtp_int_gpio);
			free_irq(client->irq, ts);
		} else {
			hrtimer_cancel(&ts->timer);
		}
	}

	printk("<<-GTP-INFO->> GTP driver removing...\n");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

/* 16. gtp_power_switch */
static int gtp_power_switch(struct i2c_client *client, int on)
{
	static struct regulator *vdd_ana;
	int ret;

	if (!vdd_ana) {
		vdd_ana = regulator_get(&client->dev, "tp");
		if (IS_ERR(vdd_ana)) {
			printk("<<-GTP-ERROR->> regulator get of vdd_ana failed\n");
			ret = (int)(long)vdd_ana;
			vdd_ana = NULL;
			return ret;
		}
	}
	if (on)
		return regulator_enable(vdd_ana);
	else
		return regulator_disable(vdd_ana);
}

/* 17. gtp_i2c_test */
static int gtp_i2c_test(struct i2c_client *client)
{
	u8 buf[3];
	int ret = 0;
	int retry = 0;

	buf[0] = 0x80;
	buf[1] = 0x47;
	buf[2] = 0;

	do {
		ret = gtp_i2c_read(client, buf, 3);
		if (ret > 0)
			break;
		printk("<<-GTP-ERROR->> GTP i2c test failed time %d.\n", ++retry);
		msleep(10);
	} while (retry < 3);

	return ret;
}

/* 18. goodix_ts_work_func */
static void goodix_ts_work_func(struct work_struct *work)
{
	struct goodix_ts_data *ts = container_of(work, struct goodix_ts_data, work);
	static u16 pre_touch = 0;
	static u8 pre_key = 0;
	static u8 pre_pen = 0;
	static u8 pre_finger = 0;
	u8 end_cmd[3] = {0x81, 0x4e, 0x00};
	u8 point_data[12 + 10 * 8] = {0};
	u8 more_points[10 * 8] = {0};
	u8 touch_num;
	int ret;
	int i;
	int key_val;
	int key_reported;
	int have_pen_event = 0;
	u16 cur_touch = 0;
	u8 pt_id = 0;
	int count = 0;

	point_data[0] = 0x81;
	point_data[1] = 0x4e;

	if (ts->rawdiff_mode)
		return;

	ret = gtp_i2c_read(ts->client, point_data, 12);
	if (ret < 0) {
		printk("<<-GTP-ERROR->> I2C transfer error. errno:%d\n \n", ret);
		return;
	}

	if (!point_data[2])
		goto exit_work_check;

	if (!(point_data[2] & 0x80))
		goto exit_work_end_cmd;

	touch_num = point_data[2] & 0x0f;
	if (touch_num > 10)
		goto exit_work_end_cmd;

	if (touch_num > 1) {
		more_points[0] = 0x81;
		more_points[1] = 0x58;
		gtp_i2c_read(ts->client, more_points, (touch_num * 8) - 6);
		memcpy(&point_data[12], &more_points[2], (touch_num * 8) - 8);
	}

	key_val = point_data[touch_num * 8 + 3];
	if (pre_key || key_val) {
		for (i = 0; i < 7; i++)
			input_event(ts->input_dev, EV_KEY, touch_key_array[i], (key_val & (1 << i)) != 0);
		key_reported = 1;
		if (pre_key != 32 && key_val != 32 && pre_key != 16 && key_val != 16 &&
		    key_val != 64 && pre_key != 64)
			touch_num = 0;
	} else {
		key_reported = 0;
	}
	pre_key = key_val;

	if (touch_num == 0 && pre_touch == 0) {
		if (key_reported)
			input_sync(ts->input_dev);
		goto exit_work_end_cmd;
	}

	if (touch_num) {
		pt_id = point_data[3];
		if (point_data[2] & 0x80) { // pen or finger
			u16 chk = (point_data[8] | (point_data[9] << 8));
			if (chk == 0) {
				if (pre_finger == 1) {
					pre_finger = 0;
					for (i = 0; i < 10; i++) {
						input_mt_slot(ts->input_dev, i);
						input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
					}
					key_reported = 1;
				}
				pre_pen = 1;
				pre_touch = touch_num;
				input_sync(ts->pen_dev);
				if (key_reported)
					input_sync(ts->input_dev);
				goto exit_work_end_cmd;
			}
			if (pre_pen == 1) {
				gtp_pen_up();
				pt_id = 9;
				pre_pen = 0;
				have_pen_event = 1;
			} else {
				have_pen_event = 0;
				pt_id = 9;
			}
			cur_touch = 512;
		} else {
			if (pre_pen == 1) {
				pre_touch = touch_num;
				if (key_reported)
					input_sync(ts->input_dev);
				goto exit_work_end_cmd;
			}
			have_pen_event = 0;
			cur_touch = 1 << point_data[3];
		}
	} else {
		if (pre_pen == 1) {
			gtp_pen_up();
			cur_touch = 0;
			pt_id = 0;
			pre_pen = 0;
			have_pen_event = 1;
		} else {
			cur_touch = 0;
			have_pen_event = 0;
			pt_id = 0;
		}
	}

	count = 0;
	for (i = 0; i < 10; i++) {
		u16 bit = (1 << i);
		if (cur_touch & bit) {
			pre_finger = 1;
			input_mt_slot(ts->input_dev, pt_id);
			input_mt_report_slot_state(ts->input_dev, pt_id == 9 ? MT_TOOL_PEN : MT_TOOL_FINGER, true);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, point_data[count * 8 + 8] | (point_data[count * 8 + 9] << 8));
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, point_data[count * 8 + 4] | (point_data[count * 8 + 5] << 8));
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, point_data[count * 8 + 6] | (point_data[count * 8 + 7] << 8));
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, point_data[count * 8 + 8] | (point_data[count * 8 + 9] << 8));
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, point_data[count * 8 + 8] | (point_data[count * 8 + 9] << 8));
			count++;
			pre_touch |= bit;
			if (touch_num > count) {
				pt_id = point_data[count * 8 + 3] & 0x0f;
				cur_touch |= (1 << pt_id);
			}
		} else {
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
			pre_touch &= ~bit;
		}
		if (pre_pen)
			break;
	}
	pre_touch = touch_num;

	if (have_pen_event) {
		input_sync(ts->pen_dev);
	}
	input_sync(ts->input_dev);

exit_work_end_cmd:
	if (!ts->rawdiff_mode) {
		ret = gtp_i2c_write(ts->client, end_cmd, 3);
		if (ret < 0)
			printk("<<-GTP-INFO->> I2C write end_cmd error!\n");
	}

exit_work_check:
	if (ts->use_irq)
		gtp_irq_enable(ts);
}

/* 19. gtp_pen_up */
static void gtp_pen_up(void)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	input_report_key(ts->pen_dev, BTN_TOUCH, 0);
	input_report_abs(ts->pen_dev, ABS_PRESSURE, 0);
	input_report_key(ts->pen_dev, BTN_TOOL_PEN, 0);
}

/* 20. gt91xx_config_read_proc */
static ssize_t gt91xx_config_read_proc(struct file *file, char __user *page,
				       size_t size, loff_t *ppos)
{
	char *p = page;
	u8 read_buf[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH];
	int i;

	memset(read_buf, 0, sizeof(read_buf));
	read_buf[0] = 0x80;
	read_buf[1] = 0x47;

	if (*ppos)
		return 0;

	strcpy(p, "==== GT9XX config init value====\n");
	p += strlen("==== GT9XX config init value====\n");
	for (i = 0; i < GTP_CONFIG_MAX_LENGTH; i++) {
		p += sprintf(p, "0x%02X ", config[i + GTP_ADDR_LENGTH]);
		if ((i & 7) == 7)
			*p++ = '\n';
	}
	*p++ = '\n';

	strcpy(p, "==== GT9XX config real value====\n");
	p += strlen("==== GT9XX config real value====\n");
	gtp_i2c_read(i2c_connect_client, read_buf, sizeof(read_buf));
	for (i = 0; i < GTP_CONFIG_MAX_LENGTH; i++) {
		p += sprintf(p, "0x%02X ", read_buf[i + GTP_ADDR_LENGTH]);
		if ((i & 7) == 7)
			*p++ = '\n';
	}
	*ppos += (p - page);
	return (p - page);
}

/* 21. gt91xx_config_write_proc */
static ssize_t gt91xx_config_write_proc(struct file *filp,
					const char __user *buffer,
					size_t count, loff_t *off)
{
	if (count > GTP_CONFIG_MAX_LENGTH)
		return -EFAULT;

	if (copy_from_user(&config[GTP_ADDR_LENGTH], buffer, count)) {
		memset(&config[GTP_ADDR_LENGTH + count - 0], 0, 0);
		printk("<<-GTP-ERROR->> copy from user fail\n\n");
		return -EFAULT;
	}

	if (gtp_send_cfg(i2c_connect_client) < 0)
		printk("<<-GTP-ERROR->> send config failed.\n");

	return count;
}

/* 22. goodix_ts_irq_handler */
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = dev_id;

	gtp_irq_disable(ts);
	queue_work(goodix_wq, &ts->work);
	return IRQ_HANDLED;
}

/* 23. goodix_ts_timer_handler */
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
	struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);

	queue_work(goodix_wq, &ts->work);
	hrtimer_start_range_ns(&ts->timer, 16000000, 0, HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

/* 24. gtp_fb_notifier_callback */
static int gtp_fb_notifier_callback(struct notifier_block *noti,
				    unsigned long event, void *data)
{
	struct fb_event *ev_data = data;
	struct goodix_ts_data *ts = container_of(noti, struct goodix_ts_data, fb_notif);
	int *blank;
	int ret;
	int retry;
	u8 sleep_cmd[3] = {0x80, 0x40, 0x05};
	u8 wake_cmd[3] = {0x80, 0x41, 0xAA};

	if (ev_data && ts && event == FB_EVENT_BLANK) {
		blank = ev_data->data;
		if (blank) {
			if (*blank == FB_BLANK_POWERDOWN) {
				if (!ts->is_suspended) {
					printk("<<-GTP-INFO->> System suspend.\n");
					ts->is_suspended = 1;
					gtp_esd_switch(ts->client, 0);
					if (ts->use_irq)
						gtp_irq_disable(ts);
					else
						hrtimer_cancel(&ts->timer);

					gpio_direction_output(gtp_int_gpio, 0);
					msleep(5);
					retry = 1;
					while (1) {
						ret = gtp_i2c_write(ts->client, sleep_cmd, 3);
						if (ret > 0)
							break;
						msleep(10);
						if (retry++ >= 5) {
							printk("<<-GTP-ERROR->> GTP send sleep cmd failed.\n");
							break;
						}
					}
					printk("<<-GTP-INFO->> GTP enter sleep!\n");
					if (ret < 0)
						printk("<<-GTP-ERROR->> GTP early suspend failed.\n");
					msleep(58);
				}
			} else if (*blank == FB_BLANK_UNBLANK) {
				if (ts->is_suspended) {
					printk("<<-GTP-INFO->> System resume.\n");
					gpio_direction_output(gtp_int_gpio, 0);
					msleep(5);
					ret = gtp_i2c_test(ts->client);
					if (ret < 1) {
						gtp_reset_guitar(ts->client, 20);
						printk("<<-GTP-ERROR->> GTP wakeup sleep failed.\n");
						if (ret < 0)
							printk("<<-GTP-ERROR->> GTP later resume failed.\n");
					} else {
						printk("<<-GTP-INFO->> GTP wakeup sleep.\n");
						gtp_int_sync(25);
						gtp_i2c_write_no_rst(ts->client, wake_cmd, 3);
					}
					gtp_send_cfg(ts->client);
					if (ts->use_irq)
						gtp_irq_enable(ts);
					else
						hrtimer_start_range_ns(&ts->timer, 1000000000, 0, HRTIMER_MODE_REL);
					ts->is_suspended = 0;
					gtp_esd_switch(ts->client, 1);
				}
			}
		}
	}
	return 0;
}

/* 25. gtp_esd_check_func */
static void gtp_esd_check_func(struct work_struct *work)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	u8 esd_data[5] = {0x80, 0x40, 0, 0, 0};
	u8 check_data[4] = {0x80, 0x40, 0, 0};
	int ret;
	int retry = 0;

	if (ts->is_suspended || ts->rawdiff_mode) {
		printk("<<-GTP-INFO->> Esd suspended!\n");
		return;
	}

	while (1) {
		ret = gtp_i2c_read_no_rst(ts->client, esd_data, 4);
		if (ret < 0)
			goto esd_fail;
		if (esd_data[2] != 0xAA && esd_data[3] == 0xAA)
			break;
		check_data[0] = 0x80;
		check_data[1] = 0x40;
		gtp_i2c_read_no_rst(ts->client, check_data, 4);
		if (check_data[2] == 0xAA || check_data[3] != 0xAA) {
esd_fail:
			printk("<<-GTP-ERROR->> IC working abnormally! Process reset guitar.\n");
			esd_data[0] = 0x80;
			esd_data[1] = 0x40;
			esd_data[2] = 0xaa;
			esd_data[3] = 0x01;
			esd_data[4] = 0x01;
			gtp_i2c_write_no_rst(ts->client, esd_data, 5);
			msleep(50);
			gtp_power_switch(ts->client, 0);
			msleep(20);
			gtp_power_switch(ts->client, 1);
			msleep(20);
			gtp_reset_guitar(ts->client, 50);
			msleep(50);
			gtp_send_cfg(ts->client);
			goto esd_out;
		}
		if (++retry >= 3)
			goto esd_fail;
	}

	esd_data[2] = esd_data[3];
	gtp_i2c_write_no_rst(ts->client, esd_data, 3);

esd_out:
	if (ts->is_suspended) {
		printk("<<-GTP-INFO->> Esd suspended!\n");
		return;
	}
	queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
}

/* 26. goodix_ts_init */
static int __init goodix_ts_init(void)
{
	printk("<<-GTP-INFO->> GTP driver installing....\n");
	goodix_wq = create_singlethread_workqueue("goodix_wq");
	if (!goodix_wq) {
		printk("<<-GTP-ERROR->> Creat workqueue failed.\n");
		return -ENOMEM;
	}
	INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
	gtp_esd_check_workqueue = create_singlethread_workqueue("gtp_esd_check");
	return i2c_add_driver(&goodix_ts_driver);
}

/* 27. goodix_ts_exit */
static void __exit goodix_ts_exit(void)
{
	printk("<<-GTP-INFO->> GTP driver exited.\n");
	i2c_del_driver(&goodix_ts_driver);
	if (goodix_wq)
		destroy_workqueue(goodix_wq);
}

module_init(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
