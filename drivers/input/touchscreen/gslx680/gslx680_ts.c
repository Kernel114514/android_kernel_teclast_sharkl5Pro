/*
 * drivers/input/touchscreen/gslx680/gslx680_ts.c
 *
 * Clean-room rewrite from stock-vmlinux disassembly/Hex-Rays
 * for Teclast M40 (Unisoc UMS512 / T618 Sharkl5Pro).
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include "cust_firmware.h"

extern void gsl_DataInit(unsigned int *conf_in);
extern void gsl_alg_id_main(struct gsl_touch_info *cinfo);
extern unsigned int gsl_mask_tiaoping(void);
extern unsigned int gsl_version_id(void);

struct gsl_ts_platform_data {
    u32 irq_gpio;
    u32 rst_gpio;
    const char *vdd_name;
    u32 virtualkeys[12];
    u32 max_x;
    u32 max_y;
};

struct gsl_ts {
    struct input_dev *input;
    u8 touch_data[44];
    u8 pad[4];
    struct i2c_client *client;
    struct work_struct work;
    struct workqueue_struct *wq;
    u8 pad2[32];
    struct gsl_ts_platform_data *pdata;
};

static DEFINE_MUTEX(gsl_i2c_lock);
static struct gsl_ts *g_gslx680_ts;
static struct i2c_client *this_client;
static struct delayed_work gsl_monitor_work;
static struct workqueue_struct *gsl_monitor_workqueue;
static int id_sign[11];
static u8 id_state_flag[11];
static u8 id_state_old_flag[11];
static u16 x_old[11];
static u16 y_old[11];
static int gsl_sw_flag;
static int gsl_halt_flag;
static u8 b0_counter;
static u8 bc_counter;
static u8 int_1st_0;
static u8 int_1st_1;
static u8 int_1st_2;
static u8 int_1st_3;
static u8 i2c_lock_flag;
static int gsl_suspend_flag;

static int gsl_read_interface(struct i2c_client *client, u8 reg, u8 *buf)
{
    u8 reg_buf[4];
    int ret;

    reg_buf[0] = reg;
    mutex_lock(&gsl_i2c_lock);
    if ((s8)reg >= 0) {
        if (reg == 124) {
            reg_buf[0] = 120;
            i2c_master_send(client, reg_buf, 1);
            i2c_master_recv(client, buf, 4);
            reg_buf[0] = 124;
        } else {
            i2c_master_send(client, reg_buf, 1);
            i2c_master_recv(client, buf, 4);
        }
    }
    i2c_master_send(client, reg_buf, 1);
    ret = i2c_master_recv(client, buf, 4);
    mutex_unlock(&gsl_i2c_lock);
    return ret;
}

static int gsl_write_interface(struct i2c_client *client, u8 reg, const u8 *buf, u32 len)
{
    struct i2c_msg msg;
    u8 send_buf[len + 1];

    send_buf[0] = reg;
    memcpy(send_buf + 1, buf, len);
    msg.addr = client->addr;
    msg.flags = client->flags & I2C_M_TEN;
    msg.len = len + 1;
    msg.buf = send_buf;

    mutex_lock(&gsl_i2c_lock);
    i2c_transfer(client->adapter, &msg, 1);
    mutex_unlock(&gsl_i2c_lock);
    return 0;
}

static void gsl_reset_core(struct i2c_client *client)
{
    u32 val;

    val = 0x88;
    gsl_write_interface(client, 0xe0, (u8 *)&val, 4);
    msleep(5);
    val = 0x04;
    gsl_write_interface(client, 0xe4, (u8 *)&val, 4);
    msleep(5);
    val = 0x00;
    gsl_write_interface(client, 0xbc, (u8 *)&val, 4);
    msleep(5);
}

static int gsl_ts_read(struct i2c_client *client, u8 reg, u8 *buf, u32 len)
{
    u8 reg_buf[128];
    int ret;

    reg_buf[0] = reg;
    ret = i2c_master_send(client, reg_buf, 1);
    if (ret < 0) {
        printk("%s set data address fail!\n", "gsl_ts_read");
        return ret;
    }
    return i2c_master_recv(client, buf, len);
}

static int check_mem_data(struct i2c_client *client);

static int gsl_sw_init(struct i2c_client *client)
{
    int i;
    u8 reg_addr = 0;
    u32 buf[32];
    u32 val;
    int buf_idx;

    if (gsl_sw_flag == 1)
        return 0;

    gsl_sw_flag = 1;
    gpiod_set_raw_value(gpio_to_desc(145), 0);
    msleep(20);
    gpiod_set_raw_value(gpio_to_desc(145), 1);
    msleep(20);

    val = 0x88;
    gsl_write_interface(client, 0xe0, (u8 *)&val, 4);
    msleep(20);
    val = 0x03;
    gsl_write_interface(client, 0x80, (u8 *)&val, 4);
    msleep(5);
    val = 0x04;
    gsl_write_interface(client, 0xe4, (u8 *)&val, 4);
    msleep(5);
    val = 0x00;
    gsl_write_interface(client, 0xe0, (u8 *)&val, 4);
    msleep(20);

    gsl_reset_core(client);

    memset(buf, 0, sizeof(buf));
    buf_idx = 1;

    for (i = 0; i < 4818; i++) {
        if (GSL1680E_FW[i].offset == 0xf0) {
            val = GSL1680E_FW[i].val;
            gsl_write_interface(client, 0xf0, (u8 *)&val, 4);
            buf_idx = 1;
        } else {
            if ((buf_idx & 0x1f) == 1)
                reg_addr = (u8)GSL1680E_FW[i].offset;
            buf[buf_idx - 1] = GSL1680E_FW[i].val;
            if ((buf_idx & 0x1f) == 0) {
                gsl_write_interface(client, reg_addr, (u8 *)buf, 128);
                buf_idx = 0;
            }
            buf_idx++;
        }
    }

    val = 0;
    gsl_write_interface(client, 0xe0, (u8 *)&val, 4);
    gsl_DataInit(gsl_config_data_id);
    gsl_sw_flag = 0;
    return 0;
}

static int check_mem_data(struct i2c_client *client)
{
    u8 read_buf[4] = {0};
    int ret;

    msleep(30);
    ret = gsl_read_interface(client, 0xb0, read_buf);
    if (read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
        return gsl_sw_init(client);
    return ret;
}

static void gsl_monitor_worker(struct work_struct *work)
{
    u8 read_buf[4] = {0};

    printk("---gsl_monitor_worker---\n");
    if (!(i2c_lock_flag & 1)) {
        i2c_lock_flag = 1;
        gsl_ts_read(this_client, 0xb0, read_buf, 4);
        if (read_buf[3] == 0x5a && read_buf[2] == 0x5a && read_buf[1] == 0x5a && read_buf[0] == 0x5a) {
            b0_counter = 0;
        } else if (++b0_counter >= 2) {
            printk("======read 0xb0: %x %x %x %x ======\n",
                    read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
            b0_counter = 0;
            gsl_sw_init(this_client);
            goto out;
        }

        gsl_ts_read(this_client, 0xb4, read_buf, 4);
        if (read_buf[3] == int_1st_3 &&
            read_buf[2] == int_1st_2 &&
            read_buf[1] == int_1st_1 &&
            read_buf[0] == int_1st_0) {
            printk("======int_1st: %x %x %x %x , int_2nd: %x %x %x %x ======\n",
                    read_buf[3], read_buf[2], int_1st_1, int_1st_0,
                    int_1st_3, int_1st_2, int_1st_1, int_1st_0);
            gsl_sw_init(this_client);
            goto out;
        } else {
            int_1st_3 = read_buf[3];
            int_1st_2 = read_buf[2];
            int_1st_1 = read_buf[1];
            int_1st_0 = read_buf[0];

            gsl_ts_read(this_client, 0xbc, read_buf, 4);
            if (read_buf[3] || read_buf[2] || read_buf[1] || read_buf[0])
                bc_counter++;
            else
                bc_counter = 0;

            if (bc_counter >= 2) {
                printk("<<<< wanghe ======read 0xbc: %x %x %x %x======\n",
                        read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
                bc_counter = 0;
                gsl_sw_init(this_client);
                goto out;
            }
        }
out:
        i2c_lock_flag = 0;
    }
    queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 200);
}

static void gslX680_ts_worker(struct work_struct *work)
{
    struct gsl_ts *ts;
    u8 *touch_data;
    struct gsl_touch_info cinfo;
    int touch_num;
    int i;
    int id;
    u16 x, y;
    u32 avg_x, avg_y;
    u16 diff_x, diff_y;
    u32 tiaoping;
    u8 send_buf[128];

    if (!(i2c_lock_flag & 1)) {
        ts = i2c_get_clientdata(this_client);
        touch_data = ts->touch_data;
        i2c_lock_flag = 1;

        if (gsl_ts_read(this_client, 0x80, touch_data, 44) < 0) {
            dev_err(&this_client->dev, "read failed\n");
        } else {
            touch_num = touch_data[0];
            if (touch_num > 10)
                touch_num = 10;

            memset(&cinfo, 0, sizeof(cinfo));
            for (i = 0; i < touch_num; i++) {
                cinfo.x[i] = touch_data[4 * i + 6] | ((touch_data[4 * i + 7] & 0x0f) << 8);
                cinfo.y[i] = touch_data[4 * i + 4] | (touch_data[4 * i + 5] << 8);
                cinfo.id[i] = touch_data[4 * i + 7] >> 4;
            }
            cinfo.finger_num = *(u32 *)touch_data;

            gsl_alg_id_main(&cinfo);
            tiaoping = gsl_mask_tiaoping();
            if (tiaoping + 1 >= 2) {
                *(u32 *)send_buf = 2800;
                send_buf[4] = 0;
                i2c_master_send(this_client, send_buf, 5);
                *(u32 *)&send_buf[1] = tiaoping;
                send_buf[0] = 8;
                i2c_master_send(this_client, send_buf, 5);
            }

            for (i = 1; i <= 10; i++) {
                if (cinfo.finger_num == 0)
                    id_sign[i] = 0;
                id_state_flag[i] = 0;
            }

            for (i = 0; i < min(cinfo.finger_num, 10); i++) {
                id = cinfo.id[i];
                if ((unsigned int)(id - 1) > 9)
                    continue;

                x = cinfo.x[i];
                y = cinfo.y[i];

                if (id_sign[id] != 0) {
                    avg_x = (x + x_old[id]) >> 1;
                    avg_y = (y + y_old[id]) >> 1;
                    diff_x = (avg_x > x_old[id]) ? (avg_x - x_old[id]) : (x_old[id] - avg_x);
                    diff_y = (avg_y > y_old[id]) ? (avg_y - y_old[id]) : (y_old[id] - avg_y);

                    if ((diff_y < 2 || diff_x <= 3) && (diff_y < 4 || diff_x < 2)) {
                        if (diff_x >= 4)
                            x_old[id] = avg_x;
                        if (diff_y < 4)
                            goto report;
                    } else {
                        x_old[id] = avg_x;
                    }
                    y_old[id] = avg_y;
                } else {
                    x_old[id] = x;
                    y_old[id] = y;
                }
report:
                id_sign[id]++;

                input_event(ts->input, EV_KEY, BTN_TOUCH, 1);
                input_event(ts->input, EV_ABS, ABS_MT_POSITION_X, x_old[id]);
                input_event(ts->input, EV_ABS, ABS_MT_POSITION_Y, y_old[id]);
                input_event(ts->input, EV_ABS, ABS_MT_TRACKING_ID, id);
                input_event(ts->input, EV_SYN, SYN_MT_REPORT, 0);
                id_state_flag[id] = 1;
            }

            for (i = 1; i <= 10; i++) {
                if (cinfo.finger_num == 0 || (id_state_old_flag[i] != 0 && id_state_flag[i] == 0))
                    id_sign[i] = 0;
                id_state_old_flag[i] = id_state_flag[i];
            }

            if (cinfo.finger_num == 0) {
                input_event(ts->input, EV_KEY, BTN_TOUCH, 0);
                input_event(ts->input, EV_SYN, SYN_MT_REPORT, 0);
            }
            input_event(ts->input, EV_SYN, SYN_REPORT, 0);
        }
        i2c_lock_flag = 0;
    }
    enable_irq(this_client->irq);
}

static irqreturn_t gslX680_ts_interrupt(int irq, void *dev_id)
{
    struct gsl_ts *ts = dev_id;

    disable_irq_nosync(this_client->irq);
    if (!work_pending(&ts->work))
        queue_work(ts->wq, &ts->work);
    return IRQ_HANDLED;
}

static ssize_t ts_suspend_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return 0;
}

static ssize_t ts_suspend_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    if (gsl_suspend_flag != 0) {
        if (*buf == '0') {
            gpiod_direction_output_raw(gpio_to_desc(g_gslx680_ts->pdata->rst_gpio), 1);
            gpiod_set_raw_value(gpio_to_desc(g_gslx680_ts->pdata->rst_gpio), 1);
            gsl_reset_core(this_client);
            {
                int zero = 0;
                gsl_write_interface(this_client, 0xe0, (u8 *)&zero, 4);
            }
            gsl_DataInit(gsl_config_data_id);
            msleep(20);
            check_mem_data(this_client);
            enable_irq(this_client->irq);
            printk("gsl_ts_resume () : queue gsl_monitor_work\n");
            queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 300);
            i2c_lock_flag = 0;
            gsl_halt_flag = 0;
        } else if (*buf == '1') {
            printk("gsl_ts_suspend () : cancel gsl_monitor_work\n");
            cancel_delayed_work_sync(&gsl_monitor_work);
            printk("[tp-gsl]the version of alg_id:%x\n", gsl_version_id());
            gsl_halt_flag = 1;
            disable_irq_nosync(this_client->irq);
            gpiod_direction_output_raw(gpio_to_desc(g_gslx680_ts->pdata->rst_gpio), 0);
            gpiod_set_raw_value(gpio_to_desc(g_gslx680_ts->pdata->rst_gpio), 0);
        }
    } else {
        gsl_suspend_flag = 1;
    }
    return count;
}

static DEVICE_ATTR(ts_suspend, 0664, ts_suspend_show, ts_suspend_store);

static struct attribute *ts_debug_attrs[] = {
    &dev_attr_ts_suspend.attr,
    NULL,
};

static struct attribute_group ts_debug_attr_group = {
    .attrs = ts_debug_attrs,
};

static int gslX680_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct device_node *np = client->dev.of_node;
    struct gsl_ts_platform_data *pdata = client->dev.platform_data;
    struct gsl_ts *ts;
    struct input_dev *input;
    struct kobject *kobj;
    u32 chip_id = 0;
    int ret;

    if (!pdata && np) {
        pdata = kzalloc(sizeof(struct gsl_ts_platform_data), GFP_KERNEL);
        if (!pdata) {
            dev_err(&client->dev, "kzalloc failed\n");
            return -ENOMEM;
        }
        pdata->rst_gpio = of_get_named_gpio_flags(np, "gpios", 0, NULL);
        pdata->irq_gpio = of_get_named_gpio_flags(np, "gpios", 1, NULL);
        if ((int)pdata->rst_gpio < 0 || (int)pdata->irq_gpio < 0) {
            dev_err(&client->dev, "fail to get reset/irq gpio\n");
            goto err_free_pdata;
        }
        if (of_property_read_string(np, "vdd_name", &pdata->vdd_name) != 0) {
            dev_err(&client->dev, "fail to get vdd_name\n");
            goto err_free_pdata;
        }
        if (of_property_read_u32_array(np, "virtualkeys", pdata->virtualkeys, 12) < 0) {
            dev_err(&client->dev, "fail to get virtualkeys\n");
            goto err_free_pdata;
        }
        if (of_property_read_u32(np, "TP_MAX_X", &pdata->max_x) < 0) {
            dev_err(&client->dev, "fail to get tp-max-x\n");
            goto err_free_pdata;
        }
        if (of_property_read_u32(np, "TP_MAX_Y", &pdata->max_y) < 0) {
            dev_err(&client->dev, "fail to get tp-max-y\n");
            goto err_free_pdata;
        }
        client->dev.platform_data = pdata;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
        return -ENODEV;

    ts = kzalloc(sizeof(struct gsl_ts), GFP_KERNEL);
    if (!ts)
        return -ENOMEM;

    if (gsl_read_interface(client, 0xfc, (u8 *)&chip_id) < 0) {
        printk("gslX680_ts_probe: read chip id error\n");
        kfree(ts);
        return 0;
    }

    printk("gsl buf[0] = %x buf[1] = %x buf[2] = %x buf[3] = %x err = %d\n",
           chip_id & 0xff, (chip_id >> 8) & 0xff, (chip_id >> 16) & 0xff, (chip_id >> 24) & 0xff, 0);

    mutex_init(&gsl_i2c_lock);
    ts->pdata = pdata;
    g_gslx680_ts = ts;
    this_client = client;
    i2c_set_clientdata(client, ts);
    ts->client = client;

    gpio_request(pdata->irq_gpio, "ts_irq_pin");
    gpio_request(pdata->rst_gpio, "ts_rst_pin");
    gpiod_direction_output_raw(gpio_to_desc(pdata->rst_gpio), 1);
    gpiod_set_raw_value(gpio_to_desc(pdata->rst_gpio), 1);
    gpiod_direction_input(gpio_to_desc(pdata->irq_gpio));
    client->irq = gpiod_to_irq(gpio_to_desc(pdata->irq_gpio));

    INIT_WORK(&ts->work, gslX680_ts_worker);
    ts->wq = create_singlethread_workqueue("gslX680_ts_workqueue");
    if (!ts->wq) {
        ret = -ESRCH;
        goto err_free_ts;
    }

    ret = request_threaded_irq(client->irq, gslX680_ts_interrupt, NULL,
                               IRQF_TRIGGER_RISING | IRQF_ONESHOT, client->name, ts);
    if (ret < 0) {
        dev_err(&client->dev, "gslX680_probe: request irq failed\n");
        goto err_destroy_wq;
    }

    disable_irq(client->irq);

    input = input_allocate_device();
    if (!input) {
        dev_err(&client->dev, "failed to allocate input device\n");
        ret = -ENOMEM;
        goto err_free_irq;
    }
    ts->input = input;

    __set_bit(INPUT_PROP_DIRECT, input->propbit);
    __set_bit(EV_KEY, input->evbit);
    __set_bit(EV_ABS, input->evbit);

    __set_bit(KEY_HOME, input->keybit);
    __set_bit(KEY_MENU, input->keybit);
    __set_bit(KEY_BACK, input->keybit);
    __set_bit(KEY_SEARCH, input->keybit);
    __set_bit(BTN_TOUCH, input->keybit);

    input_set_abs_params(input, ABS_MT_POSITION_X, 0, 1920, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_Y, 0, 1200, 0, 0);
    input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
    input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

    input->name = "gslX680";
    ret = input_register_device(input);
    if (ret) {
        dev_err(&client->dev, "gslX680_ts_probe: failed to register input device: %s\n",
                client->dev.init_name ? client->dev.init_name : dev_name(&client->dev));
        input_free_device(input);
        goto err_free_irq;
    }

    gsl_sw_init(this_client);
    msleep(20);
    check_mem_data(this_client);

    kobj = kobject_create_and_add("touchscreen", NULL);
    if (!kobj || sysfs_create_group(kobj, &ts_debug_attr_group))
        dev_err(&client->dev, "failed to create touchscreen sysfs\n");

    gsl_suspend_flag = 0;
    gsl_DataInit(gsl_config_data_id);
    printk("gsl_ts_probe () : queue gsl_monitor_work\n");

    INIT_DELAYED_WORK(&gsl_monitor_work, gsl_monitor_worker);
    gsl_monitor_workqueue = create_singlethread_workqueue("gsl_monitor_workqueue");
    queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 1000);
    enable_irq(client->irq);

    return 0;

err_free_irq:
    free_irq(client->irq, ts);
err_destroy_wq:
    cancel_work_sync(&ts->work);
    destroy_workqueue(ts->wq);
err_free_ts:
    i2c_set_clientdata(client, NULL);
    kfree(ts);
    return ret;

err_free_pdata:
    kfree(pdata);
    return -ENOMEM;
}

static int gslX680_ts_remove(struct i2c_client *client)
{
    struct gsl_ts *ts = i2c_get_clientdata(client);

    cancel_delayed_work_sync(&gsl_monitor_work);
    destroy_workqueue(gsl_monitor_workqueue);
    free_irq(client->irq, ts);
    input_unregister_device(ts->input);
    kfree(ts);
    cancel_work_sync(&ts->work);
    destroy_workqueue(ts->wq);
    i2c_set_clientdata(client, NULL);
    return 0;
}

static const struct of_device_id gsl_of_match[] = {
    { .compatible = "gslx680,gslx680_ts" },
    { }
};
MODULE_DEVICE_TABLE(of, gsl_of_match);

static const struct i2c_device_id gslX680_ts_id[] = {
    { "gslX680", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, gslX680_ts_id);

static struct i2c_driver gslX680_ts_driver = {
    .probe = gslX680_ts_probe,
    .remove = gslX680_ts_remove,
    .id_table = gslX680_ts_id,
    .driver = {
        .name = "gslX680",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(gsl_of_match),
    },
};

static int __init gslX680_ts_init(void)
{
    return i2c_register_driver(THIS_MODULE, &gslX680_ts_driver);
}

static void __exit gslX680_ts_exit(void)
{
    i2c_del_driver(&gslX680_ts_driver);
}

module_init(gslX680_ts_init);
module_exit(gslX680_ts_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GSLX680 touchscreen driver");
