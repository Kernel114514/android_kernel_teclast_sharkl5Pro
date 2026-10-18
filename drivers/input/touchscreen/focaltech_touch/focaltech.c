/*
 * drivers/input/touchscreen/focaltech.c
 *
 * FocalTech focaltech TouchScreen driver.
 *
 * Copyright (c) 2014  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VERSION      	DATE			AUTHOR
 *    1.1		  2014-09			mshl
 *
 */
#define REVO_ANDORID11
//#define REVO_PM_TP_SYSFS
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
//#include <soc/sprd/regulator.h>
#include <linux/input/mt.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#ifdef REVO_ANDORID11
#include <linux/pm_wakeup.h>
#include <linux/uaccess.h>
#else
#include <linux/wakelock.h>
#endif
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/suspend.h>
#include <linux/irq.h>
#include <linux/notifier.h>
#if(defined(CONFIG_I2C_SPRD) ||defined(CONFIG_I2C_SPRD_V1))
//#include <soc/sprd/i2c-sprd.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <uapi/linux/sched/types.h>

#ifdef TP_HAVE_PROX
#include <linux/wakelock.h> //chenjiaxi add
#endif

//#define FTS_DBG
#ifdef FTS_DBG
#define ENTER printk(KERN_INFO "[FTS_DBG] func: %s  line: %04d\n", __func__, __LINE__);
#define PRINT_DBG(x...)  printk(KERN_INFO "[FTS_DBG] " x)
#define PRINT_INFO(x...)  printk(KERN_INFO "[FTS_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[FTS_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[FTS_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#else
#define ENTER
#define PRINT_DBG(x...)
#define PRINT_INFO(x...)  printk(KERN_INFO "[FTS_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[FTS_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[FTS_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#endif


//#define APK_DEBUG
//#define SPRD_AUTO_UPGRADE
//#define SYSFS_DEBUG
//#define FTS_CTL_IIC
//#define FTS_GESTURE
//#define TP_HAVE_PROX

#include "focaltech.h"
#include "focaltech_ex_fun.h"
#include "focaltech_ctl.h"

#define	USE_WAIT_QUEUE	1
#define	USE_THREADED_IRQ	0
#define	USE_WORK_QUEUE	0

#define	TOUCH_VIRTUAL_KEYS
#define	MULTI_PROTOCOL_TYPE_B	0

#if defined(CONFIG_BOARD_L30)||defined(CONFIG_BOARD_L30A)
#define	TS_MAX_FINGER		10
#else
#define	TS_MAX_FINGER		5 //5
#endif

#define	FTS_PACKET_LENGTH	128

#if USE_WAIT_QUEUE
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int tpd_flag = 0;
#endif

//static int fw_size;

static struct fts_data *g_fts;
static struct i2c_client *this_client;

//static unsigned char suspend_flag = 0;

#ifdef TP_HAVE_PROX
#define PROXIMITY_INPUT_DEV		"alps_pxy"
static int is_incall = 0;
static struct input_dev *proximity_input_dev;
static struct class *firmware_class;
static struct device *firmware_cmd_dev;

static struct wakeup_source  *prx_wake_lock_ps;

static int ps_suspend_flag; //bird - Add by chenjiaxi
#endif
 struct Upgrade_Info fts_updateinfo[] =
{
	#ifdef V33K_FIRMWARE_UPGRADE_2
	{0x64,0x11,"FT6x36U",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x1c, 10, 2000},
	{0x64,0x7a,"FT6x36U",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x1c, 10, 2000},
	#else
	{0x55,"FT5x06",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000},
	{0x08,"FT5606",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 10, 0x79, 0x06, 100, 2000},
	{0x0a,"FT5x16",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 10, 1500},
	{0x06,"FT6x06",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000},
	{0x36,"FT6x36",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x18, 10, 2000},
	{0x64,"FT6x36U",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x1c, 10, 2000},
	{0x55,"FT5x06i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000},
	{0x14,"FT5336",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x13,"FT3316",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x12,"FT5436i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x11,"FT5336i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x54,"FT5x46",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,2, 2, 0x54, 0x2c, 10, 2000},
	#endif
};

struct Upgrade_Info fts_updateinfo_curr;
#ifdef FTS_GESTURE
#include "ft_gesture_lib.h"
static bool TP_gesture_Switch;
static char gesture = 'U';
#define GESTURE_SWITCH_FILE 		"/data/tp_wake_switch"
#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP		    0x22
#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_O		    0x30
#define GESTURE_W		    0x31
#define GESTURE_M		    0x32
#define GESTURE_E		    0x33
#define GESTURE_C		    0x34
#define GESTURE_S		    0x46
#define GESTURE_V		    0x54
#define GESTURE_Z		    0x41
#define GESTURE_A		    0x36
#define FTS_GESTURE_POINTS 255
#define FTS_GESTURE_POINTS_ONETIME  62
#define FTS_GESTURE_POINTS_HEADER 8
#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4

short pointnum = 0;
unsigned short coordinate_x[150] = {0};
unsigned short coordinate_y[150] = {0};

extern int fetch_object_sample(unsigned char *buf,short pointnum);
extern void init_para(int x_pixel,int y_pixel,int time_slot,int cut_x_pixel,int cut_y_pixel);
//suspend_state_t get_suspend_state(void);
#pragma message("focaltech gesture function is defined!\n")
#endif
static int g_chipid = 0;
static int g_fwversion = 0;
static int g_vendorid = 0;

static unsigned char fts_read_fw_ver(void);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void fts_early_suspend(struct early_suspend *h);
static void fts_late_resume(struct early_suspend *h);
#endif

struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
	u16	pressure;
    u8  touch_point;
#ifdef TP_HAVE_PROX
	int prox_event;
	int prox_value;
#endif
};

struct fts_data {
	struct input_dev	*input_dev;
#ifdef TP_HAVE_PROX
	struct input_dev    *prox_input_dev;
#endif
	struct i2c_client	*client;
	struct ts_event	event;
#if USE_WORK_QUEUE
	struct work_struct	pen_event_work;
	struct workqueue_struct	*fts_workqueue;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct work_struct		resume_work;
	struct workqueue_struct	*fts_resume_workqueue;
	struct early_suspend	early_suspend;
#endif
	struct fts_platform_data	*platform_data;
};

#ifdef FTS_GESTURE
static int gestureflag = 1;
static int gestureswitch = 1;
#endif

#ifdef FTS_GESTURE
static ssize_t show_gesture(struct device *dev, struct device_attribute *attr, char *buf)
{
    int len = -1;
    if ( buf != NULL ){
        len = snprintf( buf, FTS_PAGE_SIZE, "%c", gesture );
        printk("xxxxx mycat gesture: buf=%d\n\n", *buf);
        return len;
    }
    return len;
}
static ssize_t store_gesture(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    return 1;
}
static DEVICE_ATTR(gesture, S_IRUGO | S_IWUSR, show_gesture, store_gesture);

#endif

/*static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	if (i2c_transfer(this_client->adapter, msgs, 2) != 2) {
		ret = -EIO;
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	}

	return ret;
}*/


#ifdef TP_HAVE_PROX
static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret = 0;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

	if (i2c_transfer(this_client->adapter, msg, 1) != 1) {
		ret = -EIO;
		pr_err("%s i2c write error: %d\n", __func__, ret);
	}

	return ret;
}

/***********************************************************************************************
Name	:	 ft5x0x_write_reg

Input	:	addr -- address
                     para -- parameter

Output	:

function	:	write register of ft5x0x

***********************************************************************************************/
static int ft5x0x_write_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5x0x_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}
#endif

/***********************************************************************************************
Name	:	ft5x0x_read_reg

Input	:	addr
                     pdata

Output	:

function	:	read register of ft5x0x

***********************************************************************************************/
static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	int ret = 0;
	u8 buf[2] = {addr, 0};

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= buf+1,
		},
	};

	if (i2c_transfer(this_client->adapter, msgs, 2) != 2) {
		ret = -EIO;
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	}

	*pdata = buf[1];
	return ret;
}

//#define VIRTUAL_BOARD_PROPERTIES_SUPPORT
#if defined(TOUCHPANEL_NAME_SHOW)
#define VIRTUAL_TP_TYPE             TOUCHPANEL_NAME_SHOW
#else
#define VIRTUAL_TP_TYPE             "FT8201"
#endif
#define VIRTUAL_TP_NAME           "tp_name"

static u8 chipId;
static u8 vendorId;

#ifdef VIRTUAL_BOARD_PROPERTIES_SUPPORT
static ssize_t tp_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{  
    return sprintf(buf, "TP_NAME: %s(%x)\n"    "FW_VER=0x%x\n" , VIRTUAL_TP_TYPE ,chipId , vendorId);
}

static struct kobj_attribute virtual_tp_name_attr = {
    .attr = {
        .name = VIRTUAL_TP_NAME,
        .mode = S_IRUGO|S_IWUSR,
    },
    .show = &tp_name_show,
};
#endif

#ifdef TOUCH_VIRTUAL_KEYS

static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct fts_data *data;
	struct fts_platform_data *pdata;

	if (!this_client)
		return 0;
	data = i2c_get_clientdata(this_client);
	if (!data || !data->platform_data)
		return 0;
	pdata = data->platform_data;
	return sprintf(buf,"%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d\n"
		,__stringify(EV_KEY), __stringify(KEY_BACK),pdata ->virtualkeys[0],pdata ->virtualkeys[1],pdata ->virtualkeys[2],pdata ->virtualkeys[3]
		,__stringify(EV_KEY), __stringify(KEY_APPSELECT),pdata ->virtualkeys[4],pdata ->virtualkeys[5],pdata ->virtualkeys[6],pdata ->virtualkeys[7]
		,__stringify(EV_KEY), __stringify(KEY_HOMEPAGE),pdata ->virtualkeys[8],pdata ->virtualkeys[9],pdata ->virtualkeys[10],pdata ->virtualkeys[11]);
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.focaltech_ts",
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
#ifdef VIRTUAL_BOARD_PROPERTIES_SUPPORT
    &virtual_tp_name_attr.attr,
#endif
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static void fts_virtual_keys_init(void)
{
    int ret = 0;
    struct kobject *properties_kobj;

    pr_info("[FST] %s\n",__func__);

    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("failed to create board_properties\n");
}

#endif

/***********************************************************************************************
Name	:	 fts_read_fw_ver

Input	:	 void

Output	:	 firmware version

function	:	 read TP firmware version

***********************************************************************************************/
static unsigned char fts_read_fw_ver(void)
{
	unsigned char ver;
	fts_read_reg(this_client, FTS_REG_FIRMID, &ver);
	PRINT_INFO("fts_read_fw_ver ver = 0x%x\n", (int)ver);

	return ver;
}



static void fts_clear_report_data(struct fts_data *fts)
{
	int i;

	for(i = 0; i < TS_MAX_FINGER; i++) {
	#if MULTI_PROTOCOL_TYPE_B
		input_mt_slot(fts->input_dev, i);
		input_mt_report_slot_state(fts->input_dev, MT_TOOL_FINGER, false);
	#endif
	}
	input_report_key(fts->input_dev, BTN_TOUCH, 0);
	#if !MULTI_PROTOCOL_TYPE_B
		input_mt_sync(fts->input_dev);
	#endif
	input_sync(fts->input_dev);
}

static int fts_update_data(void)
{
	struct fts_data *data = i2c_get_clientdata(this_client);
	//struct fts_platform_data *pdata = data->platform_data;
	struct ts_event *event = &data->event;
	u8 buf[63] = {0};
	int ret = -1;
	int i;
	u16 x , y;
	u8 ft_pressure , ft_size;
#ifdef TP_HAVE_PROX
	u8 result = 0;
	ft5x0x_read_reg(0x01, &result);
	event->prox_event = 0;
	//PRINT_DBG("fts_pen_irq_work result:%x\n",result);
	if(result == 0xc0 && is_incall==1)
	{
		PRINT_DBG("[ft5x0x] face close 0n\n");
		event->prox_event = 1;
		event->prox_value = 0;

		input_report_abs(proximity_input_dev, ABS_DISTANCE, event->prox_value);
		input_sync(proximity_input_dev);
	}
	else if(result == 0xe0 && is_incall==1)
	{
		PRINT_DBG("[ft5x0x] face far away\n");
		event->prox_event = 1;
		event->prox_value = 1;

		input_report_abs(proximity_input_dev, ABS_DISTANCE, event->prox_value);
		input_sync(proximity_input_dev);
	}
#endif
	ret = fts_i2c_Read(this_client, buf, 1, buf, 63);//33 is 5 fingers , 63 is 10 fingers

	if (ret < 0) {
		pr_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x07;

	for(i = 0; i < TS_MAX_FINGER; i++) {
		if((buf[6*i+3] & 0xc0) == 0xc0)
			continue;
		x = (s16)(buf[6*i+3] & 0x0F)<<8 | (s16)buf[6*i+4];
		y = (s16)(buf[6*i+5] & 0x0F)<<8 | (s16)buf[6*i+6];
		ft_pressure = buf[6*i+7];
		if(ft_pressure > 127 || ft_pressure == 0)
			ft_pressure = 127;
		ft_size = (buf[6*i+8]>>4) & 0x0F;
		if(ft_size == 0)
		{
			ft_size = 0x09;
		}
		if((buf[6*i+3] & 0x40) == 0x0) {
		#if MULTI_PROTOCOL_TYPE_B
			input_mt_slot(data->input_dev, buf[6*i+5]>>4);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
		#else
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, buf[6*i+5]>>4);
		#endif
		//#if (PRJ_FEATURE_H_BOARD_DISP_ROTATION == 90)
		#if 0//(PRJ_FEATURE_H_BOARD_DISP_ROTATION == 90)
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, y);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, pdata->TP_MAX_X-x);
		#else
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, y);
		#endif
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, ft_pressure);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, ft_size);
			input_report_key(data->input_dev, BTN_TOUCH, 1);
		#if !MULTI_PROTOCOL_TYPE_B
			input_mt_sync(data->input_dev);
		#endif
			//pr_debug("===x%d = %d,y%d = %d ====",i, x, i, y);
			//printk("===x%d = %d,y%d = %d ====",i, x, i, y);
		}
		else {
		#if MULTI_PROTOCOL_TYPE_B
			input_mt_slot(data->input_dev, buf[6*i+5]>>4);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
		#endif
		}
	}
	if(0 == event->touch_point) {
		for(i = 0; i < TS_MAX_FINGER; i ++) {
			#if MULTI_PROTOCOL_TYPE_B
                            input_mt_slot(data->input_dev, i);
                            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			#endif
		}
		input_report_key(data->input_dev, BTN_TOUCH, 0);
		#if !MULTI_PROTOCOL_TYPE_B
			input_mt_sync(data->input_dev);
		#endif

	}
	input_sync(data->input_dev);

	return 0;
}

#ifdef FTS_GESTURE
static void check_gesture(int gesture_id)
{
	struct fts_data *data = i2c_get_clientdata(this_client);
	PRINT_DBG("gesture_id==0x%x\n ",gesture_id);
	switch(gesture_id)
	{
		case GESTURE_LEFT:
			gesture = 's' ;
			input_report_key(data->input_dev, KEY_TP_WAKE, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_TP_WAKE, 0);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_LEFT, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_LEFT, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_RIGHT:
			gesture = 'S' ;
			input_report_key(data->input_dev, KEY_TP_WAKE, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_TP_WAKE, 0);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_RIGHT, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_RIGHT, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_UP:
			gesture = 'U' ;
			input_report_key(data->input_dev, KEY_TP_WAKE, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_TP_WAKE, 0);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_UP, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_UP, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_DOWN:
			gesture = 'C' ;
			input_report_key(data->input_dev, KEY_TP_WAKE, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_TP_WAKE, 0);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_DOWN, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_DOWN, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_DOUBLECLICK:
			gesture = 'd' ;
			input_report_key(data->input_dev, KEY_TP_WAKE, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_TP_WAKE, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_O:
			gesture = 'o' ;
			input_report_key(data->input_dev, KEY_TP_WAKE, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_TP_WAKE, 0);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_O, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_O, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_W:
			gesture = 'w' ;
			input_report_key(data->input_dev, KEY_TP_WAKE, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_TP_WAKE, 0);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_W, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_W, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_M:
			gesture = 'm' ;
			input_report_key(data->input_dev, KEY_TP_WAKE, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_TP_WAKE, 0);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_M, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_M, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_E:
			gesture = 'e' ;
			input_report_key(data->input_dev, KEY_TP_WAKE, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_TP_WAKE, 0);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_E, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_E, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_C:
			gesture = 'c' ;
			input_report_key(data->input_dev, KEY_TP_WAKE, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_TP_WAKE, 0);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_C, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_C, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_S:
			gesture = 'x' ;
			input_report_key(data->input_dev, KEY_TP_WAKE, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_TP_WAKE, 0);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_S, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_S, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_V:
			gesture = 'v' ;
			input_report_key(data->input_dev, KEY_TP_WAKE, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_TP_WAKE, 0);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_V, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_V, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_Z:
        case 0x65: // two different firmware,two different gestrue_id
			gesture = 'z' ;
			input_report_key(data->input_dev, KEY_TP_WAKE, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_TP_WAKE, 0);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_Z, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_Z, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_A:
			gesture = 'a' ;
			input_report_key(data->input_dev, KEY_TP_WAKE, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_TP_WAKE, 0);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_A, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_A, 0);
			input_sync(data->input_dev);
			break;
		default:
			break;
	}
	msleep(250);
}
static int fts_read_Gestruedata(void)
{
    unsigned char buf[FTS_GESTURE_POINTS * 3] = { 0 };
    int ret = -1;
    int i = 0;
    buf[0] = 0xd3;
    int gestrue_id = 0;

    pointnum = 0;

    ret = fts_i2c_Read(this_client, buf, 1, buf, FTS_GESTURE_POINTS_HEADER);
                PRINT_DBG( "tpd read FTS_GESTURE_POINTS_HEADER.\n");

    if (ret < 0)
    {
        PRINT_DBG( "%s read touchdata failed.\n", __func__);
        return ret;
    }

    /* FW */
     if (fts_updateinfo_curr.CHIP_ID==0x54 || fts_updateinfo_curr.CHIP_ID==0x64)
     {
		gestrue_id = buf[0];
		pointnum = (short)(buf[1]) & 0xff;
		buf[0] = 0xd3;

		if((pointnum * 4 + 8)<255)
		{
	                ret = fts_i2c_Read(this_client, buf, 1, buf, (pointnum * 4 + 8));
		}
		else
		{
	                ret = fts_i2c_Read(this_client, buf, 1, buf, 255);
	                ret = fts_i2c_Read(this_client, buf, 0, buf+255, (pointnum * 4 + 8) -255);
		}
		if (ret < 0)
		{
	                PRINT_ERR( "%s read touchdata failed.\n", __func__);
	                return ret;
		}
		check_gesture(gestrue_id);
		for(i = 0;i < pointnum;i++)
		{
	                coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
	                8 | (((s16) buf[1 + (4 * i)])& 0xFF);
	                coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
	                8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
		}
		for(i = 0; i < pointnum; i++)
		{
	                PRINT_DBG("coordinate_x[%d]  = %d",i,coordinate_x[i] );
		}
		return -1;
    }

    if (0x24 == buf[0])

    {
        gestrue_id = 0x24;
        check_gesture(gestrue_id);
        return -1;
    }

    pointnum = (short)(buf[1]) & 0xff;
    buf[0] = 0xd3;

    if((pointnum * 4 + 8)<255)
    {
         ret = fts_i2c_Read(this_client, buf, 1, buf, (pointnum * 4 + 8));
    }
    else
    {
         ret = fts_i2c_Read(this_client, buf, 1, buf, 255);
         ret = fts_i2c_Read(this_client, buf, 0, buf+255, (pointnum * 4 + 8) -255);
    }
    if (ret < 0)
    {
        PRINT_ERR( "%s read touchdata failed.\n", __func__);
        return ret;
    }

   gestrue_id = fetch_object_sample(buf, pointnum);
   check_gesture(gestrue_id);

    for(i = 0;i < pointnum;i++)
    {
        coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
            8 | (((s16) buf[1 + (4 * i)])& 0xFF);
        coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
            8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
    }
    return -1;
}
#endif
#if USE_WAIT_QUEUE
static int touch_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = 5 };
	//u8 state;
	sched_setscheduler(current, SCHED_RR, &param);
	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, (0 != tpd_flag));
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);
#ifdef FTS_GESTURE
if (TP_gesture_Switch==true)
{
		i2c_smbus_read_i2c_block_data(this_client, 0xd0, 1, &state);
		PRINT_DBG( "after tpd fts_read_Gestruedata.state=%d..\n",state);
		if( state ==1)
		{

			fts_read_Gestruedata();
			continue;
		}
}
#endif
		fts_update_data();

	} while (!kthread_should_stop());

	return 0;
}
#endif

#if USE_WORK_QUEUE
static void fts_pen_irq_work(struct work_struct *work)
{
	fts_update_data();
	enable_irq(this_client->irq);
}
#endif

static irqreturn_t fts_interrupt(int irq, void *dev_id)
{
#if USE_WAIT_QUEUE
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
#endif

#if USE_WORK_QUEUE
	struct fts_data *fts = (struct fts_data *)dev_id;

	if (!work_pending(&fts->pen_event_work)) {
		queue_work(fts->fts_workqueue, &fts->pen_event_work);
	}
	return IRQ_HANDLED;
#endif

#if USE_THREADED_IRQ
	fts_update_data();
	return IRQ_HANDLED;
#endif

}

static int fts_reset_down(void)
{
	struct fts_platform_data *pdata = g_fts->platform_data;

	if (gpio_request(pdata->reset_gpio_number, "ft5x0x_reset")) {
			pr_err("Failed to request GPIO for touch_reset pin!\n");
			return -1;
		}
	gpio_direction_output(pdata->reset_gpio_number, 0);
	gpio_free(pdata->reset_gpio_number);
	return 0;
}

static void fts_reset(void)
{
	struct fts_platform_data *pdata = g_fts->platform_data;

	gpio_direction_output(pdata->reset_gpio_number, 1);
	msleep(1);
	gpio_set_value(pdata->reset_gpio_number, 0);
	msleep(10);
	gpio_set_value(pdata->reset_gpio_number, 1);
	msleep(200);
}

#ifdef TP_HAVE_PROX
static inline int ft5x0x_prox_ctl(int value)
{
	if(value == 0){
		PRINT_ERR("qiao_[elan]close the ps mod successful\n");
		is_incall = 0;
		ft5x0x_write_reg(0xB0,0x00);
	}else if(value == 1){
		PRINT_ERR("qiao_[elan]open the ps mod successful\n");
		is_incall = 1;
		ft5x0x_write_reg(0xB0,0x01);
	}else{
		PRINT_ERR("[elan]open or close the ps mod fail\n");
	}
	return 0;
}
#endif

static int fts_suspend(void)
{
	//struct fts_platform_data *pdata = g_fts->platform_data;
	int ret = -1;
    //int len;
	PRINT_INFO("==%s==\n", __FUNCTION__);
    printk("mxy fts_suspend\n");
#ifdef TP_HAVE_PROX
    if(is_incall == 1)
    {
        ps_suspend_flag = 1;
        printk("fts_suspend gesture prox is_incall!\n");
        return 0;
    }
#endif

#ifdef FTS_GESTURE
	struct file * fp=NULL;
	mm_segment_t fs;
	char buf[200]={0};
	char *p=NULL;
	int *buffer=NULL;

	fp = filp_open(GESTURE_SWITCH_FILE ,O_WRONLY|O_CREAT, 0644);
	if(IS_ERR(fp))
    {
		printk("wangcq327 --- touchpanle Open file fail !!\n");
		TP_gesture_Switch = true;
	}
    else
    {
        fs = get_fs();//get old fs;
        set_fs(KERNEL_DS);

        fp->f_pos = 0;
        len = fp->f_op->llseek(fp, 0, SEEK_END);
		/*check len*/
		printk("<0> len is %d\n", len);
        buffer = (int *) kzalloc(4, GFP_KERNEL);
		printk("<0>suspend 0\n");
		if (buffer == NULL)
		{
			printk("<0> kzlloc failed!!!!!,pp_suspend go back!\n");
			goto kz_failed;
		}
        fp->f_op->llseek(fp, 0, SEEK_SET);
		printk("<0>suspend 1\n");
        fp->f_op->read(fp, buffer, len, &fp->f_pos);
		printk("<0>suspend 2\n");
        filp_close(fp, NULL);
		printk("<0>suspend 3\n");
        set_fs(fs);
		printk("<0>suspend 4\n");

        if(*buffer == 1){
            TP_gesture_Switch = true;
        }else{
            TP_gesture_Switch = false;
        }
		printk("<0>suspend 5\n");
kz_failed:
        printk("wangcq327 --- TP_gesture_Switch == %d\n",(TP_gesture_Switch==true)?1:0);
	}

    if (TP_gesture_Switch == true)
    {
        PRINT_DBG("in gesture ts_suspend!\n");

        fts_write_reg(this_client, 0xd0, 0x01);
        if (fts_updateinfo_curr.CHIP_ID==0x54 || fts_updateinfo_curr.CHIP_ID==0x64)
        {
            fts_write_reg(this_client, 0xd1, 0xff);
            fts_write_reg(this_client, 0xd2, 0xff);
            fts_write_reg(this_client, 0xd5, 0xff);
            fts_write_reg(this_client, 0xd6, 0xff);
            fts_write_reg(this_client, 0xd7, 0xff);
            fts_write_reg(this_client, 0xd8, 0xff);
        }
        enable_irq_wake(this_client->irq);
        irq_set_irq_type(this_client->irq,IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND | IRQF_ONESHOT);
        //msleep(50);
    }
    else
    {
        ret = fts_write_reg(this_client, FTS_REG_PMODE, PMODE_HIBERNATE);
        if(ret)
        {
            PRINT_ERR("==fts_suspend==  fts_write_reg fail\n");
        }
        disable_irq(this_client->irq);
        fts_clear_report_data(g_fts);
        fts_reset_down();
    }
	return 0;
#else
    ret = fts_write_reg(this_client, FTS_REG_PMODE, PMODE_HIBERNATE);
    if(ret)
    {
        PRINT_ERR("==fts_suspend==  fts_write_reg fail\n");
    }
    disable_irq(this_client->irq);
    fts_clear_report_data(g_fts);
    fts_reset_down();
	printk("<0> delfino fts suspend func return!!!\n");
	return 0;
#endif
}

static int fts_resume(void)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct fts_data  *fts = (struct fts_data *)i2c_get_clientdata(this_client);
	queue_work(fts->fts_resume_workqueue, &fts->resume_work);
#elif defined(CONFIG_PM_SLEEP)
	//struct fts_data  *fts = (struct fts_data *)i2c_get_clientdata(this_client);
	//struct fts_platform_data *pdata = g_fts->platform_data;
	printk("mxy fts_resume\n");
	PRINT_INFO("==%s==\n", __FUNCTION__);
#ifdef TP_HAVE_PROX
    if(is_incall == 1 && ps_suspend_flag == 1) {
        ps_suspend_flag = 0;
        PRINT_INFO("fts_resume gesture prox is_incall");
        ft5x0x_prox_ctl(1);
        return 0;
    }
#endif

#ifdef FTS_GESTURE
    if(TP_gesture_Switch == true) {
        irq_set_irq_type(this_client->irq,IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
        fts_write_reg(this_client,0xD0,0x00);
        return;
    }
#endif

    fts_reset();
    enable_irq(this_client->irq);
    msleep(2);
    //fts_clear_report_data(g_fts);
#endif
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/* earlysuspend module the suspend/resume procedure */
static void fts_early_suspend(struct early_suspend *h)
{
	fts_suspend();
}

static void fts_late_resume(struct early_suspend *h)
{
	fts_resume();
}

static void fts_resume_work(struct work_struct *work)
{
	struct fts_data  *fts = (struct fts_data *)i2c_get_clientdata(this_client);
	struct fts_platform_data *pdata = g_fts->platform_data;

	pr_info("==%s==\n", __FUNCTION__);

#ifdef TP_HAVE_PROX
    if(is_incall == 1 && ps_suspend_flag == 1)
    {
        ps_suspend_flag = 0;
        PRINT_INFO("fts_resume gesture prox is_incall");
        ft5x0x_prox_ctl(1);
        return;
    }
#endif

#ifdef FTS_GESTURE
    if(TP_gesture_Switch == true)
    {
        irq_set_irq_type(this_client->irq,IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
        fts_write_reg(this_client,0xD0,0x00);
        return;
    }
#endif

    fts_reset();
    enable_irq(this_client->irq);
    msleep(2);
    //fts_clear_report_data(g_fts);
}

#elif defined(CONFIG_PM_SLEEP)
static int fts_pm_suspend(struct device *dev)
{
    return fts_suspend();
}

static int fts_pm_resume(struct device *dev)
{
	return fts_resume();
}

/* bus control the suspend/resume procedure */
static const struct dev_pm_ops fts_pm_ops = {
	SET_RUNTIME_PM_OPS(fts_pm_suspend, fts_pm_resume, NULL)
	//.suspend = fts_pm_suspend,
	//.resume = fts_pm_resume,
};
#endif

#ifdef TP_HAVE_PROX

static ssize_t show_proximity_sensor(struct device *dev, struct device_attribute *attr, char *buf)
{
	//static char temp=2;
	PRINT_ERR("tp get prox ver\n");
	if (buf != NULL) {
		sprintf(buf, "tp prox version\n");
	}

    return 0;
}

static ssize_t store_proximity_sensor(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{

	unsigned int on_off = simple_strtoul(buf, NULL, 10);
	//struct multi_tp_prv_data *p_prv_data = p_private_data;
	//struct multi_tp_cfg     *p_tp_cfg = p_private_data->p_tp_cfg;

	printk("focaltech cjx:qiao_store_proximity_sensor buf=%d,size=%d,on_off=%d\n", *buf, size, on_off);
	if(buf != NULL && size != 0)
	{
		if (0 == on_off)
		{
			ft5x0x_prox_ctl(0);
			is_incall = 0;
		}
		else if (1 == on_off)
		{
			ft5x0x_prox_ctl(1);
			is_incall = 1;
		}
	}

    return size;
}

static DEVICE_ATTR(proximity, S_IRUGO | S_IWUSR, show_proximity_sensor, store_proximity_sensor);
#endif

static ssize_t tp_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, FTS_PAGE_SIZE, "ft5x0x::chipid(0x%x)::fwversion(0x%x)\n",
		   g_chipid, g_fwversion);
}

static ssize_t tp_version_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;
    if (value) {
    }else {
    }
	return size;
}
static DEVICE_ATTR(tp_version, S_IRUGO | S_IWUSR, tp_version_show, tp_version_store);

//wangqiang add start, @2017/7/10 14:32:23
/************************************************************************
* Name: tpd_read_vendor_id
* Brief: get touchpanel vendor_id
* Input: void
* Output: vender_id
* Return:
***********************************************************************/
static unsigned char fts_read_vendor_id(void)
{
	unsigned char vendor_id;
	fts_read_reg(this_client, FTS_REG_FT5201ID, &vendor_id);
	PRINT_INFO("fts_read_vendor_id vendor_id = 0x%x\n", (int)vendor_id);

	return vendor_id;
}

/************************************************************************
* Name: tpd_show_chipinfo
* Brief: get touchpanel chipinfo
* Input: device, device attribute, char buf, char count
* Output: no
* Return:
***********************************************************************/
static ssize_t tpd_show_chipinfo(struct device *dev,struct device_attribute *attr, char *buf)
{
	char *chip_name = NULL;
	unsigned char cur_fm_ver, cur_vendor_id;

	PRINT_INFO("%s S", __func__);
	switch(g_chipid){
		case 0x55:
			chip_name = "FT5x06";
			break;
		case 0x08:
			chip_name = "FT5606";
			break;
		case 0x0a:
			chip_name = "FT5x16";
			break;
		case 0x06:
			chip_name = "FT6x06";
			break;
		case 0x36:
			chip_name = "FT6x36";
			break;
		case 0x64:
			chip_name = "FT6x36U";
			break;
		case 0x14:
			chip_name = "FT5336";
			break;
		case 0x13:
			chip_name = "FT3316";
			break;
		case 0x12:
			chip_name = "FT5436i";
			break;
		case 0x11:
			chip_name = "FT5336i";
			break;
		case 0x54:
			chip_name = "FT5x46";
			break;
		default:
			PRINT_ERR("%s unsupported chip_id(0x%x), pls check!!!\n", __func__, g_chipid);
			break;
	}

	//get vendor_id
	cur_vendor_id = fts_read_vendor_id();
	PRINT_INFO("current vendor id = 0x%x\n", (int)cur_vendor_id);

	//get firmware version
	cur_fm_ver = fts_read_fw_ver();
	PRINT_INFO("current firmware version = 0x%x\n", (int)cur_fm_ver);

	PRINT_INFO("%s E", __func__);
  	//chip||vendor_id||ic version||firmware version
  	return sprintf(buf, "%s||0x%X||%s||0x%X\n", chip_name, (int)cur_vendor_id, "NONE", (int)cur_fm_ver);
}
static DEVICE_ATTR(chipinfo, S_IRUGO, tpd_show_chipinfo, NULL);
//wangqiang add end @2017/7/10 14:32:29

static void fts_hw_init(struct fts_data *fts)
{
	struct regulator *reg_vdd;
	int error;
	struct i2c_client *client = fts->client;
	struct fts_platform_data *pdata = fts->platform_data;

	pr_info("[FST] %s [irq=%d];[rst=%d]\n",__func__,
		pdata->irq_gpio_number,pdata->reset_gpio_number);
	gpio_request(pdata->irq_gpio_number, "ts_irq_pin");
	gpio_request(pdata->reset_gpio_number, "ts_rst_pin");
	gpio_direction_output(pdata->reset_gpio_number, 1);
	gpio_direction_input(pdata->irq_gpio_number);

	reg_vdd = regulator_get(&client->dev, pdata->vdd_name);
	if (!WARN(IS_ERR(reg_vdd), "[FST] fts_hw_init regulator: failed to get %s.\n", pdata->vdd_name)) {
		if(!strcmp(pdata->vdd_name,"vdd18"))
		{
			regulator_set_voltage(reg_vdd,1800000,1800000);
		}
		if(!strcmp(pdata->vdd_name,"vdd28"))
		{
			regulator_set_voltage(reg_vdd, 2800000, 2800000);
		}
		error = regulator_enable(reg_vdd);
		if (error) {
			PRINT_ERR("Failed to enable reg_vdd: %d\n", error);
		}
	}
	msleep(100);
	fts_reset();
}

void focaltech_get_upgrade_array(struct i2c_client *client)
{

	u8 chip_id;
	u8 vendor_id;
	u32 i;

	i2c_smbus_read_i2c_block_data(client,FT_REG_CHIP_ID,1,&chip_id);
	i2c_smbus_read_i2c_block_data(client,FT_REG_VENDOR_ID,1,&vendor_id);
	chipId = chip_id;
	vendorId = vendor_id;
	printk("%s chip_id = %x\n", __func__, chip_id);
	printk("%s vendor_id = %x\n", __func__, vendor_id);
	for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info);i++)
	{
		#ifdef V33K_FIRMWARE_UPGRADE_2
		if((chip_id==fts_updateinfo[i].CHIP_ID)&&(vendor_id==fts_updateinfo[i].VENDOR_ID))
		{
			memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct Upgrade_Info));
			break;
		}
		#else
		if(chip_id==fts_updateinfo[i].CHIP_ID)
		{
			memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct Upgrade_Info));
			break;
		}
		#endif

	}

	if(i >= sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info))
	{
		#ifdef V33K_FIRMWARE_UPGRADE_2
		memcpy(&fts_updateinfo_curr, &fts_updateinfo[0], sizeof(struct Upgrade_Info));
		#else
		memcpy(&fts_updateinfo_curr, &fts_updateinfo[5], sizeof(struct Upgrade_Info));
		#endif
	}
}

#ifdef CONFIG_OF
static struct fts_platform_data *fts_parse_dt(struct device *dev)
{
	struct fts_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct fts_platform_data");
		return NULL;
	}
	pdata->reset_gpio_number = of_get_gpio(np, 0);
	if(pdata->reset_gpio_number < 0){
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	pdata->irq_gpio_number = of_get_gpio(np, 1);
	if(pdata->irq_gpio_number < 0){
		dev_err(dev, "fail to get irq_gpio_number\n");
		goto fail;
	}
	ret = of_property_read_string(np, "vdd_name", &pdata->vdd_name);
	if(ret){
		dev_err(dev, "fail to get vdd_name\n");
		goto fail;
	}
	ret = of_property_read_u32_array(np, "virtualkeys", pdata->virtualkeys, 12);
	if(ret){
		dev_err(dev, "fail to get virtualkeys\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_X", &pdata->TP_MAX_X);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_X\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_Y", &pdata->TP_MAX_Y);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_Y\n");
		goto fail;
	}

	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif


#ifdef CONFIG_SPRD_PH_INFO
extern char SPRD_TPInfo[];
#endif

#ifdef REVO_PM_TP_SYSFS
static ssize_t ts_suspend_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
  unsigned int input;

  if (kstrtouint(buf, 10, &input))
      return -EINVAL;

  if (input == 1)
      fts_suspend();
  else if (input == 0)
      fts_resume();
  else
      return -EINVAL;

  return count;
}
static DEVICE_ATTR(ts_suspend, 0664, NULL, ts_suspend_store);

static struct attribute *tp_sysfs_attrs[] = {
  &dev_attr_ts_suspend.attr,
  NULL,
};

static struct attribute_group tp_attr_group = {
  .attrs = tp_sysfs_attrs,
};

static int tp_sysfs_init(struct i2c_client *client)
{ 
  int err = 0;
	err = sysfs_create_group(&client->dev.kobj, &tp_attr_group);
	if (err < 0) {
		dev_err(&client->dev, "Fail to create debug files!");
		return -ENOMEM;
	}

	err = sysfs_create_link(NULL, &client->dev.kobj, "touchscreen");
	if (err < 0) {
		dev_err(&client->dev, "Failed to create link!");
		return -ENOMEM;
	}
  return err;
}

int glb_fts_tp_sysfs_init(struct i2c_client *client){
	return tp_sysfs_init(client);
}
#endif

static int fts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct fts_data *fts;
	struct input_dev *input_dev;
	struct fts_platform_data *pdata = client->dev.platform_data;
	int err = 0;
	unsigned char uc_reg_value;
	u8 is_have_tp;
	struct class *tp_class;
	struct device *tp_cmd_dev;
#ifdef CONFIG_OF
	struct device_node *np = client->dev.of_node;
#endif

	pr_info("[FST] %s: probe\n",__func__);
	is_have_tp = 0;
	if (is_have_tp) {
		printk("[FST] %s: probe is_have_tp\n",__func__);
		err = -1;
		return err;
	}
#ifdef CONFIG_OF
	if (np && !pdata){
		pdata = fts_parse_dt(&client->dev);
		if(pdata){
			client->dev.platform_data = pdata;
		}
		else{
			err = -ENOMEM;
			goto exit_alloc_platform_data_failed;
		}
	}
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	fts = kzalloc(sizeof(*fts), GFP_KERNEL);
	if (!fts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	g_fts = fts;
	fts->platform_data = pdata;
	this_client = client;
	fts->client = client;
	fts_hw_init(fts);
	i2c_set_clientdata(client, fts);
	//client->irq = gpio_to_irq(pdata->irq_gpio_number);

	#if(defined(CONFIG_I2C_SPRD) ||defined(CONFIG_I2C_SPRD_V1))
	//sprd_i2c_ctl_chg_clk(client->adapter->nr, 400000);
	#endif

	err = fts_read_reg(this_client, FTS_REG_CIPHER, &uc_reg_value);
	if (err < 0)
	{
		pr_err("[FST] read chip id error %x, try again\n", uc_reg_value);
		msleep(100);
		err = fts_read_reg(this_client, FTS_REG_CIPHER, &uc_reg_value);
		if (err < 0)
		{
			pr_err("[FST] read chip id error %x, quit\n", uc_reg_value);
			err = -ENODEV;
			goto exit_chip_check_failed;
		}
	}
	
	g_chipid = uc_reg_value;
	client->irq = gpio_to_irq(pdata->irq_gpio_number);

#if USE_WORK_QUEUE
	INIT_WORK(&fts->pen_event_work, fts_pen_irq_work);

	fts->fts_workqueue = create_singlethread_workqueue("focal-work-queue");
	if (!fts->fts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	INIT_WORK(&fts->resume_work, fts_resume_work);
	fts->fts_resume_workqueue = create_singlethread_workqueue("fts_resume_work");
	if (!fts->fts_resume_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
#endif
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "[FST] failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
#ifdef TOUCH_VIRTUAL_KEYS
	fts_virtual_keys_init();
#endif
	fts->input_dev = input_dev;

	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	__set_bit(KEY_APPSELECT,  input_dev->keybit);
	__set_bit(KEY_BACK,  input_dev->keybit);
	__set_bit(KEY_HOMEPAGE,  input_dev->keybit);
	__set_bit(KEY_POWER,  input_dev->keybit);
#ifdef FTS_GESTURE
	__set_bit(KEY_TP_WAKE,  input_dev->keybit);
#endif
	__set_bit(KEY_LEFT,  input_dev->keybit);
	__set_bit(KEY_RIGHT,  input_dev->keybit);
	__set_bit(KEY_UP,  input_dev->keybit);
	__set_bit(KEY_DOWN,  input_dev->keybit);
	__set_bit(KEY_U,  input_dev->keybit);
	__set_bit(KEY_O,  input_dev->keybit);
	__set_bit(KEY_W,  input_dev->keybit);
	__set_bit(KEY_M,  input_dev->keybit);
	__set_bit(KEY_E,  input_dev->keybit);
	__set_bit(KEY_C,  input_dev->keybit);
	__set_bit(KEY_S,  input_dev->keybit);
	__set_bit(KEY_V,  input_dev->keybit);
	__set_bit(KEY_Z,  input_dev->keybit);
	__set_bit(KEY_A,  input_dev->keybit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

#if MULTI_PROTOCOL_TYPE_B
	input_mt_init_slots(input_dev, TS_MAX_FINGER,0);
#endif
	//#if (PRJ_FEATURE_H_BOARD_DISP_ROTATION == 90)
	#if 0//(PRJ_FEATURE_H_BOARD_DISP_ROTATION == 90)
	input_set_abs_params(input_dev,ABS_MT_POSITION_X, 0, pdata->TP_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_POSITION_Y, 0, pdata->TP_MAX_X, 0, 0);
	#else
	input_set_abs_params(input_dev,ABS_MT_POSITION_X, 0, pdata->TP_MAX_X, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_POSITION_Y, 0, pdata->TP_MAX_Y, 0, 0);
	#endif

	input_set_abs_params(input_dev,ABS_MT_TOUCH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_PRESSURE, 0, 127, 0, 0);
#if !MULTI_PROTOCOL_TYPE_B
	input_set_abs_params(input_dev,ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#endif

#ifdef FTS_GESTURE
	input_set_capability(input_dev, EV_KEY, KEY_TP_WAKE);
#endif
	input_set_capability(input_dev, EV_KEY, KEY_F13);
	input_set_capability(input_dev, EV_KEY, KEY_F14);
	input_set_capability(input_dev, EV_KEY, KEY_F15);
	input_set_capability(input_dev, EV_KEY, KEY_F16);
	input_set_capability(input_dev, EV_KEY, KEY_F18);
	input_set_capability(input_dev, EV_KEY, KEY_F19);
	input_set_capability(input_dev, EV_KEY, KEY_F20);
	input_set_capability(input_dev, EV_KEY, KEY_POWER);

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	input_dev->name = FOCALTECH_TS_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"[FST] fts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#if USE_THREADED_IRQ
	err = request_threaded_irq(client->irq, NULL, fts_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND, client->name, fts);
#else
	err = request_irq(client->irq, fts_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND, client->name, fts);
#endif
	if (err < 0) {
		dev_err(&client->dev, "[FST] ft5x0x_probe: request irq failed %d\n",err);
		goto exit_irq_request_failed;
	}
#ifdef REVO_PM_TP_SYSFS
    err = glb_fts_tp_sysfs_init(client);
    if (err) {
        dev_err(&client->dev, "could not create sysfs device attributes , < ts_suspend >");
    }
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	fts->early_suspend.level 	= EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	fts->early_suspend.suspend 	= fts_early_suspend;
	fts->early_suspend.resume	= fts_late_resume;
	register_early_suspend(&fts->early_suspend);
#endif

focaltech_get_upgrade_array(client);

#ifdef SYSFS_DEBUG
fts_create_sysfs(client);
#endif

#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(client) < 0)
	{
		dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",	__func__);
	}
#endif

#ifdef FTS_GESTURE
	init_para(pdata->TP_MAX_X,pdata->TP_MAX_Y,0,0,0);
#endif
#ifdef SPRD_AUTO_UPGRADE
	printk("********************Enter CTP Auto Upgrade********************\n");
	fts_ctpm_auto_upgrade(client);
#endif
	err = ft5x0x_read_reg(FTS_REG_FIRMID, &uc_reg_value);
	g_fwversion = uc_reg_value;
	err = ft5x0x_read_reg(0xA8, &uc_reg_value);
	g_vendorid = uc_reg_value;

#ifdef APK_DEBUG
	fts_create_apk_debug_channel(client);
#endif

#if USE_WAIT_QUEUE
	thread = kthread_run(touch_event_handler, 0, "focal-wait-queue");
	if (IS_ERR(thread))
	{
		err = PTR_ERR(thread);
		PRINT_ERR("failed to create kernel thread: %d\n", err);
	}
#endif
#ifdef TP_HAVE_PROX
	firmware_class = class_create(THIS_MODULE,"sprd-tpd");//client->name

	if(IS_ERR(firmware_class))
		PRINT_ERR("Failed to create class(firmware)!\n");
	firmware_cmd_dev = device_create(firmware_class, NULL, 0, NULL, "device");//device

	if(IS_ERR(firmware_cmd_dev))
		PRINT_ERR("Failed to create device(firmware_cmd_dev)!\n");

	if(device_create_file(firmware_cmd_dev, &dev_attr_proximity) < 0) // /sys/class/sprd-tpd/device/proximity
	{
		PRINT_ERR("Failed to create device file(%s)!\n", dev_attr_proximity.attr.name);
	}

	prx_wake_lock_ps = wakeup_source_register("prx_wake_lock");

	//setup prox input device
	proximity_input_dev = input_allocate_device();

	//p_tp_param = &p_tp_cfg->tp;
	proximity_input_dev->name = PROXIMITY_INPUT_DEV; //p_tp_cfg->name;
	proximity_input_dev->phys = PROXIMITY_INPUT_DEV;
	proximity_input_dev->id.bustype = BUS_I2C;
	proximity_input_dev->dev.parent = &fts->client->dev;
	__set_bit(EV_ABS, proximity_input_dev->evbit);
	input_set_abs_params(proximity_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	if (!proximity_input_dev)
	{
		err = -ENOMEM;
		PRINT_ERR("%s: failed to allocate input device\n", __func__);
		goto exit_input_dev_alloc_failed;
	}
	err = input_register_device(proximity_input_dev);
	if (err)
	{
		input_put_device(proximity_input_dev);
		PRINT_ERR("Unable to register prox device %d!", err);
		goto exit_input_dev_alloc_failed;
	}

#endif
	tp_class = class_create(THIS_MODULE,"xr-tp");//client->name
	if(IS_ERR(tp_class))
		PRINT_ERR("Failed to create class(version)!\n");

	tp_cmd_dev = device_create(tp_class, NULL, 0, NULL, "device");//device
	if(IS_ERR(tp_cmd_dev))
		PRINT_ERR("Failed to create device(tp_cmd_dev)!\n");

	if(device_create_file(tp_cmd_dev, &dev_attr_tp_version) < 0) // /sys/class/xr-tp/device/tp_version
		PRINT_ERR("Failed to create device file(%s)!\n", dev_attr_tp_version.attr.name);

	if(device_create_file(tp_cmd_dev, &dev_attr_chipinfo) < 0) // /sys/class/xr-tp/device/chipinfo
		PRINT_ERR("Failed to create device file(%s)!\n", dev_attr_chipinfo.attr.name);

#ifdef FTS_GESTURE
	if(device_create_file(tp_cmd_dev, &dev_attr_gesture) < 0) // /sys/class/xr-tp/device/gesture
		PRINT_ERR("Failed to create gesture device file()!\n");
#endif

#ifdef CONFIG_SPRD_PH_INFO
	memset((void *)SPRD_TPInfo, 0, 100);
	memcpy(SPRD_TPInfo, FOCALTECH_TS_NAME, 100);
#endif
	is_have_tp = 1;
	printk("fts probe ok!!!!!!");
	return 0;

exit_irq_request_failed:
#ifdef TP_HAVE_PROX
    wakeup_source_unregister(prx_wake_lock_ps);
#endif
	input_unregister_device(input_dev);
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
#if (defined(CONFIG_HAS_EARLYSUSPEND) || USE_WORK_QUEUE)
exit_create_singlethread:
#endif
exit_chip_check_failed:
	gpio_free(pdata->irq_gpio_number);
	gpio_free(pdata->reset_gpio_number);
	kfree(fts);
	g_fts = NULL;
	this_client = NULL;
exit_alloc_data_failed:
exit_check_functionality_failed:
	fts = NULL;
	i2c_set_clientdata(client, fts);
exit_alloc_platform_data_failed:
	return err;
}

static int fts_remove(struct i2c_client *client)
{
	struct fts_data *fts = i2c_get_clientdata(client);

	pr_info("==fts_remove=\n");
	#ifdef FTS_GESTURE
	device_remove_file(&client->dev, &dev_attr_gesture);
	#endif

	#ifdef SYSFS_DEBUG
	fts_release_sysfs(client);
	#endif
	#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
	#endif
	#ifdef APK_DEBUG
	fts_release_apk_debug_channel();
	#endif

#ifdef TP_HAVE_PROX
	input_unregister_device(proximity_input_dev);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&fts->early_suspend);
#endif
	free_irq(client->irq, fts);
	input_unregister_device(fts->input_dev);
	input_free_device(fts->input_dev);
#if USE_WORK_QUEUE
	cancel_work_sync(&fts->pen_event_work);
	destroy_workqueue(fts->fts_workqueue);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	cancel_work_sync(&fts->resume_work);
	destroy_workqueue(fts->fts_resume_workqueue);
#endif
	kfree(fts);
	fts = NULL;
	i2c_set_clientdata(client, fts);

	return 0;
}

static const struct i2c_device_id fts_id[] = {
	{ FOCALTECH_TS_NAME, 0 },{ }
};
MODULE_DEVICE_TABLE(i2c, fts_id);

static const struct of_device_id focaltech_of_match[] = {
       { .compatible = "focaltech,focaltech_ts", FOCALTECH_TS_NAME,},
       { }
};
MODULE_DEVICE_TABLE(of, focaltech_of_match);
static struct i2c_driver fts_driver = {
	.probe		= fts_probe,
	.remove		= fts_remove,
	.id_table	= fts_id,
	.driver	= {
		.name	= FOCALTECH_TS_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = focaltech_of_match,
#if defined(CONFIG_PM_SLEEP)
		.pm = &fts_pm_ops,
#endif
	},
	//.suspend = fts_suspend,
	//.resume = fts_resume,
};

static int __init fts_init(void)
{
#if defined(CONFIG_BOARD_L30)
	//130+128
	#define TP_2V8 258
	if(gpio_request(TP_2V8, "tp_2v8") == 0)
	{
		gpio_direction_output(TP_2V8, 1);
	}
#endif
	return i2c_add_driver(&fts_driver);
}

static void __exit fts_exit(void)
{
	i2c_del_driver(&fts_driver);
}

module_init(fts_init);
module_exit(fts_exit);

MODULE_AUTHOR("<mshl>");
MODULE_DESCRIPTION("FocalTech fts TouchScreen driver");
MODULE_LICENSE("GPL");
