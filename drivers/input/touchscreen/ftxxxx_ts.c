/* drivers/input/touchscreen/ftxxxx_ts.c
*
* FocalTech ftxxxx TouchScreen driver.
*
* Copyright (c) 2014  Focaltech Ltd.
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
*/

#include <linux/i2c.h>
#include <linux/input.h>
/*#include <linux/earlysuspend.h>*/
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>/*ori #include <mach/irqs.h>*/
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/input/mt.h>
#include "ftxxxx_ts.h"
#include <linux/switch.h>
#include <linux/gpio.h>/*ori #include <mach/gpio.h>*/
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/proc_fs.h>







/**add by jinpeng_He for early_suspend+++**/
//#if defined(CONFIG_FB)
//#include <linux/notifier.h>
//#include <linux/fb.h>
//struct notifier_block tp_fb_notif;

//static int asus_otg_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);

//#endif

/*#include "linux/input/proximity_class.h"*/
/*#include <linux/ft3x17.h>*/
#define SYSFS_DEBUG
/*#define FTS_APK_DEBUG		not support now*/
#define FTS_PM
#define FTS_CTL_IIC 
#ifdef ASUS_FACTORY_BUILD

#else
#define FTS_GESTRUE
#endif
/*#define FTXXXX_ENABLE_IRQ*/

/*#define CONFIG_PM*/

#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif

#define PROC_AsusTouchDisable_name "asus_touch_proximity_status"

#ifdef FTS_GESTRUE
/*zax 20140922*/
#define KEY_GESTURE_U		KEY_POWER
#define KEY_GESTURE_UP		KEY_UP
#define KEY_GESTURE_DOWN		KEY_DOWN
#define KEY_GESTURE_LEFT		KEY_LEFT
#define KEY_GESTURE_RIGHT		KEY_RIGHT
#define KEY_GESTURE_M		KEY_M
#define KEY_GESTURE_L		KEY_L
/*asus use*/
#define KEY_GESTURE_V		KEY_V
#define KEY_GESTURE_Z		KEY_Z
#define KEY_GESTURE_C		KEY_C
#define KEY_GESTURE_E		KEY_E
#define KEY_GESTURE_S		KEY_S
#define KEY_GESTURE_W		KEY_W
/*asus use*/

#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP			0x22
#define GESTURE_DOWN		0x23
#define GESTURE_M			0x32
#define GESTURE_L			0x44
#define GESTURE_S			0x46
#define GESTURE_V			0x54
#define GESTURE_Z			0x65
#define GESTURE_C			0x34
#define GESTURE_E			0x33
#define GESTURE_O			0x30
#define GESTURE_W			0x31

#define FTS_GESTRUE_POINTS 255
#define FTS_GESTRUE_POINTS_ONETIME 62
#define FTS_GESTRUE_POINTS_HEADER 8
#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4

short pointnum = 0;
unsigned short coordinate_x[150] = {0};
unsigned short coordinate_y[150] = {0};
static bool dclick_flags;

int gestrue_id = 0;
//static u32 touch_control;
#endif

#ifdef SYSFS_DEBUG
#include "ftxxxx_ex_fun.h"
#endif

/*ASUS_BSP jacob kung: add for debug mask +++ */
#include <linux/module.h>
/* Debug levels */
#define NO_DEBUG       0
#define DEBUG_POWER     1
#define DEBUG_INFO  2
#define DEBUG_VERBOSE 5
#define DEBUG_RAW      8
#define DEBUG_TRACE   10

static int debug = DEBUG_INFO;
/**************add by jinpeng_He begin*********************/
static int g_ASUS_hwID=1;
static int ZE500KL_SR1=0;
u32 selftestflag=0;
/**************add by jinpeng_He end****************************/
module_param(debug, int, 0644);

MODULE_PARM_DESC(debug, "Activate debugging output");

#define focal_debug(level, ...) do { if (debug >= (level)) pr_info(__VA_ARGS__); } while (0)
/*ASUS_BSP jacob kung: add for debug mask --- */

volatile bool suspend_resume_process;
static bool disable_tp_flag;
int focal_init_success = 0;
bool FOCAL_IRQ_DISABLE = true;
int TPID = -1;
int Focal_hw_id = -1;
u8 B_VenderID;
u8 F_VenderID;
char B_projectcode[8];
u8 F_projectcode;
u8 Gesture_flag;
u8 FTS_gesture_register_d2;
u8 FTS_gesture_register_d5;
u8 FTS_gesture_register_d6;
u8 FTS_gesture_register_d7;
//u8 FTS_gesture_register_d0;

/* +++ asus jacob add for print touch location +++ */
//int report_touch_locatoin_count[10];
/* --- asus jacob add for print touch location --- */
//#ifdef FTS_PM
//static int ftxxxx_ts_suspend(struct device *dev);
//static int ftxxxx_ts_resume(struct device *dev);
//#endif
struct ftxxxx_ts_data *ftxxxx_ts;
//static bool touch_down_up_status;

#define TOUCH_MAX_X						720
#define TOUCH_MAX_Y						1280

#define ANDROID_INPUT_PROTOCOL_B

/* jacob add for i2c retey and if i2c error countor > 10 reset IC */
#define IICReadWriteRetryTime	3
static int IICErrorCountor = 0;
/* jacob add for i2c retey  and if i2c error countor > 10 reset IC */

/*#define FTXXXX_RESET_PIN	88//EXYNOS4_GPJ0(3) //S5PV210_GPB(2)*/
#define FTXXXX_RESET_PIN_NAME	"ft5x46-rst"
/*#define FTXXXX_INT_PIN	62//EXYNOS4_GPJ0(3) //S5PV210_GPB(2)*/
#define FTXXXX_INT_PIN_NAME	"ft5x46-int"

//<asus-Jeffery20151202+>
extern bool keyboard_enable;
struct point_first_location first_location[CFG_MAX_TOUCH_POINTS];
//<asus-Jeffery20151202->

extern bool proximity_check_status(void);

/*
*ftxxxx_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int ftxxxx_i2c_Read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret;
	int retry = 0;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD, //read flags,if there is no then will write
				.len = readlen,
				.buf = readbuf,
			},
		};

		for (retry = 0; retry < IICReadWriteRetryTime; retry++) {
			ret = i2c_transfer(client->adapter, msgs, 2);

			if (ret >= 0)
				break;

			msleep(1);
		}
	} else {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};

		for (retry = 0; retry < IICReadWriteRetryTime; retry++) {
			ret = i2c_transfer(client->adapter, msgs, 1);

			if (ret >= 0)
				break;

			msleep(1);
		}
	}

	if (retry == IICReadWriteRetryTime) {
		dev_err(&client->dev, "[Focal][Touch] %s: i2c read error.  error code = %d \n", __func__, ret);
		IICErrorCountor += 1;

		if (IICErrorCountor >= 10) {
			dev_err(&client->dev, "[Focal][Touch] %s: i2c read/write error over 10 times !! \n", __func__);
			dev_err(&client->dev, "[Focal][Touch] %s: excute reset IC process !! \n", __func__);
//			ASUSEvtlog("[Touch] touch i2c read/write error over 10 times, reset IC \n");
			queue_work(ftxxxx_ts->reset_wq, &ftxxxx_ts->reset_ic_work);
			return ret;
		}

		return ret;
	}

	IICErrorCountor = 0;

	return ret;
}
/*write data by i2c*/
int ftxxxx_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;
	int retry = 0;
	if(writelen>0)
	{
		struct i2c_msg msg[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
		};

		for (retry = 0; retry < IICReadWriteRetryTime; retry++) {
			ret = i2c_transfer(client->adapter, msg, 1);

			if (ret >= 0)
				break;

			msleep(1);
		}
	}
	if (retry == IICReadWriteRetryTime) {
		dev_err(&client->dev, "[Focal][Touch] %s: i2c write error.  error code = %d \n", __func__, ret);

		IICErrorCountor += 1;

		if (IICErrorCountor >= 10) {
			dev_err(&client->dev, "[Focal][Touch] %s: i2c read/write error over 10 times !! \n", __func__);
			dev_err(&client->dev, "[Focal][Touch] %s: excute reset IC process !! \n", __func__);
//			ASUSEvtlog("[Touch] touch i2c read/write error over 10 times, reset IC \n");			
			queue_work(ftxxxx_ts->reset_wq, &ftxxxx_ts->reset_ic_work);
			return ret;
		}

		return ret;
	}

	IICErrorCountor = 0;

	return ret;
}

/*ASUS_BSP Jacob : add for creating virtual_key_maps +++*/
#define VIRTURAL_KEY
#ifdef VIRTURAL_KEY

#define MAX_LEN		200
#define FT5x06_KEY_HOME 102
#define FT5x06_KEY_BACK 158
#define FT5x06_KEY_MENU 580



static ssize_t focalTP_virtual_keys_register(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{

	char *virtual_keys ;
	switch (asus_PRJ_ID) {
		//<asus-Jeffery20150323+>
		case ASUS_ZE600KL:
			#ifdef ZE600KL_HD
			virtual_keys = 	__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_BACK) ":160:1330:148:72" "\n" \

						__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_HOME) ":360:1330:172:72" "\n" \

						__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_MENU)   ":560:1330:148:72" "\n" ;			
			#endif
			#ifdef ZE601KL_FHD
			virtual_keys= 	__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_BACK) ":240:1995:222:108" "\n" \

						__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_HOME) ":540:1995:258:108" "\n" \

						__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_MENU)   ":840:1995:222:108" "\n" ;
			#endif
			break;
		//<asus-Jeffery20150323->
		case ASUS_ZX550KL:
			virtual_keys= 	__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_BACK) ":215:2045:260:250" "\n" \

						__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_HOME) ":540:2045:260:250" "\n" \

						__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_MENU)   ":865:2045:260:250" "\n" ;
			break;
		case ASUS_ZD550KL:
			virtual_keys= 	__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_BACK) ":215:2045:260:250" "\n" \

						__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_HOME) ":540:2045:260:250" "\n" \

						__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_MENU)   ":865:2045:260:250" "\n" ;
			break;
		case ASUS_ZE550KL:
		default:
			#ifdef ZE550KL_HD
			virtual_keys = 	__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_BACK) ":140:1341:180:100" "\n" \

						__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_HOME) ":360:1341:170:100" "\n" \

						__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_MENU)   ":580:1341:180:100" "\n" ;
			#endif
			#ifdef ZE551KL_FHD
			virtual_keys= 	__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_BACK) ":215:2061:260:250" "\n" \

						__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_HOME) ":540:2061:260:250" "\n" \

						__stringify(EV_KEY) ":" __stringify(FT5x06_KEY_MENU)   ":865:2061:260:250" "\n" ;
			#endif
			break;
	}

	return snprintf(buf, strnlen(virtual_keys, MAX_LEN) + 1 , "%s",	virtual_keys);

}

static struct kobj_attribute focalTP_virtual_keys_attr = {

	.attr = {
		.name = "virtualkeys.ft5x06_ts", //here the ft5x06_ts is the same as input_dev.name
		.mode = S_IRWXU | S_IRWXG | S_IROTH,
	},

	.show = &focalTP_virtual_keys_register,

};

static struct attribute *virtual_key_properties_attrs[] = {

	&focalTP_virtual_keys_attr.attr,

	NULL

};

static struct attribute_group virtual_key_properties_attr_group = {

	.attrs = virtual_key_properties_attrs,

};

struct kobject *focal_virtual_key_properties_kobj;
#endif

/*ASUS_BSP Jacob : add for creating virtual_key_maps ---*/

u8 get_focal_tp_fw(void)
{
	u8 fwver = 0;

	if (ftxxxx_read_reg(ftxxxx_ts->client, FTXXXX_REG_FW_VER, &fwver) < 0)
		return -1;
	else
		return fwver;
}
/**add by jinpeng_he ++++**/
struct touch_fw_version
{
	u8 project_id;
	char glass_vendor[10];
	u8 firmware_version;
	u8 vendor_id;
};
u8 get_focal_tp_version(struct touch_fw_version *touch_version)
{
	int ret;
	memset(touch_version,0,sizeof(struct touch_fw_version));
	ret=ftxxxx_read_reg(ftxxxx_ts->client,FTXXXX_REG_PROJECT_ID,&touch_version->project_id);
	if(ret<0)
		return ret;
	ret=ftxxxx_read_reg(ftxxxx_ts->client,FTXXXX_REG_FW_VER,&touch_version->firmware_version);
	if(ret<0)
		return ret;
	ret=ftxxxx_read_reg(ftxxxx_ts->client,FTXXXX_REG_VENDOR_ID,&touch_version->vendor_id);
	if(ret<0)
		return ret;
	switch (asus_PRJ_ID) 
	{
		//<asus-Jeffery20150408+>
		case 1:
			if(ftxxxx_ts->tp_id_value2 == 0){
				memcpy(&touch_version->glass_vendor,"TPK",strlen("TPK")+1);
			}
			else{
				memcpy(&touch_version->glass_vendor,"Jtouch",strlen("Jtouch")+1);
			}
			break;
		//<asus-Jeffery20150408->
		case 3:
			if (ftxxxx_ts->tp_id_value1 == 0 && ftxxxx_ts->tp_id_value2 == 0)
			{
				memcpy(&touch_version->glass_vendor,"Jtouch",strlen("Jtouch")+1);
			}
			else if(ftxxxx_ts->tp_id_value1 == 0 && ftxxxx_ts->tp_id_value2 == 1) 
			{
				memcpy(&touch_version->glass_vendor,"GIS",strlen("GIS")+1);
			}
			break;
		case 2:
		case 0:
		default:
			if(ftxxxx_ts->tp_id_value1==1&&ftxxxx_ts->tp_id_value2==1)
			{
				memcpy(&touch_version->glass_vendor,"TPK",strlen("TPK")+1);
			}
			else
			{
				memcpy(&touch_version->glass_vendor,"GIS",strlen("GIS")+1);
			}
			break;
	}
	return ret;
}
/**add by jinpeng_he ----**/

static ssize_t focal_show_tpfwver(struct switch_dev *sdev, char *buf)
{
	int num_read_chars = 0;
	int ret;
	struct touch_fw_version touch_version;
	ret=get_focal_tp_version(&touch_version);
	
	if (ret<0) {
		printk("[Focal][Touch] %s :  read FW fail \n ", __func__);
		num_read_chars = snprintf(buf, PAGE_SIZE, "get tp fw version fail!\n");
	} else {
		printk("[Focal][Touch] touch_version: = %s-0x%x-0x%x\n ",touch_version.glass_vendor,touch_version.project_id,touch_version.firmware_version);
		num_read_chars = snprintf(buf, PAGE_SIZE, "0x%02X-0x%02X-0x%02X\n",touch_version.project_id,touch_version.vendor_id,touch_version.firmware_version);
	}
	return num_read_chars;
}

#ifdef FTS_GESTRUE/*zax 20140922*/
static void check_gesture(struct ftxxxx_ts_data *data, int gesture_id)
{
	bool Ps_status = false;

//	printk(KERN_EMERG "[Focal][Touch] %s :  gesture_id = 0x%x\n ", __func__, gesture_id);
		if(!ftxxxx_ts->cover_mode_states)
			Ps_status = proximity_check_status();
	if (!Ps_status) {
		switch (gesture_id) {
		/* ++++ touch gesture mode support part in ZE500CL ++++ */
		
			case GESTURE_DOUBLECLICK:
				if(dclick_flags==true)
				{
					input_report_key(data->input_dev, KEY_GESTURE_U, 1);
					input_sync(data->input_dev);
					input_report_key(data->input_dev, KEY_GESTURE_U, 0);
					input_sync(data->input_dev);
					printk(KERN_EMERG "[Focal][Touch] double click\n");
					dclick_flags=false;
				}
				break;
			case GESTURE_V:
				input_report_key(data->input_dev, KEY_GESTURE_V, 1);
				input_sync(data->input_dev);
				input_report_key(data->input_dev, KEY_GESTURE_V, 0);
				input_sync(data->input_dev);
				printk(KERN_EMERG"[Focal][Touch] click GESTURE_V\n");
			
				break;

			case GESTURE_Z:
				input_report_key(data->input_dev, KEY_GESTURE_Z, 1);
				input_sync(data->input_dev);
				input_report_key(data->input_dev, KEY_GESTURE_Z, 0);
				input_sync(data->input_dev);
				printk(KERN_EMERG "[Focal][Touch] click GESTURE_Z\n");
				
				break;
			case GESTURE_E:
				input_report_key(data->input_dev, KEY_GESTURE_E, 1);
				input_sync(data->input_dev);
				input_report_key(data->input_dev, KEY_GESTURE_E, 0);
				input_sync(data->input_dev);
				printk(KERN_EMERG "[Focal][Touch] click GESTURE_E\n");
				
				break;
			case GESTURE_C:
				input_report_key(data->input_dev, KEY_GESTURE_C, 1);
				input_sync(data->input_dev);
				input_report_key(data->input_dev, KEY_GESTURE_C, 0);
				input_sync(data->input_dev);
				printk(KERN_EMERG "[Focal][Touch] click GESTURE_C\n");
				
				break;
			case GESTURE_S:
				input_report_key(data->input_dev, KEY_GESTURE_S, 1);
				input_sync(data->input_dev);
				input_report_key(data->input_dev, KEY_GESTURE_S, 0);
				input_sync(data->input_dev);
				printk(KERN_EMERG "[Focal][Touch] click GESTURE_S\n");
				
				break;

			case GESTURE_W:
				input_report_key(data->input_dev, KEY_GESTURE_W, 1);
				input_sync(data->input_dev);
				input_report_key(data->input_dev, KEY_GESTURE_W, 0);
				input_sync(data->input_dev);
				printk(KERN_EMERG "[Focal][Touch] click GESTURE_W\n");
				
				break;
			default:
				
				break;
		}
	} else {
		printk(KERN_EMERG "[Focal][Touch] %s :  Skip wake up devices !\n ", __func__);
	}
}


static int fts_read_Gestruedata(struct ftxxxx_ts_data *data)
{
	unsigned char buf[FTS_GESTRUE_POINTS * 3] = { 0 };
	int ret = -1;
	int i = 0;
	int gestrue_id = 0;
	buf[0] = 0xd3;

	pointnum = 0;

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
	printk("[Focal][Touch] %s : tpd read FTS_GESTRUE_POINTS_HEADER.\n", __func__);

	if (ret < 0) {
		printk("[Focal][Touch] %s : read touchdata failed.\n", __func__);
		return ret;
	}

	/* FW */
	/*if (fts_updateinfo_curr.CHIP_ID==0x54)*/
	/*{*/
		gestrue_id = buf[0];
		pointnum = (short)(buf[1]) & 0xff;
		buf[0] = 0xd3;

		if ((pointnum * 4 + 8) < 255) {
			ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, (pointnum * 4 + 8));
		} else {
			ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, 255);
			ret = ftxxxx_i2c_Read(data->client, buf, 0, buf+255, (pointnum * 4 + 8) - 255);
		}
		if (ret < 0) {
			printk("[Focal][Touch] %s read touchdata failed.\n", __func__);
			return ret;
		}
	check_gesture(data, gestrue_id);
	for (i = 0; i < pointnum; i++) {
		coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
		8 | (((s16) buf[1 + (4 * i)]) & 0xFF);
		coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
		8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	}
	return -1;
/*}*/
/*
	if (0x24 == buf[0]) {
		gestrue_id = 0x24;
		check_gesture(gestrue_id);
		return -1;
	}

	pointnum = (short)(buf[1]) & 0xff;
	buf[0] = 0xd3;

	if((pointnum * 4 + 8)<255) {
		ret = fts_i2c_Read(i2c_client, buf, 1, buf, (pointnum * 4 + 8));
	}
	else
	{
		 ret = fts_i2c_Read(i2c_client, buf, 1, buf, 255);
		 ret = fts_i2c_Read(i2c_client, buf, 0, buf+255, (pointnum * 4 + 8) -255);
	}
	if (ret < 0)
	{
		printk( "%s read touchdata failed.\n", __func__);
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
	return -1;*/
}
#endif

/*static int ftxxxx_read_Touchdata(struct ftxxxx_ts_data *data)
{
	unsigned char buf[FTS_GESTRUE_POINTS * 2] = { 0 };
	int ret = -1;
	int i = 0;
	buf[0] = 0xd3;

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, 8);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n", __func__);
		return ret;
	}

	if (0x24 == buf[0]) {
		gestrue_id = 0x24;
		printk(KERN_WARNING "The Gesture ID is %x %x \n",gestrue_id,pointnum);
		return -1;
	}

	pointnum = (short)(buf[1]) & 0xff;

	buf[0] = 0xd3;

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, (pointnum * 4 + 8));
	Read two times
	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, (8));
	ret = ftxxxx_i2c_Read(data->client, buf, 0, (buf+8), (8));

	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n", __func__);
		return ret;
	}

	gestrue_id = fetch_object_sample(buf,pointnum);

	printk(KERN_WARNING "The Gesture ID is %x %x \n",gestrue_id,pointnum);

	for(i = 0;i < pointnum;i++)
	{
		coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[1 + (4 * i)])& 0xFF);
		coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	}

	return -1;
}*/

/*Read touch point information when the interrupt  is asserted.*/
static int ftxxxx_read_Touchdata(struct ftxxxx_ts_data *data)
{
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&data->client->dev, "[Focal][Touch] %s : read touchdata failed.\n", __func__);
		return ret;
	}

	/*Ft_Printf_Touchdata(data,buf);*/	/*\B4\F2ӡ\B1\A8\B5\E3\B5\F7\CA\D4\D0\C5Ϣ*/

	memset(event, 0, sizeof(struct ts_event));
	event->Cur_touchpoint=buf[2]&0x0f;
	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		//printk(KERN_WARNING"hjptestftxxxx->pointid=%d\n",pointid);
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
		event->au16_x[i] =
			(((s16) buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i]) & 0x0F) <<
			8 | (((s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i])& 0xFF);
		event->au16_y[i] =
			(((s16) buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i]) & 0x0F) <<
			8 | (((s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i]) & 0xFF);
		event->au8_touch_event[i] =
			buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
			(buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		event->pressure[i] =
			(buf[FT_TOUCH_XY_POS + FT_TOUCH_STEP * i]);
		event->area[i] =
			(buf[FT_TOUCH_MISC + FT_TOUCH_STEP * i]) >> 4;
		focal_debug(DEBUG_VERBOSE, "[Focal][Touch] id=%d event=%d x=%d y=%d pressure=%d area=%d\n", event->au8_finger_id[i],
			event->au8_touch_event[i], event->au16_x[i], event->au16_y[i], event->pressure[i], event->area[i]);
		if((event->au8_touch_event[i]==0||event->au8_touch_event[i]==2)&&(event->Cur_touchpoint==0))
			return 1;
	//	printk(KERN_WARNING "id=%d event=%d x=%d y=%d pressure=%d area=%d\n", event->au8_finger_id[i],
	//		event->au8_touch_event[i], event->au16_x[i], event->au16_y[i], event->pressure[i], event->area[i]);
	}
	
	/*event->pressure = FT_PRESS;*/
	/*event->pressure = 200;*/

	return 0;
}

/*
*report the point information
*/

static int ASUS_check_point_position(u16 x, u16 y){

	int limit_y =0;

#ifdef ZE601KL_FHD
	limit_y = 1920;
#endif
#ifdef ZE600KL_HD
	limit_y = 1280;
#endif
	if((limit_y != 0) && y > limit_y){
#ifdef ZE600KL_HD		
		if( x < 86 || (234 <x && x < 274) || (446 < x && x < 486) || x > 634)
			return 1;
#endif
#ifdef ZE601KL_FHD
		if( x < 129 || (351 < x && x < 411) || (669 < x && x < 729) || x > 951)
			return 1;
#endif


if(asus_PRJ_ID == ASUS_ZE550KL){	

#ifdef ZE551KL_FHD
              limit_y = 1280;
		if( x < 85 || (345 < x && x < 410) || (670 < x && x < 735) || x > 995)
					return 1;
#endif
#ifdef ZE550KL_HD
					  limit_y = 1280;
				if( x < 50 || (230 < x && x < 275) || (445 < x && x < 490) || x > 670)
							return 1;
#endif
}

		return 0;
	}else
		return 0;

	return 0;
}

static void ftxxxx_report_value(struct ftxxxx_ts_data *data)
{	
	struct ts_event *event = &data->event;
	int i;
	int uppoint = 0;
	static u8 last_touchpoint; 
	bool report_point=true;
	u8 filter_touch_point=0;
	int limit_report=0;
	/*protocol B*/
if(asus_PRJ_ID == ASUS_ZE600KL){	
#ifdef ZE601KL_FHD
	limit_report = 23;
#endif
#ifdef ZE600KL_HD
	limit_report = 13;
#endif


}
if(asus_PRJ_ID == ASUS_ZE550KL){	

#ifdef ZE550KL_HD
		limit_report = 13;
#endif
#ifdef ZE551KL_FHD
			limit_report = 13;
#endif
}

	filter_touch_point = event->touch_point;
	for (i = 0; i < event->touch_point; i++) {
		report_point=true;

			if(asus_PRJ_ID == ASUS_ZE600KL||asus_PRJ_ID == ASUS_ZE550KL){
				if(ASUS_check_point_position(event->au16_x[i],event->au16_y[i])){
							report_point=false;
							filter_touch_point--;
				}
				
				if(!keyboard_enable && report_point){
					if( //if the point is in non-reported area at the first touch.( HD: x < 13 or x > 720-13, FHD: x<23 or x > 1080-23).
						(first_location[event->au8_finger_id[i]].x < limit_report) ||
						(first_location[event->au8_finger_id[i]].x > (ftxxxx_ts->x_max - limit_report))
						){
						if(//if the point is in the reported area, report the point and set the flag to true.
							(event->au8_touch_event[i] == 2) && (limit_report <= event->au16_x[i] && (event->au16_x[i]<= (ftxxxx_ts->x_max-limit_report))))
							{
							report_point=true;
							first_location[event->au8_finger_id[i]].filter_flag = true;
						}else{ // if the point is in non-reported area and if the flag to false, skeep the point.
							if(!first_location[event->au8_finger_id[i]].filter_flag){
								report_point=false; 
								filter_touch_point--;
							}
						}
					}
				}
			}
	
		if(!report_point)
			continue;
		
		input_mt_slot(data->input_dev,event->au8_finger_id[i]);
		if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2) {
			input_mt_report_slot_state(data->input_dev,MT_TOOL_FINGER,true);
		//A	input_report_key(data->input_dev, BTN_TOUCH, 1);             /* touch down*/
		//	input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->au8_finger_id[i]); /*ID of touched point*/
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure[i]);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->area[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);
			
		//A	input_mt_sync(data->input_dev);
		} else {
			uppoint++;
			/* +++ asus jacob add for print touch location +++ */
			//report_touch_locatoin_count[i] = 0;
			//printk("hjptest111--->[Focal][Touch] touch up !\n");
			/* --- asus jacob add for print touch location --- */
		//A	input_mt_sync(data->input_dev);
			input_mt_report_slot_state(data->input_dev,MT_TOOL_FINGER,false);
			
		}
	//	if((event->au16_x[i]<0)||(event->au16_x[i]>720)||(event->au16_y[i]<0)||(event->au16_y[i]>1440))
	//		printk(KERN_WARNING "id=%d event=%d x=%d y=%d pressure=%d area=%d\n", event->au8_finger_id[i],
	//			event->au8_touch_event[i], event->au16_x[i], event->au16_y[i], event->pressure[i], event->area[i]);
	}
	
	if((last_touchpoint>0)&&(event->Cur_touchpoint==0))
	{
		for(i=0;i<CFG_MAX_TOUCH_POINTS;i++)
		{
			input_mt_slot(data->input_dev,i);
			input_mt_report_slot_state(data->input_dev,MT_TOOL_FINGER,false);
		}
		last_touchpoint=0;
	}
	if(filter_touch_point == uppoint) {
		input_report_key(data->input_dev, BTN_TOUCH, 0);
		//touch_down_up_status = 0;
		/* +++ asus jacob add for print touch location +++ */
		//memset(report_touch_locatoin_count, 0, sizeof(report_touch_locatoin_count));
		/* --- asus jacob add for print touch location --- */
	} else {
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
		//if (touch_down_up_status == 0) {
		//	touch_down_up_status = 1;

		}

	input_sync(data->input_dev);
	last_touchpoint=event->Cur_touchpoint;
}

/*The ftxxxx device will signal the host about TRIGGER_FALLING.
*Processed when the interrupt is asserted.
*/
static irqreturn_t ftxxxx_ts_interrupt(int irq, void *dev_id)
{
/*	struct ftxxxx_ts_data *ftxxxx_ts = dev_id; ASUS jacob use globle ftxxxx_ts data*/
	int ret = 0;
	int i=0;
#ifdef FTS_GESTRUE/*zax 20140922*/
	u8 state;
#endif
	if(selftestflag==1)
	{
		printk(KERN_WARNING "[Focal][Touch] in selftest mode\n");
		return IRQ_HANDLED;
	}
	//if (suspend_resume_process == true) { //anna -for disable touch
	if ((suspend_resume_process == true) | (disable_tp_flag == true)) {
		printk("[Focal][Touch] interrupt suspend_resume in process skip !\n");
		return IRQ_HANDLED;
	}
	
	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);
	if(!FOCAL_IRQ_DISABLE)
	{
		ftxxxx_nosync_irq_disable(ftxxxx_ts->client);
		
#ifdef FTS_GESTRUE/*zax 20140922*/
						i2c_smbus_read_i2c_block_data(ftxxxx_ts->client, 0xd0, 1, &state);
						if(Gesture_flag==1)
						{
							printk("[Focal][Touch] %s: tpd fts_read_Gestruedata state=%d\n",__func__, state);
						}
						if (state == 1) {
							fts_read_Gestruedata(ftxxxx_ts);
							/*continue;*/
						} else {
#endif
			ret = ftxxxx_read_Touchdata(ftxxxx_ts);	
			//<asus-Jeffery20151202+>
			if(asus_PRJ_ID == ASUS_ZE600KL||asus_PRJ_ID == ASUS_ZE550KL){
				for (i = 0; i < ftxxxx_ts->event.touch_point; i++) {
					if(ftxxxx_ts->event.au8_touch_event[i] == 0){
					//pr_err("[Jeffery][Touch] event=0 x=%x, y=%x\n",ftxxxx_ts->event.au16_x[i],ftxxxx_ts->event.au16_y[i]);
						first_location[ftxxxx_ts->event.au8_finger_id[i]].x=ftxxxx_ts->event.au16_x[i];
						first_location[ftxxxx_ts->event.au8_finger_id[i]].filter_flag = false;
					}
				}
			}
			//<asus-Jeffery20151202->
			if ((ret == 0)&&(suspend_resume_process==false)&&(!disable_tp_flag))
				ftxxxx_report_value(ftxxxx_ts);
#ifdef FTS_GESTRUE/*zax 20140922*/
							}
#endif
		ftxxxx_irq_enable(ftxxxx_ts->client);
	}
	else
	{
		printk(KERN_EMERG"[Focal][Touch] newcode---disable touch\n");		
	}
	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);
	return IRQ_HANDLED;
}

void ftxxxx_reset_tp(int HighOrLow)
{
	pr_info("[Focal][Touch] %s : set tp reset pin to %d\n", __func__, HighOrLow);
	gpio_set_value(ftxxxx_ts->pdata->rst_gpio, HighOrLow);
}

void ftxxxx_Enable_IRQ(struct i2c_client *client, int enable)
{
	//if (FTXXXX_ENABLE_IRQ == enable)
	//if (FTXXXX_ENABLE_IRQ)
	//enable_irq(client->irq);
	//else
	//disable_irq_nosync(client->irq);
}

void ftxxxx_nosync_irq_disable(struct i2c_client *client)
{
	struct ftxxxx_ts_data *ts_data;
	unsigned long irqflags;
	ts_data = i2c_get_clientdata(client);
	
    spin_lock_irqsave(&ts_data->irq_lock, irqflags);
    if (!ts_data->irq_lock_status) {
		disable_irq_nosync(ts_data->client->irq);
		ts_data->irq_lock_status = 1;
	} else {
		printk("[Focal][Touch] %s : already disnable skip ! \n", __func__);
	}
    spin_unlock_irqrestore(&ts_data->irq_lock, irqflags);
}

void ftxxxx_irq_disable(struct i2c_client *client)
{
	struct ftxxxx_ts_data *ts_data;
	unsigned long irqflags;
	ts_data = i2c_get_clientdata(client);
	
    spin_lock_irqsave(&ts_data->irq_lock, irqflags);
    if (!ts_data->irq_lock_status) {
		disable_irq(ts_data->client->irq);
		ts_data->irq_lock_status = 1;
	} else {
		printk("[Focal][Touch] %s : already disnable skip ! \n", __func__);
	}
    spin_unlock_irqrestore(&ts_data->irq_lock, irqflags);
}

void ftxxxx_irq_enable(struct i2c_client *client)
{
	struct ftxxxx_ts_data *ts_data;
	unsigned long irqflags;
	ts_data = i2c_get_clientdata(client);

    spin_lock_irqsave(&ts_data->irq_lock, irqflags);
	
		if (ts_data->irq_lock_status) {
			enable_irq(ts_data->client->irq);
			
	        ts_data->irq_lock_status = 0;
		} 
		else {
			printk("[Focal][Touch] %s : already enable skip ! \n", __func__);
		}
    spin_unlock_irqrestore(&ts_data->irq_lock, irqflags);
}

int ftxxxx_read_tp_id(void)
{

	int err = 0;

	err = fts_ctpm_fw_upgrade_ReadVendorID(ftxxxx_ts->client, &B_VenderID);
	if (err < 0)
		B_VenderID = 0xFF;

	printk("[Focal][Touch] %s : TP Bootloadr info : vendor ID = %x !\n", __func__, B_VenderID);
	
	return B_VenderID;
}

int focal_get_HW_ID(void)
{
	Focal_hw_id = g_ASUS_hwID;
	printk("[Focal][Touch] %s : Focal get hw id %d !\n", __func__, Focal_hw_id);

	return Focal_hw_id;
}

void asus_check_touch_mode(void)
{
	uint8_t buf[2]={0};
	int err = 0;
	if(ftxxxx_ts->init_success == 1)
	{
		if(ftxxxx_ts->usb_status==1)
		{
			buf[0]=0x8B;
			buf[1]=1;
			err=ftxxxx_write_reg(ftxxxx_ts->client,buf[0],buf[1]);
			if(err<0)
				printk("[Focal][Touch] %s : switch to usb mode failed\n",__func__);
			else
				printk("[Focal][Touch] %s : switch to usb mode \n",__func__);
		}
		else
		{
			buf[0]=0x8B;
			buf[1]=0;
			err=ftxxxx_write_reg(ftxxxx_ts->client,buf[0],buf[1]);
			if(err<0)
				printk("[Focal][Touch] %s : leave  usb mode failed\n",__func__);
			else
				printk("[Focal][Touch] %s : leave  usb mode\n",__func__);
		}
	}
}

void focal_usb_detection(bool plugin)
{
	if (ftxxxx_ts == NULL) {
		printk("[Focal][Touch] %s : ftxxxx_ts is null, skip \n", __func__);
		return;
	}
	printk("[Focal][Touchh] plugin=%d\n",plugin);
	if (ftxxxx_ts->init_success == 1) {
		wake_lock(&ftxxxx_ts->wake_lock);
		mutex_lock(&ftxxxx_ts->g_device_mutex);
		if (plugin)
			ftxxxx_ts->usb_status = 1; /*AC plug in*/
		else
			ftxxxx_ts->usb_status = 0;	/*no AC */
		mutex_unlock(&ftxxxx_ts->g_device_mutex);
		wake_unlock(&ftxxxx_ts->wake_lock);
		queue_work(ftxxxx_ts->usb_wq, &ftxxxx_ts->usb_detect_work);
	}
}

static void focal_cable_status(struct work_struct *work)
{
	uint8_t buf[2] = {0};
	int status = ftxxxx_ts->usb_status;

	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	printk("[Focal][Touchh] cable_status=%d, init_success=%d.\n", status, ftxxxx_ts->init_success);

	if (ftxxxx_ts->init_success == 1) {
		if (status == 0) {	/*no AC */
			buf[0] = 0x8B;//the charge in flags address
			buf[1] = 0x00;
			ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
		} else if (status == 1) {	/*AC plug in*/
			buf[0] = 0x8B;
			buf[1] = 0x01;
			ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
		}
	}
	ftxxxx_read_reg(ftxxxx_ts->client, buf[0], &buf[1]);
	printk("[Focal][Touch] buf[1]=%d\n",buf[1]);
	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);

	return;

}

void focal_glove_switch(bool plugin)
{
	uint8_t buf[2] = {0};

	int err = 0;

	if (ftxxxx_ts == NULL) {
		printk("[Focal][Touch] %s : ftxxxx_ts is null, skip \n", __func__);
		return;
	}

	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	if (ftxxxx_ts->init_success == 1) {
		if (plugin) {//open glove mode

			buf[0] = 0xC0;

			buf[1] = 0x01;

			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

			if (err < 0)
				printk("[Focal][Touch] %s : glove mode enable fail ! \n", __func__);
			else
				printk("[Focal][Touch] %s : glove mode enable ! \n", __func__);

			ftxxxx_ts->glove_mode_eable = true; /*glove mode enable*/

		} else {//close glove mode

			buf[0] = 0xC0;

			buf[1] = 0x00;

			err = ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

			if (err < 0)
				printk("[Focal][Touch] %s : glove mode disable fail ! \n", __func__);
			else
				printk("[Focal][Touch] %s : glove mode disable ! \n", __func__);

			ftxxxx_ts->glove_mode_eable = false;	/*glove mode disable*/

		}
	}

	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);

}

static void focal_reset_ic_work(struct work_struct *work)
{

	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	ftxxxx_nosync_irq_disable(ftxxxx_ts->client);

	ftxxxx_reset_tp(0);

	msleep(20);

	ftxxxx_reset_tp(1);

	msleep(80);

	IICErrorCountor = 0;

	ftxxxx_irq_enable(ftxxxx_ts->client);

	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);

	return;

}
#ifdef FTS_PM
static void focal_suspend_work(struct work_struct *work)
{
	
	uint8_t buf[2] = {0};
	int i;
	
	struct ftxxxx_ts_data *ts = ftxxxx_ts;
	
	suspend_resume_process = true;
	
//	printk(KERN_EMERG "[Focal][Touch] %s : Touch suspend +++ ++\n", __func__);

	//wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	if (ftxxxx_ts->suspend_flag) {

		focal_debug(DEBUG_VERBOSE, "[Focal][Touch]--->[Focal][Touch] IC in suspend !! \n");

		mutex_unlock(&ftxxxx_ts->g_device_mutex);

		//wake_unlock(&ftxxxx_ts->wake_lock);

		suspend_resume_process = false;

		return;
	}
#ifdef FTS_GESTRUE/*zax 20140922*/
	dclick_flags=true;
	if ((ftxxxx_ts->dclick_mode_eable == true) || (ftxxxx_ts->gesture_mode_eable == true)) {
			printk(KERN_EMERG "[Focal][Touch] %s : Touch gesture mode \n", __func__);
			enable_irq_wake(ftxxxx_ts->client->irq);
			ftxxxx_write_reg(ts->client, 0xd0, 0x01);

			if (ftxxxx_ts->dclick_mode_eable == true) {
				printk(KERN_EMERG "[Focal][Touch] %s : open dclick mode \n", __func__);
				ftxxxx_write_reg(ts->client, 0xd1, 0x10);
			}

			if (ftxxxx_ts->gesture_mode_eable == true) {
				if (ftxxxx_ts->dclick_mode_eable == true)
					ftxxxx_write_reg(ts->client, 0xd1, 0x30);
				else
					ftxxxx_write_reg(ts->client, 0xd1, 0x20);
				
				ftxxxx_write_reg(ts->client, 0xd2, FTS_gesture_register_d2);
				ftxxxx_write_reg(ts->client, 0xd5, FTS_gesture_register_d5);
				ftxxxx_write_reg(ts->client, 0xd6, FTS_gesture_register_d6);
				ftxxxx_write_reg(ts->client, 0xd7, FTS_gesture_register_d7); //<asus-Jeffery20150427+>
			}
			Gesture_flag=1;
			{
				u8 FTS_gesture_register_d1;
				u8 FTS_gesture_register_d0;
				ftxxxx_read_reg(ts->client, 0xd1, &FTS_gesture_register_d1);
				ftxxxx_read_reg(ts->client, 0xd0, &FTS_gesture_register_d0);
				printk("[Focal][Touch]suspend FTS_gesture_register_d0=0x%x\n",FTS_gesture_register_d0);
				printk("[Focal][Touch]suspend FTS_gesture_register_d1=0x%x\n",FTS_gesture_register_d1);
			}
	} else {
		
		ftxxxx_irq_disable(ts->client);
		printk(KERN_EMERG "1[Focal][Touch] %s : Touch suspend and  disable touch\n", __func__);
		buf[0] = 0xA5;
		buf[1] = 0x03;
		ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
	}
#else
	printk(KERN_EMERG "2[Focal][Touch] %s : Touch suspend and  disable touch\n", __func__);
	ftxxxx_irq_disable(ts->client);
	buf[0] = 0xA5;
	buf[1] = 0x03;
	ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
#endif
	//release all touch points
	for(i=0;i<CFG_MAX_TOUCH_POINTS;i++)
	{
		input_mt_slot(ftxxxx_ts->input_dev, i);
		input_mt_report_slot_state(ftxxxx_ts->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(ftxxxx_ts->input_dev, false);
	input_sync(ftxxxx_ts->input_dev);
	/* +++ asus jacob add for print touch location +++ */
	//memset(report_touch_locatoin_count, 0, sizeof(report_touch_locatoin_count));
	/* --- asus jacob add for print touch location --- */

//	input_report_key(ftxxxx_ts->input_dev, BTN_TOUCH, 0);
//	input_mt_sync(ftxxxx_ts->input_dev);
//	input_sync(ftxxxx_ts->input_dev);

	ftxxxx_ts->suspend_flag = 1;

	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	//wake_unlock(&ftxxxx_ts->wake_lock);

	suspend_resume_process = false;

	printk(KERN_EMERG "[Focal][Touch] %s : Touch suspend --- \n", __func__);

	return;
}	

	
static void focal_resume_work(struct work_struct *work)
{
	uint8_t buf[2] = {0};
	struct ftxxxx_ts_data *ts = ftxxxx_ts;
	
	suspend_resume_process = true;
	disable_tp_flag=false;

	printk("[Focal][Touch] %s : Touch resume +++ \n", __func__);

	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	if (!ftxxxx_ts->suspend_flag) {

		focal_debug(DEBUG_VERBOSE, "[Focal][Touch] IC did not enter suspend !! \n");

		mutex_unlock(&ftxxxx_ts->g_device_mutex);

		wake_unlock(&ftxxxx_ts->wake_lock);

		suspend_resume_process = false;

		return;

	}

#ifdef FTS_GESTRUE	/*zax 20140922*/

	if ((ftxxxx_ts->dclick_mode_eable == true) ||(ftxxxx_ts->gesture_mode_eable == true)) {
		if (ftxxxx_ts->reset_pin_status == 1) {

			printk("[Focal][Touch] %s : Touch resume from gesture mode \n", __func__);
			disable_irq_wake(ts->client->irq);
			gpio_set_value(ts->pdata->rst_gpio, 0);

			msleep(20);

			gpio_set_value(ts->pdata->rst_gpio, 1);

			msleep(80);

			asus_check_touch_mode();

			if (ts->glove_mode_eable == true) {

				buf[0] = 0xC0;

				buf[1] = 0x01;

				ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
			}

		ftxxxx_write_reg(ts->client, 0xD0, 0x00);

			ftxxxx_irq_enable(ts->client);
			if(ftxxxx_ts->cover_mode_states == false)
			{
				ftxxxx_write_reg(ftxxxx_ts->client,0xC3,0);//the filp cover is open
				printk("[Focal][Touch] the filp cover is open in resume! \n");
			}
		} else {

			printk("[Focal][Touch] %s : ftxxxx_ts->reset_pin_status set to 0 ! skip reset IC \n", __func__);

		}
		Gesture_flag=0;
		{
			u8 FTS_gesture_register_d1;
			u8 FTS_gesture_register_d0;
			ftxxxx_read_reg(ts->client, 0xd1, &FTS_gesture_register_d1);
			ftxxxx_read_reg(ts->client, 0xd0, &FTS_gesture_register_d0);
			printk("[Focal][Touch] resume FTS_gesture_register_d0=0x%x\n",FTS_gesture_register_d0);
			printk("[Focal][Touch] resume FTS_gesture_register_d1=0x%x\n",FTS_gesture_register_d1);
		}
		ftxxxx_irq_enable(ts->client);
	} else {

		if (ftxxxx_ts->reset_pin_status == 1) {

			printk("[Focal][Touch] %s : Touch resume from sleep mode \n", __func__);

			gpio_set_value(ts->pdata->rst_gpio, 0);

			msleep(20);

			gpio_set_value(ts->pdata->rst_gpio, 1);

			msleep(80);

			asus_check_touch_mode();

			if (ts->glove_mode_eable == true) {

				buf[0] = 0xC0;

				buf[1] = 0x01;

				ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

			}

			buf[0] = 0xA5;

			buf[1] = 0x00;

			ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

			ftxxxx_irq_enable(ts->client);

		} else {

			printk("[Focal][Touch] %s : ftxxxx_ts->reset_pin_status set to 0 ! skip reset IC \n", __func__);

			ftxxxx_irq_enable(ts->client);

		}
		ftxxxx_write_reg(ftxxxx_ts->client,0xC3,0);//the filp cover is open
	}
#else
	if (ftxxxx_ts->reset_pin_status == 1) {

		gpio_set_value(ts->pdata->rst_gpio, 0);

		msleep(20);

		gpio_set_value(ts->pdata->rst_gpio, 1);

		msleep(80);

		asus_check_touch_mode();

		if (ts->glove_mode_eable == true) {

			buf[0] = 0xC0;

			buf[1] = 0x01;

			ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

		}

		buf[0] = 0xA5;

		buf[1] = 0x00;

		ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);

		ftxxxx_irq_enable(ts->client);

	} else {

		printk("[Focal][Touch] %s : ftxxxx_ts->reset_pin_status set to 0 ! skip reset IC \n", __func__);

		ftxxxx_irq_enable(ts->client);

	}
#endif

	ftxxxx_ts->suspend_flag = 0;

	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);

	suspend_resume_process = false;

	printk("[Focal][Touch] %s : Touch resume --- \n", __func__);

	return;
}
#endif

static int fts_power_on(struct ftxxxx_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->pdata->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"[Focal][Touch] Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->pdata->vcc);
	if (rc) {
		dev_err(&data->client->dev,
			"[Focal][Touch] Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->pdata->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->pdata->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"[Focal][Touch] Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->pdata->vcc);
	if (rc) {
		dev_err(&data->client->dev,
			"[Focal][Touch] Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(data->pdata->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"[Focal][Touch] Regulator vdd enable failed rc=%d\n", rc);
		}
	}

	return rc;
}

static int fts_power_init(struct device *dev,
			struct focal_i2c_platform_data *pdata, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;
	/* +++ pars regulator+++ */
	pdata->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR( pdata->vdd)) {
		rc = PTR_ERR(pdata->vdd);
		printk("[Focal][Touch] Regulator get touch vdd failed rc=%d\n", rc);
		return rc;
		}

	if (regulator_count_voltages(pdata->vdd) > 0) {
		rc = regulator_set_voltage(pdata->vdd, FT_VDD_MIN_UV,
					   FT_VDD_MAX_UV);
		if (rc) {
			printk("[Focal][Touch] Regulator set_vdd failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}
	
	pdata->vcc = devm_regulator_get(dev, "vcc_i2c");
	if (IS_ERR( pdata->vcc)) {
		rc = PTR_ERR(pdata->vcc);
		printk("[Focal][Touch] Regulator get vcc_i2c failed rc=%d\n", rc);
		goto reg_vdd_set_vtg;
		}

	if (regulator_count_voltages(pdata->vcc) > 0) {
		rc = regulator_set_voltage(pdata->vcc, FT_I2C_VCC_MIN_UV,
					   FT_I2C_VCC_MAX_UV);
		if (rc) {
			printk("[Focal][Touch] Regulator set_vcc failed vdd rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}
	/* +++ pars regulator+++ */
	return 0;
reg_vcc_i2c_put:
	regulator_put(pdata->vcc);
reg_vdd_set_vtg:
	if (regulator_count_voltages(pdata->vdd) > 0)
		regulator_set_voltage(pdata->vdd, 0, FT_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(pdata->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(pdata->vdd) > 0)
		regulator_set_voltage(pdata->vdd, 0, FT_VDD_MAX_UV);

	regulator_put(pdata->vdd);

	if (regulator_count_voltages(pdata->vcc) > 0)
		regulator_set_voltage(pdata->vcc, 0, FT_I2C_VCC_MAX_UV);

	regulator_put(pdata->vcc);
	return 0;
}

static int fts_init_gpio_hw(struct ftxxxx_ts_data *ftxxxx_ts)
{
	int ret = 0;

	ret = gpio_request(ftxxxx_ts->pdata->rst_gpio, FTXXXX_RESET_PIN_NAME);
	if (ret) {
		printk("[Focal][Touch] %s: request GPIO %s for reset failed, ret = %d\n",
			__func__, FTXXXX_RESET_PIN_NAME, ret);
		return ret;
	}
	
	ret = gpio_direction_output(ftxxxx_ts->pdata->rst_gpio, 1);	/*asus change reset set output high*/
	if (ret) {
		printk("[Focal][Touch] %s: set %s gpio to out put high failed, ret = %d\n",
			__func__, FTXXXX_RESET_PIN_NAME, ret);
		return ret;
		}
	return ret;
}

static void fts_un_init_gpio_hw(struct ftxxxx_ts_data *ftxxxx_ts)
{
	gpio_free(ftxxxx_ts->pdata->rst_gpio);
}

/* +++asus jacob add for pars dt info+++ */

static int ft5x46_get_dt_coords(struct device *dev, char *name,
				struct focal_i2c_platform_data *pdata)
{
	u32 coords[FT_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FT_COORDS_ARR_SIZE) {
		dev_err(dev, "[Focal][Touch] invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "[Focal][Touch] Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "focaltech,panel-coords")) {
		pdata->abs_x_min = coords[0];
		pdata->abs_y_min = coords[1];
		pdata->abs_x_max = coords[2];
		pdata->abs_y_max = coords[3];
	} else if (!strcmp(name, "focaltech,display-coords")) {
		pdata->abs_x_min = coords[0];
		pdata->abs_y_min = coords[1];
		pdata->abs_x_max = coords[2];
		pdata->abs_y_max = coords[3];
	} else {
		dev_err(dev, "[Focal][Touch] unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int ft5x46_parse_dt(struct device *dev,
			struct focal_i2c_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;

/*
	pdata->name = "focaltech";
	rc = of_property_read_string(np, "focaltech,name", *pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}
*/
	rc = ft5x46_get_dt_coords(dev, "focaltech,display-coords", pdata);
	
	if (rc)
		return rc;
	
	/* +++reset, irq gpio info+++ */
	pdata->rst_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
				0, &pdata->rst_gpio_flag);
	if (pdata->rst_gpio < 0)
		return pdata->rst_gpio;

	pdata->intr_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
				0, &pdata->intr_gpio_flag);
	if (pdata->intr_gpio < 0)
		return pdata->intr_gpio;
	/* ---reset, irq gpio info--- */

	return 0;
}

/* ---asus jacob add for pars dt info--- */
/************************add for test begin jinpeng_He******************************/
static char *I2C_Test[]={"i2c r/w test fail","i2c test r/w test ok"};
static char Driver_ver[]="20150128";
//get the touch status
#define TOUCH_BUF_LEN_MAX  160



static ssize_t ftxxxx_touch_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ftxxxx_ts_data *data = dev_get_drvdata(dev);
	u8 register_addr = FTXXXX_REG_FW_VER;
	u8 register_value;
	//ftxxxx_update_fw_ver(data);
	int err= ftxxxx_i2c_Read(data->client, &register_addr, 1, &register_value, 1);
	if (err < 0) 
	{
		//dev_err(&fts_i2c_client->dev, "version read failed");
		switch (asus_PRJ_ID) {
			//<asus-Jeffery20150408+>
			case ASUS_ZE600KL:
				if(data->tp_id_value2==0)
				{
					return snprintf(buf, TOUCH_BUF_LEN_MAX - 1, "ERROR:%s\n"
																"Driver version is:%s\n"
																"Config version is:%d.%d.%d\n"
																"Firmware version is:%d.%d.%d\n"
																"Glass vendor is :TPK\n",
					I2C_Test[0],
					Driver_ver,
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2],
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2]);
				}
				else
				{
					return snprintf(buf, TOUCH_BUF_LEN_MAX - 1, "ERROR:%s\n"
																"Driver version is:%s\n"
																"Config version is:%d.%d.%d\n"
																"Firmware version is:%d.%d.%d\n"
																"Glass vendor is :Jtouch\n",
					I2C_Test[0],
					Driver_ver,
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2],
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2]);
				}
				break;
			//<asus-Jeffery20150408->
			case ASUS_ZE550KL:
				if(data->tp_id_value1==1&&data->tp_id_value2==1)
				{
					return snprintf(buf, TOUCH_BUF_LEN_MAX - 1, "ERROR:%s\n"
																"Driver version is:%s\n"
																"Config version is:%d.%d.%d\n"
																"Firmware version is:%d.%d.%d\n"
																"Glass vendor is :TPK\n",
					I2C_Test[0],
					Driver_ver,
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2],
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2]);
				}
				else
				{
					return snprintf(buf, TOUCH_BUF_LEN_MAX - 1, "ERROR:%s\n"
																"Driver version is:%s\n"
																"Config version is:%d.%d.%d\n"
																"Firmware version is:%d.%d.%d\n"
																"Glass vendor is :GIS\n",
					I2C_Test[0],
					Driver_ver,
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2],
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2]);
				}
				break;
			case ASUS_ZD550KL:
				if (ftxxxx_ts->tp_id_value1 == 0 && ftxxxx_ts->tp_id_value2 == 0) {
					return snprintf(buf, TOUCH_BUF_LEN_MAX - 1, "ACK:%s\n"
																"Driver version is:%s\n"
																"Config version is:%d.%d.%d\n"
																"Firmware version is:%d.%d.%d\n"
																"Glass vendor is :Jtouch\n",
					I2C_Test[0],
					Driver_ver,
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2],
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2]);				
				}
				else if(ftxxxx_ts->tp_id_value1 == 0 && ftxxxx_ts->tp_id_value2 == 1) {
					return snprintf(buf, TOUCH_BUF_LEN_MAX - 1, "ACK:%s\n"
																"Driver version is:%s\n"
																"Config version is:%d.%d.%d\n"
																"Firmware version is:%d.%d.%d\n"
																"Glass vendor is :GIS\n",
					I2C_Test[0],
					Driver_ver,
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2],
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2]);
				}
				break;
			default:
				return snprintf(buf, TOUCH_BUF_LEN_MAX - 1, "ACK:%s\n"
																"Driver version is:%s\n"
																"Config version is:%d.%d.%d\n"
																"Firmware version is:%d.%d.%d\n"
																"Glass vendor is :default case GIS\n",
					I2C_Test[0],
					Driver_ver,
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2],
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2]);
				break;
		}				
	}
	else
	{	
		switch (asus_PRJ_ID) {
			//<asus-Jeffery20150408+>
			case ASUS_ZE600KL:
				if(data->tp_id_value2==0)
				{
					return snprintf(buf, TOUCH_BUF_LEN_MAX - 1, "ACK:%s\n"
																	"Driver version is:%s\n"
																	"Config version is:%d.%d.%d\n"
																	"Firmware version is:%d.%d.%d\n"
																	"Glass vendor is :TPK\n",
						I2C_Test[1],
						Driver_ver,
						data->fw_ver[0],data->fw_ver[1],data->fw_ver[2],
						data->fw_ver[0],data->fw_ver[1],data->fw_ver[2]);
				}
				else
				{
					return snprintf(buf, TOUCH_BUF_LEN_MAX - 1, "ACK:%s\n"
																	"Driver version is:%s\n"
																	"Config version is:%d.%d.%d\n"
																	"Firmware version is:%d.%d.%d\n"
																	"Glass vendor is :Jtouch\n",
						I2C_Test[1],
						Driver_ver,
						data->fw_ver[0],data->fw_ver[1],data->fw_ver[2],
						data->fw_ver[0],data->fw_ver[1],data->fw_ver[2]);
				}
				break;
			//<asus-Jeffery20150408->
			case ASUS_ZE550KL:
				if(data->tp_id_value1==1&&data->tp_id_value2==1)
				{
					return snprintf(buf, TOUCH_BUF_LEN_MAX - 1, "ACK:%s\n"
																	"Driver version is:%s\n"
																	"Config version is:%d.%d.%d\n"
																	"Firmware version is:%d.%d.%d\n"
																	"Glass vendor is :TPK\n",
						I2C_Test[1],
						Driver_ver,
						data->fw_ver[0],data->fw_ver[1],data->fw_ver[2],
						data->fw_ver[0],data->fw_ver[1],data->fw_ver[2]);
				}
				else
				{
					return snprintf(buf, TOUCH_BUF_LEN_MAX - 1, "ACK:%s\n"
																	"Driver version is:%s\n"
																	"Config version is:%d.%d.%d\n"
																	"Firmware version is:%d.%d.%d\n"
																	"Glass vendor is :GIS\n",
						I2C_Test[1],
						Driver_ver,
						data->fw_ver[0],data->fw_ver[1],data->fw_ver[2],
						data->fw_ver[0],data->fw_ver[1],data->fw_ver[2]);
				}
				break;
			case ASUS_ZD550KL:
				if (ftxxxx_ts->tp_id_value1 == 0 && ftxxxx_ts->tp_id_value2 == 0) {
					return snprintf(buf, TOUCH_BUF_LEN_MAX - 1, "ACK:%s\n"
																"Driver version is:%s\n"
																"Config version is:%d.%d.%d\n"
																"Firmware version is:%d.%d.%d\n"
																"Glass vendor is :Jtouch\n",
					I2C_Test[1],
					Driver_ver,
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2],
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2]);				
				}
				else if(ftxxxx_ts->tp_id_value1 == 0 && ftxxxx_ts->tp_id_value2 == 1) {
					return snprintf(buf, TOUCH_BUF_LEN_MAX - 1, "ACK:%s\n"
																"Driver version is:%s\n"
																"Config version is:%d.%d.%d\n"
																"Firmware version is:%d.%d.%d\n"
																"Glass vendor is :GIS\n",
					I2C_Test[1],
					Driver_ver,
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2],
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2]);
				}
				break;
			default:
				return snprintf(buf, TOUCH_BUF_LEN_MAX - 1, "ACK:%s\n"
																"Driver version is:%s\n"
																"Config version is:%d.%d.%d\n"
																"Firmware version is:%d.%d.%d\n"
																"Glass vendor is :default case GIS\n",
					I2C_Test[1],
					Driver_ver,
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2],
					data->fw_ver[0],data->fw_ver[1],data->fw_ver[2]);
				break;
		}		
	}
	return 0;
}


static DEVICE_ATTR(touch_status,0444,ftxxxx_touch_status_show,NULL);
/************************add for test end jinpeng_He******************************/
/***********add byjinpeng_He for update firmware in workqueue begin****************/
static void update_fw_in_wq(struct work_struct *work)
{
//	printk("update firmware in workqueue\n");
	fts_ctpm_auto_upgrade(ftxxxx_ts->client);
}
/***********add byjinpeng_He for update firmware in workqueue begin****************/



/* +++ anna  add for proximity trigger disable touch +++ */
void ftxxxx_disable_touch(bool flag)
{
	if (flag) {
		disable_tp_flag = true;
		printk("[Focal][Touch] %s: proximity trigger disable touch !\n", __func__);
	} else {
		disable_tp_flag = false;
		printk("[Focal][Touch] %s: proximity trigger enable touch  !\n", __func__);
	}
}
EXPORT_SYMBOL(ftxxxx_disable_touch);

/* --- anna add for proximity trigger disable touch --- */

/*****add by jinpeng_he for touch disable or enable from execute /proc/asus_touch_proximity_status ++++******/
struct proc_dir_entry *touchsensor_entry = NULL;
static ssize_t amax_disable_touch_write(struct file *filp, const char __user *buff, size_t len, loff_t *off)
{
	char msg[64];
	int i;
	memset(msg, 0, sizeof(msg));
	if(len > 64)
		len = 64;
	if(copy_from_user(msg, buff, len))

		return -EFAULT;
	if(strncmp(msg, "0", 1) == 0) {	// enable touch

		disable_tp_flag = 0;

		printk("[Focal][Touch] %s : enable touch !\n", __func__);

	} else if(strncmp(msg, "1", 1) == 0){

		disable_tp_flag = 1;
		
		for(i=0;i<CFG_MAX_TOUCH_POINTS;i++)
        {
               input_mt_slot(ftxxxx_ts->input_dev, i);
               input_mt_report_slot_state(ftxxxx_ts->input_dev, MT_TOOL_FINGER, 0);
        }
		input_sync(ftxxxx_ts->input_dev);
		printk("[Focal][Touch] %s : disable touch & release pointer !\n", __func__);

	} else {

		printk("[Focal][Touch] %s : Input parameter invaild !\n", __func__);

	}
	return len;

}


static const struct file_operations amax_disable_touch_fops ={
	.write =amax_disable_touch_write,
};

int create_asusproc_touchsensor_status_entry(void)
{
	touchsensor_entry = proc_create(PROC_AsusTouchDisable_name, 0664, NULL,&amax_disable_touch_fops);
	if (!touchsensor_entry)
       		 return -ENOMEM;

    	return 0;
}

/*****add by jinpeng_he for touch disable or enable from execute /proc/asus_proximity_status ----******/

static int ftxxxx_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct focal_i2c_platform_data *pdata = (struct focal_i2c_platform_data *)client->dev.platform_data;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;
	extern char asus_lcd_id[2];
	//touch_control=1
	printk("[Focal][Touch] FTxxxx probe process Start !\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
	goto exit_check_functionality_failed;
	}

	if (client->dev.of_node) {
		ftxxxx_ts = kzalloc(sizeof(struct ftxxxx_ts_data), GFP_KERNEL);
		if (!ftxxxx_ts) {
			err = -ENOMEM;
			printk("[Focal][Touch] %s: alloc ftxxxx_ts_data failed !! \n", __func__);
			goto exit_alloc_data_failed;
		}
		pdata = kzalloc(sizeof(struct focal_i2c_platform_data), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			printk("[Focal][Touch] %s: alloc focal_i2c_platform_data failed !! \n", __func__);
			goto exit_alloc_data_failed;
		}
	}

	err = ft5x46_parse_dt(&client->dev, pdata);
	if (err) {
		dev_err(&client->dev, "[Focal][Touch] DT parsing failed\n");
		return err;
	}

	i2c_set_clientdata(client, ftxxxx_ts);

	ftxxxx_ts->client = client;
	ftxxxx_ts->init_success = 0;
	ftxxxx_ts->suspend_flag = 0;
	ftxxxx_ts->usb_status = 0;
	ftxxxx_ts->glove_mode_eable = true;
	ftxxxx_ts->dclick_mode_eable = true;
	ftxxxx_ts->gesture_mode_eable = true;
	ftxxxx_ts->gesture_mode_type = 0;
	ftxxxx_ts->pdata = pdata;
	Gesture_flag=0;
	switch (asus_PRJ_ID)
	{
		case 1: //<asus-Jeffery20150408+>
			#ifdef ZE600KL_HD
			ftxxxx_ts->x_max=720;
			ftxxxx_ts->y_max=1280;
			#endif
			#ifdef ZE601KL_FHD
			ftxxxx_ts->x_max=1080;
			ftxxxx_ts->y_max=1920;
			#endif		
			break;
		case 2:
		case 3:
			ftxxxx_ts->x_max = pdata->abs_x_max;
			ftxxxx_ts->y_max = pdata->abs_y_max;
			break;
		case 0:
		default:
			#ifdef ZE550KL_HD
			ftxxxx_ts->x_max=720;
			ftxxxx_ts->y_max=1280;
			#endif
			#ifdef ZE551KL_FHD
			ftxxxx_ts->x_max=1080;
			ftxxxx_ts->y_max=1920;
			#endif
			break;
	}
	
	
	if (0 >= ftxxxx_ts->x_max)
		ftxxxx_ts->x_max = TOUCH_MAX_X;
	if (0 >= ftxxxx_ts->y_max)
		ftxxxx_ts->y_max = TOUCH_MAX_Y;
	ftxxxx_ts->irq = ftxxxx_ts->pdata->intr_gpio;
	/*add by jinpeng_He for get the id pin begin*/
	ftxxxx_ts->tp_id_gpio1=69+902;
	ftxxxx_ts->tp_id_gpio2=74+902;
	
	ftxxxx_ts->lcd_vendor=asus_lcd_id[0];
//	printk("hjptest--->ftxxxx_ts->lcd_vendor=0x%x\n",ftxxxx_ts->lcd_vendor);
	/*add by jinpeng_He for get the id pin end*/

	if (fts_init_gpio_hw (ftxxxx_ts) < 0)
		goto exit_init_gpio;

	ftxxxx_ts->reset_pin_status = 1;
//<asus-Jeffery20150424+>
	switch (asus_PRJ_ID)
	{
		case ASUS_ZE600KL: 
			gpio_set_value(ftxxxx_ts->pdata->rst_gpio, 0);
			msleep(70);
			gpio_set_value(ftxxxx_ts->pdata->rst_gpio, 1);
			msleep(10);
			break;
		case ASUS_ZE550KL:
		case ASUS_ZD550KL:
		case ASUS_ZX550KL:
		default:
			gpio_set_value(ftxxxx_ts->pdata->rst_gpio, 0);
			msleep(20);
			gpio_set_value(ftxxxx_ts->pdata->rst_gpio, 1);
			msleep(80);
			break;
	}
//<asus-Jeffery20150424->

	if (fts_power_init(&client->dev, ftxxxx_ts->pdata, true) < 0)
		goto exit_init_gpio;

	if (fts_power_on(ftxxxx_ts, true) < 0)
		goto exit_power_init;

	if (gpio_request(ftxxxx_ts->irq, FTXXXX_INT_PIN_NAME)) {
		printk("[Focal][Touch] %s: gpio %d request for interrupt fail.\n", __func__, ftxxxx_ts->irq);
		goto exit_irq_request_failed;
	}
	gpio_direction_input(ftxxxx_ts->irq);
	/*add by jinpeng_He for get the id pin begin*/
	if(gpio_request(ftxxxx_ts->tp_id_gpio1,"tpiidpin1"))
	{
		printk(" [Focal][Touch] %s: gpio %d request for id pin 1 fail.\n", __func__, ftxxxx_ts->tp_id_gpio1);
		goto exit_tp_id_gpio1_request_failed;
	}
	gpio_direction_input(ftxxxx_ts->tp_id_gpio1);
	if(gpio_request(ftxxxx_ts->tp_id_gpio2,"tpiidpin2"))
	{
		printk("[Focal][Touch]  %s: gpio %d request for id pin 2 fail.\n", __func__, ftxxxx_ts->tp_id_gpio2);
		goto exit_tp_id_gpio2_request_failed;
	}
	gpio_direction_input(ftxxxx_ts->tp_id_gpio2);
	/*add by jinpeng_He for get the id pin end*/
	ftxxxx_ts->client->irq = gpio_to_irq(ftxxxx_ts->irq);
	err = request_threaded_irq(ftxxxx_ts->client->irq, NULL, ftxxxx_ts_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->dev.driver->name,
		ftxxxx_ts);
	//enable_irq_wake(ftxxxx_ts->irq);
	if (ftxxxx_ts->client->irq < 0) {
		dev_err(&client->dev, "[[Focal][Touch] %s: request irq fail. \n", __func__);
		goto exit_irq_request_failed;
	}

	spin_lock_init(&ftxxxx_ts->irq_lock);

	ftxxxx_irq_disable(ftxxxx_ts->client);	/*need mutex protect, should add latter*/

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "[Focal][Touch] %s: failed to allocate input device\n", __func__);
		goto exit_input_dev_alloc_failed;
	}
	

	ftxxxx_ts->input_dev = input_dev;
	
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_APPSELECT, input_dev->keybit);
	//set_bit(KEY_POWER, input_dev->keybit);

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
//	printk("maxx=%d,maxy=%d\n",ftxxxx_ts->x_max,ftxxxx_ts->y_max);
	input_mt_init_slots(input_dev, CFG_MAX_TOUCH_POINTS, 0);
//	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, CFG_MAX_TOUCH_POINTS, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 31, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ftxxxx_ts->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ftxxxx_ts->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, PRESS_MAX, 0, 0);

	input_dev->name = Focal_input_dev_name;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "[Focal][Touch] %s: failed to register input device: %s\n", __func__, dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
	/*make sure CTP already finish startup process */

#ifdef SYSFS_DEBUG
	//printk("[Focal][Touch] ftxxxx_create_sysfs Start !\n");
	ftxxxx_create_sysfs(client);
	//printk("[Focal][Touch] ftxxxx_create_sysfs End !\n");

	mutex_init(&ftxxxx_ts->g_device_mutex);

	wake_lock_init(&ftxxxx_ts->wake_lock, WAKE_LOCK_SUSPEND, "focal_touch_wake_lock");

	ftxxxx_ts->usb_wq = create_singlethread_workqueue("focal_usb_wq");
	if (!ftxxxx_ts->usb_wq) {
		printk("[Focal][Touch] %s: create usb workqueue failed\n", __func__);
		goto err_create_wq_failed;
	}
	INIT_WORK(&ftxxxx_ts->usb_detect_work, focal_cable_status);
#ifdef FTS_PM
	ftxxxx_ts->suspend_resume_wq = create_singlethread_workqueue("focal_suspend_resume_wq");
	if (!ftxxxx_ts->suspend_resume_wq) {
		printk("[Focal][Touch] %s: create resume workqueue failed\n", __func__);
		goto err_create_wq_failed;
	}
	INIT_WORK(&ftxxxx_ts->suspend_work, focal_suspend_work);
	INIT_WORK(&ftxxxx_ts->resume_work, focal_resume_work);
#endif



//#if defined(CONFIG_FB)
//tp_fb_notif.notifier_call = asus_otg_fb_notifier_callback;
//	err = fb_register_client(&tp_fb_notif);
//	if (err)
//		printk( "[Focal][Touch]Unable to register fb_notifier: %d\n", err);
//#endif

	ftxxxx_ts->reset_wq = create_singlethread_workqueue("focal_reset_ic_wq");
	if (!ftxxxx_ts->reset_wq) {
		printk("[Focal][Touch] %s: create reset ic workqueue failed\n", __func__);
		goto err_create_wq_failed;
	}
	INIT_WORK(&ftxxxx_ts->reset_ic_work, focal_reset_ic_work);

	ftxxxx_ts->touch_sdev.name = "touch";
	ftxxxx_ts->touch_sdev.print_name = focal_show_tpfwver;

	if (switch_dev_register(&ftxxxx_ts->touch_sdev) < 0)
	{
		printk("[Focal][Touch] %s: failed to register switch_dev \n", __func__);
		goto exit_err_sdev_register_fail;
	} 

	/*ASUS_BSP Jacob: add for creating virtual_key_maps +++*/
	#ifdef VIRTURAL_KEY
	focal_virtual_key_properties_kobj = kobject_create_and_add("board_properties", NULL);

	if (focal_virtual_key_properties_kobj)

		err = sysfs_create_group(focal_virtual_key_properties_kobj, &virtual_key_properties_attr_group);

	if (!focal_virtual_key_properties_kobj || err)

		printk("[Focal][Touch] %s : failed to create novaTP virtual key map! \n", __func__);
	#endif

	/*ASUS_BSP Jacob: add for creating virtual_key_maps ---*/
#endif
/*********************add by jinpeng_He for get the touch status and disable or enable touch**********/
	err = device_create_file(&client->dev, &dev_attr_touch_status);
	if (err) {
		dev_err(&client->dev, "[Focal][Touch] sys file creation failed\n");
		goto free_touch_status_sys;
	}
/*end jinpeng_He*/
#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(client) < 0)
		dev_err(&client->dev, "[Focal][Touch] %s : create fts control iic driver failed\n", __func__);
#endif
#ifdef FTS_APK_DEBUG
	ftxxxx_create_apk_debug_channel(client);
#endif

 	err = create_asusproc_touchsensor_status_entry();
	if(err<0)
	{
		printk("[Focal][Touch]create proc fialed\n");
	}
	/*get some register information */
	uc_reg_addr = FTXXXX_REG_FW_VER;
	err = ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (err < 0)
		ftxxxx_ts->init_success = 0;
	else {
		ftxxxx_ts->init_success = 1;
	//	printk("hjptest--->[Focal][Touch] Firmware version = 0x%x\n", uc_reg_value);
		}

	uc_reg_addr = FTXXXX_REG_POINT_RATE;
	err = ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (err < 0)
		ftxxxx_ts->init_success = 0;
	else {
		ftxxxx_ts->init_success = 1;
	//	printk("hjptest--->[Focal][Touch] report rate is %dHz.\n", uc_reg_value * 10);
		}

	uc_reg_addr = FTXXXX_REG_THGROUP;
	err = ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (err < 0)
		ftxxxx_ts->init_success = 0;
	else {
		ftxxxx_ts->init_success = 1;
	//	printk("hjptest--->[Focal][Touch] touch threshold is %d.\n", uc_reg_value * 4);
		}
		
	uc_reg_addr = FTXXXX_REG_VENDOR_ID;
	err = ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (err < 0)
		ftxxxx_ts->init_success = 0;
	else {
		ftxxxx_ts->init_success = 1;
	//	printk("hjptest--->[Focal][Touch] VENDOR ID = 0x%x\n", uc_reg_value);
		}

//	printk("hjptest--->[Focal][Touch] TP vendor ID = %d \n", ftxxxx_read_tp_id());

#ifdef FTS_GESTRUE	/*zax 20140922*/
	/* ++++ touch gesture mode support part in ZE500CL ++++ */
	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_C);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);

	__set_bit(KEY_POWER, input_dev->keybit);
	__set_bit(KEY_GESTURE_V, input_dev->keybit);
	__set_bit(KEY_GESTURE_Z, input_dev->keybit);
	__set_bit(KEY_GESTURE_C, input_dev->keybit);
	__set_bit(KEY_GESTURE_E, input_dev->keybit);
	__set_bit(KEY_GESTURE_S, input_dev->keybit);
	__set_bit(KEY_GESTURE_W, input_dev->keybit);
	/* ---- touch gesture mode support part in ZE500CL ---- */
#endif

	printk("[Focal][Touch][INFO] client name = %s irq = %d\n", client->name, client->irq);
//	printk("hjptest--->[Focal][Touch] X-RES = %d, Y-RES = %d, RST gpio = %d, gpio irq = %d, client irq = %d\n",
//		ftxxxx_ts->pdata->abs_x_max, ftxxxx_ts->pdata->abs_y_max, ftxxxx_ts->pdata->rst_gpio, ftxxxx_ts->irq, ftxxxx_ts->client->irq);

	if (ftxxxx_ts->init_success == 1) {
		focal_init_success = 1;
		if (g_ASUS_hwID >= ZE500KL_SR1 )
			FOCAL_IRQ_DISABLE = false;
	}
	ftxxxx_update_fw_ver(ftxxxx_ts);
	/*add by jinpeng_He for get the id pin state begin*/
	ftxxxx_ts->tp_id_value1=gpio_get_value(ftxxxx_ts->tp_id_gpio1);
	ftxxxx_ts->tp_id_value2=gpio_get_value(ftxxxx_ts->tp_id_gpio2);
	printk(KERN_WARNING "[Focal][Touch]id_value1=%d,id_value2=%d\n",ftxxxx_ts->tp_id_value1,ftxxxx_ts->tp_id_value2);
	/*add by jinpeng_He for get the id pin state end*/
	ftxxxx_ts->update_fw_wq=create_singlethread_workqueue("focal_fw_wq");
	if(!ftxxxx_ts->update_fw_wq)
	{
		printk("[Focal][Touch]--->%s:create update firmware woqkqueue failed\n",__func__);
		goto err_create_update_fw_wq_failed;
	}
	INIT_WORK(&ftxxxx_ts->update_fw_work,update_fw_in_wq);
	//fts_ctpm_auto_upgrade(client);
	queue_work(ftxxxx_ts->update_fw_wq,&ftxxxx_ts->update_fw_work);
	ftxxxx_irq_enable(ftxxxx_ts->client);
	printk("[Focal][Touch] FTxxxx prob process end !\n");
	return 0;

err_create_update_fw_wq_failed:
	if (ftxxxx_ts->update_fw_wq) {
		destroy_workqueue(ftxxxx_ts->update_fw_wq);
	}
free_touch_status_sys:
	device_remove_file(&client->dev, &dev_attr_touch_status);
exit_err_sdev_register_fail:
	switch_dev_unregister(&ftxxxx_ts->touch_sdev);
exit_input_register_device_failed:
	input_free_device(input_dev);

exit_input_dev_alloc_failed:
	free_irq(client->irq, ftxxxx_ts);
#ifdef CONFIG_PM
#if 0
exit_request_reset:
	gpio_free(ftxxxx_ts->pdata->reset);
#endif
#endif

err_create_wq_failed:
//#if defined(CONFIG_FB)
//	fb_unregister_client(&tp_fb_notif);
//#endif
	if (ftxxxx_ts->reset_wq) {
		destroy_workqueue(ftxxxx_ts->reset_wq);
	}
#ifdef FTS_PM
	if (ftxxxx_ts->suspend_resume_wq) {
		destroy_workqueue(ftxxxx_ts->suspend_resume_wq);
	}
#endif
	if (ftxxxx_ts->usb_wq) {
		destroy_workqueue(ftxxxx_ts->usb_wq);
	}
	
exit_tp_id_gpio2_request_failed:
exit_tp_id_gpio1_request_failed:
exit_irq_request_failed:
	fts_power_on(ftxxxx_ts, true);
exit_power_init:
	fts_power_init(&client->dev, ftxxxx_ts->pdata, false);
exit_init_gpio:
	fts_un_init_gpio_hw(ftxxxx_ts);

	i2c_set_clientdata(client, NULL);
	kfree(ftxxxx_ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

//#if defined(CONFIG_FB)
//static void ftxxxx_ts_early_suspend(void)
//{
//	ftxxxx_suspend();
//}


//static void ftxxxx_ts_late_resume(void)
//{
//	
//	queue_work(ftxxxx_ts->resume_wq, &ftxxxx_ts->resume_work);
//}
//static int asus_otg_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
//{
//	struct fb_event *evdata = data;
//	static int blank_old = 0;
//	int *blank;

//	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
//		blank = evdata->data;
//		if (*blank == FB_BLANK_UNBLANK) {
//			if (blank_old == FB_BLANK_POWERDOWN) {
//				blank_old = FB_BLANK_UNBLANK;
//				ftxxxx_ts_late_resume();
//			}
//		} else if (*blank == FB_BLANK_POWERDOWN) {
//			if (blank_old == 0 || blank_old == FB_BLANK_UNBLANK) {
//				blank_old = FB_BLANK_POWERDOWN;
//				ftxxxx_ts_early_suspend();
//			}
//		}
//	}
//	return 0;
//}
//#else
//static int ftxxxx_ts_suspend(struct device *dev)
//{
//	ftxxxx_suspend();
//	return 0;
//}
//static int ftxxxx_ts_resume(struct device *dev);
//{
//	queue_work(ftxxxx_ts->resume_wq, &ftxxxx_ts->resume_work);
//	return 0;
//}
//static const struct dev_pm_ops ftxxxx_ts_pm_ops = {
//	.suspend = ftxxxx_ts_suspend,
//	.resume = ftxxxx_ts_resume,
//};

//#endif
#ifdef FTS_PM
void ftxxxx_ts_suspend(void)
{
	queue_work(ftxxxx_ts->suspend_resume_wq,&ftxxxx_ts->suspend_work);
	return;
}
EXPORT_SYMBOL(ftxxxx_ts_suspend);
void ftxxxx_ts_resume(void)
{
	queue_work(ftxxxx_ts->suspend_resume_wq,&ftxxxx_ts->resume_work);
	return;
}
EXPORT_SYMBOL(ftxxxx_ts_resume);
#endif
static int ftxxxx_ts_remove(struct i2c_client *client)
{
	struct ftxxxx_ts_data *ftxxxx_ts;
	ftxxxx_ts = i2c_get_clientdata(client);
	input_unregister_device(ftxxxx_ts->input_dev);

#ifdef CONFIG_PM
	gpio_free(ftxxxx_ts->pdata->rst_gpio);
#endif
#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
#endif
#ifdef SYSFS_DEBUG
	ftxxxx_remove_sysfs(client);
#endif
#ifdef FTS_APK_DEBUG
	ftxxxx_release_apk_debug_channel();
#endif

	fts_un_init_gpio_hw(ftxxxx_ts);

	free_irq(client->irq, ftxxxx_ts);

	kfree(ftxxxx_ts);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id ftxxxx_ts_id[] = {
	{ FTXXXX_NAME, 0 },
	{ }
};
//in the dtsi msm8939-cdp-ze550kl.dtsi
//focaltech@38 {
//			compatible = "focaltech,5x06";

#ifdef CONFIG_OF
static struct of_device_id ft5x46_match_table[] = {
	{ .compatible = "focaltech,5X06",}, //here will compare with the dtsi
	{ },
};
#else
#define ft5x46_match_table NULL
#endif

/*MODULE_DEVICE_TABLE(i2c, ftxxxx_ts_id);*/

static struct i2c_driver ftxxxx_ts_driver = {
	.probe = ftxxxx_ts_probe,
	.remove = ftxxxx_ts_remove,
	.id_table = ftxxxx_ts_id,
	.driver = {
		.name = FTXXXX_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ft5x46_match_table,
	//	#ifdef CONFIG_FB
	//	#else
	//	   .pm = &ftxxxx_ts_pm_ops,
	//	#endif
	},
};

static int __init ftxxxx_ts_init(void)
{
	int ret;
	printk("[Focal][Touch] %s : ftxxxx_ts_init !\n", __func__);
	ret = i2c_add_driver(&ftxxxx_ts_driver);
	if (ret) {
		printk(KERN_WARNING " [Focal][Touch] Adding ftxxxx driver failed "
			"(errno = %d)\n", ret);
	} else {
		printk("[Focal][Touch] %s : Successfully added driver %s\n", __func__,
			ftxxxx_ts_driver.driver.name);
	}
	return ret;
}

static void __exit ftxxxx_ts_exit(void)
{
	i2c_del_driver(&ftxxxx_ts_driver);
}

module_init(ftxxxx_ts_init);
module_exit(ftxxxx_ts_exit);

MODULE_AUTHOR("<OSTeam>");
MODULE_DESCRIPTION("FocalTech TouchScreen driver");
MODULE_LICENSE("GPL");
