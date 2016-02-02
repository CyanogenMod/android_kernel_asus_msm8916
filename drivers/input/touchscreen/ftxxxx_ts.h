#ifndef __LINUX_ftxxxx_TS_H__
#define __LINUX_ftxxxx_TS_H__

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
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
//#include <linux/input/mt.h>
#include <linux/switch.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>

//#define FTS_APK_DEBUG		//not support now*/






/* -- dirver configure -- */
#define CFG_MAX_TOUCH_POINTS	10
#define FT_VDD_MIN_UV		2800000
#define FT_VDD_MAX_UV		3300000
#define FT_I2C_VCC_MIN_UV	1800000
#define FT_I2C_VCC_MAX_UV	1800000
/**add by jinpeng_He +++++**/
#define ZE550KL		0x0a
#define ZE551KL		0x0b
#define ZX550KL		0x05
#define ZD550KL		0x0e
#define ZE600KL		0x0c
#define ZE601KL		0x0f
#define LCD_VENDOR_CPT_HD		0x30
#define LCD_VENDOR_TM_HD		0x31
#define LCD_VENDOR_TM_FHD		0x32
#define LCD_VENDOR_AUO_FHD		0x33
#define LCD_VENDOR_CPT_HD_ZE600KL		0x34
#define LCD_VENDOR_IVO_HD_ZE600KL		0x35
#define LCD_VENDOR_AUO_FHD_ZE600KL		0x36




/**add by jinpeng_He -----**/


#define FT_COORDS_ARR_SIZE	4

#define PRESS_MAX	0xFF
#define FT_PRESS	0x08

#define FTXXXX_NAME	"Ft5x46"
//#define FTXXXX_NAME "ft5x06_ts"

#define Focal_input_dev_name	"ft5x06_ts"


#define FT_MAX_ID	0x0F
#define FT_TOUCH_STEP	6
#define FT_TOUCH_X_H_POS		3
#define FT_TOUCH_X_L_POS		4
#define FT_TOUCH_Y_H_POS		5
#define FT_TOUCH_Y_L_POS		6
#define FT_TOUCH_XY_POS			7
#define FT_TOUCH_MISC			8
#define FT_TOUCH_EVENT_POS		3
#define FT_TOUCH_ID_POS			5

#define POINT_READ_BUF	(3 + FT_TOUCH_STEP * CFG_MAX_TOUCH_POINTS)

/*register address*/
#define FTXXXX_REG_FW_VER		0xA6
#define FTXXXX_REG_POINT_RATE	0x88
#define FTXXXX_REG_THGROUP	0x80
#define FTXXXX_REG_VENDOR_ID	0xA8
#define FTXXXX_REG_PROJECT_ID	0xA1

#define FTXXXX_ENABLE_IRQ	1
#define FTXXXX_DISABLE_IRQ	0

/*************IO control setting***************/
#define TOUCH_TP_ON			1
#define TOUCH_TP_OFF			0
#define TOUCH_EC_ON			1
#define TOUCH_EC_OFF			0
#define TOUCH_FW_UPGRADE_ON		1
#define TOUCH_FW_UPGRADE_OFF		0
#define TOUCH_WIFI_ON			1
#define TOUCH_WIFI_OFF			0
#define TOUCH_IOC_MAGIC			0xF4
#define TOUCH_IOC_MAXNR			7
#define TOUCH_INIT			_IOR(TOUCH_IOC_MAGIC,	1,	int)
#define TOUCH_FW_UPDATE_FLAG		_IOR(TOUCH_IOC_MAGIC,	2,	int)
#define TOUCH_FW_UPDATE_PROCESS		_IOR(TOUCH_IOC_MAGIC,	3,	int)
#define TOUCH_TP_FW_check		_IOR(TOUCH_IOC_MAGIC,	4,	int)

#define TOUCH_FW_UPGRADE_SUCCESS	"0"
#define TOUCH_FW_UPGRADE_FAIL		"1"
#define TOUCH_FW_UPGRADE_PROCESS	"2"
#define TOUCH_FW_UPGRADE_INIT		"3"
/*************IO control setting***************/

#ifdef CONFIG_TOUCHSCREEN_FT5X46
#define FOCAL_TS_NAME	"Ft5x46"
#endif
#define TOUCH_SDEV_NAME	"touch"

struct focal_i2c_platform_data
{
	const char *name;
	uint16_t version;
	u32 abs_x_min;
	u32 abs_x_max;
	u32 abs_y_min;
	u32 abs_y_max;
	u32 intr_gpio;
	u32 intr_gpio_flag;
	u32 rst_gpio;
	u32 rst_gpio_flag;
	struct regulator *vdd;
	struct regulator *vcc;
};


int ftxxxx_i2c_Read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
int ftxxxx_i2c_Write(struct i2c_client *client, char *writebuf, int writelen);

void ftxxxx_reset_tp(int HighOrLow);
int ftxxxx_read_tp_id(void);
u8 get_focal_tp_fw(void);
void focal_glove_switch(bool plugin);
void ftxxxx_Enable_IRQ(struct i2c_client *client, int enable);
int focal_get_HW_ID(void);
void ftxxxx_nosync_irq_disable(struct i2c_client *client);
void ftxxxx_irq_disable(struct i2c_client *client);
void ftxxxx_irq_enable(struct i2c_client *client);
/* The platform data for the Focaltech ftxxxx touchscreen driver */


struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:0 -- down; 1-- contact; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u8 au8_finger_weight[CFG_MAX_TOUCH_POINTS];	/*touch weight */
	u8 pressure[CFG_MAX_TOUCH_POINTS];
	u8 area[CFG_MAX_TOUCH_POINTS];
	u8 touch_point;
	u8 Cur_touchpoint;
};

struct ftxxxx_ts_data {
	u32 irq;
	u32 x_max;
	u32 y_max;
	u32 init_success;
	bool suspend_flag;
	bool usb_status;
	bool glove_mode_eable;
	bool dclick_mode_eable;
	bool cover_mode_states;
	bool cover_mode_eable;
	bool gesture_mode_eable;
	u8 gesture_mode_type;
	bool reset_pin_status;
	bool irq_lock_status;
	spinlock_t irq_lock;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	struct switch_dev touch_sdev;
	struct focal_i2c_platform_data *pdata;
	struct mutex g_device_mutex;
	struct workqueue_struct *usb_wq;
	struct work_struct usb_detect_work;
	struct workqueue_struct *reset_wq;
	struct work_struct reset_ic_work;
	struct workqueue_struct *suspend_resume_wq;
	struct work_struct resume_work;
	struct work_struct suspend_work;
	/********add by jinpeng_He begin***********/
	u8 fw_ver[3];
	u32 tp_id_gpio1;
	u32 tp_id_gpio2;
	u32 tp_id_value1;
	u32 tp_id_value2;
	struct workqueue_struct *update_fw_wq;
	struct work_struct update_fw_work;
	u8 lcd_vendor;
	/********add by jinpeng_He end*****************/
	
	/* Wakelock Protect */
	struct wake_lock wake_lock;
	/* Wakelock Protect */
};

#endif
