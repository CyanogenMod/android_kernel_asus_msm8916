/*
 * Copyright (C) 2012 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input/kionix_gsensor.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/regulator/consumer.h>

int build_version = 0;
bool bypass_for_eng_mode = true ;

/************************************************
 * Use to open/close the debugmessage
 *************************************************/
int KX022_DEBUG_MESSAGE = 0;
int KX022_REG_MESSAGE = 0;
int KX022_CALIBRATED_MESSAGE = 1;

#define KX022_RESUME_DISABLE		0
#define KX022_RESUME_ENABLE		1
#define KX022_RESUME_MISSDISABLE	2
#define KX022_RESUME_MISSENABLE	3
/* POWER SUPPLY VOLTAGE RANGE */
#define kx022_VDD_MIN_UV	2000000
#define kx022_VDD_MAX_UV	3300000
#define kx022_VIO_MIN_UV	1750000
#define kx022_VIO_MAX_UV	1950000


//<ASUS-annacheng20150129>>>>>>>>>>>>
#define Driverversion  "1.0.0"  
#define VENDOR  "KX022-1020"    
struct proc_dir_entry *kx022sensor_entry=NULL;
//<ASUS-annacheng20150129><<<<<<<<<<<<+
static int Gsensor_status = 0;

/******
 * 	The following table lists the maximum appropriate poll interval for each available output data rate.
 *	Default Setting :
 *	  kxtj9_odr_table[] = {
 *		{ 1,		ODR1600F },
 *		{ 3,		ODR800F },
 *		{ 5,		ODR400F },
 *		{ 10,	ODR200F },
 *		{ 20,	ODR100F },
 *		{ 40,	ODR50F  },
 *		{ 80,	ODR25F  },
 *		{ 0xFFFFFFFF,	ODR12_5F},
 *	//	{ 0xFFFFFFFF,		ODR3_125F,	KXTJ9_RES_8BIT},				// 320,	251~max	NO POLL
 *	};
 */

struct kionix_odr_table {
	unsigned int cutoff;
	u8 mask;
	int RES;
};
static const struct kionix_odr_table kx022_odr_table[] = {
															/*  ms,	range,	mode */
	{ 15,	 			ODR200F,	KX022_RES_16BIT},			/*  2.5,	6~ 10	FASTEST MODE , full power mode */
	{ 35, 			ODR50F,		KX022_RES_16BIT},			/*  20,	21~30	GAME MODE */
	{ 70, 			ODR25F,		KX022_RES_16BIT},			/*  66,	31~70	UI MODE */
	{ 250,			ODR12_5F,	KX022_RES_16BIT},			/*  80,	71~250	NORMAL MODE */
	{ 0xFFFFFFFF,		ODR0_781F,	KX022_RES_16BIT},			/*  1280,	251~max	NO POLL */
};

/*************************************************
 * 	kionix kernel driver data struture
 *************************************************/
struct ASUS_Gsensor_data	{
	struct i2c_client		*client;
	struct input_dev		*input_dev;
	struct mutex			lock;				/* For muxtex lock */
	unsigned int			last_poll_interval;
	atomic_t		enabled;
	u8		ctrl_reg1;
	u8		data_ctrl;
	u8		int_ctrl;
	int		suspend_resume_state;
	int		resume_enable;
	int		irq_status;
	int		irq;
 	bool   power_enabled;
	struct regulator *vdd;
	struct regulator *vio;
#ifdef CONFIG_INPUT_KXTJ9_POLLED_MODE
	struct input_polled_dev *poll_dev;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND	
	struct early_suspend gsensor_early_suspendresume;
#endif
// for enable motion detect , added by cheng_kao 2014.02.12 ++
	u8 wufe_rate;
	u8 wufe_timer;
	u8 wufe_thres;
// for enable motion detect , added by cheng_kao 2014.02.12 --
// added by cheng_kao 2013.06.01  for sensors calibration ++
	int accel_cal_data[6];
	int accel_cal_offset[3];
	int accel_cal_sensitivity[3];
// added by cheng_kao 2013.06.01  for sensors calibration --
// for enable motion detect , added by cheng_kao 2014.02.12 ++
	int motion_detect_threshold_x;
	int motion_detect_threshold_y;
	int motion_detect_threshold_z;
	int motion_detect_timer;
	int chip_interrupt_mode;
// for enable motion detect , added by cheng_kao 2014.02.12 --
};

static struct ASUS_Gsensor_data *kionix_Gsensor_data;

//<asus-annacheng20150129>>>>>>>>>>>>>>>>>>>>>>>+

static int kx022sensor_proc_show(struct seq_file *m, void *v) {
	
		int ret = 0,retval=0;		
		struct ASUS_Gsensor_data  *kx022=kionix_Gsensor_data;	
		
		retval = i2c_smbus_read_byte_data(kx022->client, WHO_AM_I);
		if (retval < 0)
			printk(KERN_INFO "[kx022] i2c err\n");
								
		printk("anna-sensor idReg : 0x00=0x%x \n", retval);
	      
	  	if(retval<0){
			ret =seq_printf(m," ERROR: i2c r/w test fail\n");	
		}else{
			ret =seq_printf(m," ACK: i2c r/w test ok\n");	
		}
		if(Driverversion != NULL){
			ret =seq_printf(m," Driver version:%s\n",Driverversion);
		}
		else{
			ret =seq_printf(m," Driver version:NULL\n");
		}
	  	if(retval==0x1f){//who am i
	  		ret =seq_printf(m," Vendor:%s(0x%x)\n", VENDOR,retval);
		}else{
			ret =seq_printf(m," Vendor:%s(0x%x)\n", VENDOR,retval);
		}

		retval = i2c_smbus_read_byte_data(kx022->client, WHO_AM_I);
		printk("anna-sensor idReg33 : 0x00=0x%x \n", retval);
		
		if(retval<0){	  		
			ret =seq_printf(m," Device status:error\n");			
		}else{
			ret =seq_printf(m," Device status:ok\n");			
		}
	     	    
   	return ret;

}


static int kx022sensor_proc_open(struct inode *inode, struct  file *file) {
  return single_open(file, kx022sensor_proc_show, NULL);
}

static const struct file_operations kx022sensor_proc_fops = {
  .owner = THIS_MODULE,
  .open = kx022sensor_proc_open,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

int create_asusproc_kx022sensor_status_entry(void){
	kx022sensor_entry = proc_create("gsensor_status", S_IWUGO| S_IRUGO, NULL,&kx022sensor_proc_fops);
 	if (!kx022sensor_entry)
       		 return -ENOMEM;

    	return 0;
}
//<asus-annacheng20150129><<<<<<<<<<<<<<<<<+
static int kx022_into_drdy_function(void)
{
	int result = 0;
	// turn off power
	result = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG1, 0);
	if (result < 0){
		printk("[Gsensor] alp : drdy motion_detect_power_off_fail !\n");
		return result;
	}

	// turn on power
	kionix_Gsensor_data->ctrl_reg1 |= DRDYE;
	kionix_Gsensor_data->ctrl_reg1 &= WUFE_OFF;
	if (KX022_DEBUG_MESSAGE)
		printk("[Gsensor] alp : kx022_into_drdy_function (%d)\n", kionix_Gsensor_data->ctrl_reg1);
	result = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG1, kionix_Gsensor_data->ctrl_reg1); // enable motion detect
	if (result < 0){
		printk("[Gsensor] alp : kx022_into_drdy_function : into fail !\n");
		return result;
	}
	mdelay(50); // 50 ms

	return result;
}
/*
static int kx022_into_motion_detect_function(void)
{
	int result = 0;
	// turn off power
	result = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG1, 0);
	if (result < 0){
		printk("[Gsensor] alp : motion_detect_power_off_fail !\n");
		return result;
	}

	// set the chip timer
	result = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, WAKEUP_TIMER, kionix_Gsensor_data->wufe_timer );
	if (result < 0){
		printk("[Gsensor] alp : set_timer_fail !\n");
		return result;
	}

	// set the chip threshold
	result = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, WAKEUP_THRES, kionix_Gsensor_data->wufe_thres);
	if (result < 0){
		printk("[Gsensor] alp : set_chip_threshold_fail !\n");
		return result;
	}

	// set the chip wufe rate
	result = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG2, kionix_Gsensor_data->wufe_rate);
	if (result < 0){
		printk("[Gsensor] alp : set_chip_wufe_rate_fail !\n");
		return result;
	}

	// turn on power
	kionix_Gsensor_data->ctrl_reg1 |= WUFE_ON;
	kionix_Gsensor_data->ctrl_reg1 &= DRDYE_OFF;
	if (KX022_DEBUG_MESSAGE)
		printk("[Gsensor] alp : kx022_into_motion_detect_function (%d)\n", kionix_Gsensor_data->ctrl_reg1);
	result = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG1, kionix_Gsensor_data->ctrl_reg1); // enable motion detect
	if (result < 0){
		printk("[Gsensor] alp : kx022_into_motion_detect_function : into fail !\n");
		return result;
	}
	mdelay(50); // 50 ms

	return result;
}
*/

static int kx022_reset_function(struct i2c_client *client)
{
	int result = 0;
	kionix_Gsensor_data->ctrl_reg1 &= PC1_OFF;
	result = i2c_smbus_write_byte_data(client, CTRL_REG2, 0x84); // SRST set to 1
	if (result < 0){
		printk("[Gsensor] alp : kx022_reset_function : reset fail (%d)\n",result);
		result = 1;
		return result;
	}
	mdelay(50); // 50 ms

	/* ensure that PC1 is cleared before updating control registers */
	result = i2c_smbus_write_byte_data(client, CTRL_REG1, 0);
	if (result < 0){
		result = 3;
		return result;
	}
	/* only write INT_CTRL_REG1 if in irq mode */
	if (kionix_Gsensor_data->irq) {
		result = i2c_smbus_write_byte_data(client,INT_CTRL1, kionix_Gsensor_data->int_ctrl);
		result = i2c_smbus_write_byte_data(client,INT_CTRL4, KX022_DRDYI1);
		if (result < 0){
			result = 4;
			return result;
		}
	}

	/* turn on outputs */
	kionix_Gsensor_data->ctrl_reg1 |= PC1_ON;
	result = i2c_smbus_write_byte_data(client, CTRL_REG1, kionix_Gsensor_data->ctrl_reg1);
	if (result < 0){
		result = 5;
		return result;
	}

	/* clear initial interrupt if in irq mode */
	if (kionix_Gsensor_data->irq) {
		result = i2c_smbus_read_byte_data(client, INT_REL);
		if (result < 0){
			result = 6;
			return result;
		}
	}
	atomic_set(&kionix_Gsensor_data->enabled, 1);
		
	return result;
}

static int kx022_i2c_read(u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = kionix_Gsensor_data->client->addr,
			.flags = kionix_Gsensor_data->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = kionix_Gsensor_data->client->addr,
			.flags = kionix_Gsensor_data->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	return i2c_transfer(kionix_Gsensor_data->client->adapter, msgs, 2);
}

static void kx022_report_acceleration_data(void)
{
	unsigned char acc_data[6];
	int rawdata_x=0,rawdata_y=0,rawdata_z=0,err=0, ireset=0;
	int reportdata_x=0,reportdata_y=0,reportdata_z=0;
	err = kx022_i2c_read(XOUT_L, (u8 *)acc_data, 6);
	if (err < 0){
		dev_err(&kionix_Gsensor_data->client->dev, "accelerometer data read failed\n");
		return;
	}
	//rawdata_x = ( acc_data[0] | (acc_data[1]<<8) );
	//rawdata_y = ( acc_data[2] | (acc_data[3]<<8) );
	rawdata_x = ( acc_data[2] | (acc_data[3]<<8) );
	rawdata_y = ( acc_data[0] | (acc_data[1]<<8) );
	rawdata_z = ( acc_data[4] | (acc_data[5]<<8) );

	if( (g_kxtj_for_camera_x==rawdata_x)&&(g_kxtj_for_camera_y==rawdata_y)&&(g_kxtj_for_camera_z==rawdata_z) ){
		printk("[Gsensor] alp : the same rawdata!!!\n");
		ireset = KX022_RESET_FOR_SAME_RAWDATA;
	}

	if( (rawdata_x==0)&&(rawdata_y==0)&&(rawdata_z==0) ){
		printk("[Gsensor] alp : the zero rawdata!!!\n");
		ireset = KX022_RESET_FOR_ZERO_RAWDATA;
	}

	if(ireset!=KX022_VALUE_FOR_NOT_NEED_RESET){
		err=kx022_reset_function(kionix_Gsensor_data->client);
		printk("[Gsensor] alp :  kx022_reset_function : %d\n",err);		
	}

	if(rawdata_x > 16383)
		rawdata_x = rawdata_x-65536;
	if(rawdata_y > 16383)
		rawdata_y = rawdata_y-65536;
	if(rawdata_z > 16383)
		rawdata_z = rawdata_z-65536;

	//rawdata_x = rawdata_x*(-1);
	rawdata_y = rawdata_y*(-1);

	// transfromed by chip location
	switch(g_ilocation){
		case KX022_CHIP_LOCATION_SR_ZC500KL:
			if ((kionix_Gsensor_data->accel_cal_sensitivity[0] == 0) || (kionix_Gsensor_data->accel_cal_sensitivity[1] == 0) || (kionix_Gsensor_data->accel_cal_sensitivity[2] == 0)) {
				reportdata_x = rawdata_y*(-1);
				reportdata_y = rawdata_x;
				reportdata_z = rawdata_z;
				if (KX022_DEBUG_MESSAGE) {
					printk("[Gsensor] alp : LOCATION_ZC500KL_SR default\n");
				}
			} else {
				reportdata_x = -1024 * (rawdata_y - kionix_Gsensor_data->accel_cal_offset[1]) / kionix_Gsensor_data->accel_cal_sensitivity[1];
				reportdata_y = 1024 * (rawdata_x - kionix_Gsensor_data->accel_cal_offset[0]) / kionix_Gsensor_data->accel_cal_sensitivity[0];
				reportdata_z = 1024 * (rawdata_z - kionix_Gsensor_data->accel_cal_offset[2]) / kionix_Gsensor_data->accel_cal_sensitivity[2];
				if (KX022_DEBUG_MESSAGE) {
					printk("[Gsensor] alp : LOCATION_ZC500KL_SR calibration\n");
				}
			}
		break;
	}

	if(KX022_DEBUG_MESSAGE) printk("[Gsensor] report_acceleration data : (%d), (%d), (%d)\n",reportdata_x, reportdata_y, reportdata_z);
	g_kxtj_for_camera_x = reportdata_x;
	g_kxtj_for_camera_y = reportdata_y;
	g_kxtj_for_camera_z = reportdata_z;
	input_report_abs(kionix_Gsensor_data->input_dev, ABS_X, reportdata_x);
	input_report_abs(kionix_Gsensor_data->input_dev, ABS_Y, reportdata_y);
	input_report_abs(kionix_Gsensor_data->input_dev, ABS_Z, reportdata_z);
	input_sync(kionix_Gsensor_data->input_dev);
}

static irqreturn_t kx022_isr(int irq, void *dev)
{
	int err;

	/* data ready is the only possible interrupt type */
	kx022_report_acceleration_data();
	err = i2c_smbus_read_byte_data(kionix_Gsensor_data->client, INT_REL);
	if (err < 0){
		printk("[Gsensor] alp : kx022_isr err(%x)\n",err);
	}

	return IRQ_HANDLED;
}

static int kx022_update_odr(unsigned int poll_interval)
{
	int err;
	int i;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kx022_odr_table); i++) {
		if (poll_interval < kx022_odr_table[i].cutoff)
			break;
	}
	kionix_Gsensor_data->data_ctrl = kx022_odr_table[i].mask;
	printk("[Gsensor] alp : kx022_update_odr  i(%d), cutoff(%d), mask(%d), poll(%d)\n", i, kx022_odr_table[i].cutoff, kx022_odr_table[i].mask, poll_interval);
	printk("[Gsensor] alp : data_ctrl(%x), ctrl_reg(%x)\n", kionix_Gsensor_data->data_ctrl, kionix_Gsensor_data->ctrl_reg1);

	kionix_Gsensor_data->ctrl_reg1 |=RES_16bit;

	err = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, DATA_CTRL, kionix_Gsensor_data->data_ctrl);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG1, kionix_Gsensor_data->ctrl_reg1);
	if (err < 0)
		return err;

	return 0;
}

static int kx022_power_on(bool on)
{
	int rc = 0;

	if (!on && kionix_Gsensor_data->power_enabled) {
		rc = regulator_disable(kionix_Gsensor_data->vdd);
		if (rc) {
			dev_err(&kionix_Gsensor_data->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(kionix_Gsensor_data->vio);
		if (rc) {
			dev_err(&kionix_Gsensor_data->client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			rc = regulator_enable(kionix_Gsensor_data->vdd);
			if (rc) {
				dev_err(&kionix_Gsensor_data->client->dev,
					"Regulator vdd enable failed rc=%d\n",
					rc);
			}
		}
		kionix_Gsensor_data->power_enabled = false;
	} else if (on && !kionix_Gsensor_data->power_enabled) {
		rc = regulator_enable(kionix_Gsensor_data->vdd);
		if (rc) {
			dev_err(&kionix_Gsensor_data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(kionix_Gsensor_data->vio);
		if (rc) {
			dev_err(&kionix_Gsensor_data->client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			regulator_disable(kionix_Gsensor_data->vdd);
		}
		kionix_Gsensor_data->power_enabled = true;
	} else {
		dev_warn(&kionix_Gsensor_data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, kionix_Gsensor_data->power_enabled);
	}

	return rc;
}

static int kx022_power_init(bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(kionix_Gsensor_data->vdd) > 0)
			regulator_set_voltage(kionix_Gsensor_data->vdd, 0, kx022_VDD_MAX_UV);

		regulator_put(kionix_Gsensor_data->vdd);

		if (regulator_count_voltages(kionix_Gsensor_data->vio) > 0)
			regulator_set_voltage(kionix_Gsensor_data->vio, 0, kx022_VIO_MAX_UV);

		regulator_put(kionix_Gsensor_data->vio);
	} else {
		kionix_Gsensor_data->vdd = regulator_get(&kionix_Gsensor_data->client->dev, "vdd");
		if (IS_ERR(kionix_Gsensor_data->vdd)) {
			rc = PTR_ERR(kionix_Gsensor_data->vdd);
			dev_err(&kionix_Gsensor_data->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(kionix_Gsensor_data->vdd) > 0) {
			rc = regulator_set_voltage(kionix_Gsensor_data->vdd, kx022_VDD_MIN_UV,
						   kx022_VDD_MAX_UV);
			if (rc) {
				dev_err(&kionix_Gsensor_data->client->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		kionix_Gsensor_data->vio = regulator_get(&kionix_Gsensor_data->client->dev, "vio");
		if (IS_ERR(kionix_Gsensor_data->vio)) {
			rc = PTR_ERR(kionix_Gsensor_data->vio);
			dev_err(&kionix_Gsensor_data->client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(kionix_Gsensor_data->vio) > 0) {
			rc = regulator_set_voltage(kionix_Gsensor_data->vio, kx022_VIO_MIN_UV,
						   kx022_VIO_MAX_UV);
			if (rc) {
				dev_err(&kionix_Gsensor_data->client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(kionix_Gsensor_data->vio);
reg_vdd_set:
	if (regulator_count_voltages(kionix_Gsensor_data->vdd) > 0)
		regulator_set_voltage(kionix_Gsensor_data->vdd, 0, kx022_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(kionix_Gsensor_data->vdd);
	return rc;
}
static int kx022_device_power_on(void)
{
	int err = 0;

		err = kx022_power_on(true);
		if (err) {
			dev_err(&kionix_Gsensor_data->client->dev, "power on failed");
			goto err_exit;
		}
		/* Use 80ms as vendor suggested. */
		msleep(80);

err_exit:
	dev_dbg(&kionix_Gsensor_data->client->dev, "soft power on complete err=%d.\n", err);
	return err;
}

static void kx022_device_power_off(void)
{
	int err;

	kionix_Gsensor_data->ctrl_reg1 &= PC1_OFF;
	err = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG1, kionix_Gsensor_data->ctrl_reg1);
	if (err < 0)
		dev_err(&kionix_Gsensor_data->client->dev, "soft power off failed\n");

	kx022_power_on(false);

	dev_dbg(&kionix_Gsensor_data->client->dev, "soft power off complete.\n");
	return ;
}

static int kx022_enable(void)
{
	int err;
	if (KX022_DEBUG_MESSAGE)
		printk("[Gsensor] alp:kx022_enable ++\n");

	err = kx022_device_power_on();
	if (err < 0)	
		 return err;

	if (kionix_Gsensor_data->suspend_resume_state == 1)	{
		printk("[Gsensor] alp : kx022_enable  already suspend return !\n");
		kionix_Gsensor_data->resume_enable = KX022_RESUME_MISSENABLE;
		return 0;
	}

	if (atomic_read(&kionix_Gsensor_data->enabled) == 1)	{
		printk("[Gsensor] alp : kx022_enable  already enable , return 0\n");
		return 0;
	}
	
	if(kionix_Gsensor_data->irq_status==0){
		enable_irq(kionix_Gsensor_data->irq);
		printk("[Gsensor] alp : kx022_enable  irq (%d)\n", kionix_Gsensor_data->irq);
		kionix_Gsensor_data->irq_status = 1;
	}

	/* ensure that PC1 is cleared before updating control registers */
	err = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	/* only write INT_CTRL_REG1 if in irq mode */
	if (kionix_Gsensor_data->irq) {
		err = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, INT_CTRL1, kionix_Gsensor_data->int_ctrl);
		err = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, INT_CTRL4, KX022_DRDYI1);
		if (err < 0){
			printk("[Gsensor] alp : kx022_enable fail 1 irq = %d\n", kionix_Gsensor_data->irq);
			return err;
		}
		if (KX022_DEBUG_MESSAGE)
			printk("[Gsensor] alp : kx022_enable  irq = %d(0x%x)\n", kionix_Gsensor_data->irq, kionix_Gsensor_data->int_ctrl);
	}

	/* turn on outputs */
	if (KX022_DEBUG_MESSAGE)
		printk("[Gsensor] alp : turn on ++\n");
	kionix_Gsensor_data->ctrl_reg1 |= PC1_ON;
	err = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG1, kionix_Gsensor_data->ctrl_reg1);
	if (KX022_DEBUG_MESSAGE)
		printk("[Gsensor] alp : turn on -- (%d)\n", kionix_Gsensor_data->ctrl_reg1);
	if (err < 0)	{
		printk("[Gsensor] alp : turn on fail -- (%d)\n", err);
		return err;
	}

	err = kx022_update_odr(kionix_Gsensor_data->last_poll_interval);
	if (err < 0)
		return err;

	/* clear initial interrupt if in irq mode */
	if (kionix_Gsensor_data->irq) {
		err = i2c_smbus_read_byte_data(kionix_Gsensor_data->client, STATUS_REG);
		if(KX022_DEBUG_MESSAGE)
			printk("[Gsensor]  STATUS_REG(%x)\n", err);
		err = i2c_smbus_read_byte_data(kionix_Gsensor_data->client, INT_REL);
		if (err < 0) {
			if (KX022_DEBUG_MESSAGE)
				printk("[Gsensor] alp : error clearing interrupt: %d\n", err)	;
			goto fail;
		}
	}
	atomic_set(&kionix_Gsensor_data->enabled, 1);

	return 0;

fail:
	kx022_device_power_off();
	return err;
}

static void kx022_disable(void)
{
	if(atomic_read(&kionix_Gsensor_data->enabled)==0){
		printk("[Gsensor] alp : kx022_disable  already disable , return !!\n");
		return ;
	}

	if(kionix_Gsensor_data->irq_status==1){
		disable_irq(kionix_Gsensor_data->irq);
		kionix_Gsensor_data->irq_status = 0;
	}

	kx022_device_power_off();
	atomic_set(&kionix_Gsensor_data->enabled, 0);
}

static int kx022_enable_by_orientation(void)
{
	int err;

	if(KX022_DEBUG_MESSAGE) printk("[Gsensor] alp:kx022_enable_by_orientation ++\n");

	err = kx022_device_power_on();
	if (err < 0)	
		 return err;

	/* ensure that PC1 is cleared before updating control registers */
	err = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	/* only write INT_CTRL_REG1 if in irq mode */
	if (kionix_Gsensor_data->irq) {
		err = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, INT_CTRL1, kionix_Gsensor_data->int_ctrl);
		err = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, INT_CTRL4, KX022_DRDYI1);
		if (err < 0){
			if(KX022_DEBUG_MESSAGE) printk("[Gsensor] alp : kx022_enable fail 1 irq = %d\n", kionix_Gsensor_data->irq);
			return err;
		}
		if(KX022_DEBUG_MESSAGE) printk("[Gsensor] alp : kx022_enable  irq = %d\n", kionix_Gsensor_data->irq);
	}

	/* turn on outputs */
	kionix_Gsensor_data->ctrl_reg1 |= PC1_ON;
	err = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG1, kionix_Gsensor_data->ctrl_reg1);
	if (err < 0)
		return err;

	/* clear initial interrupt if in irq mode */
	if (kionix_Gsensor_data->irq) {
		err = i2c_smbus_read_byte_data(kionix_Gsensor_data->client, INT_REL);
		if (err < 0) {
			if(KX022_DEBUG_MESSAGE) printk("[Gsensor] alp : error clearing interrupt: %d\n",err)	;
			goto fail;
		}
	}
	return 0;

fail:
	kx022_device_power_off();
	return err;
}

static int kx022_setup_input_device(void)
{
	int err;
	printk("[Gsensor] alp :  kx022_init_input_device ++\n");

	kionix_Gsensor_data->input_dev = input_allocate_device();
	if (!kionix_Gsensor_data->input_dev) {
		dev_err(&kionix_Gsensor_data->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	set_bit(EV_ABS, kionix_Gsensor_data->input_dev->evbit);
	set_bit(EV_SYN, kionix_Gsensor_data->input_dev->evbit);
	set_bit(EV_SW, kionix_Gsensor_data->input_dev->evbit);
	input_set_abs_params(kionix_Gsensor_data->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(kionix_Gsensor_data->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(kionix_Gsensor_data->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	kionix_Gsensor_data->input_dev->name = "kx022_accel";
	kionix_Gsensor_data->input_dev->id.bustype = BUS_I2C;
	kionix_Gsensor_data->input_dev->dev.parent = &kionix_Gsensor_data->client->dev;

	err = input_register_device(kionix_Gsensor_data->input_dev);
	if (err) {
		dev_err(&kionix_Gsensor_data->client->dev,
			"unable to register input polled device %s: %d\n",
			kionix_Gsensor_data->input_dev->name, err);
		input_free_device(kionix_Gsensor_data->input_dev);
		return err;
	}
	printk("[Gsensor] alp :  kx022_init_input_device --\n");

	return 0;
}

/*
 * When IRQ mode is selected, we need to provide an interface to allow the user
 * to change the output data rate of the part.  For consistency, we are using
 * the set_poll method, which accepts a poll interval in milliseconds, and then
 * calls update_odr() while passing this value as an argument.  In IRQ mode, the
 * data outputs will not be read AT the requested poll interval, rather, the
 * lowest ODR that can support the requested interval.  The client application
 * will be responsible for retrieving data from the input node at the desired
 * interval.
 */
static ssize_t gsensor_chip_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int retval=0;

	retval = i2c_smbus_read_byte_data(kionix_Gsensor_data->client, WHO_AM_I);

	printk("[Gsensor] alp : whoami = %d\n", retval);

	if(retval == 0x09)
		printk("[Gsensor] alp : use kxtj2!!\n");
	else if(retval == 0x08)
		printk("[Gsensor] alp : use kxtj9!!\n");
	else if(retval < 0)
		printk("[Gsensor] alp : read fail!!\n");

	return sprintf(buf, "%d\n", retval);
}

static ssize_t gsensor_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int err=0;

	err = i2c_smbus_read_byte_data(kionix_Gsensor_data->client, WHO_AM_I);
	if (err < 0)
		Gsensor_status = 0;
	else
		Gsensor_status = 1;

	return sprintf(buf, "%d\n", Gsensor_status);
}

static ssize_t gsensor_dump_reg(struct device *dev, struct device_attribute *attr, char *buf)	
{
	int i = 0, val = 0;

	for(i = 0; i <0x3b; i++)
	{
		val = i2c_smbus_read_byte_data(kionix_Gsensor_data->client, i);
		printk("[Gsensor] cmd =%02x value = %02x\n", i, val);
	}
	val = i2c_smbus_read_byte_data(kionix_Gsensor_data->client, 0x6A);
	printk("[Gsensor] cmd = 6A value = %02x\n", val);
	
	return sprintf(buf, "%d\n", Gsensor_status);
}

static ssize_t gsensor_read_raw(struct device *dev, struct device_attribute *attr, char *buf)
{
	int retval = 0,rawdata_x=0,rawdata_y=0,rawdata_z=0 ;
	unsigned char acc_data[6];
	static bool user_enable_sensor = false;
	
	retval = atomic_read(&kionix_Gsensor_data->enabled);

	printk("[Gsensor] alp : get_rawdata enable state=(%d)\n",retval);
	if(retval==0){
		retval = kx022_enable();
		mdelay(100);
		if (retval < 0){
			printk("[Gsensor] ATTR power on fail\n");
			return retval;
		}
		user_enable_sensor = true;
	}

	retval = kx022_i2c_read(XOUT_L, (u8 *)acc_data, 6);
	if (retval < 0){
		dev_err(&kionix_Gsensor_data->client->dev, "accelerometer data read failed\n");
		return sprintf(buf, "get no data\n");
	}

	/* ZE500KL Project */
	rawdata_x = (acc_data[2] | (acc_data[3]<<8));
	rawdata_y = (acc_data[0] | (acc_data[1]<<8));
	rawdata_z = (acc_data[4] | (acc_data[5]<<8));

	if (rawdata_x > 16383)
		rawdata_x = rawdata_x - 65536;
	if (rawdata_y > 16383)
		rawdata_y = rawdata_y - 65536;
	if (rawdata_z > 16383)
		rawdata_z = rawdata_z - 65536;

	//rawdata_x = rawdata_x*(-1);
        rawdata_y = rawdata_y*(-1);

	if (user_enable_sensor == true)
	{
		kx022_disable();
		user_enable_sensor = false;
	}

	return sprintf(buf, "%d %d %d\n", rawdata_x, rawdata_y, rawdata_z);
}

static ssize_t kxtj9_r_en_mt(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = (bypass_for_eng_mode == true) ? (0) : (1);

	return sprintf(buf, "en(%d)  bypass(%d) \n", ret, bypass_for_eng_mode);
}

static ssize_t kxtj9_w_en_mt(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = simple_strtoul(buf, NULL, 10);

	if (val == 1) { /* enable mt => disable bypass mode */
		bypass_for_eng_mode = false;
	} else {
		bypass_for_eng_mode = true;
		kx022_into_drdy_function();
	}
	printk(KERN_INFO "[KXTJ9] en(%d)  bypass(%d) \n", val, bypass_for_eng_mode);

	return count;
}

static ssize_t gsensor_show_message(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("[Gsensor] alp : kx022_message DEBUG(%d), REG(%d), CALI(%d)\n",
			KX022_DEBUG_MESSAGE, KX022_REG_MESSAGE, KX022_CALIBRATED_MESSAGE);
	return sprintf(buf, "\n[Gsensor] alp : kx022_message DEBUG(%d), REG(%d), CALI(%d)\n",
			KX022_DEBUG_MESSAGE, KX022_REG_MESSAGE, KX022_CALIBRATED_MESSAGE);
}

static ssize_t gsensor_set_message(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = simple_strtoul(buf, NULL, 10);
	switch(val){
		case 0:
			KX022_DEBUG_MESSAGE = 0;
			KX022_REG_MESSAGE = 0;
			KX022_CALIBRATED_MESSAGE = 0;
			printk("[Gsensor] alp : disable all message !!!\n");
		break;

		case 1:
			KX022_DEBUG_MESSAGE = 1;
			KX022_REG_MESSAGE = 1;
			KX022_CALIBRATED_MESSAGE = 1;
			printk("[Gsensor] alp : enable all message !!!\n");
		break;

		case 2:
			KX022_REG_MESSAGE = 1;
			printk("[Gsensor] alp : enable  REG_MESSAGE !!!\n");
		break;

		case 3:
			KX022_CALIBRATED_MESSAGE = 1;
			printk("[Gsensor] alp : enable  CALIBRATED_MESSAG !!!\n");
		break;

		case 4:
			KX022_DEBUG_MESSAGE = 1;
			printk("[Gsensor] alp : enable  DEBUG_MESSAGE !!!\n");
		break;

		default:
			printk("[Gsensor] alp : error input !!!\n");
		break;
	}
	return count;
}

/* Returns currently selected poll interval (in ms) */
static ssize_t gsensor_get_poll(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int gsensor_ctrl = 0;
	gsensor_ctrl = i2c_smbus_read_byte_data(client, DATA_CTRL);
	printk("[Gsensor] alp : gsensor_get_poll (%d) (%d)\n", kionix_Gsensor_data->last_poll_interval, gsensor_ctrl);
	return sprintf(buf, "%d , %d\n", kionix_Gsensor_data->last_poll_interval, gsensor_ctrl);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t gsensor_set_poll(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int interval=0;

	interval = simple_strtoul(buf, NULL, 10);
	if (KX022_DEBUG_MESSAGE)
		printk("[Gsensor] alp : interval (%u)\n", interval);
	if (KX022_DEBUG_MESSAGE)
		printk("[Gsensor] alp : gsensor_set_poll buf (%s)\n", buf);
	if (KX022_DEBUG_MESSAGE)
		printk("[Gsensor] alp : last_poll_interval (%u), New_poll(%d)\n", kionix_Gsensor_data->last_poll_interval, interval);

	// Lock the device to prevent races with open/close (and itself)
	mutex_lock(&kionix_Gsensor_data->lock);
	 //Set current interval to the greater of the minimum interval or the requested interval
	kx022_update_odr(interval);
	kionix_Gsensor_data->last_poll_interval = interval;
	mutex_unlock(&kionix_Gsensor_data->lock);

	return count;
}

static ssize_t gsensor_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("[Gsensor] alp : kx022_enable_show (%d)\n", atomic_read(&kionix_Gsensor_data->enabled));
	return sprintf(buf, "%d\n", atomic_read(&kionix_Gsensor_data->enabled));
}

static ssize_t gsensor_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = simple_strtoul(buf, NULL, 10);
	switch(val)	{
		case 0:
			printk("[Gsensor] alp : kx022_disable_by_acceleation !!!\n");
			kx022_disable();
			break;
		case 1:
			printk("[Gsensor] alp : kxtj9_enable_by_acceleation !!!\n");
			kx022_enable();
			break;
		case 2:
			printk("[Gsensor] alp : kx022_disable_by_orientation !!!\n");
			kx022_device_power_off();
			break;
		case 3:
			printk("[Gsensor] alp : kx022_enable_by_orientation !!!\n");
			kx022_enable_by_orientation();
			break;
	}
	return count;
}

static ssize_t get_Gsensor_state(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int who_am_i_data = 0 , ret=0;
	who_am_i_data = i2c_smbus_read_byte_data(client, WHO_AM_I);
	printk("[Gsensor] get_Gsensor_state wia : %d\n", who_am_i_data);

	if ( (who_am_i_data == WHOAMI_VALUE_FOR_KXTJ9) || 
			(who_am_i_data == WHOAMI_VALUE_FOR_KXTJ2) || 
			(who_am_i_data == WHOAMI_VALUE_FOR_KX022) )
		ret = 1;
	else
		ret = 0;
	return sprintf(buf, "%d\n",ret);

}

static ssize_t get_rawdata(struct device *dev, struct device_attribute *devattr, char *buf)
{	
	int retval = 0, rawdata_x = 0, rawdata_y = 0, rawdata_z = 0;
	unsigned char acc_data[6];

	retval = atomic_read(&kionix_Gsensor_data->enabled);
	printk("[Gsensor] alp : get_rawdata enable state=(%d)\n",retval);
	if(retval==0){
		retval = kx022_enable();
		mdelay(100);
		if (retval < 0){
			printk("[Gsensor] ATTR power on fail\n");
			return retval;
		}
	}
	retval = kx022_i2c_read(XOUT_L, (u8 *)acc_data, 6);
	if (retval < 0){
		dev_err(&kionix_Gsensor_data->client->dev, "accelerometer data read failed\n");
		return sprintf(buf, "get no data\n");
	}

	/* ZE500KL Project */
	rawdata_x = (acc_data[2] | (acc_data[3]<<8));
	rawdata_y = (acc_data[0] | (acc_data[1]<<8));
	rawdata_z = (acc_data[4] | (acc_data[5]<<8));

	if (rawdata_x > 16383)
		rawdata_x = rawdata_x - 65536;
	if (rawdata_y > 16383)
		rawdata_y = rawdata_y - 65536;
	if (rawdata_z > 16383)
		rawdata_z = rawdata_z - 65536;

	//rawdata_x = rawdata_x*(-1);
	rawdata_y = rawdata_y*(-1);

	return sprintf(buf, "%d %d %d\n", rawdata_x, rawdata_y, rawdata_z);
}

static ssize_t read_gsensor_resolution(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data;
	data = i2c_smbus_read_byte_data(client, CTRL_REG1);
	printk("[Gsensor] alp : read_gsensor_resolution (%d)\n",data);
	return sprintf(buf, "%d\n", data);
}

static ssize_t write_gsensor_resolution(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = simple_strtoul(buf, NULL, 10);
	switch(val){
		case 0:
			printk("[Gsensor] alp : 8-bit !!!\n");
			kionix_Gsensor_data->ctrl_reg1 &=RES_8bit;
			i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG1, kionix_Gsensor_data->ctrl_reg1);
		break;
		case 1:
			printk("[Gsensor] alp : 16-bit !!!\n");
			kionix_Gsensor_data->ctrl_reg1 |=RES_16bit;
			i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG1, kionix_Gsensor_data->ctrl_reg1);
		break;

		default:
			printk("[Gsensor] alp : error input !!!\n");
		break;
	}
	return count;
}

static ssize_t read_gsensor_wufe(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data[2]={0};
	data[0] = i2c_smbus_read_byte_data(client, WAKEUP_TIMER);
	data[1] = i2c_smbus_read_byte_data(client, WAKEUP_THRES);
	printk("[Gsensor] alp : read_gsensor_wufe time(%d) , thres(%d)\n",data[0],data[1]);
	return sprintf(buf, "%d & %d\n",data[0],data[1]);
}

static ssize_t write_gsensor_wufe(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 wufe_data[2]={0,0};
	int result = 0;
	int val = simple_strtoul(buf, NULL, 10);

	// turn off power
	result = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG1, 0);
	if (result < 0){
		printk("[Gsensor] alp : motion_detect_power_off_fail !\n");
		return result;
	}
	wufe_data[0] = 0x01; 
	switch(val){
		case 0:
			wufe_data[0] = 0x00; wufe_data[1] = 0x01;
		break;

		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
			wufe_data[1] = val;
		break;

		default:
			printk("[Gsensor] alp : not support !!!\n");
		break;
	}
	kionix_Gsensor_data->wufe_timer = wufe_data[0];
	kionix_Gsensor_data->wufe_thres = wufe_data[1];

	// set the chip timer
	result = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, WAKEUP_TIMER, wufe_data[0]);
	if (result < 0){
		printk("[Gsensor] alp : ATTR set_timer_fail !\n");
		return result;
	}

	// set the chip threshold
	result = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, WAKEUP_THRES, wufe_data[1]);
	if (result < 0){
		printk("[Gsensor] alp : ATTR set_chip_threshold_fail !\n");
		return result;
	}

	// turn on power
	kionix_Gsensor_data->ctrl_reg1 |= DRDYE;
	kionix_Gsensor_data->ctrl_reg1 &= WUFE_OFF;
	result = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG1, kionix_Gsensor_data->ctrl_reg1); // enable motion detect
	if (result < 0){
		printk("[Gsensor] alp : write_gsensor_wufe into DRDY fail (%d)\n",result);
		return result;
	}
	mdelay(50); // 50 ms
	
	return count;
}

static ssize_t read_gsensor_reg2_rate(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data;
	data = i2c_smbus_read_byte_data(client, CTRL_REG2);
	printk("[Gsensor] alp : read_gsensor_reg2_rate reg2 = %d\n",data);
	return sprintf(buf, "%d\n",data);
}

static ssize_t write_gsensor_reg2_rate(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int result = 0;
	int val = simple_strtoul(buf, NULL, 10);
	unsigned char data;

	// turn off power
	result = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG1, 0);
	if (result < 0)	{
		printk("[Gsensor] alp : motion_detect_power_off_fail !\n");
		return result;
	}
	switch(val){
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
			data = val;
		break;

		default:
			printk("[Gsensor] alp : not support !!!\n");
		break;
	}
	kionix_Gsensor_data->wufe_rate = data;
	// set the chip wufe rate
	result = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG2, data);
	if (result < 0){
		printk("[Gsensor] alp : ATTR write_gsensor_reg2_rate !\n");
		return result;
	}

	// turn on power
	kionix_Gsensor_data->ctrl_reg1 |= DRDYE;
	kionix_Gsensor_data->ctrl_reg1 &= WUFE_OFF;
	result = i2c_smbus_write_byte_data(kionix_Gsensor_data->client, CTRL_REG1, kionix_Gsensor_data->ctrl_reg1); // enable motion detect
	if (result < 0){
		printk("[Gsensor] alp : write_gsensor_reg2_rate power on fail ! (%d)\n", result);
		return result;
	}
	mdelay(50); // 50 ms
	
	return count;
}

static ssize_t reset_gsensor(struct device *dev, struct device_attribute *attr, char *buf)
{
	int res=0;
	kx022_reset_function(kionix_Gsensor_data->client);
	return sprintf(buf, "%d\n", res);
}


static DEVICE_ATTR(message, 0660, gsensor_show_message, gsensor_set_message);
static DEVICE_ATTR(delay, 0660, gsensor_get_poll, gsensor_set_poll);
static DEVICE_ATTR(enable, 0660,gsensor_enable_show,gsensor_enable_store);
static DEVICE_ATTR(rawdata, S_IRUGO, get_rawdata, NULL);
static DEVICE_ATTR(state, S_IRUGO, get_Gsensor_state, NULL);
static DEVICE_ATTR(resolution, 0660, read_gsensor_resolution, write_gsensor_resolution);
static DEVICE_ATTR(wufe, 0660, read_gsensor_wufe, write_gsensor_wufe);
static DEVICE_ATTR(reg2_rate, 0660, read_gsensor_reg2_rate, write_gsensor_reg2_rate);
static DEVICE_ATTR(reset, 0660, reset_gsensor, NULL);

static DEVICE_ATTR(Gsensor_chip_id, 0664, gsensor_chip_id_show, NULL);
static DEVICE_ATTR(Gsensor_status, 0664, gsensor_status_show, NULL);
static DEVICE_ATTR(Gsensor_raw, 0664, gsensor_read_raw, NULL);
static DEVICE_ATTR(Gsensor_dump_reg, 0440, gsensor_dump_reg, NULL);
static DEVICE_ATTR(Gsensor_en_mt, 0660, kxtj9_r_en_mt, kxtj9_w_en_mt);

static struct attribute *kx022_attributes[] = {
	&dev_attr_message.attr,
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_rawdata.attr,
	&dev_attr_state.attr,
	&dev_attr_resolution.attr,
	&dev_attr_wufe.attr,
	&dev_attr_reg2_rate.attr,
	&dev_attr_reset.attr,

	&dev_attr_Gsensor_chip_id.attr,
	&dev_attr_Gsensor_status.attr,
	&dev_attr_Gsensor_raw.attr,
	&dev_attr_Gsensor_dump_reg.attr,
	&dev_attr_Gsensor_en_mt.attr,
	NULL
};

static struct attribute_group kx022_attribute_group = {
	.attrs = kx022_attributes
};


#ifdef CONFIG_INPUT_KXTJ9_POLLED_MODE
static void kx022_poll(struct input_polled_dev *dev)
{
	unsigned int poll_interval = dev->poll_interval;

	kx022_report_acceleration_data();

	if (poll_interval != kionix_Gsensor_data->last_poll_interval) {
		kx022_update_odr(poll_interval);
		kionix_Gsensor_data->last_poll_interval = poll_interval;
	}
	if(KX022_DEBUG_MESSAGE) 
		printk("[Gsensor] alp : kx022_poll poll_interval(%d)\n",kionix_Gsensor_data->last_poll_interval);
}

static void kx022_polled_input_open(struct input_polled_dev *dev)
{
	kx022_enable();
}

static void kx022_polled_input_close(struct input_polled_dev *dev)
{
	kx022_disable();
}

static int kx022_setup_polled_device(void)
{
	int err;
	struct input_polled_dev *poll_dev;
	poll_dev = input_allocate_polled_device();

	if (!poll_dev) {
		dev_err(&kionix_Gsensor_data->client->dev,
			"Failed to allocate polled device\n");
		return -ENOMEM;
	}

	kionix_Gsensor_data->poll_dev = poll_dev;
	kionix_Gsensor_data->input_dev = poll_dev->input;

	poll_dev->private = kionix_Gsensor_data;
	poll_dev->poll = kx022_poll;
	poll_dev->open = kx022_polled_input_open;
	poll_dev->close = kx022_polled_input_close;

	kx022_setup_input_device();

	err = input_register_polled_device(poll_dev);
	if (err) {
		dev_err(&kionix_Gsensor_data->client->dev,
			"Unable to register polled device, err=%d\n", err);
		input_free_polled_device(poll_dev);
		return err;
	}

	return 0;
}

static void __devexit kx022_teardown_polled_device(void)
{
	input_unregister_polled_device(kionix_Gsensor_data->poll_dev);
	input_free_polled_device(kionix_Gsensor_data->poll_dev);
}

#else

static inline int kx022_setup_polled_device(void)
{
	return -ENOSYS;
}

static inline void kx022_teardown_polled_device(void)
{
}

#endif

static int KIONIX_HW_vertify(void)
{
	int retval=0;
	if(KX022_DEBUG_MESSAGE) printk("[Gsensor] KIONIX_HW_vertify ++\n");

	retval = i2c_smbus_read_byte_data(kionix_Gsensor_data->client, WHO_AM_I);
	printk("[Gsensor] KIONIX_HW_vertify ret = %d\n",retval);
	if (retval < 0) {
		printk("[Gsensor] read fail ret = %d\n", retval);
		dev_err(&kionix_Gsensor_data->client->dev, "read err int source\n");
		goto out;
	}
	if(KX022_DEBUG_MESSAGE) printk("[Gsensor] alp : read retval = %d\n", retval);

	if(retval == WHOAMI_VALUE_FOR_KX022)
		printk("[Gsensor] Use kx022!!\n");
	else if(retval == WHOAMI_VALUE_FOR_KXTJ2)
		printk("[Gsensor] Use kxtj2!!\n");
	else if(retval == WHOAMI_VALUE_FOR_KXTJ9)
		printk("[Gsensor] Use kxtj9!!\n");
	else
		printk("[Gsensor] Use other!!\n");
out:
	//kx022_device_power_off();
	if(KX022_DEBUG_MESSAGE) printk("[Gsensor] alp : KIONIX_HW_vertify --\n");
	return retval;
}

/*
static struct file_operations kx022_fops = {
	.owner			= 	THIS_MODULE,
//	.poll 			= 	kx022_poll,
//	.read 			= 	kxtj9_read,
	.unlocked_ioctl	=	kxtj9_ioctl,
//	.compat_ioctl		=	kxtj9_ioctl,
	.open			=	kxtj9_open,
	.release			=	kxtj9_release,
};
*/

static struct miscdevice kx022_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "kx022_dev",
//	.fops = &kx022_fops,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void kx022_early_suspend(struct early_suspend *h)
{
	int status=0;

	mutex_lock(&kionix_Gsensor_data->lock);
	kionix_Gsensor_data->suspend_resume_state = 1;

	status = 	kionix_Gsensor_data->ctrl_reg1 >> 7;
	printk("[Gsensor] alp : kx022_early_suspend enable(%d)\n", status);
	if(status == KX022_RESUME_ENABLE){
		printk("[Gsensor] alp : kxtj, need to enable after resume!\n");
		kionix_Gsensor_data->resume_enable = KX022_RESUME_ENABLE;
	} else	{
		kionix_Gsensor_data->resume_enable = KX022_RESUME_DISABLE;
	}

	kx022_disable();
	mutex_unlock(&kionix_Gsensor_data->lock);
	printk("[Gsensor] alp : kx022_early_suspend irq(%d)\n", kionix_Gsensor_data->irq);
}

static void kx022_late_resume(struct early_suspend *h)
{
	mutex_lock(&kionix_Gsensor_data->lock);
	kionix_Gsensor_data->suspend_resume_state = 0;
	if ( (kionix_Gsensor_data->resume_enable == KX022_RESUME_ENABLE) ||
			(kionix_Gsensor_data->resume_enable==KX022_RESUME_MISSENABLE) )	{
		kx022_enable();
		printk("[Gsensor] alp : kx022_late_resume enable\n");
	}else
		printk("[Gsensor] alp : kx022_late_resume pass enable\n");
	mutex_unlock(&kionix_Gsensor_data->lock);
	printk("[Gsensor] alp : kx022_late_resume irq(%d)\n", kionix_Gsensor_data->irq);
}
#endif

static int kx022_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	int gpio=0, iloop=0;
	int err = 0;
	printk("[Gsensor] Probe KX022 Gsensor i2c driver\n");

	/* Setting i2c client and device_data */
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	kionix_Gsensor_data = kzalloc(sizeof(struct ASUS_Gsensor_data), GFP_KERNEL);
	if (!kionix_Gsensor_data) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}
	memset(kionix_Gsensor_data, 0, sizeof(struct ASUS_Gsensor_data));
	kionix_Gsensor_data->client = client;
	i2c_set_clientdata(client, kionix_Gsensor_data);
	mutex_init(&kionix_Gsensor_data->lock);

	/* Init calibration data */
	for(iloop=0;iloop<6;iloop++){
		kionix_Gsensor_data->accel_cal_data[iloop]=0;
	}
	for(iloop=0;iloop<3;iloop++){
		kionix_Gsensor_data->accel_cal_offset[iloop]=0;
		kionix_Gsensor_data->accel_cal_sensitivity[iloop]=1024;
	}

	kionix_Gsensor_data->power_enabled = false;

	err = kx022_power_init(true);
	if (err < 0) {
		dev_err(&kionix_Gsensor_data->client->dev, "power init failed! err=%d", err);
		goto err_pdata_exit;
	}

	err = kx022_device_power_on();
	if (err < 0) {
		dev_err(&client->dev, "power on failed! err=%d\n", err);
		goto err_power_deinit;
	}


	/* Setting input event & device */
	err = kx022_setup_input_device();
	if (err){
		printk("[Gsensor] Unable to request gsensor input device\n");
		goto err_power_off;
	}
	/* Setting gpio & irq */
	gpio = of_get_named_gpio(np, "kionix,interrupt-gpio", 0);
	
	printk("[Gsensor] Get Gsensor interrupt : %d\n", gpio);
	err = gpio_request(gpio,"accel_kx022_interrupt");
	if (err)
		printk("[Gsensor] Unable to request gpio %d (kx022-irq)\n", gpio);
	err = gpio_direction_input(gpio);
	if (err)
		printk("[Gsensor] Unable to set the direction of gpio %d (kx022-irq)\n", gpio);
	
	kionix_Gsensor_data->irq = gpio_to_irq(gpio);
	printk("[Gsensor] Setting Final irq=%d\n", kionix_Gsensor_data->irq);


	err = KIONIX_HW_vertify();
	if (err == 0)
		Gsensor_status = 1;

	if (err < 0) {
		dev_err(&client->dev, "[Gsensor] device not recognized\n");
		goto err_destroy_input;
	}
	kionix_Gsensor_data->ctrl_reg1 |= (RES_16bit | GRP4_G_4G);
	kionix_Gsensor_data->last_poll_interval = 0;
	printk("[Gsensor] Setting init reg1 = 0x%X \n",kionix_Gsensor_data->ctrl_reg1);

	if (kionix_Gsensor_data->irq) {
		/* If in irq mode, populate INT_CTRL_REG1 and enable DRDY. */
		kionix_Gsensor_data->int_ctrl |= (KX022_IEN | KX022_IEA);

		kionix_Gsensor_data->ctrl_reg1 |= DRDYE;

		err = request_threaded_irq(kionix_Gsensor_data->irq, NULL,
					kx022_isr,IRQF_TRIGGER_RISING | IRQF_ONESHOT, "kx022-irq", kionix_Gsensor_data);
		if (err < 0)
			printk("[Gsensor] Gsensor request_irq() error %d.\n", err);
		else		{
			printk("[Gsensor] Gsensor request_irq ok.(0x%x)\n", kionix_Gsensor_data->int_ctrl);
			/* Clean interrupt bit */
			i2c_smbus_read_byte_data(kionix_Gsensor_data->client, INT_REL);
			disable_irq(kionix_Gsensor_data->irq);
			kionix_Gsensor_data->irq_status = 0;
		}
		if (err){
			dev_err(&client->dev, "request irq failed: %d\n", err);
			goto err_destroy_input;
		}
		/* Setting system file */
		err = sysfs_create_group(&client->dev.kobj, &kx022_attribute_group);
		if (err) {
			dev_err(&client->dev, "sysfs create failed: %d\n", err);	
			goto err_free_irq;
		}
		err = misc_register(&kx022_device);
		if (err) {
			printk("[Gsensor] alp :  kx022_misc_register failed\n");
			goto err_free_irq;
		}
	} else {
		err = kx022_setup_polled_device();
		goto err_destroy_input;
	}
	printk("[Gsensor] Final reg1 = 0x%X \n", kionix_Gsensor_data->ctrl_reg1);
	//atomic_set(&kionix_Gsensor_data->enabled, 0);
	kx022_disable();

	g_ilocation = KX022_CHIP_LOCATION_SR_ZC500KL;
//<asus-annacheng20150129>>>>>>>>>>>>>>>>>>>>>>>+
	if(create_asusproc_kx022sensor_status_entry())
		printk("[%s] : ERROR to create kx022sensor proc entry\n",__func__);
//<asus-annacheng20150129><<<<<<<<<<<<<<<<<+
	
// for enable motion detect , added by cheng_kao 2014.02.12 ++
	if(build_version==1){	// 1:eng ; 2:user ; 3:userdebug 
		bypass_for_eng_mode = true;
		printk("[Gsensor] alp.D. : kx022 G-sensor for eng mode !!\n");
	}
	kionix_Gsensor_data->motion_detect_threshold_x=0;
	kionix_Gsensor_data->motion_detect_threshold_y=0;
	kionix_Gsensor_data->motion_detect_threshold_z=0;
	kionix_Gsensor_data->motion_detect_timer=-1;	// init counter
	kionix_Gsensor_data->chip_interrupt_mode=INT_MODE_DRDY;
	kionix_Gsensor_data->wufe_rate=WUFE25F;		// 	rate
	kionix_Gsensor_data->wufe_timer=0x01;			//	1 / rate = delay (sec)
	kionix_Gsensor_data->wufe_thres=0x01;			//	counts 16 = 1g
// for enable motion detect , added by cheng_kao 2014.02.12 --

#ifdef CONFIG_HAS_EARLYSUSPEND
	kionix_Gsensor_data->gsensor_early_suspendresume.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;
	kionix_Gsensor_data->gsensor_early_suspendresume.suspend = kx022_early_suspend;
	kionix_Gsensor_data->gsensor_early_suspendresume.resume = kx022_late_resume;
	register_early_suspend(&kionix_Gsensor_data->gsensor_early_suspendresume);
#endif

	kionix_Gsensor_data->suspend_resume_state = 0;
	kionix_Gsensor_data->resume_enable = 0;

	printk("[Gsensor] kx022_probe (%d) --\n", g_ilocation);

	return err;

err_free_irq:
	free_irq(kionix_Gsensor_data->irq, NULL);
err_destroy_input:
	input_unregister_device(kionix_Gsensor_data->input_dev);
err_power_off:
	kx022_device_power_off();
err_power_deinit:
	kx022_power_init(false);
err_pdata_exit:
	kfree(kionix_Gsensor_data);
	dev_err(&client->dev, "%s: kxtj9_probe err=%d\n", __func__, err);
	return err;

}

static int kx022_remove(struct i2c_client *client)
{
	if (kionix_Gsensor_data->irq) {
		sysfs_remove_group(&client->dev.kobj, &kx022_attribute_group);
		free_irq(kionix_Gsensor_data->irq, NULL);
		input_unregister_device(kionix_Gsensor_data->input_dev);
	} else {
		kx022_teardown_polled_device();
	}

	kx022_device_power_off();
	kx022_power_init(false);

	kfree(kionix_Gsensor_data);

	return 0;
}

//#ifdef CONFIG_PM_SLEEP
static int kx022_suspend(struct device *dev)
{

	int status=0;

	//if (KX022_DEBUG_MESSAGE)
		printk("[Gsensor] alp : kx022_suspend\n");

	mutex_lock(&kionix_Gsensor_data->lock);
	kionix_Gsensor_data->suspend_resume_state = 1;

	status = 	kionix_Gsensor_data->ctrl_reg1 >> 7;
	printk("[Gsensor] alp : kx022_suspend enable(%d)\n", status);
	if(status == KX022_RESUME_ENABLE){
		printk("[Gsensor] alp : kxtj, need to enable after resume!\n");
		kionix_Gsensor_data->resume_enable = KX022_RESUME_ENABLE;
	} else	{
		kionix_Gsensor_data->resume_enable = KX022_RESUME_DISABLE;
	}

	kx022_disable();
	mutex_unlock(&kionix_Gsensor_data->lock);
	printk("[Gsensor] alp : kx022_early_suspend irq(%d)\n", kionix_Gsensor_data->irq);

	return 0;
}

static int kx022_resume(struct device *dev)
{
	//if (KX022_DEBUG_MESSAGE)
		printk("[Gsensor] alp : kx022_resume\n");


	mutex_lock(&kionix_Gsensor_data->lock);
	kionix_Gsensor_data->suspend_resume_state = 0;
	if ( (kionix_Gsensor_data->resume_enable == KX022_RESUME_ENABLE) ||
			(kionix_Gsensor_data->resume_enable==KX022_RESUME_MISSENABLE) )	{
		kx022_enable();
		printk("[Gsensor] alp : kx022_resume enable\n");
	}else
		printk("[Gsensor] alp : kx022_resume pass enable\n");
	mutex_unlock(&kionix_Gsensor_data->lock);
	printk("[Gsensor] alp : kx022_late_resume irq(%d)\n", kionix_Gsensor_data->irq);

	return 0;
}
//#endif

static SIMPLE_DEV_PM_OPS(kx022_pm_ops, kx022_suspend, kx022_resume);

static struct of_device_id kionix_gsensor_match_table[] = {
	{ .compatible = "kionix, kx022",},
	{ },
};

static const struct i2c_device_id kx022_id[] = {
	{ ASUS_GSENSOR_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, kx022_id);

static struct i2c_driver kx022_driver = {
	.driver = {
		.name			= ASUS_GSENSOR_NAME,
		.owner			= THIS_MODULE,
		.of_match_table	= kionix_gsensor_match_table,
		.pm				= &kx022_pm_ops,
	},
	.probe	= kx022_probe,
	.remove	= kx022_remove,
	.id_table	= kx022_id,
};

static int __init kx022_init(void)
{
	int res = 0;
	pr_info("[Gsensor] Gsensor driver: initialize.\n");
	res = i2c_add_driver(&kx022_driver);
	if (res != 0)
		printk("[Gsensor] I2c_add_driver fail, Error : %d\n", res);
	return res;
}
module_init(kx022_init);

static void __exit kx022_exit(void)
{
	i2c_del_driver(&kx022_driver);
	//<asus-annacheng20150129>>>>>>>>>>>>>>>>>>+
	if(kx022sensor_entry)
		remove_proc_entry("gsensor_status", NULL);
	//<asus-annacheng20150129><<<<<<<<<<<<<<<<<+
}
module_exit(kx022_exit);

MODULE_DESCRIPTION("KX022 accelerometer driver");
MODULE_LICENSE("GPL");
