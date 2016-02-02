/* drivers/misc/akm09911.c - akm09911 compass driver
 *
 * Copyright (C) 2007-2008 HTC Corporation.
 * Author: Hou-Kun Chen <houkun.chen@gmail.com>
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

/*#define DEBUG*/
/*#define VERBOSE_DEBUG*/

#include <linux/input/akm09911.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>

#define AKM_DEBUG_IF			1
#define AKM_HAS_RESET			1
#define AKM_INPUT_DEVICE_NAME	"compass"
#define AKM_DRDY_TIMEOUT_MS		100
#define AKM_BASE_NUM			10
#define AKM_DEBUG_MESSAGE		1
#define AKM_ATTR_ATD_TEST		1
//<ASUS-annacheng20150129>>>>>>>>>>>>
#define Driverversion  "1.0.0"  
#define VENDOR  "AK09911"    
struct proc_dir_entry *compasssensor_entry=NULL;
//<ASUS-annacheng20150129><<<<<<<<<<<<+
/* POWER SUPPLY VOLTAGE RANGE */
#define AKM09911_VDD_MIN_UV	2000000
#define AKM09911_VDD_MAX_UV	3300000
#define AKM09911_VIO_MIN_UV	1750000
#define AKM09911_VIO_MAX_UV	1950000
struct akm_sensor_state {
	bool power_on;
	uint8_t mode;
};
struct akm_compass_data {
	struct i2c_client	*i2c;
	struct input_dev	*input;
	struct device		*class_dev;
	struct class		*compass;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pin_default;
	struct pinctrl_state	*pin_sleep;
	struct mutex		op_mutex;

	wait_queue_head_t	drdy_wq;
	wait_queue_head_t	open_wq;

	/* These two buffers are initialized at start up.
	   After that, the value is not changed */
	uint8_t sense_info[AKM_SENSOR_INFO_SIZE];
	uint8_t sense_conf[AKM_SENSOR_CONF_SIZE];

	struct	mutex sensor_mutex;
	uint8_t	sense_data[AKM_SENSOR_DATA_SIZE];
	/* Positive value means the device is working.
	   0 or negative value means the device is not woking,
	   i.e. in power-down mode. */
	int8_t	is_busy;

	struct mutex	val_mutex;
	uint32_t		enable_flag;
	int64_t			delay;

	atomic_t	active;
	atomic_t	drdy;

	char layout;
	int	irq;
	int	gpio_rstn;

	struct	regulator		*vdd;
	struct	regulator		*vio;
	bool	power_enabled;
	struct	akm_sensor_state		state;
	/* ASUS BSP Peter_Lu for E-compass Debug & ATD tool */
	bool compass_debug_switch;
};

static struct akm_compass_data *s_akm;

static int akm_compass_power_set(struct akm_compass_data *data, bool on);

/***** I2C I/O function ***********************************************/
static int akm_i2c_rxdata(
	struct i2c_client *i2c,
	uint8_t *rxData,
	int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};
	uint8_t addr = rxData[0];

	ret = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.\n", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&i2c->dev, "%s: transfer failed(size error).\n",
				__func__);
		return -ENXIO;
	}

	dev_vdbg(&i2c->dev, "RxData: len=%02x, addr=%02x, data=%02x",
		length, addr, rxData[0]);

	return 0;
}

static int akm_i2c_txdata(
	struct i2c_client *i2c,
	uint8_t *txData,
	int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msg)) {
		dev_err(&i2c->dev, "%s: transfer failed(size error).",
				__func__);
		return -ENXIO;
	}

	dev_vdbg(&i2c->dev, "TxData: len=%02x, addr=%02x data=%02x",
		length, txData[0], txData[1]);

	return 0;
}


//<asus-annacheng20150129>>>>>>>>>>>>>>>>>>>>>>>+
static int Compasssensor_proc_show(struct seq_file *m, void *v) {
	
		int ret = 0,err=0,i=0;
		uint8_t buffer[2];
		uint8_t idReg[2];
		struct akm_compass_data  *akm =s_akm;
				
		
		for (i=0;i<2;i++){
			buffer[0]=0x00+i;
			err = akm_i2c_rxdata(akm->i2c, buffer, 1);
			idReg[i]=buffer[0];
			printk(KERN_INFO "[AKM] i2c reg value %d=0x%x\n",i,buffer[0]);
			
			
		}
		if (err < 0)
			printk(KERN_INFO "[AKM] i2c err\n");
			
			
		
		//struct CM36283_info *lpi = lp_info_cm36283;   
		//ret1 = _CM36283_I2C_Read_Word(lpi->slave_addr, ID_REG, &idReg);
		printk("anna-sensor idReg : 0x00=0x%x \n", buffer[0]);
	      
	  	if(err<0){
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
	  	if((idReg[0]==0x48)&&(idReg[1]==0X05)){//vendorid  ,deviceid
	  		ret =seq_printf(m," Vendor:%s(0x0%x%x)\n", VENDOR,idReg[1],idReg[0]);
		}else{
			ret =seq_printf(m," Vendor:%s(0x0%x%x)\n", VENDOR,idReg[1],idReg[0]);
		}

		buffer[0]=0x00;
		err = akm_i2c_rxdata(akm->i2c, buffer, 1);
		printk("anna-sensor idReg33 : 0x00=0x%x \n", buffer[0]);
		if(err<0){	  		
			ret =seq_printf(m," Device status:error\n");			
		}else{
			ret =seq_printf(m," Device status:ok\n");			
		}
	     	    
   	return ret;

}


static int Compasssensor_proc_open(struct inode *inode, struct  file *file) {
  return single_open(file, Compasssensor_proc_show, NULL);
}

static const struct file_operations compasssensor_proc_fops = {
  .owner = THIS_MODULE,
  .open = Compasssensor_proc_open,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

int create_asusproc_compasssensor_status_entry(void){
	compasssensor_entry = proc_create("ecompass_status", S_IWUGO| S_IRUGO, NULL,&compasssensor_proc_fops);
 	if (!compasssensor_entry)
       		 return -ENOMEM;

    	return 0;
}

//<asus-annacheng20150129><<<<<<<<<<<<<<<<<+

/***** akm miscdevice functions *************************************/
static int AKECS_Set_CNTL(
	struct akm_compass_data *akm,
	uint8_t mode)
{
	uint8_t buffer[2];
	int err;

	/***** lock *****/
	mutex_lock(&akm->sensor_mutex);
	/* Busy check */
	if (akm->is_busy > 0) {
		dev_err(&akm->i2c->dev,
				"%s: device is busy.\n", __func__);
		err = -EBUSY;
	} else {
		/* Set measure mode */
		buffer[0] = AKM_REG_MODE;
		buffer[1] = mode;
		err = akm_i2c_txdata(akm->i2c, buffer, 2);
		if (err < 0) {
			dev_err(&akm->i2c->dev,
					"%s: Can not set CNTL.", __func__);
		} else {
			dev_vdbg(&akm->i2c->dev,
					"Mode is set to (%d).", mode);
			/* Set flag */
			akm->is_busy = 1;
			atomic_set(&akm->drdy, 0);
			/* wait at least 100us after changing mode */
			udelay(100);
		}
		akm->is_busy = 0;
	}

	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

	return err;
}

static int AKECS_Set_PowerDown(
	struct akm_compass_data *akm)
{
	uint8_t buffer[2];
	int err;

	/***** lock *****/
	mutex_lock(&akm->sensor_mutex);

	/* Set powerdown mode */
	buffer[0] = AKM_REG_MODE;
	buffer[1] = AKM_MODE_POWERDOWN;
	err = akm_i2c_txdata(akm->i2c, buffer, 2);
	if (err < 0) {
		dev_err(&akm->i2c->dev,
			"%s: Can not set to powerdown mode.", __func__);
	} else {
		dev_dbg(&akm->i2c->dev, "Powerdown mode is set.");
		/* wait at least 100us after changing mode */
		udelay(100);
	}
	/* Clear status */
	akm->is_busy = 0;
	atomic_set(&akm->drdy, 0);

	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

	return err;
}

static int AKECS_Reset(
	struct akm_compass_data *akm,
	int hard)
{
	int err;

#if AKM_HAS_RESET
	uint8_t buffer[2];

	/***** lock *****/
	if (s_akm->compass_debug_switch)
		printk("[E-compass] AKECS_Reset +\n");
	mutex_lock(&akm->sensor_mutex);
	if (hard != 0) {
		gpio_set_value(akm->gpio_rstn, 0);
		udelay(5);
		gpio_set_value(akm->gpio_rstn, 1);
		// No error is returned
		err = 0;
	} else {
		buffer[0] = AKM_REG_RESET;
		buffer[1] = AKM_RESET_DATA;
		err = akm_i2c_txdata(akm->i2c, buffer, 2);
		if (err < 0) {
			dev_err(&akm->i2c->dev,
				"%s: Can not set SRST bit.", __func__);
		} else {
			dev_dbg(&akm->i2c->dev, "Soft reset is done.");
		}
	}
	/* Device will be accessible 100 us after */
	udelay(100);
	/* Clear status */
	akm->is_busy = 0;
	atomic_set(&akm->drdy, 0);

	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/
	if (s_akm->compass_debug_switch)
		printk("[E-compass] AKECS_Reset -\n");

#else
	err = AKECS_Set_PowerDown(akm);
#endif

	return err;
}

static int AKECS_SetMode(
	struct akm_compass_data *akm,
	uint8_t mode)
{
	int err;

	if (s_akm->compass_debug_switch)	{
		switch (mode & 0x1F) {
		case AKM_MODE_SNG_MEASURE:
			printk("[E-compass] AKM Set SNG_MEASURE function mode(%d) \n", mode);
			break;
		case AKM_MODE_SELF_TEST:
			printk("[E-compass] AKM Set SELF_TEST function mode(%d) \n", mode);
			break;
		case AKM_MODE_FUSE_ACCESS:
			printk("[E-compass] AKM Set FUSE_ACCESS function mode(%d) \n", mode);
			break;
		case AKM_MODE_POWERDOWN:
			printk("[E-compass] AKM Power down(%d) \n", mode);
			break;
		}
	}

	switch (mode & 0x1F) {
	case AKM_MODE_SNG_MEASURE:
	case AKM_MODE_SELF_TEST:
	case AKM_MODE_FUSE_ACCESS:
		err = AKECS_Set_CNTL(akm, mode);
		break;
	case AKM_MODE_POWERDOWN:
		err = AKECS_Set_PowerDown(akm);
		break;
	default:
		dev_err(&akm->i2c->dev,
			"%s: Unknown mode(%d).", __func__, mode);
		return -EINVAL;
	}
	akm->state.mode = mode;
	return err;
}

static void AKECS_SetYPR(
	struct akm_compass_data *akm,
	int *rbuf)
{
	if (s_akm->compass_debug_switch)
		printk("[E-compass] alp : AKECS_SetYPR +\n");
	dev_vdbg(&akm->i2c->dev, "%s: flag =0x%X", __func__, rbuf[0]);
	dev_vdbg(&akm->input->dev, "  Acc [LSB]   : %6d,%6d,%6d stat=%d",
		rbuf[1], rbuf[2], rbuf[3], rbuf[4]);

	/* No events are reported */
	if (!rbuf[0]) {
		dev_dbg(&akm->i2c->dev, "Don't waste a time.");
		return;
	}

	/* Report magnetic vector information */
	/* Before copy values to buffer, check AKM_YPR_DATA_SIZE */
	input_report_abs(akm->input, ABS_RX, rbuf[1]);
	input_report_abs(akm->input, ABS_RY, rbuf[2]);
	input_report_abs(akm->input, ABS_RZ, rbuf[3]);
	input_report_abs(akm->input, ABS_RUDDER, rbuf[4]);

	input_sync(akm->input);
	if (s_akm->compass_debug_switch)
		printk("[E-compass] alp : AKECS_SetYPR -\n");
}

/* This function will block a process until the latest measurement
 * data is available.
 */
static int AKECS_GetData(
	struct akm_compass_data *akm,
	uint8_t *rbuf,
	int size)
{
	int err;

	/* Block! */
	if (s_akm->compass_debug_switch)
		printk("[E-compass] alp : AKECS_GetData +\n");
	err = wait_event_interruptible_timeout(
			akm->drdy_wq,
			atomic_read(&akm->drdy),
			msecs_to_jiffies(AKM_DRDY_TIMEOUT_MS));

	if (err < 0) {
		dev_err(&akm->i2c->dev,
			"[E-compass] %s: wait_event failed (%d).", __func__, err);
		return err;
	}
	if (!atomic_read(&akm->drdy)) {
		dev_err(&akm->i2c->dev,
			"[E-compass] %s: DRDY is not set.\n", __func__);
		return -ENODATA;
	}

	/***** lock *****/
	mutex_lock(&akm->sensor_mutex);

	memcpy(rbuf, akm->sense_data, size);
	atomic_set(&akm->drdy, 0);

	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

	if (s_akm->compass_debug_switch)
		printk("[E-compass] alp : AKECS_GetData -\n");
	return 0;
}

static int AKECS_GetData_Poll(
	struct akm_compass_data *akm,
	uint8_t *rbuf,
	int size)
{
	uint8_t buffer[AKM_SENSOR_DATA_SIZE];
	int err;

	if (s_akm->compass_debug_switch)
		printk("[E-compass] alp : AKECS_GetData_Poll +\n");
	/* Read status */
	buffer[0] = AKM_REG_STATUS;
	err = akm_i2c_rxdata(akm->i2c, buffer, 1);
	if (err < 0) {
		dev_err(&akm->i2c->dev, "%s failed.", __func__);
		return err;
	}

	/* Check ST bit */
	if (!(AKM_DRDY_IS_HIGH(buffer[0])))
		return -EAGAIN;

	/* Read rest data */
	buffer[1] = AKM_REG_STATUS + 1;
	err = akm_i2c_rxdata(akm->i2c, &(buffer[1]), AKM_SENSOR_DATA_SIZE-1);
	if (err < 0) {
		dev_err(&akm->i2c->dev, "%s failed.", __func__);
		return err;
	}

	memcpy(rbuf, buffer, size);
	atomic_set(&akm->drdy, 0);

	/***** lock *****/
	mutex_lock(&akm->sensor_mutex);
	akm->is_busy = 0;
	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

	return 0;
}

static int AKECS_GetOpenStatus(
	struct akm_compass_data *akm)
{
	return wait_event_interruptible(
			akm->open_wq, (atomic_read(&akm->active) > 0));
}

static int AKECS_GetCloseStatus(
	struct akm_compass_data *akm)
{
	return wait_event_interruptible(
			akm->open_wq, (atomic_read(&akm->active) <= 0));
}

static int AKECS_Open(struct inode *inode, struct file *file)
{

	printk("zxtest AKECS_Open\n");
	file->private_data = s_akm;
	return nonseekable_open(inode, file);
}

static int AKECS_Release(struct inode *inode, struct file *file)
{
	printk("zxtest AKECS_Release\n");
	return 0;
}

static long
AKECS_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
//	struct akm_compass_data *akm = file->private_data;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	uint8_t i2c_buf[AKM_RWBUF_SIZE];		/* for READ/WRITE */
	uint8_t dat_buf[AKM_SENSOR_DATA_SIZE];/* for GET_DATA */
	int32_t ypr_buf[AKM_YPR_DATA_SIZE];		/* for SET_YPR */
	int64_t delay;		/* for GET_DELAY */
	uint8_t mode;		/* for SET_MODE*/
	int status;			/* for OPEN/CLOSE_STATUS */
	int ret = 0;		/* Return value. */

	if (s_akm->compass_debug_switch)
		printk("[E-compass] alp : AKECS_ioctl : (0x%4X) +\n", cmd & 0xFF);

	switch (cmd) {
	case ECS_IOCTL_READ:
	case ECS_IOCTL_WRITE:
		if (argp == NULL) {
			dev_err(&s_akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&i2c_buf, argp, sizeof(i2c_buf))) {
			dev_err(&s_akm->i2c->dev, "copy_from_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_MODE:
		if (argp == NULL) {
			dev_err(&s_akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&mode, argp, sizeof(mode))) {
			dev_err(&s_akm->i2c->dev, "copy_from_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_YPR:
		if (argp == NULL) {
			dev_err(&s_akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&ypr_buf, argp, sizeof(ypr_buf))) {
			dev_err(&s_akm->i2c->dev, "copy_from_user failed.");
			return -EFAULT;
		}
	case ECS_IOCTL_GET_INFO:
	case ECS_IOCTL_GET_CONF:
	case ECS_IOCTL_GET_DATA:
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
	case ECS_IOCTL_GET_DELAY:
	case ECS_IOCTL_GET_LAYOUT:
		/* Check buffer pointer for writing a data later. */
		if (argp == NULL) {
			dev_err(&s_akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		dev_vdbg(&s_akm->i2c->dev, "IOCTL_READ called.");
		if ((i2c_buf[0] < 1) || (i2c_buf[0] > (AKM_RWBUF_SIZE-1))) {
			dev_err(&s_akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		ret = akm_i2c_rxdata(s_akm->i2c, &i2c_buf[1], i2c_buf[0]);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_WRITE:
		dev_vdbg(&s_akm->i2c->dev, "IOCTL_WRITE called.");
		if ((i2c_buf[0] < 2) || (i2c_buf[0] > (AKM_RWBUF_SIZE-1))) {
			dev_err(&s_akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		ret = akm_i2c_txdata(s_akm->i2c, &i2c_buf[1], i2c_buf[0]);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_RESET:
		dev_vdbg(&s_akm->i2c->dev, "IOCTL_RESET called.");
		ret = AKECS_Reset(s_akm, s_akm->gpio_rstn);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_SET_MODE:
		dev_vdbg(&s_akm->i2c->dev, "IOCTL_SET_MODE called.");
		ret = AKECS_SetMode(s_akm, mode);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_SET_YPR:
		dev_vdbg(&s_akm->i2c->dev, "IOCTL_SET_YPR called.");
		AKECS_SetYPR(s_akm, ypr_buf);
		break;
	case ECS_IOCTL_GET_DATA:
		dev_vdbg(&s_akm->i2c->dev, "IOCTL_GET_DATA called.");
		if (s_akm->irq)
			ret = AKECS_GetData(s_akm, dat_buf, AKM_SENSOR_DATA_SIZE);
		else
			ret = AKECS_GetData_Poll(
					s_akm, dat_buf, AKM_SENSOR_DATA_SIZE);

		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
		dev_vdbg(&s_akm->i2c->dev, "IOCTL_GET_OPEN_STATUS called.");
		ret = AKECS_GetOpenStatus(s_akm);
		if (ret < 0) {
			printk(KERN_ERR "[E-compass] open status stop ret(%d).\n", ret);
			return ret;
		}
		break;
	case ECS_IOCTL_GET_CLOSE_STATUS:
		dev_vdbg(&s_akm->i2c->dev, "IOCTL_GET_CLOSE_STATUS called.");
		ret = AKECS_GetCloseStatus(s_akm);
		if (ret < 0) {
			printk(KERN_ERR "[E-compass] close status stop ret(%d).\n", ret);
			return ret;
		}
		break;
	case ECS_IOCTL_GET_DELAY:
		dev_vdbg(&s_akm->i2c->dev, "IOCTL_GET_DELAY called.");
		mutex_lock(&s_akm->val_mutex);
		delay = (s_akm->enable_flag ? s_akm->delay : -1);
		mutex_unlock(&s_akm->val_mutex);
		break;
	case ECS_IOCTL_GET_INFO:
		dev_vdbg(&s_akm->i2c->dev, "IOCTL_GET_INFO called.");
		break;
	case ECS_IOCTL_GET_CONF:
		dev_vdbg(&s_akm->i2c->dev, "IOCTL_GET_CONF called.");
		break;
	case ECS_IOCTL_GET_LAYOUT:
		dev_vdbg(&s_akm->i2c->dev, "IOCTL_GET_LAYOUT called.");
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		/* +1  is for the first byte */
		if (copy_to_user(argp, &i2c_buf, i2c_buf[0]+1)) {
			dev_err(&s_akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_INFO:
		if (copy_to_user(argp, &s_akm->sense_info,
					sizeof(s_akm->sense_info))) {
			dev_err(&s_akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_CONF:
		if (copy_to_user(argp, &s_akm->sense_conf,
					sizeof(s_akm->sense_conf))) {
			dev_err(&s_akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_DATA:
		if (copy_to_user(argp, &dat_buf, sizeof(dat_buf))) {
			dev_err(&s_akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
		status = atomic_read(&s_akm->active);
		if (copy_to_user(argp, &status, sizeof(status))) {
			dev_err(&s_akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_DELAY:
		if (copy_to_user(argp, &delay, sizeof(delay))) {
			dev_err(&s_akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_LAYOUT:
		if (copy_to_user(argp, &s_akm->layout, sizeof(s_akm->layout))) {
			dev_err(&s_akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	default:
		break;
	}

	return 0;
}

static const struct file_operations AKECS_fops = {
	.owner = THIS_MODULE,
	.open = AKECS_Open,
	.release = AKECS_Release,
	.unlocked_ioctl = AKECS_ioctl,
};

static struct miscdevice akm_compass_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AKM_MISCDEV_NAME,
	.fops = &AKECS_fops,
};

/***** akm sysfs functions ******************************************/
static int create_device_attributes(
	struct device *dev,
	struct device_attribute *attrs)
{
	int i;
	int err = 0;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i) {
		err = device_create_file(dev, &attrs[i]);
		if (err)
			break;
	}

	if (err) {
		for (--i; i >= 0 ; --i)
			device_remove_file(dev, &attrs[i]);
	}

	return err;
}

static void remove_device_attributes(
	struct device *dev,
	struct device_attribute *attrs)
{
	int i;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i)
		device_remove_file(dev, &attrs[i]);
}

/*********************************************************************
 *
 * SysFS attribute functions
 *
 * directory : /sys/class/compass/akmXXXX/
 * files :
 *  - enable_mag [rw] [t] : enable flag for magnetometer
 *  - delay_mag  [rw] [t] : delay in nanosecond for magnetometer
 *
 * debug :
 *  - mode       [w]  [t] : E-Compass mode
 *  - bdata      [r]  [t] : buffered raw data
 *  - asa        [r]  [t] : FUSEROM data
 *  - regs       [r]  [t] : read all registers
 *
 * [b] = binary format
 * [t] = text format
 */

/***** sysfs enable *************************************************/
static void akm_compass_sysfs_update_status(
	struct akm_compass_data *akm)
{
	uint32_t en;
	mutex_lock(&akm->val_mutex);
	en = akm->enable_flag;
	mutex_unlock(&akm->val_mutex);

	if (en == 0) {
		if (atomic_cmpxchg(&akm->active, 1, 0) == 1) {
			wake_up(&akm->open_wq);
			dev_dbg(akm->class_dev, "Deactivated");
		}
	} else {
		if (atomic_cmpxchg(&akm->active, 0, 1) == 0) {
			wake_up(&akm->open_wq);
			dev_dbg(akm->class_dev, "Activated");
		}
	}
	dev_dbg(&akm->i2c->dev,
		"Status updated: enable=0x%X, active=%d",
		en, atomic_read(&akm->active));
}

static ssize_t akm_compass_sysfs_enable_show(
	struct akm_compass_data *akm, char *buf)
{
	int flag;

	mutex_lock(&akm->val_mutex);
	flag = ((akm->enable_flag) & 1);
	mutex_unlock(&akm->val_mutex);

	return scnprintf(buf, PAGE_SIZE, "%d\n", flag);
}

static ssize_t akm_compass_sysfs_enable_store(
	struct akm_compass_data *akm, char const *buf, size_t count)
{
	long en = 0;
	int ret = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtol(buf, AKM_BASE_NUM, &en))
		return -EINVAL;

	en = en ? 1 : 0;

	mutex_lock(&akm->op_mutex);

	ret = akm_compass_power_set(akm, en);
	if (ret) {
		dev_err(&akm->i2c->dev,
			"Fail to configure device power!\n");
		goto exit;
	}

	mutex_lock(&akm->val_mutex);
	akm->enable_flag &= ~(1);
	akm->enable_flag |= ((uint32_t)(en));
	mutex_unlock(&akm->val_mutex);

	akm_compass_sysfs_update_status(akm);

exit:
	mutex_unlock(&akm->op_mutex);
	return ret ? ret : count;
}

/***** Magnetic field ***/
// ASUS_BSP +++ Jiunhau_Wang "[PF450CL][Sensor][NA][Spec] Porting 9-axis sensor"
static ssize_t akm_chip_id_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", akm->sense_info[0]);
}
static int akm09911_i2c_check_device(struct i2c_client *client);
static ssize_t akm_status_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int err;

	err = akm09911_i2c_check_device(s_akm->i2c);
	if (err < 0)
		return sprintf(buf, "%d\n", 0);
	else
		return sprintf(buf, "%d\n", 1);
}

static ssize_t akm_i2c_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int err;

	err = akm09911_i2c_check_device(s_akm->i2c);
	if (err < 0)
		return sprintf(buf, "%d\n", 0);
	else
		return sprintf(buf, "%d\n", 1);
}

static ssize_t akm_read_raw(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int err;
	uint8_t buffer[AKM_SENSOR_DATA_SIZE];
	int i;
	int x = 0,y = 0,z = 0;
	int raw[6];

	memset(buffer, 0, sizeof(buffer));
	
	if (1)
	{
		err = AKECS_SetMode(akm, AK09911_MODE_SNG_MEASURE);
		if (err < 0)
		{
			return sprintf(buf, "0 0 0 \n");
		}
			
		mutex_lock(&akm->sensor_mutex);
		buffer[0] = AKM_REG_STATUS;
		err = akm_i2c_rxdata(akm->i2c, buffer, AKM_SENSOR_DATA_SIZE);
		if (err < 0)
		{
			mutex_unlock(&akm->sensor_mutex);
			return sprintf(buf, "0 0 0 \n");
		}
#if 0
		if (!(AKM_DRDY_IS_HIGH(buffer[0])))
		{
			mutex_unlock(&akm->sensor_mutex);
			return sprintf(buf, "0 0 0 \n");
		}
#endif
		mdelay(10);
		for(i = 0; i < 6; i++)
		{
			buffer[0] = 0x11 + i;
			err = akm_i2c_rxdata(akm->i2c, buffer, AKM_SENSOR_DATA_SIZE);
			if (err < 0)
				printk(KERN_INFO "[AKM] i2c err\n");
			raw[i] = buffer[0];
			//msleep(5);
		}
		x = (short) (raw[0] | (raw[1] << 8));
		y = (short) (raw[2] | (raw[3] << 8));
		z = (short) (raw[4] | (raw[5] << 8));

		mutex_unlock(&akm->sensor_mutex);
		
		err = AKECS_SetMode(akm, AK09911_MODE_POWERDOWN);
		if (err < 0)
		{
			return sprintf(buf, "0 0 0 \n");
		}
	}
	return sprintf(buf, "%d %d %d \n", x, y, z);
#if 0
	atomic_set(&akm->drdy, 1);
	wake_up(&akm->drdy_wq);
#endif
}

// ASUS_BSP --- Jiunhau_Wang "[PF450CL][Sensor][NA][Spec] Porting 9-axis sensor"
static ssize_t akm_enable_mag_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf);
}
static ssize_t akm_enable_mag_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count);
}

/***** sysfs delay **************************************************/
static ssize_t akm_compass_sysfs_delay_show(
	struct akm_compass_data *akm, char *buf)
{
	int64_t val;

	mutex_lock(&akm->val_mutex);
	val = akm->delay;
	mutex_unlock(&akm->val_mutex);

	return scnprintf(buf, PAGE_SIZE, "%lld\n", val);
}

static ssize_t akm_compass_sysfs_delay_store(
	struct akm_compass_data *akm, char const *buf, size_t count)
{
	long long val = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtoll(buf, AKM_BASE_NUM, &val))
		return -EINVAL;

	printk("[E-compass] Setting compass delay : %lld\n", val);

	mutex_lock(&akm->val_mutex);
	akm->delay = val;
	mutex_unlock(&akm->val_mutex);

	return count;
}

/***** Magnetic field ***/
static ssize_t akm_delay_mag_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_delay_show(
		dev_get_drvdata(dev), buf);
}
static ssize_t akm_delay_mag_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_delay_store(
		dev_get_drvdata(dev), buf, count);
}


#if AKM_DEBUG_IF
static ssize_t akm_sysfs_mode_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	long mode = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtol(buf, AKM_BASE_NUM, &mode))
		return -EINVAL;

	if (AKECS_SetMode(akm, (uint8_t)mode) < 0)
		return -EINVAL;

	return 1;
}

static ssize_t akm09911_sysfs_bdata_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	/* AK09911 specific function */
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	uint8_t rbuf[AKM_SENSOR_DATA_SIZE];

	mutex_lock(&akm->sensor_mutex);
	memcpy(&rbuf, akm->sense_data, sizeof(rbuf));
	mutex_unlock(&akm->sensor_mutex);

	return scnprintf(buf, PAGE_SIZE,
		"0x%02X,0x%02X,0x%02X,0x%02X,"
		"0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\n",
		rbuf[0], rbuf[1], rbuf[2], rbuf[3],
		rbuf[4], rbuf[5], rbuf[6], rbuf[7], rbuf[8]);
}

static ssize_t akm09911_sysfs_asa_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	/* AK09911 specific function */
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int err;
	uint8_t asa[3];

	err = AKECS_SetMode(akm, AK09911_MODE_FUSE_ACCESS);
	if (err < 0)
		return err;

	asa[0] = AK09911_FUSE_ASAX;
	err = akm_i2c_rxdata(akm->i2c, asa, 3);
	if (err < 0)
		return err;

	err = AKECS_SetMode(akm, AK09911_MODE_POWERDOWN);
	if (err < 0)
		return err;

	return scnprintf(buf, PAGE_SIZE,
			"0x%02X,0x%02X,0x%02X\n", asa[0], asa[1], asa[2]);
}

static ssize_t akm09911_sysfs_regs_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	/* AK09911 specific function */
	/* The total number of registers depends on the device. */
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int err;
	uint8_t regs[13];
	int sz;
	int n;
	char *cur;

	/* This function does not lock mutex obj */
	regs[0] = AK09911_REG_WIA1;
	err = akm_i2c_rxdata(akm->i2c, regs, 13);
	if (err < 0)
		return err;

	cur = buf;
	sz = snprintf(cur, PAGE_SIZE, "(HEX):");
	cur += sz;
	for (n = 0; n < 13; n++) {
		sz = snprintf(cur, 4, "%02X,", regs[n]);
		cur += sz;
	}

	return (ssize_t)(cur - buf);
}
#endif

static struct device_attribute akm_compass_attributes[] = {
	__ATTR(enable_mag, 0660, akm_enable_mag_show, akm_enable_mag_store),
	__ATTR(delay_mag,  0660, akm_delay_mag_show,  akm_delay_mag_store),
// ASUS_BSP +++ Jiunhau_Wang "[PF450][Sensor][NA][Spec] Porting 9-axis sensor"
	__ATTR(akm_chip_id, 0444, akm_chip_id_show, NULL),
	__ATTR(akm_status, 0444, akm_status_show, NULL),
	__ATTR(akm_09911_raw, 0444, akm_read_raw, NULL),
	__ATTR(akm_i2c, 0444, akm_i2c_show, NULL),
// ASUS_BSP --- Jiunhau_Wang "[PF450][Sensor][NA][Spec] Porting 9-axis sensor"
#if AKM_DEBUG_IF
	__ATTR(mode,  0220, NULL, akm_sysfs_mode_store),
	__ATTR(bdata, 0440, akm09911_sysfs_bdata_show, NULL),
	__ATTR(asa,   0440, akm09911_sysfs_asa_show, NULL),
	__ATTR(regs,  0440, akm09911_sysfs_regs_show, NULL),
#endif
	__ATTR_NULL,
};

static char const *const device_link_name = "i2c";
static dev_t const akm_compass_device_dev_t = MKDEV(MISC_MAJOR, 240);

static int create_sysfs_interfaces(struct akm_compass_data *akm)
{
	int err;

	if (NULL == akm)
		return -EINVAL;

	err = 0;

	akm->compass = class_create(THIS_MODULE, AKM_SYSCLS_NAME);
	if (IS_ERR(akm->compass)) {
		err = PTR_ERR(akm->compass);
		goto exit_class_create_failed;
	}

	akm->class_dev = device_create(
						akm->compass,
						NULL,
						akm_compass_device_dev_t,
						akm,
						AKM_SYSDEV_NAME);
	if (IS_ERR(akm->class_dev)) {
		err = PTR_ERR(akm->class_dev);
		goto exit_class_device_create_failed;
	}

	err = sysfs_create_link(
			&akm->class_dev->kobj,
			&akm->i2c->dev.kobj,
			device_link_name);
	if (0 > err)
		goto exit_sysfs_create_link_failed;

	err = create_device_attributes(
			akm->class_dev,
			akm_compass_attributes);
	if (0 > err)
		goto exit_device_attributes_create_failed;

	return err;

exit_device_attributes_create_failed:
	sysfs_remove_link(&akm->class_dev->kobj, device_link_name);
exit_sysfs_create_link_failed:
	device_destroy(akm->compass, akm_compass_device_dev_t);
exit_class_device_create_failed:
	akm->class_dev = NULL;
	class_destroy(akm->compass);
exit_class_create_failed:
	akm->compass = NULL;
	return err;
}

static void remove_sysfs_interfaces(struct akm_compass_data *akm)
{
	if (NULL == akm)
		return;

	if (NULL != akm->class_dev) {
		remove_device_attributes(
			akm->class_dev,
			akm_compass_attributes);
		sysfs_remove_link(
			&akm->class_dev->kobj,
			device_link_name);
		akm->class_dev = NULL;
	}
	if (NULL != akm->compass) {
		device_destroy(
			akm->compass,
			akm_compass_device_dev_t);
		class_destroy(akm->compass);
		akm->compass = NULL;
	}
}

#ifdef CONFIG_I2C_STRESS_TEST
#include <linux/i2c_testcase.h>
#define I2C_TEST_compass_sensor_FAIL (-1)

static int Compass_sensor_I2C_stress_test_function(void)
{	
	uint8_t buffer[AKM_SENSOR_DATA_SIZE];
	int err = 0;
	int rawdata_x = 0, rawdata_y = 0, rawdata_z = 0;

	s_akm->sense_info[0] = AK09911_REG_CNTL1;
	err = akm_i2c_rxdata(s_akm->i2c, s_akm->sense_info, 1);
	if (s_akm->compass_debug_switch)
		printk("[E-compass] get_akm09911_rawdata CNTL1 : (%d)\n", s_akm->sense_info[0]);
	if (err < 0)	{
		printk("[E-compass] akm_i2c_rxdata AKM_REG_STATUS read fail\n");
		return err;
	}
	if (s_akm->sense_info[0] == 0)	{
		err = AKECS_SetMode(s_akm, AK09911_MODE_SNG_MEASURE);
		if (err < 0)	{
			printk("[E-compass] get_akm09911_rawdata AK09911_MODE_SNG_MEASURE fail\n");
			return err;
		}
	} else	{
		err = AKECS_SetMode(s_akm, AK09911_MODE_POWERDOWN);
		if (err < 0)	{
			printk("[E-compass] get_akm09911_rawdata AK09911_MODE_POWERDOWN fail\n");
			return err;
		}
		msleep(50);
		err = AKECS_SetMode(s_akm, AK09911_MODE_SNG_MEASURE);
		if (err < 0)	{
			printk("[E-compass] get_akm09911_rawdata AK09911_MODE_SNG_MEASURE fail\n");
			return err;
		}
	}
	/* Read whole data and rest data */
	buffer[0] = AKM_REG_STATUS ;
	err = akm_i2c_rxdata(s_akm->i2c, buffer, AKM_SENSOR_DATA_SIZE);
	if (err < 0)	{
		printk("[E-compass] akm_i2c_rxdata AKM_REG_STATUS+1 fail\n");
		return err;
	}

	if (s_akm->compass_debug_switch)
		printk("[E-compass] rawdataX : (%d) | (%d)\n", buffer[1], buffer[2]);
	if (s_akm->compass_debug_switch)
		printk("[E-compass] rawdataY : (%d) | (%d)\n", buffer[3], buffer[4]);
	if (s_akm->compass_debug_switch)
		printk("[E-compass] rawdataZ : (%d) | (%d)\n", buffer[5], buffer[6]);
	rawdata_x = ( buffer[1] | (buffer[2]<<8) );
	rawdata_y = ( buffer[3] | (buffer[4]<<8) );
	rawdata_z = ( buffer[5] | (buffer[6]<<8) );

	if (rawdata_x > 0x7FFF)
		rawdata_x = rawdata_x-0x10000;
	if (rawdata_y > 0x7FFF)
		rawdata_y = rawdata_y-0x10000;
	if (rawdata_z > 0x7FFF)
		rawdata_z = rawdata_z-0x10000;
	if (s_akm->compass_debug_switch)
		printk("[E-compass] Rawdata : (%d), (%d), (%d)\n", rawdata_x, rawdata_y, rawdata_z);

	err = AKECS_SetMode(s_akm, AK09911_MODE_POWERDOWN);
	if (err < 0)	{
		printk("[E-compass] get_akm09911_rawdata AK09911_MODE_POWERDOWN fail\n");
		return err;
	}

	return 0;
}

static int Compass_sensor_I2C_stress_test(struct i2c_client *apClient)
{
	int lnResult = I2C_TEST_PASS;
	int err = 0;

	i2c_log_in_test_case("TestCompassSensorI2C ++\n");

	err = Compass_sensor_I2C_stress_test_function();
	if (err < 0)	{
		i2c_log_in_test_case("Fail to Test CompassSensorI2C!!!\n");
		lnResult = I2C_TEST_compass_sensor_FAIL;
		goto error_1;
	}
	i2c_log_in_test_case("TestCompassSensorI2C --\n");

error_1:
	return lnResult;
}

static struct i2c_test_case_info CompassSensorTestCaseInfo[] =	{
	__I2C_STRESS_TEST_CASE_ATTR(Compass_sensor_I2C_stress_test),
};
#endif

/***** akm input device functions ***********************************/
static int akm_compass_input_init(
	struct input_dev **input)
{
	int err = 0;

	/* Declare input device */
	*input = input_allocate_device();
	if (!*input)
		return -ENOMEM;

	/* Setup input device */
	set_bit(EV_ABS, (*input)->evbit);
	/* Magnetic field (limited to 16bit) */
	input_set_abs_params(*input, ABS_RX,
			-32768, 32767, 0, 0);
	input_set_abs_params(*input, ABS_RY,
			-32768, 32767, 0, 0);
	input_set_abs_params(*input, ABS_RZ,
			-32768, 32767, 0, 0);
	input_set_abs_params(*input, ABS_RUDDER,
			0, 3, 0, 0);

	/* Set name */
	(*input)->name = AKM_INPUT_DEVICE_NAME;

	/* Register */
	err = input_register_device(*input);
	if (err) {
		input_free_device(*input);
		return err;
	}

	return err;
}

/***** akm functions ************************************************/
static irqreturn_t akm_compass_irq(int irq, void *handle)
{
//	struct akm_compass_data *akm = handle;
	uint8_t buffer[AKM_SENSOR_DATA_SIZE];
	int err;

	memset(buffer, 0, sizeof(buffer));
	if (AKM_DEBUG_MESSAGE)
		printk("[E-compass] alp : akm_compass_irq +\n");
	/***** lock *****/
	mutex_lock(&s_akm->sensor_mutex);

	/* Read whole data */
	buffer[0] = AKM_REG_STATUS;
	err = akm_i2c_rxdata(s_akm->i2c, buffer, AKM_SENSOR_DATA_SIZE);
	if (err < 0) {
		dev_err(&s_akm->i2c->dev, "[E-compass] IRQ I2C error.");
		s_akm->is_busy = 0;
		mutex_unlock(&s_akm->sensor_mutex);
		/***** unlock *****/

		return IRQ_HANDLED;
	}
	/* Check ST bit */
	if (!(AKM_DRDY_IS_HIGH(buffer[0]))){
		if (AKM_DEBUG_MESSAGE)
			printk("[E-compass] alp : akm_compass_irq check_bit error\n");
		goto work_func_none;
	}
	memcpy(s_akm->sense_data, buffer, AKM_SENSOR_DATA_SIZE);
	s_akm->is_busy = 0;

	mutex_unlock(&s_akm->sensor_mutex);
	/***** unlock *****/

	atomic_set(&s_akm->drdy, 1);
	wake_up(&s_akm->drdy_wq);
	if (AKM_DEBUG_MESSAGE)
		printk("[E-compass] alp : akm_compass_irq -\n");
	dev_vdbg(&s_akm->i2c->dev, "[E-compass] IRQ handled.");
	return IRQ_HANDLED;

work_func_none:
	mutex_unlock(&s_akm->sensor_mutex);
	/***** unlock *****/

	dev_vdbg(&s_akm->i2c->dev, "[E-compass] IRQ not handled.");
	return IRQ_NONE;
}

static int akm_compass_suspend(struct device *dev)
{

	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int ret = 0;

	printk("[E-compass] alp : akm_compass_suspend !! \n");

	//if (AKM_IS_MAG_DATA_ENABLED() &&
	//	akm->use_poll &&
	//	akm->pdata->auto_report)
	//	cancel_delayed_work_sync(&akm->dwork);

	akm->state.power_on = akm->power_enabled;
	if (akm->state.power_on) {
		akm_compass_power_set(akm, false);
		/* Clear status */
		akm->is_busy = 0;
		atomic_set(&akm->drdy, 0);
	}

	ret = pinctrl_select_state(akm->pinctrl, akm->pin_sleep);
	if (ret)
		dev_err(dev, "Can't select pinctrl state\n");

	dev_dbg(&akm->i2c->dev, "suspended\n");

	return 0;

}

static int akm_compass_resume(struct device *dev)
{

	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int ret = 0;

	printk("[E-compass] alp : akm_compass_resume !! \n");

	ret = pinctrl_select_state(akm->pinctrl, akm->pin_default);
	if (ret)
		dev_err(dev, "Can't select pinctrl state\n");

	if (akm->state.power_on) {
		ret = akm_compass_power_set(akm, true);
		if (ret) {
			dev_err(dev, "Sensor power resume fail!\n");
			goto exit;
		}

		ret = AKECS_SetMode(akm, akm->state.mode);
		if (ret) {
			dev_err(dev, "Sensor state resume fail!\n");
			goto exit;
		}

		//if (AKM_IS_MAG_DATA_ENABLED() &&
		//	akm->use_poll &&
		//	akm->pdata->auto_report)
		//	schedule_delayed_work(&akm->dwork,
		//		(unsigned long)nsecs_to_jiffies64(
		//		akm->delay[MAG_DATA_FLAG]));
	}
	dev_dbg(&akm->i2c->dev, "resumed\n");

exit:
	return 0;
}

static int akm09911_i2c_check_device(
	struct i2c_client *client)
{
	/* AK09911 specific function */
	struct akm_compass_data *akm = i2c_get_clientdata(client);
	int err;

	s_akm->sense_info[0] = AK09911_REG_WIA1;
	err = akm_i2c_rxdata(client, s_akm->sense_info, AKM_SENSOR_INFO_SIZE);
	if (err < 0)	{
		printk("[E-compass] %s AKM read info fail(%d)\n", __func__, err);
		return err;
	}

	/* Set FUSE access mode */
	err = AKECS_SetMode(s_akm, AK09911_MODE_FUSE_ACCESS);
	if (err < 0)	{
		printk("[E-compass] %s AKECS_SetMode fai(%d)\n", __func__, err);
		return err;
	}

	akm->sense_conf[0] = AK09911_FUSE_ASAX;
	err = akm_i2c_rxdata(client, akm->sense_conf, AKM_SENSOR_CONF_SIZE);
	if (err < 0)	{
		printk("[E-compass] %s AKM read config fai(%d)\n", __func__, err);
		return err;
	}

	err = AKECS_SetMode(s_akm, AK09911_MODE_POWERDOWN);
	if (err < 0)	{
		printk("[E-compass] %s AKECS_SetMode fail(%d)\n", __func__, err);
		return err;
	}

	/* Check read data */
	if (akm->sense_info[0] != AKM_WIA_VALE) {
		dev_err(&client->dev,
			"[E-compass] %s: The device is not AKM Compass.", __func__);
		return -ENXIO;
	}

	return err;
}



static int akm_compass_power_set(struct akm_compass_data *data, bool on)
{
	int rc = 0;

	if (!on && data->power_enabled) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			goto err_vdd_disable;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			goto err_vio_disable;
		}
		data->power_enabled = false;
		return rc;
	} else if (on && !data->power_enabled) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_enable;
		}
		data->power_enabled = true;

		/*
		 * The max time for the power supply rise time is 50ms.
		 * Use 80ms to make sure it meets the requirements.
		 */
		msleep(80);
		return rc;
	} else {
		dev_warn(&data->i2c->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
		return rc;
	}

err_vio_enable:
	regulator_disable(data->vio);
err_vdd_enable:
	return rc;

err_vio_disable:
	if (regulator_enable(data->vdd))
		dev_warn(&data->i2c->dev, "Regulator vdd enable failed\n");
err_vdd_disable:
	return rc;
}

static int akm_compass_power_init(struct akm_compass_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				AKM09911_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				AKM09911_VIO_MAX_UV);

		regulator_put(data->vio);

	} else {
		data->vdd = regulator_get(&data->i2c->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->i2c->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				AKM09911_VDD_MIN_UV, AKM09911_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->i2c->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->i2c->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			dev_err(&data->i2c->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				AKM09911_VIO_MIN_UV, AKM09911_VIO_MAX_UV);
			if (rc) {
				dev_err(&data->i2c->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, AKM09911_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

static int akm09911_pinctrl_init(struct akm_compass_data *s_akm)
{
	struct i2c_client *client = s_akm->i2c;

	s_akm->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(s_akm->pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
		return PTR_ERR(s_akm->pinctrl);
	}

	s_akm->pin_default = pinctrl_lookup_state(s_akm->pinctrl,
			"default");
	if (IS_ERR_OR_NULL(s_akm->pin_default)) {
		dev_err(&client->dev, "Failed to look up default state\n");
		return PTR_ERR(s_akm->pin_default);
	}

	s_akm->pin_sleep = pinctrl_lookup_state(s_akm->pinctrl,
			"sleep");
	if (IS_ERR_OR_NULL(s_akm->pin_sleep)) {
		dev_err(&client->dev, "Failed to look up sleep state\n");
		return PTR_ERR(s_akm->pin_sleep);
	}

	return 0;
}




#if AKM_ATTR_ATD_TEST
//	add the attr for ATD Test
static ssize_t get_akm09911_state(struct device *dev, struct device_attribute *devattr, char *buf)
{	

//	struct i2c_client *client = to_i2c_client(dev);
//	struct akm_compass_data *akm = i2c_get_clientdata(client);
	int ak09911_wia = 0;
	int err=0;

	s_akm->sense_info[0] = AK09911_REG_WIA1;
	if (s_akm->compass_debug_switch)
		printk("[E-compass] alp : get_akm09911_state 1+\n");
	err = akm_i2c_rxdata(s_akm->i2c, s_akm->sense_info, AKM_SENSOR_INFO_SIZE);
	if (s_akm->compass_debug_switch)
		printk("[E-compass] alp : get_akm09911_state 1-\n");
	if (err < 0)
		if (s_akm->compass_debug_switch)
			printk("get_akm09911_state wia : %d\n", err);
	if (s_akm->compass_debug_switch)
		printk("[E-compass] get_akm09911_state wia : %d, %d\n", s_akm->sense_info[0], s_akm->sense_info[1]);
	if (s_akm->sense_info[0] == 72)
		ak09911_wia =  1;
	else
		ak09911_wia =  0;
	return sprintf(buf, "%d\n",ak09911_wia);
}

static ssize_t get_akm09911_rawdata(struct device *dev, struct device_attribute *devattr, char *buf)
{	
//	struct i2c_client *client = to_i2c_client(dev);
//	struct akm_compass_data *akm = i2c_get_clientdata(client);
	uint8_t buffer[AKM_SENSOR_DATA_SIZE];
	int retval = 0, err=0;
	int rawdata_x=0,rawdata_y=0,rawdata_z=0;

	s_akm->sense_info[0] = AK09911_REG_CNTL1;
	err = akm_i2c_rxdata(s_akm->i2c, s_akm->sense_info, 1);
	if (s_akm->compass_debug_switch)
		printk("[E-compass] get_akm09911_rawdata CNTL1 : (%d)\n", s_akm->sense_info[0]);
	if (err < 0)	{
		printk("[E-compass] akm_i2c_rxdata AKM_REG_STATUS read fail\n");
		return retval;
	}
	if (s_akm->sense_info[0] == 0)	{
		err = AKECS_SetMode(s_akm, AK09911_MODE_SNG_MEASURE);
		if (err < 0)
			printk("[E-compass] get_akm09911_rawdata AK09911_MODE_SNG_MEASURE fail\n");
	} else	{
		err = AKECS_SetMode(s_akm, AK09911_MODE_POWERDOWN);
		if (err < 0)
			printk("[E-compass] get_akm09911_rawdata AK09911_MODE_POWERDOWN fail\n");
		msleep(50);
		err = AKECS_SetMode(s_akm, AK09911_MODE_SNG_MEASURE);
		if (err < 0)
			printk("[E-compass] get_akm09911_rawdata AK09911_MODE_SNG_MEASURE fail\n");
	}
	// Read whole data and rest data
	buffer[0] = AKM_REG_STATUS ;
	err = akm_i2c_rxdata(s_akm->i2c, buffer, AKM_SENSOR_DATA_SIZE);
	if (err < 0)	{
		printk("[E-compass] akm_i2c_rxdata AKM_REG_STATUS+1 fail\n");
		return retval;
	}

	if (s_akm->compass_debug_switch)
		printk("[E-compass] rawdataX : (%d) | (%d)\n", buffer[1], buffer[2]);
	if (s_akm->compass_debug_switch)
		printk("[E-compass] rawdataY : (%d) | (%d)\n", buffer[3], buffer[4]);
	if (s_akm->compass_debug_switch)
		printk("[E-compass] rawdataZ : (%d) | (%d)\n", buffer[5], buffer[6]);
	rawdata_x = ( buffer[1] | (buffer[2]<<8) );
	rawdata_y = ( buffer[3] | (buffer[4]<<8) );
	rawdata_z = ( buffer[5] | (buffer[6]<<8) );

	if (rawdata_x > 0x7FFF)
		rawdata_x = rawdata_x-0x10000;
	if (rawdata_y > 0x7FFF)
		rawdata_y = rawdata_y-0x10000;
	if (rawdata_z > 0x7FFF)
		rawdata_z = rawdata_z-0x10000;
	if (s_akm->compass_debug_switch)
		printk("[E-compass] Rawdata : (%d), (%d), (%d)\n", rawdata_x, rawdata_y, rawdata_z);

	return sprintf(buf, "[E-compass] Rawdata : %d , %d , %d\n", rawdata_x, rawdata_y, rawdata_z);
}
static DEVICE_ATTR(state, S_IRUGO, get_akm09911_state, NULL);
static DEVICE_ATTR(rawdata, S_IRUGO, get_akm09911_rawdata, NULL);
#endif
/*
static ssize_t get_akm09911_debug_state(struct device *dev, struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "[E-compass] switch : %s\n", (s_akm->compass_debug_switch ? "on" : "off"));
}

static ssize_t Set_akm09911_debug_state(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
	char debugStr[12];

	memset(debugStr, 0, 12);
	strncpy(debugStr, buf, count);
	s_akm->compass_debug_switch = (simple_strtoul(&debugStr[0], NULL, 10) ? true : false);

	return sprintf(buf, "[E-compass] switch : %s\n", (s_akm->compass_debug_switch ? "on" : "off"));
}
static DEVICE_ATTR(akm09911_debug_state, 770, get_akm09911_debug_state, Set_akm09911_debug_state);
*/

static struct attribute *akm09911_attributes[] = {
#if AKM_ATTR_ATD_TEST
	&dev_attr_state.attr,
	&dev_attr_rawdata.attr,
#endif
	//&dev_attr_akm09911_debug_state.attr,
	NULL
};

static struct attribute_group akm09911_attribute_group = {
	.attrs = akm09911_attributes
};

int akm_compass_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;

//	dev_dbg(&client->dev, "start probing.");
	printk(KERN_INFO "[Progress][%s] Probe starts \n", AKM_I2C_NAME);
	printk("[E-compass] alp akm_compass_probe ++\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
				"[E-compass] %s: check_functionality failed.", __func__);
		err = -ENODEV;
		goto exit0;
	}

	/* Allocate memory for driver data */
	s_akm = kzalloc(sizeof(struct akm_compass_data), GFP_KERNEL);
	if (!s_akm) {
		dev_err(&client->dev,
				"[E-compass] %s: memory allocation failed.", __func__);
		err = -ENOMEM;
		goto exit1;
	}

	/**** initialize variables in akm_compass_data *****/
	init_waitqueue_head(&s_akm->drdy_wq);
	init_waitqueue_head(&s_akm->open_wq);

	mutex_init(&s_akm->sensor_mutex);
	mutex_init(&s_akm->val_mutex);
	mutex_init(&s_akm->op_mutex);

	atomic_set(&s_akm->active, 0);
	atomic_set(&s_akm->drdy, 0);

	/* ASUS BSP Peter_Lu for E-compass Debug & ATD tool */
	s_akm->compass_debug_switch = false;

	s_akm->is_busy = 0;
	s_akm->enable_flag = 0;
	s_akm->delay = -1;


	
	s_akm->gpio_rstn  = of_get_named_gpio(np, "akm,gpio_rstn", 0);

	printk("[E-compass] alp : akm09911 gpio_RSTN =%d \n", s_akm->gpio_rstn);

	err = gpio_request(s_akm->gpio_rstn, "ECOMP_RST#");
	if (err)
		pr_err("[E-compass] %s: unable to request gpio %d (ECOMP_RST#)\n", __func__, s_akm->gpio_rstn);

	err = gpio_direction_output(s_akm->gpio_rstn, 1);
	if (err < 0)
		pr_err("[E-compass] %s: unable to set the direction of gpio %d\n", __func__, s_akm->gpio_rstn);

	s_akm->layout = 0;

	/***** I2C initialization *****/
	s_akm->i2c = client;
	/* set client data */
	i2c_set_clientdata(client, s_akm);


	/* check connection */

	err = akm_compass_power_init(s_akm, true);
	if (err < 0){
		pr_err("[E-compass] %s: fail\n", __func__);
		goto exit1;
	}
	err = akm_compass_power_set(s_akm, true);
	if (err < 0){
		pr_err("[E-compass] %s: fail\n", __func__);
		goto err_compass_pwr_init;
	}

	/* initialize pinctrl */
	if (!akm09911_pinctrl_init(s_akm)) {
		err = pinctrl_select_state(s_akm->pinctrl, s_akm->pin_default);
		if (err) {
			dev_err(&client->dev, "Can't select pinctrl state\n");
			goto err_compass_pwr_off;
		}
	}

	/***** input *****/
	err = akm_compass_input_init(&s_akm->input);
	if (err) {
		dev_err(&client->dev,
			"%s: input_dev register failed", __func__);
		goto err_compass_pwr_off;
	}
	input_set_drvdata(s_akm->input, s_akm);

	/***** IRQ setup *****/
	/*
	 * ASUS +++ Peter_Lu Did not use irq , daemon control
	 * s_akm->irq = client->irq;
	 */
	s_akm->irq = 0;

	dev_dbg(&client->dev, "[E-compass] %s: IRQ is #%d.", __func__, s_akm->irq);

	if (s_akm->irq) {
		err = request_threaded_irq(
				s_akm->irq,
				NULL,
				akm_compass_irq,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"akm09911-irq",//dev_name(&client->dev),
				s_akm);
		if (err < 0) {
			if (s_akm->compass_debug_switch)
				printk("[E-compass] alp enable_irq fail err =%d \n", err);
			goto exit4;
		}
	}else{
		printk("[E-compass] alp : did not use irq , daemon control!! \n");
	}

	/***** misc *****/
	err = misc_register(&akm_compass_dev);
	if (err) {
		dev_err(&client->dev,
			"[E-compass] %s: akm_compass_dev register failed", __func__);
		goto exit5;
	}

	/***** sysfs *****/
	err = create_sysfs_interfaces(s_akm);
	if (0 > err) {
		dev_err(&client->dev,
			"[E-compass] %s: create sysfs failed.", __func__);
		goto exit6;
	}

	err = sysfs_create_group(&client->dev.kobj,&akm09911_attribute_group);
	if (err) {
		 printk("[E-compass] alp : sysfs_create_group fail\n");
	}

#ifdef CONFIG_I2C_STRESS_TEST
	printk("[I2CST] Compass Senor add test case\n");
	i2c_add_test_case(client, "CompassSensorTest", ARRAY_AND_SIZE(CompassSensorTestCaseInfo));
#endif

	/* check connection */
	err = akm09911_i2c_check_device(client);
	if (err < 0)
		goto exit1;

//<asus-annacheng20150129>>>>>>>>>>>>>>>>>>>>>>>+
	if(create_asusproc_compasssensor_status_entry())
		printk("[%s] : ERROR to create compasssensor proc entry\n",__func__);
//<asus-annacheng20150129><<<<<<<<<<<<<<<<<+
//	dev_info(&client->dev, "successfully probed.");
	printk("[E-compass] alp akm_compass_probe --\n");
	printk(KERN_INFO "[Progress][%s] Probe ends \n", AKM_I2C_NAME);
	
	return 0;

exit6:
	misc_deregister(&akm_compass_dev);
exit5:
	if (s_akm->irq)
		free_irq(s_akm->irq, s_akm);
exit4:
	input_unregister_device(s_akm->input);


err_compass_pwr_off:
	akm_compass_power_set(s_akm, false);
err_compass_pwr_init:
	akm_compass_power_init(s_akm, false);

//exit3:
	kfree(s_akm);
exit1:
exit0:
	return err;
}

static int akm_compass_remove(struct i2c_client *client)
{
	struct akm_compass_data *akm = i2c_get_clientdata(client);

	if (akm_compass_power_set(akm, false))
		dev_err(&client->dev, "power off failed.\n");
	if (akm_compass_power_init(akm, false))
		dev_err(&client->dev, "power deinit failed.\n");

	remove_sysfs_interfaces(akm);
	if (misc_deregister(&akm_compass_dev) < 0)
		dev_err(&client->dev, "misc deregister failed.");
	if (akm->irq)
		free_irq(akm->irq, akm);
	input_unregister_device(akm->input);
	kfree(akm);
#if AKM_ATTR_ATD_TEST
	sysfs_remove_group(&client->dev.kobj, &akm09911_attribute_group);
#endif
	dev_info(&client->dev, "successfully removed.");
	return 0;
}

static struct of_device_id akm_compass_match_table[] = {
	{ .compatible = "ak,ak09911",},
	{ },
};

static const struct i2c_device_id akm_compass_id[] = {
	{AKM_I2C_NAME, 0 },
	{ }
};

static const struct dev_pm_ops akm_compass_pm_ops = {
	.suspend	= akm_compass_suspend,
	.resume		= akm_compass_resume,
};

static struct i2c_driver akm_compass_driver = {
	.probe		= akm_compass_probe,
	.remove		= akm_compass_remove,
	.id_table	= akm_compass_id,
	.driver = {
		.name	= AKM_I2C_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= akm_compass_match_table,
		.pm		= &akm_compass_pm_ops,
	},
};

static int __init akm_compass_init(void)
{
	int res = 0;
	pr_info("[E-compass] AKM compass driver: initialize.\n");
	res = i2c_add_driver(&akm_compass_driver);
	if (res != 0)
		printk("[E-compass] I2c_add_driver fail, Error : %d\n", res);
	return res;
}

static void __exit akm_compass_exit(void)
{
	pr_info("AKM compass driver: release.");
	i2c_del_driver(&akm_compass_driver);
	//<asus-annacheng20150129>>>>>>>>>>>>>>>>>>+
	if(compasssensor_entry)
		remove_proc_entry("compasssensor_status", NULL);
	//<asus-annacheng20150129><<<<<<<<<<<<<<<<<+
}

module_init(akm_compass_init);
module_exit(akm_compass_exit);

MODULE_AUTHOR("viral wang <viral_wang@htc.com>");
MODULE_DESCRIPTION("AKM compass driver");
MODULE_LICENSE("GPL");
