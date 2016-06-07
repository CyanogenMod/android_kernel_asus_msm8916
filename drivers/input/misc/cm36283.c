/* drivers/input/misc/CM36283.c - CM36283 optical sensors driver
 *    
 * Copyright (C) 2012 Capella Microsystems Inc.
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

#include <linux/delay.h>
//#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/sensors.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/lightsensor.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

//#include <asm/mach-types.h>

//<ASUS-danielchan20150601>>>>>>>>>+
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/file.h>
//<ASUS-danielchan20150601><<<<<<<<+

#include <linux/cm36283.h>
#include <linux/capella_cm3602.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/of_gpio.h>
//#include <linux/HWVersion.h>
//+++++ward+++++
#include <linux/seq_file.h> 
//------ward-----
#include <linux/regulator/consumer.h>
extern int Read_PROJ_ID(void);


#define D(x...) pr_info(x)
#define sensitivity_20  20
#define sensitivity_25  25
#define Driverversion  "1.0.0"  //<ASUS-annacheng20150129+>
#define VENDOR  "CM6686"    
/* POWER SUPPLY VOLTAGE RANGE */
#define CM36283_VDD_MIN_UV	2700000
#define CM36283_VDD_MAX_UV	3300000
#define CM36283_VI2C_MIN_UV	1750000
#define CM36283_VI2C_MAX_UV	1950000

#define CONFIG_ASUS_FACTORY_SENSOR_MODE  1
//<ASUS-danielchan20150921>>>>>>>>>+
static uint16_t ZD_CLOSE_THD=0x90;
static uint16_t ZD_AWAY_THD=0x50;
//<ASUS-danielchan20150921><<<<<<<<+
static int sensitivity_x = 20;
static bool newold = 0 ; //20140523 Eve_Wen default = 0 means cm36283
static int proximity_state = 1; //<asus-wx20150429+>
static int proximity_int_away = 0; //<asus-wx20150506+>
static int cm36283_probe_fail = 0; //<asus-wx20150805+>
static int cm36283_is_no_present= 0;
//<-- ASUS-Bevis_Chen + -->

bool enLSensorConfig_flag =0 ;
bool enPSensorConfig_flag =0 ;

int LSensor_CALIDATA[2] = {0}; //input calibration data . Format : "200 lux -->lux value ; 1000 lux -->lux value"
int PSensor_CALIDATA[2] = {0}; //input calibration data . Format : "near 3cm :--> value ; far  5cm :--> value"
struct proc_dir_entry *lpsensor_entry = NULL;
 //<ASUS-annacheng20150129+>>>>+
struct proc_dir_entry *lightsensor_entry = NULL;
struct proc_dir_entry *proximitysensor_entry = NULL;
 //<ASUS-annacheng20150129+><<<<<+
 extern bool proximityap3426_check_status(void);
 extern void ftxxxx_disable_touch(bool flag);
 extern int get_audiomode(void);
//<++++++ward_du+++++++>
static int lpsensor_proc_show(struct seq_file *m, void *v) {
	if(!lpsensor_entry)
	return seq_printf(m, "-1\n");
	else
	return seq_printf(m, "1\n");
}

static int lpsensor_proc_open(struct inode *inode, struct  file *file) {
  return single_open(file, lpsensor_proc_show, NULL);
}

static const struct file_operations lpsensor_proc_fops = {
  .owner = THIS_MODULE,
  .open = lpsensor_proc_open,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};


int create_asusproc_lpsensor_status_entry( void )
{   
   lpsensor_entry = proc_create("asus_lpsensor_status", S_IWUGO| S_IRUGO, NULL,&lpsensor_proc_fops);
    if (!lpsensor_entry)

        return -ENOMEM;

    	return 0;

}

//<-------ward_du------->
//<-- ASUS-Bevis_Chen - -->

//=========================================

//     Calibration Formula:

//     y = f(x) 

//  -> ax - by = constant_k

//     a is f(x2) - f(x1) , b is x2 - x1

////=========================================            

int static calibration_light(int x_big, int x_small, int report_lux){        

                    int y_1000 = 1000;
                    int y_200 = 200;
                    int constant_k = (y_1000 - y_200)*x_small - (x_big - x_small)*y_200; 

					    if ( report_lux*(y_1000 - y_200) < constant_k){
                              return 0; 
                        }else {   
                             return ((report_lux*(y_1000 - y_200) - constant_k) / (x_big - x_small));			
                        }
}

#define I2C_RETRY_COUNT 10
#define NEAR_DELAY_TIME ((100 * HZ) / 1000)
#define CONTROL_INT_ISR_REPORT        0x00
#define CONTROL_ALS                   0x01
#define CONTROL_PS                    0x02
#define LS_RAWDATA_WINDOW             (10)  // 0.05 step/lux

static int record_init_fail = 0;
static void sensor_irq_do_work(struct work_struct *work);
static void light_sensor_initial_value_work_routine(struct work_struct *work);
static void proximity_initial_value_work_routine(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);
static DECLARE_DELAYED_WORK(light_sensor_initial_value_work, light_sensor_initial_value_work_routine);
static DECLARE_DELAYED_WORK(proximity_initial_value_work, proximity_initial_value_work_routine);

struct CM36283_info {
	struct class *CM36283_class;
	struct device *ls_dev;
	struct device *ps_dev;
	struct input_dev *ls_input_dev;
	struct input_dev *ps_input_dev;
	//struct early_suspend early_suspend;
	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;
	int intr_pin;
	int als_enable;
	int ps_enable;
	int ps_irq_flag;
	uint16_t *adc_table;
	uint16_t cali_table[10];
	int irq;
	int ls_calibrate;
	int (*power)(int, uint8_t); /* power to the chip */
	uint32_t als_kadc_cm36283;
	uint32_t als_gadc;
	uint16_t golden_adc;
	struct wake_lock ps_wake_lock;
	int psensor_opened;
	int lightsensor_opened;
	uint8_t slave_addr;
	uint16_t ps_close_thd_set;
	uint16_t ps_away_thd_set;	
	int current_level;
	uint16_t current_adc;
	uint16_t ps_conf1_val;
	uint16_t ps_conf3_val;
	uint16_t ls_cmd;
	uint8_t record_clear_int_fail;
	uint8_t psensor_sleep_becuz_suspend;
	uint8_t lsensor_sleep_becuz_early_suspend;
	uint8_t status_calling;
	int last_initial_report_lux;
	struct regulator *vdd;
	struct regulator *vio;

};

static struct CM36283_info *lp_info_cm36283 = NULL;
//#define DEBUG_VEMMC2
#ifdef DEBUG_VEMMC2
#include <linux/regulator/driver.h>
#include <linux/regulator/intel_pmic.h> 
#include <linux/mfd/intel_msic.h>

static struct device* vemmc2_userCtrl_dev;
static struct class* vemmc2_userCtrl_class;
//static struct class* gpio_userCtrl_class;
//static struct device* gpio_userCtrl_dev;

static struct regulator *vemmc2_reg;
static ssize_t vemmc2_reg_show(struct device *dev,
            struct device_attribute *attr, char *buf){

/*	

    int reg_err;	

	reg_err = reg_err = intel_msic_reg_write(VEMMC2CNT_ADDR, 0x07);

	if(!reg_err)

	     D("VEMMC2_cm32863 ---regulator write success !!\n");

    else 

     	D("VEMMC2_cm32863 ---regulator write fail !!\n");	

*/

/*



     struct CM36283_info *lpi = lp_info_cm36283;

     struct input_dev* find_ps_input_dev_ptr = lpi->ls_input_dev;

	 struct list_head node_start = find_ps_input_dev_ptr->node;

	 char* self_name = find_ps_input_dev_ptr->name;	 

	 list_for_each_entry_continue( find_ps_input_dev_ptr, 

	                              &node_start,

								   node ){ 

        D("input_dev is %s!!\n",find_ps_input_dev_ptr->name);								   

         if (find_ps_input_dev_ptr->name =="proximit"){

               D("find ps_input_dev success !!!\n");		

               break;   			   

		}

		

		if (find_ps_input_dev_ptr->name == self_name){

	           D("find ps_input_dev fail !!\n");	

               break;			   

	    }

	}/config/wifi

*/     

/*



        int projid;

        projid = Read_PROJ_ID();

        D("projid = %d\n", projid);

*/



/*

	int res=0;

    FILE  *sysfsfp;

    int* var;

	char *filename = "/config/wifi/psensor.txt";

    sysfsfp = fopen(filename, "r");

    if (sysfsfp!=NULL) {

        fscanf(sysfsfp, "%d\n", var);

        fclose(sysfsfp);

    } else {

        ALOGE("open file %s to read with error %d", filename, res);

    }

    D(" read test value is %d\n", var);

*/

return 0;
}

static ssize_t vemmc2_reg_store(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf, size_t count){
	struct CM36283_info *lpi = lp_info_cm36283;			    
    int vemmc2_en, reg_err; 
	vemmc2_en = -1;
	sscanf(buf, "%d", &vemmc2_en);

	if (vemmc2_en != 0 && vemmc2_en != 1
		&& vemmc2_en != 2 && vemmc2_en != 3 && vemmc2_en != 4)
		return -EINVAL;


	if (vemmc2_en==0) {
		D("[proximity-Touch test] %s: report =%d\n",
			__func__, vemmc2_en);

		 input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 0);   
		 input_sync(lpi->ps_input_dev);  

	}else if(vemmc2_en==1){

 		D("[proximity-Touch test] %s: report =%d\n",
			__func__, vemmc2_en);

		 input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 1);   
		 input_sync(lpi->ps_input_dev);  

    }else if(vemmc2_en==2){

 		D("[proximity-Touch test] %s: report =%d\n",
			__func__, vemmc2_en);

		 input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 2);   
		 input_sync(lpi->ps_input_dev);  

    }else if(vemmc2_en==3){

 		D("[proximity-Touch test] %s: report =%d\n",
			__func__, vemmc2_en);

		 input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 3);   
		 input_sync(lpi->ps_input_dev);  
    }

return count;
}
DEVICE_ATTR(vemmc2_ctrl, 0664, vemmc2_reg_show, vemmc2_reg_store);

/*
static ssize_t gpio_test_tool_show(struct device *dev,
              struct device_attribute *attr, char *buf){

     return 0;
}


static ssize_t gpio_test_tool_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count){

     int gpio_num, reg_err; 
         gpio_num = -1;
         sscanf(buf, "%d", &gpio_num);
         gpio_free(gpio_num);

return count;
}


DEVICE_ATTR(gpio_ctrl, 0664, gpio_test_tool_show, gpio_test_tool_store);
*/
#endif // DEBUG_VEMMC2


int fLevel_cm36283=-1;

static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;
static struct mutex ps_enable_mutex, ps_disable_mutex, ps_get_adc_mutex;
static struct mutex CM36283_control_mutex;
static int lightsensor_enable(struct CM36283_info *lpi);
static int lightsensor_disable(struct CM36283_info *lpi);
static int initial_CM36283(struct CM36283_info *lpi);
static void psensor_initial_cmd(struct CM36283_info *lpi);
static int cm36283_power_set(struct CM36283_info *info, bool on);

int32_t als_kadc_cm36283;
static int control_and_report(struct CM36283_info *lpi, uint8_t mode, uint16_t param);

static int I2C_RxData(uint16_t slaveAddr, uint8_t cmd, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	int val;
	struct CM36283_info *lpi = lp_info_cm36283;
    uint8_t subaddr[1];


	struct i2c_msg msgs[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = 1,
		 .buf = subaddr,
		 },

		{
		 .addr = slaveAddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },		 
	};

  subaddr[0] = cmd;	
	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info_cm36283->i2c_client->adapter, msgs, 2) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[PS][CM36283 error] %s, i2c err, slaveAddr 0x%x ISR gpio %d  = %d, record_init_fail %d \n",
				__func__, slaveAddr, lpi->intr_pin, val, record_init_fail);
		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[PS_ERR][CM36283 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}



static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)

{
	uint8_t loop_i;
	int val;
	struct CM36283_info *lpi = lp_info_cm36283;
	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info_cm36283->i2c_client->adapter, msg, 1) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[PS][CM36283 error] %s, i2c err, slaveAddr 0x%x, value 0x%x, ISR gpio%d  = %d, record_init_fail %d\n",
				__func__, slaveAddr, txData[0], lpi->intr_pin, val, record_init_fail);


		msleep(10);
	}


	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[PS_ERR][CM36283 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);

		return -EIO;
	}

	return 0;
}

static int _CM36283_I2C_Read_Word(uint16_t slaveAddr, uint8_t cmd, uint16_t *pdata)
{
	uint8_t buffer[2];
	int ret = 0;

	if (pdata == NULL)
		return -EFAULT;

	ret = I2C_RxData(slaveAddr, cmd, buffer, 2);

	if (ret < 0) {
		pr_err(
			"[PS_ERR][CM36283 error]%s: I2C_RxData fail [0x%x, 0x%x]\n",
			__func__, slaveAddr, cmd);

		return ret;
	}

	*pdata = (buffer[1]<<8)|buffer[0];
#if 0
	/* Debug use */
	printk(KERN_DEBUG "[CM36283] %s: I2C_RxData[0x%x, 0x%x] = 0x%x\n",

		__func__, slaveAddr, cmd, *pdata);
#endif
	return ret;
}


static int _CM36283_I2C_Write_Word(uint16_t SlaveAddress, uint8_t cmd, uint16_t data)
{
	char buffer[3];
	int ret = 0;
#if 0
	/* Debug use */
	printk(KERN_DEBUG
	"[CM36283] %s: _CM36283_I2C_Write_Word[0x%x, 0x%x, 0x%x]\n",
		__func__, SlaveAddress, cmd, data);
#endif

	buffer[0] = cmd;
	buffer[1] = (uint8_t)(data&0xff);
	buffer[2] = (uint8_t)((data&0xff00)>>8);	

	ret = I2C_TxData(SlaveAddress, buffer, 3);
	if (ret < 0) {
		pr_err("[PS_ERR][CM36283 error]%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	return ret;
}

static int get_ls_adc_value(uint16_t *als_step, bool resume)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	uint32_t tmpResult;
	int ret = 0;

	if (als_step == NULL)
		return -EFAULT;

	/* Read ALS data: */
	ret = _CM36283_I2C_Read_Word(lpi->slave_addr, ALS_DATA, als_step);
	if (ret < 0) {
		pr_err(
			"[LS][CM36283 error]%s: _CM36283_I2C_Read_Word fail\n",
			__func__);
		return -EIO;
	}

//<ASUS-danielchan20150520>>>>>>>>>+
    switch (asus_PRJ_ID) {
        //  case 0://ASUS_ZE550KL
        //
        //      break;
  // <asus-jhw20150618+>
		case 1://ASUS_ZE600KL
			if(*als_step <= 0x0A){//workaround get ls adc
				*als_step=0x00;
			}
			break;
  // <asus-jhw20150618->
        //  case 2://ASUS_ZX550KL
        //
        //      break;
        case 3://ASUS_ZD550KL
		if(*als_step <= 0x0A){//workaround get ls adc
		    *als_step=0x00;
		}
            break;
        default:
            break;
    }
//<ASUS-danielchan20150520><<<<<<<<+


    if (!lpi->ls_calibrate) {
		tmpResult = (uint32_t)(*als_step) * lpi->als_gadc / lpi->als_kadc_cm36283;
		
        if (tmpResult > 0xFFFF)
			*als_step = 0xFFFF;
		else
		  *als_step = tmpResult;  			


	}
	//D("[LS][CM36283] %s: raw adc = 0x%X, ls_calibrate = %d\n",
	//	__func__, *als_step, lpi->ls_calibrate);

	return ret;
}

static int set_lsensor_range(uint16_t low_thd, uint16_t high_thd)
{
	int ret = 0;
	struct CM36283_info *lpi = lp_info_cm36283;
	_CM36283_I2C_Write_Word(lpi->slave_addr, ALS_THDH, high_thd);
	_CM36283_I2C_Write_Word(lpi->slave_addr, ALS_THDL, low_thd);
	return ret;
}

static int get_ps_adc_value(uint16_t *data)
{
	int ret = 0;
	struct CM36283_info *lpi = lp_info_cm36283;

	if (data == NULL)
		return -EFAULT;	

	ret = _CM36283_I2C_Read_Word(lpi->slave_addr, PS_DATA, data);
        if(newold == 0)//20140522Eve
        {
                (*data) &= 0xFF;
                printk("Eve_Wen Psensor in cm36283\n");
        }
                if (ret < 0) {
		pr_err(
			"[PS][CM36283 error]%s: _CM36283_I2C_Read_Word fail\n",
			__func__);
		return -EIO;

	}else{
		pr_err(
			"[PS][CM36283 OK]%s: _CM36283_I2C_Read_Word OK 0x%x\n",
			__func__, *data);
	}

	return ret;
}

static uint16_t mid_value(uint16_t value[], uint8_t size)
{
	int i = 0, j = 0;
	uint16_t temp = 0;

	if (size < 3)
		return 0;


	for (i = 0; i < (size - 1); i++)
		for (j = (i + 1); j < size; j++)
			if (value[i] > value[j]) {
				temp = value[i];
				value[i] = value[j];
				value[j] = temp;
			}

	return value[((size - 1) / 2)];
}

static int get_stable_ps_adc_value(uint16_t *ps_adc)
{
	uint16_t value[3] = {0, 0, 0}, mid_val = 0;
	int ret = 0;
	int i = 0;
	int wait_count = 0;
	struct CM36283_info *lpi = lp_info_cm36283;

	for (i = 0; i < 3; i++) {
		/*wait interrupt GPIO high*/
		while (gpio_get_value(lpi->intr_pin) == 0) {
			msleep(10);
			wait_count++;

			if (wait_count > 12) {
				pr_err("[PS_ERR][CM36283 error]%s: interrupt GPIO low,"
					" get_ps_adc_value\n", __func__);
				return -EIO;
			}
		}

		ret = get_ps_adc_value(&value[i]);
		if (ret < 0) {
			pr_err("[PS_ERR][CM36283 error]%s: get_ps_adc_value\n",
				__func__);
			return -EIO;
		}

		if (wait_count < 60/10) {/*wait gpio less than 60ms*/
			msleep(60 - (10*wait_count));
		}

		wait_count = 0;
	}
	/*D("Sta_ps: Before sort, value[0, 1, 2] = [0x%x, 0x%x, 0x%x]",
		value[0], value[1], value[2]);*/
	mid_val = mid_value(value, 3);
	D("Sta_ps: After sort, value[0, 1, 2] = [0x%x, 0x%x, 0x%x]",
		value[0], value[1], value[2]);
	*ps_adc = (mid_val & 0xFF);

	return 0;
}

static void proximity_initial_value_work_routine(struct work_struct *work){
        struct CM36283_info *lpi = lp_info_cm36283;
		uint16_t ps_adc_value_init;
        int ret;
        ret = get_ps_adc_value(&ps_adc_value_init);
        if(!ret){
             if(ps_adc_value_init > lpi->ps_close_thd_set){
			       input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 0);     
	               input_sync(lpi->ps_input_dev);    
                               proximity_state = 0; //<asus-wx20150429+>
			       D("[PS][CM36283] proximity initial NEAR\n");
			 }else{
			       input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 1);     
	               input_sync(lpi->ps_input_dev);    
                               proximity_state = 1; //<asus-wx20150429+>
			       D("[PS][CM36283] proximity initial FAR\n");  
				 ftxxxx_disable_touch(false);
			 }
        }

	
	
         enable_irq(lpi->irq);		
}

bool proximity_check_status(void){
    struct CM36283_info *lpi;
	uint16_t ps_adc_value_init = 0 , data =0, data1= 0;
    int ret = 0;
	int p_value; //<asus-wx20150814+>

	pr_err("anna proximity_check_status default \n");
	if (cm36283_is_no_present) {
		pr_err("[PS][ap3426]  is ap3426\n"); 
		return proximityap3426_check_status();		
	}   				
//<asus-wx20150805>+>>
	if (cm36283_probe_fail) {
		pr_err("proximity_check_status default return FAR\n");
		goto error_return_far;
	}

	mutex_lock(&ps_enable_mutex);

	lpi = lp_info_cm36283;
	ret = _CM36283_I2C_Read_Word(lpi->slave_addr, PS_CONF1, &data);
	if (ret < 0) goto error_return_far;
//<asus-wx20150805>+<<

	if ( data & CM36283_PS_SD ) {
	         data1 = data & CM36283_PS_SD_MASK; //disable = 0
//<asus-wx20150805>+>>
	        ret = _CM36283_I2C_Write_Word(lpi->slave_addr, PS_CONF1, data1);   	
	        if (ret < 0) goto error_return_far;
//<asus-wx20150805>+<<
	}
	
	msleep(50); //need some delay

        ret = get_ps_adc_value(&ps_adc_value_init);
        if(!ret){	      
             if(ps_adc_value_init > lpi->ps_close_thd_set){
			       D("[PS][CM36283] proximity initial NEAR\n");
				p_value = 1; //<asus-wx20150814>
			 }else{
			       D("[PS][CM36283] proximity initial FAR\n");  
				p_value = 0; //<asus-wx20150814>
				ftxxxx_disable_touch(false);
			 }
        }
//<asus-wx20150814>+>>
	else {
		goto error_return_far;
	}
//<asus-wx20150814>+<<

	if ( data & CM36283_PS_SD) {
		data1 = data | CM36283_PS_SD; //disable = 1
//<asus-wx20150805>+>>
	        ret = _CM36283_I2C_Write_Word(lpi->slave_addr, PS_CONF1, data1);   
	        if (ret < 0) goto error_return_far;
//<asus-wx20150805>+<<
	}

	mutex_unlock(&ps_enable_mutex);
	return p_value;
error_return_far: //<asus-wx20150805+>
	return 0;
}
EXPORT_SYMBOL(proximity_check_status);
	
static void light_sensor_initial_value_work_routine(struct work_struct *work){
         int ret;
		 uint16_t adc_value;
         uint16_t ls_low_thd, ls_high_thd;
		 struct CM36283_info *lpi = lp_info_cm36283;
		  int report_lux=0;
		 
//<ASUS-danielchan20150730>>>>>>>>>+
#if 0
        switch (asus_PRJ_ID) {
          //  case 0://ASUS_ZE550KL
          //
          //      break;
          //  case 1://ASUS_ZE600KL
          //
          //      break;
          //  case 2://ASUS_ZX550KL
          //
          //      break;
          //  case 3://ASUS_ZD550KL
		//    msleep(100);//ER workaround report wrong lux
        //        break;
            default:
                break;
        }
#endif
//<ASUS-danielchan20150730><<<<<<<<+

         get_ls_adc_value(&adc_value, 0);

		 if (adc_value > LS_RAWDATA_WINDOW){
		      ls_low_thd = adc_value - LS_RAWDATA_WINDOW;
		  }else{
		      ls_low_thd = 0;
		  }

		  if (adc_value < 0xFFFF - LS_RAWDATA_WINDOW){
		      ls_high_thd = adc_value + LS_RAWDATA_WINDOW;    
		  }else{
		      ls_high_thd = 0xFFFF -1 ;
		  }

/*		  
    	  ret = set_lsensor_range(((i == 0) || (adc_value == 0)) ? 0 :
    		   	*(lpi->cali_table + (i - 1)) + 1,
    		    *(lpi->cali_table + i));
 */
//////// modified to increase sensitivity of lux interrupt 
           ret = set_lsensor_range(ls_low_thd,ls_high_thd);
		   //<-- ASUS-Bevis_Chen + -->
		   report_lux = adc_value;

           if(enLSensorConfig_flag == 1 ){//calibration enable 
					if(LSensor_CALIDATA[0] > 0&&LSensor_CALIDATA[1] > 0 
					&&LSensor_CALIDATA[1] >LSensor_CALIDATA[0] ){ //in case of zero divisor error

					//D("CM36283---Before 1st calibration, report_lux is %d\n", report_lux);
                    report_lux = calibration_light(LSensor_CALIDATA[1], LSensor_CALIDATA[0], report_lux);
                    //D("CM36283---After 1st calibration, report_lux is %d\n", report_lux);
					report_lux = report_lux/sensitivity_x;
			        D("CM36283--initial-After 1st devided by 20, report_lux is %d\n", report_lux);
					}else{
       					printk("%s:ASUS initial input LSensor_CALIDATA was invalid .error !!!!!\n",__func__);
				    }

			}else{	
#ifndef CONFIG_ASUS_FACTORY_SENSOR_MODE // in user build and no calibration data			   
                   report_lux = report_lux*15;
				   D("CM36283--initial- NO calibration data, use default config\n"); 
#else
	               D("CM36283--initial- NO calibration data, Factory branch , NO default config\n");
#endif // end of CONFIG_ASUS_FACTORY_SENSOR_MODE

                   report_lux = report_lux/sensitivity_x;
			       D("CM36283--initial-After devided by 20, report_lux is %d\n", report_lux);

		//<-- ASUS-Bevis_Chen - -->

                 }			

            if(report_lux %2 == 1)
			       report_lux -= 1;


            if(report_lux == lpi->last_initial_report_lux){
                  report_lux += 2;			  
      		  }	  

			input_report_abs(lpi->ls_input_dev, ABS_MISC,report_lux);
            input_sync(lpi->ls_input_dev);
			lpi->last_initial_report_lux = report_lux;
}

static void sensor_irq_do_work(struct work_struct *work)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	uint16_t intFlag;
    _CM36283_I2C_Read_Word(lpi->slave_addr, INT_FLAG, &intFlag);
 //  D("CM36283-----sensor_irq_do_work, intFlag = 0x%X\n", intFlag);
	control_and_report(lpi, CONTROL_INT_ISR_REPORT, intFlag);  
//	D("CM36283 --- before enable_irq \n");  
	enable_irq(lpi->irq);
//	D("CM36283 --- before wake_unlock \n");
//<asus-wx20150506>+>>
	if (proximity_int_away == 1 || proximity_state == 0) {
		wake_unlock(&(lpi->ps_wake_lock));
		proximity_int_away = 0;
	}
//<asus-wx20150506>+<<
//	D("CM36283 --- End of sensor_irq_do_work\n");
}


//<asus-annacheng20150129>>>>>>>>>>>>>>>+

static int lightsensor_proc_show(struct seq_file *m, void *v) {
	
	//if(!lightsensor_entry){
		int ret = 0,ret1=0;
		uint16_t idReg;
		//uint8_t i;				
		
		struct CM36283_info *lpi = lp_info_cm36283;   
		ret1 = _CM36283_I2C_Read_Word(lpi->slave_addr, ID_REG, &idReg);
		printk("anna-sensor idReg : 0x0c=0x%x \n", idReg);
	      
	  	if(ret1<0){
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
	  	if((idReg==0x0186)){
	  		ret =seq_printf(m," Vendor:%s(0x0%x)\n", VENDOR,idReg);
		}else{
			ret =seq_printf(m," Vendor:%s(0x0%x)\n", VENDOR,idReg);
		}

		//for ( i=0x00 ;i<=0x0c;i++){
		//	 ret1 = _CM36283_I2C_Read_Word(lpi->slave_addr, i, &idReg);
		//	 printk("anna-sensor idReg : 0x%x=0x%x \n",i, idReg);
			 
		//	  }
		// }
		ret1 = _CM36283_I2C_Read_Word(lpi->slave_addr, ID_REG, &idReg);
		if(ret1<0){	  		
			ret =seq_printf(m," Device status:error\n");
			//return seq_printf(m, " ERROR: i2c r/w test fail\n Driver version:1.0.0\n Vendor:0x0%x\n Device status error.\n",idReg);
		}else{
			ret =seq_printf(m," Device status:ok\n");			
		}
	     	    
   	return ret;

}

static int Proximitysensor_proc_show(struct seq_file *m, void *v) {
	
	//if(!lightsensor_entry){
		int ret = 0,ret1=0;
		uint16_t idReg;
		//uint8_t i;				
		
		struct CM36283_info *lpi = lp_info_cm36283;   
		ret1 = _CM36283_I2C_Read_Word(lpi->slave_addr, ID_REG, &idReg);
		printk("anna-sensor idReg : 0x0c=0x%x \n", idReg);
	      
	  	if(ret1<0){
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
	  	if((idReg==0x0186)){
	  		ret =seq_printf(m," Vendor:%s(0x0%x)\n", VENDOR,idReg);
		}else{
			ret =seq_printf(m," Vendor:%s(0x0%x)\n", VENDOR,idReg);
		}

		//for ( i=0x00 ;i<=0x0c;i++){
		//	 ret1 = _CM36283_I2C_Read_Word(lpi->slave_addr, i, &idReg);
		//	 printk("anna-sensor idReg : 0x%x=0x%x \n",i, idReg);
			 
		//	  }
		// }
		ret1 = _CM36283_I2C_Read_Word(lpi->slave_addr, ID_REG, &idReg);
		if(ret1<0){	  		
			ret =seq_printf(m," Device status:error\n");
			//return seq_printf(m, " ERROR: i2c r/w test fail\n Driver version:1.0.0\n Vendor:0x0%x\n Device status error.\n",idReg);
		}else{
			ret =seq_printf(m," Device status:ok\n");			
		}
	     	    
   	return ret;

}

static int lightsensor_proc_open(struct inode *inode, struct  file *file) {
  return single_open(file, lightsensor_proc_show, NULL);
}

static const struct file_operations lightsensor_proc_fops = {
  .owner = THIS_MODULE,
  .open = lightsensor_proc_open,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

static int Proximitysensor_proc_open(struct inode *inode, struct  file *file) {
  return single_open(file, Proximitysensor_proc_show, NULL);
}

static const struct file_operations Proximitysensor_proc_fops = {
  .owner = THIS_MODULE,
  .open = Proximitysensor_proc_open,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

int create_asusproc_lightsensor_status_entry(void){
	lightsensor_entry = proc_create("lightsensor_status", S_IWUGO| S_IRUGO, NULL,&lightsensor_proc_fops);
 	if (!lightsensor_entry)
       		 return -ENOMEM;

    	return 0;
}

int create_asusproc_Proximitysensor_status_entry(void){
	proximitysensor_entry = proc_create("Proximitysensor_status", S_IWUGO| S_IRUGO, NULL,&Proximitysensor_proc_fops);
	if (!proximitysensor_entry)
       		 return -ENOMEM;

    	return 0;
}
//<asus-<asus-annacheng20150129>><<<<<<<<<<<<<+
static irqreturn_t CM36283_irq_handler(int irq, void *data)
{

	struct CM36283_info *lpi = lp_info_cm36283;
	
   // D("CM36283 --- Enter into CM36283_irq_handler\n");

//<asus-wx20150429->	wake_lock(&(lpi->ps_wake_lock));
    if (proximity_state == 0) wake_lock_timeout(&(lpi->ps_wake_lock), 1 * HZ); //<asus-wx20150429+>
	disable_irq_nosync(lpi->irq);
	queue_work(lpi->lp_wq, &sensor_irq_work);

  //  D("CM36283 --- End of CM36283_irq_handler\n");
	return IRQ_HANDLED;
}

static int als_power(int enable)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	if (lpi->power)
		lpi->power(LS_PWR_ON, 1);

	return 0;
}

//<ASUS-danielchan20150921>>>>>>>>>+
static void psensor_calibration_check(struct CM36283_info *lpi,uint16_t close_thd_value,uint16_t away_thd_value)
{
	printk("[PS][CM36283] psensor_calibration_check lpi->ps_away_thd_set:0x%x lpi->ps_close_thd_set:0x%x\n",lpi->ps_away_thd_set,lpi->ps_close_thd_set);
	if((lpi->ps_away_thd_set == 0x100) &&  (lpi->ps_close_thd_set == 0x200) ) { //without calibration
		if(newold ==0) {
		    lpi->ps_away_thd_set = away_thd_value &0xFF;
		    lpi->ps_close_thd_set = (close_thd_value &0xFF00)>>8;
		} else {
			lpi->ps_close_thd_set = close_thd_value;
	        lpi->ps_away_thd_set = away_thd_value;
		}
        printk("[PS][CM36283] without calibration: ps_away_thd_set:0x%x ,ps_close_thd_set:0x%x\n",lpi->ps_away_thd_set,lpi->ps_close_thd_set);
	} else if(lpi->ps_away_thd_set > lpi->ps_close_thd_set ) {//wrong calibration
		if(newold ==0) {
		    lpi->ps_away_thd_set = away_thd_value &0xFF;
		    lpi->ps_close_thd_set = (close_thd_value &0xFF00)>>8;
		} else {
			lpi->ps_close_thd_set = close_thd_value;
	        lpi->ps_away_thd_set = away_thd_value;
		}
		printk("[PS][CM36283] wrong calibration: ps_away_thd_set:0x%x ,ps_close_thd_set:0x%x\n",lpi->ps_away_thd_set,lpi->ps_close_thd_set);
	}
}
//<ASUS-danielchan20150921><<<<<<<<+

static void ls_initial_cmd(struct CM36283_info *lpi)
{	
	/*must disable l-sensor interrupt befrore IST create*//*disable ALS func*/
	lpi->ls_cmd &= CM36283_ALS_INT_MASK;
    lpi->ls_cmd |= CM36283_ALS_SD;
   _CM36283_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);  
}

static void psensor_initial_cmd(struct CM36283_info *lpi)
{

	/*must disable p-sensor interrupt befrore IST create*/
	lpi->ps_conf1_val |= CM36283_PS_SD;
	lpi->ps_conf1_val &= CM36283_PS_INT_MASK;

	lpi->ps_conf1_val = 0x3d7;
	lpi->ps_conf3_val = 0x210;

	//<ASUS-danielchan20150921>>>>>>>>>+
    switch (asus_PRJ_ID) {
        case 3://ASUS_ZD550KL
            psensor_calibration_check(lpi,ZD_CLOSE_THD,ZD_AWAY_THD);
            break;
        default:
            break;
    }
	//<ASUS-danielchan20150921><<<<<<<<+

	_CM36283_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);
	_CM36283_I2C_Write_Word(lpi->slave_addr, PS_CONF3, lpi->ps_conf3_val);

	_CM36283_I2C_Write_Word(lpi->slave_addr, PS_THDL, lpi->ps_away_thd_set);
	_CM36283_I2C_Write_Word(lpi->slave_addr, PS_THDH, lpi->ps_close_thd_set);

	D("[PS][CM36283] %s, finish\n", __func__);	
}

static int psensor_enable(struct CM36283_info *lpi)
{
	int ret = -EIO;
	mutex_lock(&ps_enable_mutex);
	D("[PS][CM36283] %s\n", __func__);

	if ( lpi->ps_enable ) {
		D("[PS][CM36283] %s: already enabled\n", __func__);
		ret = 0;
	}else{
	// +++	
//     psensor_initial_cmd(lpi);
/*
	_CM36283_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);   
    _CM36283_I2C_Write_Word(lpi->slave_addr, PS_CONF3, lpi->ps_conf3_val);
	_CM36283_I2C_Write_Word(lpi->slave_addr, PS_THD, (lpi->ps_close_thd_set <<8)| lpi->ps_away_thd_set);
*/		
//    enable_irq(lpi->irq);
// ---	
  	disable_irq_nosync(lpi->irq);
	ret = control_and_report(lpi, CONTROL_PS, 1);
// +++	
/*     
	 input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 1);     
	 input_sync(lpi->ps_input_dev);  
*/
// ---
	queue_delayed_work(lpi->lp_wq, &proximity_initial_value_work, 10);
    }

	mutex_unlock(&ps_enable_mutex);
	if (ret!=0){
         D("[PS][CM36283] psensor_enable--fail!!!\n");    		
     }else{
	       D("[PS][CM36283] psensor_enable--success!!!\n");
	 }

	 return ret;
}

static int psensor_disable(struct CM36283_info *lpi)
{
	int ret = -EIO;
	mutex_lock(&ps_disable_mutex);
	D("[PS][CM36283] %s\n", __func__);
	if ( lpi->ps_enable == 0 ) {
		D("[PS][CM36283] %s: already disabled\n", __func__);
		ret = 0;
	} else{
  	    ret = control_and_report(lpi, CONTROL_PS,0);
   }
//	disable_irq(lpi->irq);
	mutex_unlock(&ps_disable_mutex); //For next time event be guaranteed to be sent! 
	input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 4);     
	input_sync(lpi->ps_input_dev);    
	ftxxxx_disable_touch(false);
	return ret;
}

static int psensor_open(struct inode *inode, struct file *file)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	D("[PS][CM36283] %s\n", __func__);
	if (lpi->psensor_opened)
		return -EBUSY;

	lpi->psensor_opened = 1;
	return 0;
}

static int psensor_release(struct inode *inode, struct file *file)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	D("[PS][CM36283] %s\n", __func__);
	lpi->psensor_opened = 0;

//	return psensor_disable(lpi);
	return 0;
}

static long psensor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int val;
	int rc ;
	struct CM36283_info *lpi = lp_info_cm36283;
	char enPcalibration_flag = 0 ;
	void __user *argp = (void __user *)arg;

	
	D("[PS][CM36283] %s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case CAPELLA_CM3602_IOCTL_ENABLE:
		if (get_user(val, (unsigned int __user *)arg))
			return -EFAULT;

		if (val){
			return psensor_enable(lpi);
		}else{
			return psensor_disable(lpi);
		}

		break;
	case CAPELLA_CM3602_IOCTL_GET_ENABLED:
		return put_user(lpi->ps_enable, (unsigned int __user *)arg);
		break;
	//<----------- ASUS-Bevis_Chen + --------------->
	/*case ASUS_PSENSOR_IOCTL_START:	
	        printk("%s:ASUS ASUS_PSENSOR_IOCTL_START  \n", __func__);
	        break;

      case ASUS_PSENSOR_IOCTL_CLOSE:				
	 	  printk("%s:ASUS ASUS_PSENSOR_IOCTL_CLOSE \n", __func__);
	        break;*/
	case ASUS_PSENSOR_IOCTL_GETDATA:
		{
			uint16_t ps_adc_value = 0;	
			int ret=0;

			rc = 0 ;
			printk("%s:ASUS ASUS_PSENSOR_IOCTL_GETDATA \n", __func__);
			ret = get_ps_adc_value(&ps_adc_value);
			if (ret < 0) {
					printk("%s:ASUS failed to get_ps_adc_value. \n",__func__);
					rc = -EIO;
					goto pend;
			}

			 if ( copy_to_user(argp, &ps_adc_value, sizeof(ps_adc_value) ) ) {
			            printk("%s:ASUS failed to copy psense data to user space.\n",__func__);
						rc = -EFAULT;	
						goto pend;
			  }		

			  printk("%s:ASUS_PSENSOR_IOCTL_GETDATA end\n", __func__);
		}
		break;
	case ASUS_PSENSOR_SETCALI_DATA:
		printk("%s:ASUS ASUS_PSENSOR_SETCALI_DATA \n", __func__);
		rc = 0 ;
		memset(PSensor_CALIDATA, 0, 2*sizeof(int));
		if (copy_from_user(PSensor_CALIDATA, argp, sizeof(PSensor_CALIDATA))){
			rc = -EFAULT;
			goto pend;
		}	

		printk("%s:ASUS_PSENSOR SETCALI_DATA : PSensor_CALIDATA[0] :  %d ,PSensor_CALIDATA[1]:  %d \n", 
			__func__, PSensor_CALIDATA[0],PSensor_CALIDATA[1]);

		if(PSensor_CALIDATA[0] <= 0||PSensor_CALIDATA[1] <= 0 
			||PSensor_CALIDATA[0] <= PSensor_CALIDATA[1] )
			rc =  -EINVAL;
			

	 	lpi->ps_away_thd_set = PSensor_CALIDATA[1] ;
		lpi->ps_close_thd_set = PSensor_CALIDATA[0];
		D("enPSensorConfig_flag is 1, ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x\n",lpi->ps_close_thd_set, lpi->ps_away_thd_set);
		if(newold == 0)
		{
   	    	        _CM36283_I2C_Write_Word(lpi->slave_addr, PS_THD, (lpi->ps_close_thd_set <<8)| lpi->ps_away_thd_set);
		}
		else
		{
			//<ASUS-danielchan20150921>>>>>>>>>+
			switch (asus_PRJ_ID) {
				case 3://ASUS_ZD550KL
					psensor_calibration_check(lpi,ZD_CLOSE_THD,ZD_AWAY_THD);
					break;
				default:
					break;
			}
			//<ASUS-danielchan20150921><<<<<<<<+
			_CM36283_I2C_Write_Word(lpi->slave_addr, PS_THDL, lpi->ps_away_thd_set);
			_CM36283_I2C_Write_Word(lpi->slave_addr, PS_THDH, lpi->ps_close_thd_set);
		}
		break;

	case ASUS_PSENSOR_EN_CALIBRATION:
		printk("%s:ASUS ASUS_PSENSOR_EN_CALIBRATION \n", __func__);
		rc = 0 ;
		if (copy_from_user(&enPcalibration_flag , argp, sizeof(enPcalibration_flag ))){
			rc = -EFAULT;
			goto pend;
		}	
		enPSensorConfig_flag =  enPcalibration_flag ;
		printk("%s: ASUS_PSENSOR_EN_CALIBRATION : enPSensorConfig_flag is : %d  \n",__func__,enPSensorConfig_flag); 
		break;		
	//<----------- ASUS-Bevis_Chen - ------------->
	default:
		pr_err("[PS][CM36283 error]%s: invalid cmd %d\n",
			__func__, _IOC_NR(cmd));
		return -EINVAL;
	}

	pend:
 		return rc;
}

static const struct file_operations psensor_fops = {
	.owner = THIS_MODULE,
	.open = psensor_open,
	.release = psensor_release,
	.unlocked_ioctl = psensor_ioctl
};

struct miscdevice psensor_misc_cm36283 = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cm3602",
	.fops = &psensor_fops
};

void lightsensor_set_kvalue_cm36283(struct CM36283_info *lpi)
{
	if (!lpi) {
		pr_err("[LS][CM36283 error]%s: ls_info is empty\n", __func__);
		return;
	}

	D("[LS][CM36283] %s: ALS calibrated als_kadc_cm36283=0x%x\n",
			__func__, als_kadc_cm36283);

	if (als_kadc_cm36283 >> 16 == ALS_CALIBRATED)
		lpi->als_kadc_cm36283 = als_kadc_cm36283 & 0xFFFF;
	else{
		lpi->als_kadc_cm36283 = 0;
		D("[LS][CM36283] %s: no ALS calibrated\n", __func__);
	}

	if (lpi->als_kadc_cm36283 && lpi->golden_adc > 0) {
		lpi->als_kadc_cm36283 = (lpi->als_kadc_cm36283 > 0 && lpi->als_kadc_cm36283 < 0x1000) ?
				lpi->als_kadc_cm36283 : lpi->golden_adc;
		lpi->als_gadc = lpi->golden_adc;
	}else{
		lpi->als_kadc_cm36283 = 1;
		lpi->als_gadc = 1;
	}

	D("[LS][CM36283] %s: als_kadc_cm36283=0x%x, als_gadc=0x%x\n",
		__func__, lpi->als_kadc_cm36283, lpi->als_gadc);
}

static int lightsensor_update_table(struct CM36283_info *lpi)
{
	uint32_t tmpData[10];
	int i;
	for (i = 0; i < 10; i++) {
		tmpData[i] = (uint32_t)(*(lpi->adc_table + i))
				* lpi->als_kadc_cm36283 / lpi->als_gadc ;

		if( tmpData[i] <= 0xFFFF ){
               lpi->cali_table[i] = (uint16_t) tmpData[i];		
         }else{
               lpi->cali_table[i] = 0xFFFF;    
          }         

		//D("[LS][CM36283] %s: Calibrated adc_table: data[%d], %x\n",
		//	__func__, i, lpi->cali_table[i]);

	}

	return 0;
}

static int lightsensor_enable(struct CM36283_info *lpi)
{
	int ret = -EIO;
	mutex_lock(&als_enable_mutex);
	D("[LS][CM36283] %s\n", __func__);

	if (lpi->als_enable) {
		D("[LS][CM36283] %s: already enabled\n", __func__);
		ret = 0;
	}else{
       ret = control_and_report(lpi, CONTROL_ALS, 1);
    }

	mutex_unlock(&als_enable_mutex);
	return ret;
}

static int lightsensor_disable(struct CM36283_info *lpi)
{
	int ret = -EIO;
	mutex_lock(&als_disable_mutex);
	D("[LS][CM36283] %s\n", __func__);

	if ( lpi->als_enable == 0 ) {
		D("[LS][CM36283] %s: already disabled\n", __func__);
		ret = 0;
	}else{
        ret = control_and_report(lpi, CONTROL_ALS, 0);
    }

	mutex_unlock(&als_disable_mutex);
	return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	int rc = 0;
	//D("[LS][CM36283] %s\n", __func__);

	if (lpi->lightsensor_opened) {
		pr_err("[LS][CM36283 error]%s: already opened\n", __func__);
		rc = -EBUSY;
	}

	lpi->lightsensor_opened = 1;
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	//D("[LS][CM36283] %s\n", __func__);
	lpi->lightsensor_opened = 0;
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	int rc, val;
	struct CM36283_info *lpi = lp_info_cm36283;
	char encalibration_flag = 0 ;
	void __user *argp = (void __user *)arg;
	/*D("[CM36283] %s cmd %d\n", __func__, _IOC_NR(cmd));*/
	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned int __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		//D("[LS][CM36283] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
		//	__func__, val);

		rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		 val = lpi->als_enable;
	   	 D("[LS][CM36283] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
			__func__, val);

		rc = put_user(val, (unsigned int __user *)arg);
		break;
	//<----------- ASUS-Bevis_Chen + --------------->
	/*case ASUS_LIGHTSENSOR_IOCTL_START:	
	        printk("%s:ASUS ASUS_LIGHTSENSOR_IOCTL_START  \n", __func__);
	        break;
	case ASUS_LIGHTSENSOR_IOCTL_CLOSE:				
	 	  printk("%s:ASUS ASUS_LIGHTSENSOR_IOCTL_CLOSE \n", __func__);
	        break;*/
	case ASUS_LIGHTSENSOR_IOCTL_GETDATA:
       	{
			 uint16_t adc_value = 0;	
			int report_lux;
			 rc = 0 ;
			 printk("%s:ASUS ASUS_LIGHTSENSOR_IOCTL_GETDATA \n", __func__);
				
			 mutex_lock(&CM36283_control_mutex);
	         	get_ls_adc_value(&adc_value, 0);
			 mutex_unlock(&CM36283_control_mutex);

			 report_lux = adc_value;
			 
			if(enLSensorConfig_flag == 1 ){//calibration enable 
				  if(LSensor_CALIDATA[0] > 0&&LSensor_CALIDATA[1] > 0 
	  	                     &&LSensor_CALIDATA[1] >LSensor_CALIDATA[0] ){ //in case of zero divisor error
	                            
	                              D("CM36283---Before calibration, ASUS_LIGHTSENSOR_IOCTL_GETDATA,  report_lux is %d\n", report_lux);
	                              report_lux = calibration_light(LSensor_CALIDATA[1], LSensor_CALIDATA[0], report_lux);
	                              D("CM36283---After calibration, ASUS_LIGHTSENSOR_IOCTL_GETDATA, report_lux is %d\n", report_lux);
	                              report_lux = report_lux/sensitivity_x;
	                              D("CM36283---After devided by 20, ASUS_LIGHTSENSOR_IOCTL_GETDATA, report_lux is %d\n", report_lux);	

	                      }else{
						        rc = -EINVAL;
						        printk("%s:ASUS input LSensor_CALIDATA was invalid .error !!!!!\n",__func__);
						       }
	              }else{
#ifndef CONFIG_ASUS_FACTORY_SENSOR_MODE // in user build and no calibration data			   
	                     report_lux = report_lux*15;
					     D("CM36283--- NO calibration data, use default config\n"); 
#else
		                 D("CM36283--- NO calibration data, Factory branch , NO default config\n"); 	
#endif // end of CONFIG_ASUS_FACTORY_SENSOR_MODE				
					     report_lux = report_lux/sensitivity_x;
	                     D("CM36283---After devided by 20, ASUS_LIGHTSENSOR_IOCTL_GETDATA, report_lux is %d\n", report_lux);	
	              }

	                if ( copy_to_user(argp, &report_lux, sizeof(report_lux) ) ) {
	                       printk("%s:ASUS failed to copy lightsense data to user space.\n",__func__);
	                       rc = -EFAULT;			
	                       goto end;
		           }		
			printk("%s:ASUS_LIGHTSENSOR_IOCTL_GETDATA end\n", __func__);
		}//ASUS_LIGHTSENSOR_IOCTL_GETDATA
		break;
	case ASUS_LIGHTSENSOR_SETCALI_DATA:

		printk("%s:ASUS ASUS_LIGHTSENSOR_SETCALI_DATA \n", __func__);
		rc = 0 ;
		memset(LSensor_CALIDATA, 0, 2*sizeof(int));

		if (copy_from_user(LSensor_CALIDATA, argp, sizeof(LSensor_CALIDATA)))
		{
			rc = -EFAULT;
			goto end;
		}	

		printk("%s:ASUS_LIGHTSENSOR SETCALI_DATA : LSensor_CALIDATA[0] :  %d ,LSensor_CALIDATA[1]:  %d \n", 
			__func__, LSensor_CALIDATA[0],LSensor_CALIDATA[1]);

		if(LSensor_CALIDATA[0] <= 0||LSensor_CALIDATA[1] <= 0 
			||LSensor_CALIDATA[0] >= LSensor_CALIDATA[1] )
			rc =  -EINVAL;


		break;
	case ASUS_LIGHTSENSOR_EN_CALIBRATION:
		printk("%s:ASUS ASUS_LIGHTSENSOR_EN_CALIBRATION \n", __func__);
		rc = 0 ;
		if (copy_from_user(&encalibration_flag , argp, sizeof(encalibration_flag )))
		{
			rc = -EFAULT;
			goto end;
		}	
		enLSensorConfig_flag =  encalibration_flag ;

		printk("%s: ASUS_LIGHTSENSOR_EN_CALIBRATION : enLSensorConfig_flag is : %d  \n",__func__,enLSensorConfig_flag); 
		break;		
	//<----------- ASUS-Bevis_Chen - ------------->

	default:
		pr_err("[LS][CM36283 error]%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

end:
	
    return rc;
}

static const struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};

static ssize_t ps_adc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint16_t value;
	int ret;
	struct CM36283_info *lpi = lp_info_cm36283;
	int intr_val;

	psensor_enable(lpi);//anna

	intr_val = gpio_get_value(lpi->intr_pin);
	get_ps_adc_value(&value);
       ret = sprintf(buf, "ADC[0x%04X], ENABLE = %d, intr_pin = %d\n", value, lpi->ps_enable, intr_val);
	return ret;
}

static ssize_t ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct CM36283_info *lpi = lp_info_cm36283;
    int ps_en; 
	ps_en = -1;
	sscanf(buf, "%d", &ps_en);

	if (ps_en != 0 && ps_en != 1
		&& ps_en != 10 && ps_en != 13 && ps_en != 16)
		return -EINVAL;


	if (ps_en) {
		D("[PS][CM36283] %s: ps_en=%d\n", __func__, ps_en);
		psensor_enable(lpi);
	}else{
		psensor_disable(lpi);
    }

	D("[PS][CM36283] %s\n", __func__);

	return count;
}
static DEVICE_ATTR(ps_adc, 0664, ps_adc_show, ps_enable_store);
unsigned PS_cmd_test_value_cm36283;

static ssize_t ps_parameters_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct CM36283_info *lpi = lp_info_cm36283;

	ret = sprintf(buf, "PS_close_thd_set = 0x%x, PS_away_thd_set = 0x%x, PS_cmd_cmd:value = 0x%x\n",
		lpi->ps_close_thd_set, lpi->ps_away_thd_set, PS_cmd_test_value_cm36283);

	return ret;
}

static ssize_t ps_parameters_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	char *token[10];
	int i;
	printk(KERN_INFO "[PS][CM36283] %s\n", buf);
	for (i = 0; i < 3; i++)
		token[i] = strsep((char **)&buf, " ");


	lpi->ps_close_thd_set = simple_strtoul(token[0], NULL, 16);
	lpi->ps_away_thd_set = simple_strtoul(token[1], NULL, 16);	
	PS_cmd_test_value_cm36283 = simple_strtoul(token[2], NULL, 16);

	printk(KERN_INFO "[PS][CM36283]Set PS_close_thd_set = 0x%x, PS_away_thd_set = 0x%x, PS_cmd_cmd:value = 0x%x\n",
		lpi->ps_close_thd_set, lpi->ps_away_thd_set, PS_cmd_test_value_cm36283);

	D("[PS][CM36283] %s\n", __func__);

	return count;
}

static DEVICE_ATTR(ps_parameters, 0664, ps_parameters_show, ps_parameters_store);

static ssize_t ps_conf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	return sprintf(buf, "PS_CONF1 = 0x%x, PS_CONF3 = 0x%x\n", lpi->ps_conf1_val, lpi->ps_conf3_val);
}

static ssize_t ps_conf_store(struct device *dev, struct device_attribute *attr,	const char *buf, size_t count)
{
	int code1, code2;
	struct CM36283_info *lpi = lp_info_cm36283;

	sscanf(buf, "0x%x 0x%x", &code1, &code2);
	D("[PS]%s: store value PS conf1 reg = 0x%x PS conf3 reg = 0x%x\n", __func__, code1, code2);
    lpi->ps_conf1_val = code1;
    lpi->ps_conf3_val = code2;

	_CM36283_I2C_Write_Word(lpi->slave_addr, PS_CONF3, lpi->ps_conf3_val );  
	_CM36283_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val );

	return count;
}


static DEVICE_ATTR(ps_conf, 0664, ps_conf_show, ps_conf_store);



static ssize_t cm36283_dump_reg(struct device *dev, struct device_attribute *attr, char *buf)	
{

	struct CM36283_info *lpi = lp_info_cm36283;
	uint8_t 	i = 0;
	uint16_t value;
	int ret;
	
	for(i = 0; i <0xD; i++)
	{
		ret = _CM36283_I2C_Read_Word(lpi->slave_addr, i, &value); 
		printk("cmd =%02x value = %02x\n", i, value);
	}
	
	return sprintf(buf, "%d\n", ret);
}
static DEVICE_ATTR(cm36283_dump, 0440, cm36283_dump_reg, NULL);


static ssize_t ps_thd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	int dummy;
	struct CM36283_info *lpi = lp_info_cm36283;
    if(newold==0)
    {
	uint16_t thd;

        ret = _CM36283_I2C_Read_Word(lpi->slave_addr, PS_THD, &thd); 

        if (!ret){
        D("[PS][CM36283]threshold = 0x%x\n", thd);
        }else{
        D("[PS][CM36283] --- fail to read threshold  \n");
        }	


        dummy = sprintf(buf, "%s ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x\n", __func__, lpi->ps_close_thd_set, lpi->ps_away_thd_set);

        return dummy;
    }
    else
    {
  	ret = sprintf(buf, "[PS][CM36686]PS Hi/Low THD ps_close_thd_set = 0x%04x(%d), ps_away_thd_set = 0x%04x(%d)\n", lpi->ps_close_thd_set, lpi->ps_close_thd_set, lpi->ps_away_thd_set, lpi->ps_away_thd_set);
    return ret;

    }
}

static ssize_t ps_thd_store(struct device *dev,	struct device_attribute *attr, const char *buf, size_t count)
{
	if(newold ==0)
	{
		int code;
		int ret=0;
		struct CM36283_info *lpi = lp_info_cm36283;

		sscanf(buf, "0x%x", &code);
		D("[PS]%s: store value = 0x%x\n", __func__, code);

		lpi->ps_away_thd_set = code &0xFF;
		lpi->ps_close_thd_set = (code &0xFF00)>>8;	
	    	_CM36283_I2C_Write_Word(lpi->slave_addr, PS_THD, (lpi->ps_close_thd_set <<8)| lpi->ps_away_thd_set);
		D("[PS]%s: ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x\n", __func__, lpi->ps_close_thd_set, lpi->ps_away_thd_set);


		//ret = sprintf(buf,"[PS]%s: ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x\n", __func__, lpi->ps_close_thd_set, lpi->ps_away_thd_set );

		return ret;
	}
	else
	{
		int code1, code2;		
		struct CM36283_info *lpi = lp_info_cm36283;

	        sscanf(buf, "0x%x 0x%x", &code1, &code2);
			
	        lpi->ps_close_thd_set = code1;  
	        lpi->ps_away_thd_set = code2;
			//<ASUS-danielchan20150921>>>>>>>>>+
			switch (asus_PRJ_ID) {
				case 3://ASUS_ZD550KL
					psensor_calibration_check(lpi,ZD_CLOSE_THD,ZD_AWAY_THD);
					break;
				default:
					break;
			}
			//<ASUS-danielchan20150921><<<<<<<<+
	        _CM36283_I2C_Write_Word(lpi->slave_addr, PS_THDH, lpi->ps_close_thd_set );
	        _CM36283_I2C_Write_Word(lpi->slave_addr, PS_THDL, lpi->ps_away_thd_set );
	        D("[PS][CM36686]%s: ps_close_thd_set = 0x%04x(%d), ps_away_thd_set = 0x%04x(%d)\n", __func__, lpi->ps_close_thd_set, lpi->ps_close_thd_set, lpi->ps_away_thd_set, lpi->ps_away_thd_set);

        //ret = sprintf(buf,"[PS][CM36686]%s: ps_close_thd_set = 0x%04x(%d), ps_away_thd_set = 0x%04x(%d)\n", __func__, lpi->ps_close_thd_set, lpi->ps_close_thd_set, lpi->ps_away_thd_set, lpi->ps_away_thd_set);
        return count; //anna
	}
}


static DEVICE_ATTR(ps_thd, 0664, ps_thd_show, ps_thd_store);


//anna>>>
static ssize_t ps_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct CM36283_info *lpi = lp_info_cm36283;	
	int val;
       uint16_t ps_data = 0;
    if(newold==0)
    {
	
	 psensor_enable(lpi);
	 get_stable_ps_adc_value(&ps_data);
             val = (ps_data >= lpi->ps_close_thd_set) ? 0 : 1;
 	 // ret  = _CM36283_I2C_Read_Word(lpi->slave_addr, INT_FLAG, &thd); 

	    D("[PS][CM36283]INT_FLAG = 0x%x\n", ps_data);
		
	ret = sprintf(buf, "%s ps status= %x\n", __func__, val);
	return ret;
    }
    else
    {

	 psensor_enable(lpi);
	 get_stable_ps_adc_value(&ps_data);
             val = (ps_data >= lpi->ps_close_thd_set) ? 0 : 1;
 	 // ret  = _CM36283_I2C_Read_Word(lpi->slave_addr, INT_FLAG, &thd); 

        D("[PS][CM36283]INT_FLAG = 0x%x\n", ps_data);
       
		
	ret = sprintf(buf, "%s ps status= %x\n", __func__, val);
	return ret;

    }
	
}

static DEVICE_ATTR(ps_status, 0440, ps_status_show, NULL);
//anna<<<<

static ssize_t ps_hw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct CM36283_info *lpi = lp_info_cm36283;

	ret = sprintf(buf, "PS1: reg = 0x%x, PS3: reg = 0x%x, ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x\n",
	lpi->ps_conf1_val, lpi->ps_conf3_val, lpi->ps_close_thd_set, lpi->ps_away_thd_set);

	return ret;
}

static ssize_t ps_hw_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int code;
//	struct CM36283_info *lpi = lp_info_cm36283;

	sscanf(buf, "0x%x", &code);
	D("[PS]%s: store value = 0x%x\n", __func__, code);

	return count;
}

static DEVICE_ATTR(ps_hw, 0664, ps_hw_show, ps_hw_store);

static ssize_t ps_calling_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct CM36283_info *lpi = lp_info_cm36283;
    int ret = 0;
	ret = sprintf(buf,"[PS][CM36283]%s: calling status is %d\n", __func__, lpi->status_calling);

	return ret;
}

static ssize_t ps_calling_store(struct device *dev, 
                                struct device_attribute *attr,
                                const char *buf, 
                                size_t count)
{

	struct CM36283_info *lpi = lp_info_cm36283;			    
    int status_calling; 
	status_calling = -1;

	sscanf(buf, "%d", &status_calling);
    lpi->status_calling = status_calling;
	D("[PS][CM36283]%s: calling status is %d\n", __func__, lpi->status_calling);

	return count;
}

static DEVICE_ATTR(ps_calling, 0664, ps_calling_show, ps_calling_store);

static ssize_t ls_adc_show(struct device *dev,
				           struct device_attribute *attr, 
                           char *buf)
{
	int ret;
	uint16_t als_step;
	struct CM36283_info *lpi = lp_info_cm36283;
	lightsensor_enable(lpi); //anna
   	get_ls_adc_value(&als_step, 0);
	D("[LS][CM36283] %s: ADC = 0x%x !!!\n",	__func__, als_step);

	ret = sprintf(buf, "ADC[0x%x] !!!\n", als_step);

	return ret;
}

static ssize_t ls_adc_store(struct device *dev,
 				            struct device_attribute *attr,
				            const char *buf, size_t count)
{   // NOP
	return count;
}

static DEVICE_ATTR(ls_adc, 0664, ls_adc_show, ls_adc_store);

static ssize_t ls_enable_show(struct device *dev,
				              struct device_attribute *attr, 
                              char *buf)
{
	int ret = 0;
	struct CM36283_info *lpi = lp_info_cm36283;

	ret = sprintf(buf, "Light sensor Auto Enable = %d\n", lpi->als_enable);

	return ret;
}


static ssize_t ls_enable_store(struct device *dev,
				               struct device_attribute *attr,
				               const char *buf, 
                               size_t count)
{
	int ret = 0;
	int ls_auto;
	struct CM36283_info *lpi = lp_info_cm36283;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147)
		return -EINVAL;


	if (ls_auto) {
		lpi->ls_calibrate = (ls_auto == 147) ? 1 : 0;
		ret = lightsensor_enable(lpi);
	}else{
		lpi->ls_calibrate = 0;
		ret = lightsensor_disable(lpi);
	}

	D("[LS][CM36283] %s: lpi->als_enable = %d, lpi->ls_calibrate = %d, ls_auto=%d\n",
		__func__, lpi->als_enable, lpi->ls_calibrate, ls_auto);

	if (ret < 0)
		pr_err("[LS][CM36283 error]%s: set auto light sensor fail\n",__func__);


	return count;
}

static DEVICE_ATTR(ls_auto, 0664, ls_enable_show, ls_enable_store);

static ssize_t ls_kadc_show(struct device *dev,
                            struct device_attribute *attr, 
                            char *buf)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	int ret;

	ret = sprintf(buf, "kadc = 0x%x", lpi->als_kadc_cm36283);

	return ret;
}

static ssize_t ls_kadc_store(struct device *dev,
        				     struct device_attribute *attr,
				             const char *buf, 
                             size_t count)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	int kadc_temp = 0;
	sscanf(buf, "%d", &kadc_temp);

	mutex_lock(&als_get_adc_mutex);

  if(kadc_temp != 0) {
		lpi->als_kadc_cm36283 = kadc_temp;

            if(  lpi->als_gadc != 0){
                        if (lightsensor_update_table(lpi) < 0)
			    	        printk(KERN_ERR "[LS][CM36283 error] %s: update ls table fail\n", __func__);

            }else{
			       printk(KERN_INFO "[LS]%s: als_gadc =0x%x wait to be set\n",	__func__, lpi->als_gadc);
            }		

  }else{
		printk(KERN_INFO "[LS]%s: als_kadc_cm36283 can't be set to zero\n",	__func__);

       }

	mutex_unlock(&als_get_adc_mutex);
	return count;
}

static DEVICE_ATTR(ls_kadc, 0664, ls_kadc_show, ls_kadc_store);

static ssize_t ls_gadc_show(struct device *dev,
				            struct device_attribute *attr, 
                            char *buf)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	int ret;

	ret = sprintf(buf, "gadc = 0x%x\n", lpi->als_gadc);
	return ret;
}

static ssize_t ls_gadc_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, 
                             size_t count)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	int gadc_temp = 0;

	sscanf(buf, "%d", &gadc_temp);
	mutex_lock(&als_get_adc_mutex);

  if(gadc_temp != 0) {
		lpi->als_gadc = gadc_temp;

                if(  lpi->als_kadc_cm36283 != 0){
  		               if (lightsensor_update_table(lpi) < 0)
                            printk(KERN_ERR "[LS][CM36283 error] %s: update ls table fail\n", __func__);
  	   
               }else{
			            printk(KERN_INFO "[LS]%s: als_kadc_cm36283 =0x%x wait to be set\n",	__func__, lpi->als_kadc_cm36283);
  	                }		

	}else{
		        printk(KERN_INFO "[LS]%s: als_gadc can't be set to zero\n", __func__);

	}

	mutex_unlock(&als_get_adc_mutex);

	return count;
}


static DEVICE_ATTR(ls_gadc, 0664, ls_gadc_show, ls_gadc_store);

static ssize_t ls_adc_table_show(struct device *dev,
			                     struct device_attribute *attr, 
                                 char *buf)
{
	unsigned length = 0;
	int i;

	for (i = 0; i < 10; i++) {
		length += sprintf(buf + length,
			"[CM36283]Get adc_table[%d] =  0x%x ; %d, Get cali_table[%d] =  0x%x ; %d, \n",
			i, *(lp_info_cm36283->adc_table + i),
			*(lp_info_cm36283->adc_table + i),
			i, 
            *(lp_info_cm36283->cali_table + i),
			*(lp_info_cm36283->cali_table + i));
	}

	return length;
}



static ssize_t ls_adc_table_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, 
                                  size_t count)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	char *token[10];
	uint16_t tempdata[10];
	int i;

	printk(KERN_INFO "[LS][CM36283]%s\n", buf);

	for (i = 0; i < 10; i++) {
		token[i] = strsep((char **)&buf, " ");
		tempdata[i] = simple_strtoul(token[i], NULL, 16);

		if (tempdata[i] < 1 || tempdata[i] > 0xffff) {
			printk(KERN_ERR "[LS][CM36283 error] adc_table[%d] =  0x%x Err\n", i, tempdata[i]);

			return count;
		}

	}

	mutex_lock(&als_get_adc_mutex);

	for (i = 0; i < 10; i++) {
		lpi->adc_table[i] = tempdata[i];

		printk(KERN_INFO

		"[LS][CM36283]Set lpi->adc_table[%d] =  0x%x\n", i, *(lp_info_cm36283->adc_table + i));

	}

	if (lightsensor_update_table(lpi) < 0)
		printk(KERN_ERR "[LS][CM36283 error] %s: update ls table fail\n", __func__);


	mutex_unlock(&als_get_adc_mutex);
	D("[LS][CM36283] %s\n", __func__);

	return count;
}

static DEVICE_ATTR(ls_adc_table, 0664, ls_adc_table_show, ls_adc_table_store);

static ssize_t ls_conf_show(struct device *dev,
				            struct device_attribute *attr, 
                            char *buf)
{
	struct CM36283_info *lpi = lp_info_cm36283;

	return sprintf(buf, "ALS_CONF = 0x%x\n", lpi->ls_cmd);
}

static ssize_t ls_conf_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, 
                             size_t count)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	int value = 0;

	sscanf(buf, "0x%x", &value);

	lpi->ls_cmd = value;
	printk(KERN_INFO "[LS]set ALS_CONF = %x\n", lpi->ls_cmd);

	_CM36283_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);

	return count;
}

static DEVICE_ATTR(ls_conf, 0664, ls_conf_show, ls_conf_store);



static ssize_t ls_fLevel_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
	return sprintf(buf, "fLevel_cm36283 = %d\n", fLevel_cm36283);
}

static ssize_t ls_fLevel_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	int value=0;

	sscanf(buf, "%d", &value);
	(value>=0)?(value=min(value,10)):(value=max(value,-1));

	fLevel_cm36283=value;

	input_report_abs(lpi->ls_input_dev, ABS_MISC, fLevel_cm36283);
	input_sync(lpi->ls_input_dev);

	printk(KERN_INFO "[LS]set fLevel_cm36283 = %d\n", fLevel_cm36283);

	msleep(1000);

	fLevel_cm36283=-1;

	return count;
}

static DEVICE_ATTR(ls_flevel, 0664, ls_fLevel_show, ls_fLevel_store);

static int lightsensor_setup(struct CM36283_info *lpi)
{
	int ret;
	lpi->ls_input_dev = input_allocate_device();

	if (!lpi->ls_input_dev) {
		pr_err(	"[LS][CM36283 error]%s: could not allocate ls input device\n", __func__);

		return -ENOMEM;
	}

	lpi->ls_input_dev->name = "lightsensor-level";
	set_bit(EV_ABS, lpi->ls_input_dev->evbit);

	input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(lpi->ls_input_dev);

	if (ret < 0) {
		pr_err("[LS][CM36283 error]%s: can not register ls input device\n", __func__);

		goto err_free_ls_input_device;
	}

	ret = misc_register(&lightsensor_misc);

	if (ret < 0) {
		pr_err("[LS][CM36283 error]%s: can not register ls misc device\n", __func__);

		goto err_unregister_ls_input_device;
	}

	return ret;

err_unregister_ls_input_device:

	input_unregister_device(lpi->ls_input_dev);

err_free_ls_input_device:

	input_free_device(lpi->ls_input_dev);

	return ret;
}

static int psensor_setup(struct CM36283_info *lpi)
{
	int ret;
	lpi->ps_input_dev = input_allocate_device();

	if (!lpi->ps_input_dev) {
		pr_err( "[PS][CM36283 error]%s: could not allocate ps input device\n", __func__);
		return -ENOMEM;
	}

	lpi->ps_input_dev->name = "proximity";
	
    set_bit(EV_ABS, lpi->ps_input_dev->evbit);
	input_set_abs_params(lpi->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(lpi->ps_input_dev);

	if (ret < 0) {
               pr_err( "[PS][CM36283 error]%s: could not register ps input device\n", __func__);
               goto err_free_ps_input_device;
	}

	ret = misc_register(&psensor_misc_cm36283);

	if (ret < 0) {
             pr_err( "[PS][CM36283 error]%s: could not register ps misc device\n", __func__);
 		     goto err_unregister_ps_input_device;
	}


	return ret;

err_unregister_ps_input_device:
	input_unregister_device(lpi->ps_input_dev);

err_free_ps_input_device:
	input_free_device(lpi->ps_input_dev);


	return ret;
}
/*
static int I2C_RxData_to_judge_cm3628(unsigned short slaveAddr, unsigned char *rxData, int length)
{
    unsigned char loop_i;
    int val;

    struct i2c_msg msgs[] = {
        {
            .addr = slaveAddr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = rxData,
        },
    };

    for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {

           if (i2c_transfer(lp_info_cm36283->i2c_client->adapter, msgs, 1) > 0)
               break;

        val = gpio_get_value(lp_info_cm36283->intr_pin);
        //check intr GPIO when i2c error
           if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
                 D("[ALS+PS][CM3628-AD3 error] %s, i2c err, slaveAddr 0x%x ISR gpio %d  = %d, record_init_fail %d \n",
                    __func__, slaveAddr, lp_info_cm36283->intr_pin, val, record_init_fail);


        msleep(10);
    }

    if (loop_i >= I2C_RETRY_COUNT) {
            printk(KERN_ERR "[ALS+PS_ERR][CM3628-AD3 error] %s retry over %d\n",  __func__, I2C_RETRY_COUNT);
            return -EIO;
    }


    return 0;
}
*/
static int initial_CM36283(struct CM36283_info *lpi)
{
	int val;
	int ret;
//	uint8_t dummy;
	uint16_t idReg;

    val = gpio_get_value(lpi->intr_pin);
	D("[PS][CM36283] %s, INTERRUPT GPIO val = %d\n", __func__, val);

	ret = _CM36283_I2C_Read_Word(lpi->slave_addr, ID_REG, &idReg);
        idReg = idReg&0x00FF;
        printk("P-sensor idReg : %d \n", idReg);
	if ((ret < 0) || ((idReg != 0x0083) && (idReg != 0x0086))) {
  		    if (record_init_fail == 0)
  		   	      record_init_fail = 1;   
        printk("ret = %d \n", ret);			    
        printk("p-sensor init fail \n");
        return -ENOMEM;/*If devices without CM36283/CM36686 chip and did not probe driver*/	
     }
        if(idReg == 0x0083)
        {
                sensitivity_x = sensitivity_20;
                newold = 0; // if newold = 1 means cm36686
        }
        else
        {
                sensitivity_x = sensitivity_25;
                newold = 1;
        }
                printk("p-sensor init OK!\n");
        printk("p-sensor sensitivity = %d\n", sensitivity_x);
	return 0;
}

static int CM36283_setup(struct CM36283_info *lpi)
{
	int ret = 0;
	als_power(1);
	msleep(5);

	ret = gpio_request(lpi->intr_pin, "gpio_CM36283_intr");

	if (ret < 0) {
		pr_err("[PS][CM36283 error]%s: gpio %d request failed (%d)\n", __func__, lpi->intr_pin, ret);
		return ret;
	}

	ret = gpio_direction_input(lpi->intr_pin);

	if (ret < 0) {
   		   pr_err( "[PS][CM36283 error]%s: fail to set gpio %d as input (%d)\n", __func__, lpi->intr_pin, ret);
		   goto fail_free_intr_pin;
	}

	ret = initial_CM36283(lpi);

	if (ret < 0) {
		   pr_err( "[PS_ERR][CM36283 error]%s: fail to initial CM36283 (%d)\n", __func__, ret);
           goto fail_free_intr_pin;
	}

	/*Default disable P sensor and L sensor*/
    ls_initial_cmd(lpi);
	psensor_initial_cmd(lpi);

	ret = request_irq (lpi->irq,
                       CM36283_irq_handler,
                       IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
             			"cm3628",
            			lpi);

	if (ret < 0) {
   		   pr_err( "[PS][CM36283 error]%s: req_irq(%d) fail for gpio %d (%d)\n", __func__, lpi->irq, lpi->intr_pin, ret);
           goto fail_free_intr_pin;
	}
//<asus-wx20150429>->>
/*
	ret = enable_irq_wake(lpi->irq);

	if (ret < 0) {
           pr_err( "[PS][CM36283 error]%s: req_irq(%d) fail for gpio %d (%d)\n", __func__, lpi->irq, lpi->intr_pin, ret);
           goto fail_free_intr_pin;
	}
*/
//<asus-wx20150429>-<<
	return ret;

fail_free_intr_pin:
	gpio_free(lpi->intr_pin);

	return ret;
}
/*
static void CM36283_early_suspend(struct early_suspend *h)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	D("[LS][CM36283] %s\n", __func__);

	if (lpi->als_enable){
            lightsensor_disable(lpi);
            lpi->lsensor_sleep_becuz_early_suspend = 1;
	}
}

static void CM36283_late_resume(struct early_suspend *h)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	D("[ALS][CM36283] %s\n", __func__);

	if (lpi->lsensor_sleep_becuz_early_suspend){
		    lightsensor_enable(lpi);
            lpi->lsensor_sleep_becuz_early_suspend = 0;
            D("[LS][CM36283] lightsensor late_resume\n");    
	}
}
*/
static int cm36283_parse_dt(struct device *dev,
				struct CM36283_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32	levels[CM36283_LEVELS_SIZE], i;
	u32 temp_val;
	int rc;

	rc = of_get_named_gpio_flags(np, "capella,interrupt-gpio",
			0, NULL);
	if (rc < 0) {
		dev_err(dev, "Unable to read interrupt pin number\n");
		return rc;
	} else {
	
		//dev_err(dev, "zxtest rc = %d\n",rc);
		pdata->intr = rc;
	}

	rc = of_property_read_u32_array(np, "capella,levels", levels,
			CM36283_LEVELS_SIZE);
	if (rc) {
		dev_err(dev, "Unable to read levels data\n");
		return rc;
	} else {
		for (i = 0; i < CM36283_LEVELS_SIZE; i++)
			pdata->levels[i] = levels[i];
	}

	rc = of_property_read_u32(np, "capella,ps_close_thd_set", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_close_thd_set\n");
		return rc;
	} else {
		
		pdata->ps_close_thd_set = (u16)temp_val;
		//dev_err(dev, "zxtest pdata->ps_close_thd_set= %u \n",pdata->ps_close_thd_set);
	}

	rc = of_property_read_u32(np, "capella,ps_away_thd_set", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_away_thd_set\n");
		return rc;
	} else {
		pdata->ps_away_thd_set = (u16)temp_val;
	}

	rc = of_property_read_u32(np, "capella,ls_cmd", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ls_cmd\n");
		return rc;
	} else {
		pdata->ls_cmd = (u16)temp_val;
	}

	rc = of_property_read_u32(np, "capella,ps_conf1_val", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_conf1_val\n");
		return rc;
	} else {
		pdata->ps_conf1_val = (u16)temp_val;
	}

	rc = of_property_read_u32(np, "capella,ps_conf3_val", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_conf3_val\n");
		return rc;
	} else {
		pdata->ps_conf3_val = (u16)temp_val;
	}

	pdata->polling =  0;//of_property_read_bool(np, "capella,use-polling");

	return 0;
}

//<ASUS-danielchan20150601>>>>>>>>>+
static ssize_t psensor_auto_calibration_read(struct file *dev, char *buf, size_t count, loff_t *ppos) {
    int ret;
	struct CM36283_info *lpi = lp_info_cm36283;
    ret = sprintf(buf, "ASUS ---@ps_close_thd_set:%d ,ps_away_thd_set:%d \n",lpi->ps_close_thd_set,lpi->ps_away_thd_set);
    return ret;
}

static ssize_t psensor_auto_calibration_write(struct file *dev,const char *buf, size_t count, loff_t *ppos){
    int send_buff = -1;
	int CalidataX,CalidataL,CalidataH;
	uint16_t init_value=0x00;
	uint16_t close_thd_value,away_thd_value;
	struct CM36283_info *lpi = lp_info_cm36283;
	struct file *filp = NULL;
	char  tmpbuf[20] = "";
	int tmplen=0;
	mm_segment_t old_fs;
	int ret = -EIO;
	// enable psensor
	mutex_lock(&ps_enable_mutex);
	if ( lpi->ps_enable ) {
		D("[PS][CM36283] %s: already enabled\n", __func__);
		ret = 0;
	}else{
        disable_irq_nosync(lpi->irq);
        ret = control_and_report(lpi, CONTROL_PS, 1);
        queue_delayed_work(lpi->lp_wq, &proximity_initial_value_work, 10);
    }

	mutex_unlock(&ps_enable_mutex);
	if (ret!=0){
         D("[PS][CM36283] psensor_enable--fail!!!\n");
     }else{
	       D("[PS][CM36283] psensor_enable--success!!!\n");
	 }

    sscanf(buf, "%d", &send_buff);
	get_ps_adc_value(&init_value);

	if(init_value==0x00) {
	    printk("[PS][CM36686] driver not ready");
	    return count;
	}

	if(send_buff==1) { // close_thd_value ,away_thd_value
        printk("[PS][CM36686] psensor_calibration:%d \n",  send_buff);
        get_ps_adc_value(&init_value);
        switch (asus_PRJ_ID) {
          //  case 0://ASUS_ZE550KL
          //
          //      break;
          //  case 1://ASUS_ZE600KL
          //
          //      break;
          //  case 2://ASUS_ZX550KL
          //
          //      break;
            case 3://ASUS_ZD550KL
                if(init_value>10) {
                    close_thd_value= (abs(init_value/3)+25)+init_value;
                    away_thd_value= (abs(init_value/6)+15)+init_value;
                }else {
                    close_thd_value= (init_value+24)+init_value;
                    away_thd_value= (init_value+13)+init_value;
                }
                break;
            default:
				close_thd_value=init_value+60;
		        away_thd_value=init_value+25;
                break;
        }

		if(newold ==0) {
		    lpi->ps_away_thd_set = away_thd_value &0xFF;
		    lpi->ps_close_thd_set = (close_thd_value &0xFF00)>>8;
		    _CM36283_I2C_Write_Word(lpi->slave_addr, PS_THD, (lpi->ps_close_thd_set <<8)| lpi->ps_away_thd_set);
		} else {
			lpi->ps_close_thd_set = close_thd_value;
	        lpi->ps_away_thd_set = away_thd_value;
			_CM36283_I2C_Write_Word(lpi->slave_addr, PS_THDH, lpi->ps_close_thd_set );
	        _CM36283_I2C_Write_Word(lpi->slave_addr, PS_THDL, lpi->ps_away_thd_set );
		}
		printk("[PS][CM36686] %s: ps_close_thd_set = (0x%x), ps_away_thd_set = (0x%x)\n", __func__, lpi->ps_close_thd_set , lpi->ps_away_thd_set);
	} else if(send_buff==2) { //save calibration
        if(init_value>10) {
            CalidataX=init_value-4;
        } else {
            CalidataX=init_value;
        }
		CalidataL=lpi->ps_away_thd_set;
		CalidataH=lpi->ps_close_thd_set;
		filp = filp_open("/factory/PSensor_Calibration.ini", O_RDWR | O_CREAT,0660);
		if (IS_ERR(filp)) {
			printk("[PS][CM36686] can't open /factory/PSensor_Calibration.ini \n");
		} else {
		    sprintf(tmpbuf,"%d %d %d ", CalidataX,CalidataH,CalidataL);
			tmplen=strlen(tmpbuf);
	        printk("[PS][CM36686] tmpbuf:%s , tmplen:%d  \n",tmpbuf, tmplen);
	        old_fs = get_fs();
            set_fs(KERNEL_DS);
			filp->f_op->write(filp, tmpbuf,  tmplen, &filp->f_pos);
	        set_fs(old_fs);
            filp_close(filp, NULL);
            printk("[PS][CM36686] save /factory/PSensor_Calibration.ini \n");
		}
	} else {
	    printk("[PS][CM36686] not support command\n");
	}
    return count;
}
static const struct file_operations proc_psensor_auto_calibration_send = {
     .read       = psensor_auto_calibration_read,
     .write      = psensor_auto_calibration_write,
};
//<ASUS-danielchan20150601<<<<<<<<<<+

static int CM36283_probe (struct i2c_client *client,
	                      const struct i2c_device_id *id)
{   
	int ret = 0;
	struct CM36283_info *lpi;
	struct CM36283_platform_data *pdata;

			int status = 0;
//<ASUS-danielchan20150601>>>>>>>>>+
    void* dummy = NULL;
    struct proc_dir_entry* proc_psensor_calibration  = NULL;
//<ASUS-danielchan20150603>>>>>>>>>+
#ifdef CONFIG_ASUS_SENSOR_ENG_CMD
    proc_psensor_calibration = proc_create_data("psensor_auto_calibration", 0660, NULL, &proc_psensor_auto_calibration_send, dummy);
#else
    proc_psensor_calibration=NULL;
    dummy = NULL;
#endif
//<ASUS-danielchan20150603<<<<<<<<<<+
//<ASUS-danielchan20150601<<<<<<<<<<+
			
	D("[PS][CM36283] %s\n", __func__);

	lpi = kzalloc(sizeof(struct CM36283_info), GFP_KERNEL);

	if (!lpi)
		return -ENOMEM;

	lpi->i2c_client = client;

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory for pdata\n");
			ret = -ENOMEM;
			goto err_platform_data_null;
		}

		ret = cm36283_parse_dt(&client->dev, pdata);
		pdata->slave_addr = client->addr;
		if (ret) {
			dev_err(&client->dev, "Failed to get pdata from device tree\n");
			goto err_platform_data_null;
		}
	} else {
		pdata = client->dev.platform_data;

	if (!pdata) {
		pr_err("[PS][CM36283 error]%s: Assign platform_data error!!\n", __func__);
 		ret = -EBUSY;
		goto err_platform_data_null;
		}
	}

	
	//pr_err("%s: zxtest CM36283  present test!\n", __func__);
	
    	status = i2c_smbus_read_word_data(client, ID_REG);
	if (status < 0) {
		pr_err("[PS_ERR][CM36283 error]%s: CM36283 is not present!\n", __func__);
		cm36283_is_no_present=1;
		ret = -ENODEV;
		goto err_platform_data_null;
	}

	//dev_err(&client->dev, "zxtest select pinctrl state\n");
	ret = pinctrl_select_state( devm_pinctrl_get(&client->dev), pinctrl_lookup_state(devm_pinctrl_get(&client->dev), "default"));
		if (ret) {
			dev_err(&client->dev, "Can't select pinctrl state\n");
		}
	
//{
//	int gp36value =
//		gpio_get_value(pdata->intr);
//
//	D("[PS][CM36283] %s: reset pin pdata->intr %d = %d", __func__,pdata->intr,gp36value);
//}


/******* modified start  *******/
	lpi->irq = client->irq;
	// lpi->irq =  gpio_to_irq(pdata->intr);
	D("[PS][CM36283] %s: lpi->irq = %d\n",
					__func__, lpi->irq);
	i2c_set_clientdata(client, lpi);
	
    lpi->intr_pin = pdata->intr;
	lpi->adc_table = pdata->levels;
	lpi->power = pdata->power;
	lpi->slave_addr = pdata->slave_addr;
	lpi->ps_away_thd_set = pdata->ps_away_thd_set;
	lpi->ps_close_thd_set = pdata->ps_close_thd_set;	
	//printk("zxtest pdata->ps_close_thd_set= %u \n",lpi->ps_close_thd_set);

	lpi->ps_conf1_val = pdata->ps_conf1_val;
	lpi->ps_conf3_val = pdata->ps_conf3_val;
	lpi->status_calling = 0;
	lpi->ls_cmd  = pdata->ls_cmd;
	lpi->record_clear_int_fail=0;

	D("[PS][CM36283] %s: ls_cmd 0x%x\n", __func__, lpi->ls_cmd);

	if (pdata->ls_cmd == 0) {
		lpi->ls_cmd  = CM36283_ALS_IT_160ms | CM36283_ALS_GAIN_2;

	}

	lp_info_cm36283 = lpi;

	mutex_init(&CM36283_control_mutex);
	mutex_init(&als_enable_mutex);
	mutex_init(&als_disable_mutex);
	mutex_init(&als_get_adc_mutex);

	ret = lightsensor_setup(lpi);

	if (ret < 0) {
		pr_err("[LS][CM36283 error]%s: lightsensor_setup error!!\n", __func__);
		goto err_lightsensor_setup;
	}

	mutex_init(&ps_enable_mutex);
	mutex_init(&ps_disable_mutex);
	mutex_init(&ps_get_adc_mutex);

	ret = psensor_setup(lpi);

	if (ret < 0) {
		pr_err("[PS][CM36283 error]%s: psensor_setup error!!\n", __func__);
		goto err_psensor_setup;
	}


  //SET LUX STEP FACTOR HERE
  // if adc raw value one step = 5/100 = 1/20 = 0.05 lux
  // the following will set the factor 0.05 = 1/20
  // and lpi->golden_adc = 1;  
  // set als_kadc_cm36283 = (ALS_CALIBRATED <<16) | 20;

   als_kadc_cm36283 = (ALS_CALIBRATED <<16) | 20;
   lpi->golden_adc = 1;
  //ls calibrate always set to 1 
   lpi->ls_calibrate = 1;
   lightsensor_set_kvalue_cm36283(lpi);

   ret = lightsensor_update_table(lpi);

	if (ret < 0) {
 		pr_err("[LS][CM36283 error]%s: update ls table fail\n", __func__);
		goto err_lightsensor_update_table;
	}

	lpi->lp_wq = create_singlethread_workqueue("CM36283_wq");

	if (!lpi->lp_wq) {
		pr_err("[PS][CM36283 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

	wake_lock_init(&(lpi->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");

	ret = cm36283_power_set(lpi, true);
	if (ret < 0) {
		dev_err(&client->dev, "%s:cm36283 power on error!\n", __func__);
		goto err_cm36283_power_on;
	}


	ret = CM36283_setup(lpi);

	if (ret < 0) {
		pr_err("[PS_ERR][CM36283 error]%s: CM36283_setup error!\n", __func__);
		goto err_CM36283_setup;
	}

	lpi->CM36283_class = class_create(THIS_MODULE, "optical_sensors");

	if (IS_ERR(lpi->CM36283_class)) {
		ret = PTR_ERR(lpi->CM36283_class);
		lpi->CM36283_class = NULL;
		goto err_create_class;
	}

	lpi->ls_dev = device_create (lpi->CM36283_class,
                                  NULL,
                                  0,
                                  "%s", 
                                  "lightsensor");

	if (unlikely(IS_ERR(lpi->ls_dev))) {
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_ls_device;
	}

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_adc);

	if (ret)
		goto err_create_ls_device_file;

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_auto);

	if (ret)
		goto err_create_ls_device_file;

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_kadc);

	if (ret)
		goto err_create_ls_device_file;

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_gadc);

	if (ret)
		goto err_create_ls_device_file;

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_adc_table);

	if (ret)
		goto err_create_ls_device_file;

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_conf);

	if (ret)
		goto err_create_ls_device_file;

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_flevel);

	if (ret)
		goto err_create_ls_device_file;

	lpi->ps_dev = device_create( lpi->CM36283_class,
                                 NULL,
                                  0, 
                                  "%s", 
                                  "proximity");

	if (unlikely(IS_ERR(lpi->ps_dev))) {
 		  ret = PTR_ERR(lpi->ps_dev);
          lpi->ps_dev = NULL;
          goto err_create_ls_device_file;
	}

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_adc);

	if (ret)
		goto err_create_ps_device;

	ret = device_create_file( lpi->ps_dev, &dev_attr_ps_parameters);

	if (ret)
		goto err_create_ps_device;

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_conf);

	if (ret)
		goto err_create_ps_device;

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_thd);

	if (ret)
		goto err_create_ps_device;
	
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_status);//annatest
	if (ret)
		goto err_create_ps_device;

		/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_cm36283_dump);

	if (ret)
		goto err_create_ps_device;

	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_hw);

	if (ret)
  	    goto err_create_ps_device;

	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_calling);

	if (ret)
		goto err_create_ps_device;

#ifdef DEBUG_VEMMC2
    vemmc2_userCtrl_class = class_create(THIS_MODULE, "vemmc2_userCtrl_dev");
    vemmc2_userCtrl_dev = device_create(vemmc2_userCtrl_class, NULL, 0, "%s", "vemmc2_userCtrl");
 	ret = device_create_file(vemmc2_userCtrl_dev, &dev_attr_vemmc2_ctrl);

    if(ret){
         device_unregister(vemmc2_userCtrl_dev);
    }
/*
    gpio_userCtrl_class = class_create(THIS_MODULE, "gpio_userCtrl_dev");
    gpio_userCtrl_dev = device_create(gpio_userCtrl_class, NULL, 0, "%s", "gpio_userCtrl");
        
    ret = device_create_file(gpio_userCtrl_dev, &dev_attr_gpio_ctrl);

    if(ret){
         device_unregister(gpio_userCtrl_dev);
    }
    */
/*	
	vemmc2_reg = regulator_get(vemmc2_userCtrl_dev, "vemmc2");     
	if (IS_ERR(vemmc2_reg)) { 
	    D(" VEMMC2_cm32863 ---regulator get fail !!!\n");
	}
	int reg_err;
	uint8_t reg;
*/
/*	
    reg_err = intel_msic_reg_read(VEMMC2CNT_ADDR, &reg);
	if (reg_err){
              D(" VEMMC2_cm32863 ---regulator read VEMMC2CNT_ADDR fail !!!\n");
	}else{
	     D(" VEMMC2_cm32863 ---regulator read VEMMC2CNT_ADDR = %d\n", reg);
	}

	reg_err = intel_msic_reg_write(VEMMC2CNT_ADDR, (reg|0x07));

   	if (reg_err){
	     D(" VEMMC2_cm32863 ---regulator write VEMMC2CNT_ADDR to set NORMAL mode fail !!!\n");
	}

	reg = 0;
    reg_err = intel_msic_reg_read(VEMMC2CNT_ADDR, &reg);

	if (!reg_err){
	     D(" VEMMC2_cm32863 ---regulator read VEMMC2CNT_ADDR final = %d 2!!!\n",reg);
	}
*/
/*
    reg_err = regulator_set_mode(vemmc2_reg, 0x02);
	
    if(!reg_err)
	     D("VEMMC2_cm32863 ---regulator setmode success !!\n");
    else 
     	D("VEMMC2_cm32863 ---regulator setmode fail !!\n");
*/
#endif //DEBUG_VEMMC2
//	lpi->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
//	lpi->early_suspend.suspend = CM36283_early_suspend;
//	lpi->early_suspend.resume = CM36283_late_resume;
//	register_early_suspend(&lpi->early_suspend);

	D("[PS][CM36283] %s: Probe success!\n", __func__);
	//<-- ASUS-Bevis_chen + -->
#ifdef CONFIG_ASUS_FACTORY_SENSOR_MODE
	if(create_asusproc_lpsensor_status_entry( ))
		printk("[%s] : ERROR to create lpsensor proc entry\n",__func__);

//<ASUS-<asus-annacheng20150129>>>>>>>>>>>>>>+
	if(create_asusproc_lightsensor_status_entry( ))
		printk("[%s] : ERROR to create lightsensor proc entry\n",__func__);

	if(create_asusproc_Proximitysensor_status_entry( ))
		printk("[%s] : ERROR to create Proximitysensor proc entry\n",__func__);
//<ASUS-<asus-annacheng20150129><<<<<<<<<<<<+
	
#endif
	//<-- ASUS-Bevis_chen - -->

	return ret;

err_create_ps_device:
	device_unregister(lpi->ps_dev);

err_create_ls_device_file:
	device_unregister(lpi->ls_dev);

err_create_ls_device:
	class_destroy(lpi->CM36283_class);

err_create_class:

err_CM36283_setup:
	cm36283_power_set(lpi, false);
err_cm36283_power_on:
	destroy_workqueue(lpi->lp_wq);
	wake_lock_destroy(&(lpi->ps_wake_lock));

	input_unregister_device(lpi->ls_input_dev);
	input_free_device(lpi->ls_input_dev);
	input_unregister_device(lpi->ps_input_dev);
	input_free_device(lpi->ps_input_dev);

err_create_singlethread_workqueue:

err_lightsensor_update_table:
	misc_deregister(&psensor_misc_cm36283);

err_psensor_setup:
	mutex_destroy(&CM36283_control_mutex);
	mutex_destroy(&ps_enable_mutex);
	mutex_destroy(&ps_disable_mutex);
	mutex_destroy(&ps_get_adc_mutex);
	misc_deregister(&lightsensor_misc);

err_lightsensor_setup:
	mutex_destroy(&als_enable_mutex);
    mutex_destroy(&als_disable_mutex);
	mutex_destroy(&als_get_adc_mutex);

err_platform_data_null:
	kfree(lpi);
	cm36283_probe_fail = 1; //<asus-wx20150805+>

	return ret;
}
	static int debug_log_count=0;
static int control_and_report( struct CM36283_info *lpi, uint8_t mode, uint16_t param ) {
     int ret=0;
	 uint16_t adc_value = 0;
	 uint16_t ps_data = 0;	
 	 int level = 0, i, val;

     mutex_lock(&CM36283_control_mutex);

    if( mode == CONTROL_ALS ){
	        int als_wr_result;
            if(param){
                lpi->ls_cmd &= CM36283_ALS_SD_MASK;      
            }else{
                lpi->ls_cmd |= CM36283_ALS_SD;
            }


            als_wr_result =_CM36283_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);
        
            if (!als_wr_result){
                 lpi->als_enable=param;
	        }
  }else if( mode == CONTROL_PS ){
              int ps_wr_result;
            if(param){ 
                  lpi->ps_conf1_val &= CM36283_PS_SD_MASK;
                  lpi->ps_conf1_val |= CM36283_PS_INT_IN_AND_OUT;      
            }else{
                  lpi->ps_conf1_val |= CM36283_PS_SD;
                  lpi->ps_conf1_val &= CM36283_PS_INT_MASK;
            }


	        ps_wr_result = _CM36283_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);    

            if (!ps_wr_result){ 
                    lpi->ps_enable=param;
	        }
  }

  if(mode == CONTROL_PS){  
        if( param==1 ){
			//<ASUS-Lotta_Lu-20150603-Add for reduce enable Psensor Time>
		     msleep(20);  
         }
  }

  if(lpi->als_enable){

	          uint16_t ls_low_thd, ls_high_thd;
			  int report_lux;
        if( mode == CONTROL_ALS 
            ||( mode == CONTROL_INT_ISR_REPORT 
               && ((param&INT_FLAG_ALS_IF_L)||(param&INT_FLAG_ALS_IF_H)))){
 
    	  lpi->ls_cmd &= CM36283_ALS_INT_MASK;
    	  ret = _CM36283_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);  
          get_ls_adc_value(&adc_value, 0);

        if( lpi->ls_calibrate ) {
        	for (i = 0; i < 10; i++) {
      	  	     if (adc_value <= (*(lpi->cali_table + i))) {
      		     	level = i;

      			     if (*(lpi->cali_table + i))
      				      break;
      		      }

      		  if ( i == 9) {/*avoid  i = 10, because 'cali_table' of size is 10 */
      			  level = i;
      			  break;
      		  }
      	  }
        }else{
        	  for (i = 0; i < 10; i++) {
            		  if (adc_value <= (*(lpi->adc_table + i))) {
      		              level = i;

      			          if (*(lpi->adc_table + i))
               				  break;
      		          }

            		  if ( i == 9) {/*avoid  i = 10, because 'cali_table' of size is 10 */
               			  level = i;
            			  break;
     		           }
      	       }

    	}



		  if (adc_value > LS_RAWDATA_WINDOW){
       	      ls_low_thd = adc_value - LS_RAWDATA_WINDOW;
		  }else{
		      ls_low_thd = 0;
		  }

		  if (adc_value < 0xFFFF - LS_RAWDATA_WINDOW){
		      ls_high_thd = adc_value + LS_RAWDATA_WINDOW;    
		  }else{
		      ls_high_thd = 0xFFFF -1 ;
		  }
/*		  
    	  ret = set_lsensor_range(((i == 0) || (adc_value == 0)) ? 0 :
    		   	*(lpi->cali_table + (i - 1)) + 1,
    		    *(lpi->cali_table + i));
 */
//////// modified to increase sensitivity of lux interrupt 
           ret = set_lsensor_range(ls_low_thd,ls_high_thd);
/////////////////////end modified ///////////////
           lpi->ls_cmd |= CM36283_ALS_INT_EN;
           ret = _CM36283_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);  

	 if (debug_log_count > 10){
		debug_log_count = 0;
		
           if ((i == 0) || (adc_value == 0))
    			D("[LS][CM36283] %s: ADC=0x%03X, Level=%d, l_thd equal 0, h_thd = 0x%x \n", __func__, adc_value, level, *(lpi->cali_table + i));
    		else
    			D("[LS][CM36283] %s: ADC=0x%03X, Level=%d, l_thd = 0x%x, h_thd = 0x%x \n", __func__, adc_value, level, *(lpi->cali_table + (i - 1)) + 1, *(lpi->cali_table + i));

	  }else{
		
		debug_log_count++;
	  }

    		lpi->current_level = level;
    		lpi->current_adc = adc_value;    
		//<-- ASUS-Bevis_Chen + -->
		    report_lux = adc_value;

			if(enLSensorConfig_flag == 1 ){//calibration enable 

					if( LSensor_CALIDATA[0] > 0&&LSensor_CALIDATA[1] > 0 
                       &&LSensor_CALIDATA[1] >LSensor_CALIDATA[0] ){ //in case of zero divisor error

                          //D("CM36283---Before calibration, report_lux is %d\n", report_lux);
                          report_lux = calibration_light(LSensor_CALIDATA[1], LSensor_CALIDATA[0], report_lux);
                          //D("CM36283---After calibration, report_lux is %d\n", report_lux);
                          report_lux = report_lux/sensitivity_x;
			 //             D("CM36283---After devided by 20, report_lux is %d\n", report_lux);
                	}else{
                          printk("%s:ASUS input LSensor_CALIDATA was invalid .error !!!!!\n",__func__);
				    }
         	}else{	
#ifndef CONFIG_ASUS_FACTORY_SENSOR_MODE // in user build and no calibration data			   
//                  report_lux = report_lux*15;
                  D("CM36283--- NO calibration data, use default config\n"); 
#else
                  D("CM36283--- NO calibration data, Factory branch , NO default config\n"); 				  
#endif // end of CONFIG_ASUS_FACTORY_SENSOR_MODE
                  report_lux = report_lux/sensitivity_x*(200/15);
  		  //        D("CM36283---After devided by 20, report_lux is %d\n", report_lux);	
		//<-- ASUS-Bevis_Chen - -->
             }			

        if(mode == CONTROL_ALS){
		    queue_delayed_work(lpi->lp_wq, &light_sensor_initial_value_work, (0.15)*HZ);
		}else{

//<ASUS-danielchan20150624>>>>>>>>>+
        switch (asus_PRJ_ID) {
          //  case 0://ASUS_ZE550KL
          //
          //      break;
  // <asus-jhw20150629+>
			case 1://ASUS_ZE600KL
				if(adc_value<=10){
					report_lux = 0;
				}
				break;
  // <asus-jhw20150629->
          //  case 2://ASUS_ZX550KL
          //
          //      break;
            case 3://ASUS_ZD550KL
                if(report_lux <2) { //cal function will report_lux=1 when adc=0
                    report_lux =0; 
				}  
                break;
            default:
		    if(report_lux %2 == 0) 
			        report_lux += 1;
                break;
        }
//<ASUS-danielchan20150624><<<<<<<<+
			input_report_abs(lpi->ls_input_dev, ABS_MISC,report_lux);
            input_sync(lpi->ls_input_dev);
		}
    }
  }

#define PS_CLOSE 1
#define PS_AWAY  (1<<1)
#define PS_CLOSE_AND_AWAY PS_CLOSE+PS_AWAY

  if(lpi->ps_enable){
        int ps_status = 0;

  if( mode == CONTROL_PS ){
      ps_status = PS_CLOSE_AND_AWAY;   

  }else if(mode == CONTROL_INT_ISR_REPORT ){  
           if ( param & INT_FLAG_PS_IF_CLOSE )
                 ps_status |= PS_CLOSE;      

           if ( param & INT_FLAG_PS_IF_AWAY )
                 ps_status |= PS_AWAY;
  }

   if (ps_status!=0){
         switch(ps_status){
               case PS_CLOSE_AND_AWAY:
			   			//<ASUS-Lotta_Lu-20150603-Add for reduce enable Psensor time>
                        //get_stable_ps_adc_value(&ps_data);
                        get_ps_adc_value(&ps_data);
                        val = (ps_data >= lpi->ps_close_thd_set) ? 0 : 1;
                        D("[PS][CM36283] proximity --ps_status = PS_CLOSE_AND_AWAY\n");
		                mutex_unlock(&CM36283_control_mutex);

                        return ret;
                        break;
           
               case PS_AWAY:
                       val = 1;
					   proximity_int_away = 1; //<asus-wx20150506+>
                       D("[PS][CM36283] proximity --ps_status = PS_AWAY\n");
                       break;

               case PS_CLOSE:
                       val = 0;
                       D("[PS][CM36283] proximity --ps_status = PS_CLOSE\n");
                       break;
        };

	  	//pr_err("[PS][CM36283]ZXTEST proximity %s, val=%d\n", val ? "FAR" : "NEAR", val);	
        input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, val);
		
        input_sync(lpi->ps_input_dev);
		proximity_state = val; //<asus-wx20150506+>
//<anna-cheng>for proximity near close touch in call
	if(proximity_state == 0){
		if (2 == get_audiomode()) {
			ftxxxx_disable_touch(true);
			//pr_err("[PS][CM36283] proximity --anna close  in call \n");
		}else{
			ftxxxx_disable_touch(false);
			//pr_err("[PS][CM36283] proximity --anna close not  in call \n");
		}	
	}else{
		ftxxxx_disable_touch(false);
		//pr_err("[PS][CM36283] proximity --anna far far awayl \n");
	}
//<anna-cheng>for proximity near close touch in call		
   }
 }

  mutex_unlock(&CM36283_control_mutex);
  return ret;
}

static const struct i2c_device_id CM36283_i2c_id[] = {
	{CM36283_I2C_NAME, 0},
	{}
};


/*
static void CM36283_early_suspend(struct early_suspend *h)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	D("[LS][CM36283] %s\n", __func__);

	if (lpi->als_enable){
            lightsensor_disable(lpi);
            lpi->lsensor_sleep_becuz_early_suspend = 1;
	}
}

static void CM36283_late_resume(struct early_suspend *h)
{
	struct CM36283_info *lpi = lp_info_cm36283;
	D("[ALS][CM36283] %s\n", __func__);

	if (lpi->lsensor_sleep_becuz_early_suspend){
		    lightsensor_enable(lpi);
            lpi->lsensor_sleep_becuz_early_suspend = 0;
            D("[LS][CM36283] lightsensor late_resume\n");    
	}
}
*/

static int cm36283_power_set(struct CM36283_info *info, bool on)
{
	int rc;

	if (on) {
		info->vdd = regulator_get(&info->i2c_client->dev, "vdd");
		if (IS_ERR(info->vdd)) {
			rc = PTR_ERR(info->vdd);
			dev_err(&info->i2c_client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			goto err_vdd_get;
		}

		if (regulator_count_voltages(info->vdd) > 0) {
			rc = regulator_set_voltage(info->vdd,
					CM36283_VDD_MIN_UV, CM36283_VDD_MAX_UV);
			if (rc) {
				dev_err(&info->i2c_client->dev,
					"Regulator set failed vdd rc=%d\n", rc);
				goto err_vdd_set_vtg;
			}
		}

		info->vio = regulator_get(&info->i2c_client->dev, "vio");
		if (IS_ERR(info->vio)) {
			rc = PTR_ERR(info->vio);
			dev_err(&info->i2c_client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto err_vio_get;
		}

		if (regulator_count_voltages(info->vio) > 0) {
			rc = regulator_set_voltage(info->vio,
				CM36283_VI2C_MIN_UV, CM36283_VI2C_MAX_UV);
			if (rc) {
				dev_err(&info->i2c_client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto err_vio_set_vtg;
			}
		}

		rc = regulator_enable(info->vdd);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_ena;
		}

		rc = regulator_enable(info->vio);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_ena;
		}

	} else {
		rc = regulator_disable(info->vdd);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}
		if (regulator_count_voltages(info->vdd) > 0)
			regulator_set_voltage(info->vdd, 0, CM36283_VDD_MAX_UV);

		regulator_put(info->vdd);

		rc = regulator_disable(info->vio);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			return rc;
		}
		if (regulator_count_voltages(info->vio) > 0)
			regulator_set_voltage(info->vio, 0,
					CM36283_VI2C_MAX_UV);

		regulator_put(info->vio);
	}

	return 0;

err_vio_ena:
	regulator_disable(info->vdd);
err_vdd_ena:
	if (regulator_count_voltages(info->vio) > 0)
		regulator_set_voltage(info->vio, 0, CM36283_VI2C_MAX_UV);
err_vio_set_vtg:
	regulator_put(info->vio);
err_vio_get:
	if (regulator_count_voltages(info->vdd) > 0)
		regulator_set_voltage(info->vdd, 0, CM36283_VDD_MAX_UV);
err_vdd_set_vtg:
	regulator_put(info->vdd);
err_vdd_get:
	return rc;
}

static int CM36283_suspend(struct device *dev)
{   
    struct CM36283_info *lpi = lp_info_cm36283;
	int status_calling = lpi->ps_enable; //<asus-wx20150429>
   	pr_err("CM32683:  CM36283_suspend\n");
	
	lpi->status_calling = status_calling; //<asus-wx20150429+>
//<asus-wx20150506>->> don't power off when suspend
/*
	if (lpi->ps_enable && !status_calling){
		  pr_err("CM32683:  CM36283_suspend  no.2 \n");
  		  psensor_disable(lpi);
  	      lpi->psensor_sleep_becuz_suspend =1;
		 pr_err("CM32683:  CM36283_suspend  no.3 \n");
//		disable_irq_nosync(lp_info_cm36283->irq);
	}

	if (lpi->als_enable){ //<asus-wx20150429>
            lightsensor_disable(lpi);
            lpi->lsensor_sleep_becuz_early_suspend = 1;
		 pr_err("CM36283 lightsensor suspend\n");    
	}
*/
//<asus-wx20150506>-<< don't power off when suspend
//<asus-wx20150429>+>>
	if (!status_calling) {
//<asus-wx20150506>->> don't power off when suspend
/*
		if (cm36283_power_set(lpi, 0))
			goto out;
*/
//<asus-wx20150506>-<< don't power off when suspend
	}
	else {
		enable_irq_wake(lpi->irq);
	}
//<asus-wx20150429>+<<
	return 0;
//<asus-wx20150506>->> don't power off when suspend
/*
out:
	dev_err(&lpi->i2c_client->dev, "%s:failed during resume operation.\n",
			__func__);
	return -EIO;
*/
//<asus-wx20150506>-<< don't power off when suspend
}

static int CM32683_resume(struct device *dev)
{
	 struct CM36283_info *lpi = lp_info_cm36283;
	int status_calling = lpi->status_calling; //<asus-wx20150429+>
     pr_err("CM36283:  CM32683_resume\n");

//<asus-wx20150429>+>>
	if (!status_calling) {
//<asus-wx20150506>->> don't power off when suspend
/*
		if (cm36283_power_set(lpi, 1))
			goto out;
*/
//<asus-wx20150506>-<< don't power off when suspend
	}
	else {
		if (proximity_state == 0) {
			wake_lock_timeout(&(lpi->ps_wake_lock), 1 * HZ);
		}
		disable_irq_wake(lpi->irq);
	}
//<asus-wx20150429>+<<
//<asus-wx20150506>->> don't power off when suspend
/*
	ls_initial_cmd(lpi);
	if (!status_calling) { //<asus-wx20150429+>
		psensor_initial_cmd(lpi);
	} //<asus-wx20150429+>

	 if (!lpi->ps_enable && lpi->psensor_sleep_becuz_suspend){
	 	 pr_err("CM32683:  CM32683_resume  no.2 \n");
		 psensor_enable(lpi);
		  pr_err("CM32683:  CM32683_resume  no.3 \n");
         lpi->psensor_sleep_becuz_suspend =0;
//		enable_irq(lp_info_cm36283->irq);
    }

	if (!lpi->als_enable && lpi->lsensor_sleep_becuz_early_suspend){
		    lightsensor_enable(lpi);
            lpi->lsensor_sleep_becuz_early_suspend = 0;
            pr_err("[LS][CM36283] lightsensor late_resume\n");    
	}
*/
//<asus-wx20150506>-<< don't power off when suspend	 
	return 0;
//<asus-wx20150506>->> don't power off when suspend
/*
out:
	dev_err(&lpi->i2c_client->dev, "%s:failed during resume operation.\n",
			__func__);
	return -EIO;
*/
//<asus-wx20150506>-<< don't power off when suspend
}

static UNIVERSAL_DEV_PM_OPS(cm36283_pm, CM36283_suspend, CM32683_resume, NULL);

static const struct i2c_device_id cm36283_i2c_id[] = {
	{CM36283_I2C_NAME, 0},
	{}
};

static struct of_device_id cm36283_match_table[] = {
	{ .compatible = "capella,cm36283",},
	{ },
};
static struct i2c_driver CM36283_driver = {
	.id_table = CM36283_i2c_id,
	.probe = CM36283_probe,
	.driver = {
		.name = CM36283_I2C_NAME,
		.owner = THIS_MODULE,
		.pm = &cm36283_pm,
		.of_match_table = cm36283_match_table,
	},
};    

static int __init CM36283_init(void)
{  
    return i2c_add_driver(&CM36283_driver);      
}

static void __exit CM36283_exit(void)
{
//<-- ASUS-Bevis_Chen + -->
#ifdef CONFIG_ASUS_FACTORY_SENSOR_MODE
	if (lpsensor_entry)
		remove_proc_entry("asus_lpsensor_status", NULL);
//<asus-annacheng20150129>>>>>>>>>>>>>>>>>>+	
	if(proximitysensor_entry)
		remove_proc_entry("Proximitysensor_status", NULL);
	if(lightsensor_entry)
		remove_proc_entry("lightsensor_status", NULL);
//<asus-annacheng20150129><<<<<<<<<<<<<<<<<+
#endif
//<-- ASUS-Bevis_Chen - -->

	i2c_del_driver(&CM36283_driver);
}

module_init(CM36283_init);
module_exit(CM36283_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM36283 Driver");

