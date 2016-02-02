/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>

struct backlight_chip
{
	struct i2c_client *client;
	u32 bkl_en_gpio;
};

struct backlight_chip *rt4532_chip = NULL;

void rt4532_set(void)
{	
    unsigned char buf[] = {0x2, 0x87};
    i2c_master_send(rt4532_chip->client, buf, 2);
	buf[0] = 0x04;
	buf[1] = 0x8f;
	i2c_master_send(rt4532_chip->client, buf, 2);
    printk(KERN_EMERG "[backlight]rt4532_set\n");
}
EXPORT_SYMBOL(rt4532_set);
void rt4532_clear(void)
{	
    unsigned char buf[] = {0x2, 0x87};
    i2c_master_send(rt4532_chip->client, buf, 2);
	buf[0] = 0x04;
	buf[1] = 0x00;
	i2c_master_send(rt4532_chip->client, buf, 2);
    printk(KERN_EMERG "[backlight]rt4532_clear\n");
}
EXPORT_SYMBOL(rt4532_clear);


/* Backlight IC power on sequence*/
static int rt4532_power_on(void)
{
	int r = 0;
	unsigned char buf[] = {0x4, 0xff};
	dev_err(&rt4532_chip->client->dev, "rt4532_power_on 0\n");

//=============================
	buf[0] = 0x04;
	buf[1] = 0xff;
	r = i2c_master_send(rt4532_chip->client, buf, 2);
	if (r < 0)
		goto err_presence_check;
	dev_err(&rt4532_chip->client->dev, "rt4532_power_on 1\n");

//=============================

	buf[0] = 0x04;
	buf[1] = 0x00;
	r = i2c_master_send(rt4532_chip->client, buf, 2);
	if (r < 0)
		goto err_presence_check;
	dev_err(&rt4532_chip->client->dev, "rt4532_power_on 2\n");
//=============================

	buf[0] = 0x02;
	buf[1] = 0x87;	
	r = i2c_master_send(rt4532_chip->client, buf, 2);
	if (r < 0)
		goto err_presence_check;
	dev_err(&rt4532_chip->client->dev, "rt4532_power_on 3\n");

	buf[0] = 0x04;
	buf[1] = 0x8f;
	r = i2c_master_send(rt4532_chip->client, buf, 2);
	if (r < 0)
		goto err_presence_check;
	dev_err(&rt4532_chip->client->dev, "rt4532_power_on 4\n");

	r = 0;
	dev_err(&rt4532_chip->client->dev, "rt4532_power_on done\n");	
	return r;

err_presence_check:
	dev_err(&rt4532_chip->client->dev, "rt4532_power_on failed\n");	
	r = -ENXIO;
	return r;
}

static int rt4532_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int r = 0;
	u32 bkl_en_gpio = 0;
	u32 bkl_addr = 0;
	struct device_node *np = client->dev.of_node;
	struct backlight_chip *chip = NULL;

	printk("backlight driver probe \n");

	chip = kzalloc(sizeof(struct backlight_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "mem alloc failed\n");
		return -ENOMEM;
	}
	rt4532_chip = chip;
	
	r = of_property_read_u32(np, "reg", &bkl_addr);
	if (r)
		return -EINVAL;
	printk("check i2c slave address: 0x%x \n",bkl_addr);
	client->addr = bkl_addr;
	chip->client = client;
	
	bkl_en_gpio = of_get_named_gpio(np, "qcom,bl-en-gpio", 0);
	if ((!gpio_is_valid(bkl_en_gpio)))
		printk("bkl_en_gpio is not valid! \n");
	printk("check bkl_en_gpio: %d \n",bkl_en_gpio);
	chip->bkl_en_gpio = bkl_en_gpio;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "bkl-pwr probe: need I2C_FUNC_I2C\n");
		return -ENODEV;
	}

	r = rt4532_power_on();

	return 0;
}

static int rt4532_remove(struct i2c_client *client)
{

	return 0;
}

static int rt4532_suspend(struct device *device)
{

	return 0;
}

static int rt4532_resume(struct device *device)
{
    printk("[backlight]rt4532_resume\n");
	//rt4532_power_on();
	return 0;
}

static struct of_device_id msm_match_table[] = {
	{.compatible = "qcom,bkl-pwr"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_match_table);

static const struct i2c_device_id rt4532_id[] = {
	{"rt4532-i2c", 0},
	{}
};

static const struct dev_pm_ops bkl_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(rt4532_suspend, rt4532_resume)
};

static struct i2c_driver rt4532 = {
	.id_table = rt4532_id,
	.probe = rt4532_probe,
	.remove = rt4532_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "bkl-pwr",
		.of_match_table = msm_match_table,
		.pm = &bkl_pm_ops,
	},
};

/*
 * module load/unload record keeping
 */
static int __init rt4532_dev_init(void)
{
	return i2c_add_driver(&rt4532);
}
module_init(rt4532_dev_init);

static void __exit rt4532_dev_exit(void)
{
	i2c_del_driver(&rt4532);
}
module_exit(rt4532_dev_exit);

MODULE_DESCRIPTION("BACKLIGHT DRIVER RT4532");
MODULE_LICENSE("GPL v2");

