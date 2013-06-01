/* fp9928-core.c
 *
 * Copyright (c) 2011 Foxconn TMSBG Co., Ltd.
 *
 * Core file for Fitipower 9928 EPD PMIC I2C driver
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/fp9928.h>

static	struct fp9928_data *fp9928_dev;

struct fp9928_vcom_programming_data {
	int vcom_min_uV;
	int vcom_max_uV;
	int vcom_step_uV;
};

static struct fp9928_vcom_programming_data vcom_data = {
	-4000000,
	-300000,
	10780,
};

/* Convert uV to the VCOM register setting value*/
static inline u8 vcom_uV_to_rs(int uV)
{
	return 28 + (((((vcom_data.vcom_max_uV - uV) * 10) 
					/ vcom_data.vcom_step_uV) + 5) / 10);
}

/* Convert the VCOM register value to uV */
static inline int vcom_rs_to_uV(u8 rs)
{
	return vcom_data.vcom_max_uV - 
		((rs - 28) * vcom_data.vcom_step_uV);
}

int fp9928_i2c_device_read(u8 reg, u8 *dest)
{
	struct i2c_client *client = fp9928_dev->i2c_client;
	int ret;
		
	mutex_lock(&fp9928_dev->rwlock);
	ret = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&fp9928_dev->rwlock);
	if (ret < 0) {
		dev_err(fp9928_dev->dev,
		"Unable to read fp9928 register via I2C:reg=0x%x, dest=%d\n", reg, *dest);
		printk("Unable to read fp9928 register via I2C:reg=0x%x\n", reg);
		return ret;
	}
	
	ret &= 0xff;
	*dest = ret;
	
	return 0;
}
EXPORT_SYMBOL(fp9928_i2c_device_read);

int fp9928_i2c_device_write(u8 reg, u8 value)
{
	struct i2c_client *client = fp9928_dev->i2c_client;
	int ret;

	mutex_lock(&fp9928_dev->rwlock);
	ret = i2c_smbus_write_byte_data(client, reg, value);
	mutex_unlock(&fp9928_dev->rwlock);
	if (ret < 0) {
		dev_err(fp9928_dev->dev,
		"Unable to write fp9928 register via I2C:reg=0x%x, value=%d\n", reg, value);
		printk("Unable to write fp9928 register via I2C:reg=0x%x, value=0x%x\n", reg, value);
		return ret;
	}
	
	return 0;
}
EXPORT_SYMBOL(fp9928_i2c_device_write);
/*
static ssize_t show_temperature(struct device *dev,
						 struct device_attribute *attr, char *buf)
{
	char temp_val = 0;
	
	fp9928_i2c_device_read(REG_TMST_VALUE, &temp_val);
	
	return sprintf(buf, "%d\'C\n", temp_val);
}

static DEVICE_ATTR(temp, S_IRUGO, show_temperature, NULL);
*/
static ssize_t set_vcom_voltage(struct device *dev,
						 struct device_attribute *attr, 
						 const char *buf, size_t count)
{
	long vol_val = simple_strtol(buf, NULL, 10);
	u8 reg_val;
	
	//pr_info("Input voltage is %ldmV\n", vol_val);
	
	if ((vol_val * 1000 < vcom_data.vcom_min_uV)
		|| (vol_val * 1000 > vcom_data.vcom_max_uV)) {
		pr_err("Warning : supported VCOM voltage range : -4000mv ~ -300mv\n");
		return -EINVAL;
	}

	if(vol_val >= -2501){
		fp9928_i2c_device_write(REG_FUNC_ADJUST, 0x01);
		reg_val = vcom_uV_to_rs(vol_val * 1000);
//		pr_info("set register raw value : %d\n", reg_val);
		fp9928_i2c_device_write(REG_VCOM_SETTING, reg_val);
	
		msleep(10);

		fp9928_i2c_device_read(REG_VCOM_SETTING, &reg_val);
//		pr_info("read register raw value : %d\n", reg_val);
	}
    else{
		printk("The set value is between -2500mV and -4000mV.\n");
		printk("Aquire vcom voltage from OPIN pin, you can check it by multimeter in this situation.\n");
		fp9928_i2c_device_write(REG_FUNC_ADJUST, 0x00);
	}
	return count;
}

static ssize_t get_vcom_voltage(struct device *dev,
						 struct device_attribute *attr, char *buf)
{
	u8 reg_val;
	int vol_val;
	
	fp9928_i2c_device_read(REG_FUNC_ADJUST, &reg_val);

	if(reg_val == 0x01){
		fp9928_i2c_device_read(REG_VCOM_SETTING, &reg_val);
	
		//pr_info("read register raw value : %d\n", reg_val);
		vol_val = vcom_rs_to_uV(reg_val);	
		
		//pr_info("vol_val value : %d\n", vol_val);
		if (vol_val < vcom_data.vcom_min_uV) {
			vol_val = vcom_data.vcom_min_uV;
		}
		
		return sprintf(buf, "%dmV\n", vol_val / 1000);
	}
	else
		return sprintf(buf,"Depending on OPIN pin's voltage, test the OPIN or VCOM pin\n");
}

static DEVICE_ATTR(vcom_voltage, S_IRUGO | S_IWUSR, get_vcom_voltage, set_vcom_voltage);

static struct attribute *fp9928_attributes[] = {
//	&dev_attr_temp.attr,
	&dev_attr_vcom_voltage.attr,
	NULL
};

static const struct attribute_group fp9928_attr_group = {
	.attrs = fp9928_attributes,
};

static int init_fp9928_reg()
{
	int ret1,ret2;

	ret1=fp9928_i2c_device_write(REG_FUNC_ADJUST, 0x01);
	ret2=fp9928_i2c_device_write(REG_VCOM_SETTING, 0x74);

	if(ret1 | ret2)
		return 1;
	return 0;

}

static int __devinit fp9928_pmic_probe(struct i2c_client *client,
				   				  const struct i2c_device_id *id)
{
	int ret = 0;
	struct fp9928_platform_data *pdata = client->dev.platform_data;
	
	if (!pdata || !pdata->init)
		return -ENODEV;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	};
	
	/* Create the PMIC data structure */
	fp9928_dev = kzalloc(sizeof(struct fp9928_data), GFP_KERNEL);	
	if (fp9928_dev == NULL) {
		dev_err(&client->dev, "failed to allocate driver data!\n");
		kfree(client);
		return -ENOMEM;
	}
	
	/* Initialize the PMIC data structure */
	i2c_set_clientdata(client, fp9928_dev);
	fp9928_dev->dev = &client->dev;
	fp9928_dev->i2c_client = client;
	mutex_init(&fp9928_dev->rwlock);
	
	/* FP9928 regulator initialize */
	ret = pdata->init(fp9928_dev);
	if (ret) {
		dev_err(&client->dev, "failed to initialize regulator!\n");
		goto err;
	}
	
//	ret = init_fp9928_reg();
	if (ret) {
		dev_err(&client->dev, "failed to initialize fp9928_register!\n");
		goto err;
	}
	/* Register sysfs hooks */
	ret = sysfs_create_group(&client->dev.kobj, &fp9928_attr_group);
	if (ret < 0) {
		dev_err(&client->dev, "failed to create sysfs hooks!\n");
		goto err;
	}	
	
	dev_info(&client->dev, "PMIC FP9928 for E-Ink EPD Display!\n");

	return 0;

err:
	i2c_set_clientdata(client, NULL);
	kfree(fp9928_dev);
	return ret;
}

static int __devexit fp9928_pmic_remove(struct i2c_client *client)
{	
	sysfs_remove_group(&client->dev.kobj, &fp9928_attr_group);
	kfree(fp9928_dev);
	return 0;
}
 
#ifdef CONFIG_PM

static int fp9928_pmic_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int fp9928_pmic_resume(struct i2c_client *client)
{
	return 0;
}

#else

#define fp9928_pmic_suspend		NULL
#define fp9928_pmic_resume		NULL

#endif /* CONFIG_PM */ 

static const struct i2c_device_id fp9928_pmic_id[] = {
	{ "fp9928_pmic", 0 },
	{ },
}
MODULE_DEVICE_TABLE(i2c, fp9928_pmic_id);

static struct i2c_driver fp9928_pmic_driver = {
	.driver = {
		.name   = "fp9928-pmic-i2c",
		.owner	= THIS_MODULE,
	},
	.probe    = fp9928_pmic_probe,
	.remove   = __devexit_p(fp9928_pmic_remove),
	.suspend  = fp9928_pmic_suspend,
	.resume   = fp9928_pmic_resume,
	.id_table = fp9928_pmic_id,
};

static int __init fp9928_pmic_init(void)
{
	int ret;

	pr_info("%s\n", __func__);
	ret = i2c_add_driver(&fp9928_pmic_driver);
	if (ret)
		pr_err("Failed to register fp9928 pmic i2c driver: %d\n", ret);

	return ret;
}
subsys_initcall(fp9928_pmic_init);

static void __exit fp9928_pmic_exit(void)
{
	i2c_del_driver(&fp9928_pmic_driver);
}
module_exit(fp9928_pmic_exit);

MODULE_AUTHOR("Jerry, Luo <luokun237234@gmail.com>");
MODULE_DESCRIPTION("fp9928 EPD PMIC I2C driver");
MODULE_LICENSE("GPL");
