/* fp9928-hwmon.c
 *
 * Copyright (c) 2011 Foxconn TMSBG Co., Ltd.
 *
 * Temperature Sensor HWmon Driver for Fitipower 9928
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
 
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/mfd/fp9928.h>

/*
 * Client data (each client gets its own)
 */
struct fp9928_hwmon_data {
	struct device *hwmon_dev;
	struct regulator *temp_regulator;
	struct delayed_work temp_read_work;
};

//extern int mxc_i2c_suspended;

/* Global eink temperature variable */
volatile char g_eink_temp = 20;
EXPORT_SYMBOL(g_eink_temp);

static struct fp9928_hwmon_data *fp9928_data;

static int get_eink_temperature(void)
{
	int ret = 0;
	char temp_val = 0;

	ret = regulator_enable(fp9928_data->temp_regulator);
	if (IS_ERR((void *)ret)) {
		pr_err("fp9928_hwmon : Unable to enable DISPLAY regulator."
			"err = 0x%x\n", ret);
		return ret;
	}
	
	msleep(1);
	
	fp9928_i2c_device_read(REG_TMST_VALUE, &temp_val);
	
	regulator_disable(fp9928_data->temp_regulator);
		
	return temp_val;
}

/*
 * Sysfs stuff
 */
static ssize_t show_temperature(struct device *dev,
						 struct device_attribute *attr, char *buf)
{
	int ret = 0;
	char temp_val = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct fp9928_hwmon_data *data = platform_get_drvdata(pdev);

	ret = regulator_enable(data->temp_regulator);
	if (IS_ERR((void *)ret)) {
		dev_err(&pdev->dev, "Unable to enable DISPLAY regulator."
			"err = 0x%x\n", ret);
		return ret;
	}
	
	msleep(1);
	
	fp9928_i2c_device_read(REG_TMST_VALUE, &temp_val);
	
	regulator_disable(data->temp_regulator);
		
	return sprintf(buf, "%d\'C\n", temp_val);
}

static DEVICE_ATTR(temperature, S_IRUGO, show_temperature, NULL);

static struct attribute *fp9928_hwmon_attributes[] = {
	&dev_attr_temperature.attr,
	NULL
};

static const struct attribute_group fp9928_hwmon_group = {
	.attrs = fp9928_hwmon_attributes,
};

static void temp_read_cycle_work(struct work_struct *work)
{
	struct fp9928_hwmon_data *data = container_of(work, 
					struct fp9928_hwmon_data, temp_read_work.work);

	//pr_info("mxc_i2c_suspend is %d\n", mxc_i2c_suspended);
	
	/* i2c bus is suspended */
	//if (mxc_i2c_suspended)
	//	return;
	
	g_eink_temp = get_eink_temperature();
	
	//pr_info("Temperature is %d\'C\n", g_eink_temp);

	schedule_delayed_work(&data->temp_read_work, \
			msecs_to_jiffies(30000));	//cycle time : 30s
	return ;
}

static int fp9928_sensor_probe(struct platform_device *pdev)
{
	struct fp9928_hwmon_data *data;
	int err;

	data = kzalloc(sizeof(struct fp9928_hwmon_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	//INIT_DELAYED_WORK(&data->temp_read_work, temp_read_cycle_work);

	/* Register sysfs hooks */
	err = sysfs_create_group(&pdev->dev.kobj, &fp9928_hwmon_group);
	if (err)
		goto exit_free;

	data->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove_files;
	}

	platform_set_drvdata(pdev, data);
	
	data->temp_regulator = regulator_get(NULL, "DISPLAY");
	if (IS_ERR(data->temp_regulator)) {
		dev_err(&pdev->dev, "Unable to get display PMIC regulator."
			"err = 0x%x\n", (int)data->temp_regulator);
		err = -ENODEV;
		goto exit_remove_files;
	}

	fp9928_data = data;

	//schedule_delayed_work(&data->temp_read_work, msecs_to_jiffies(0));
	
	return 0;

exit_remove_files:
	sysfs_remove_group(&pdev->dev.kobj, &fp9928_hwmon_group);
exit_free:
	kfree(data);
exit:
	return err;
}

static int fp9928_sensor_remove(struct platform_device *pdev)
{
	struct fp9928_hwmon_data *data = platform_get_drvdata(pdev);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&pdev->dev.kobj, &fp9928_hwmon_group);
	regulator_put(data->temp_regulator);

	kfree(data);
	return 0;
}

#ifdef CONFIG_PM
static int fp9928_sensor_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct fp9928_hwmon_data *data = platform_get_drvdata(pdev);

	pr_info("%s\n", __func__);
	
	//cancel_delayed_work_sync(&data->temp_read_work);

	return 0;
}

static int fp9928_sensor_resume(struct platform_device *pdev)
{
	struct fp9928_hwmon_data *data = platform_get_drvdata(pdev);
	
	pr_info("%s\n", __func__);
	
	//schedule_delayed_work(&data->temp_read_work, msecs_to_jiffies(0));	
	
	return 0;
}
#else
#define	fp9928_sensor_suspend	NULL
#define fp9928_sensor_resume	NULL
#endif

static struct platform_driver fp9928_sensor_driver = {
	.probe   = fp9928_sensor_probe,
	.remove  = fp9928_sensor_remove,
	.suspend = fp9928_sensor_suspend,
	.resume  = fp9928_sensor_resume,
	.driver  = {
		.name = "fp9928_sensor",
	},
};

static int __init sensors_fp9928_init(void)
{
	return platform_driver_register(&fp9928_sensor_driver);
}
module_init(sensors_fp9928_init);

static void __exit sensors_fp9928_exit(void)
{
	platform_driver_unregister(&fp9928_sensor_driver);
}
module_exit(sensors_fp9928_exit);

MODULE_DESCRIPTION("fp9928 temperature sensor driver");
MODULE_AUTHOR("Jerry, Luo <luokun237234@gmail.com>");
MODULE_LICENSE("GPL");

