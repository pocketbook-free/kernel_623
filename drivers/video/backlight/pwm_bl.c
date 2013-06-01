/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include <linux/hw_ops.h>
#include <linux/suspend.h>

#define LIGHT_POWER_OFF gpio_direction_output(169,0)
#define LIGHT_POWER_ON  gpio_direction_output(169,1)
#define FL_EN	(5*32 + 9)

struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
	int			(*notify)(struct device *,
					  int brightness);
};

static int pwm_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;
	int duty = brightness * pb->period / max;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (pb->notify)
		brightness = pb->notify(pb->dev, brightness);

	if (brightness == 0) {
		pwm_config(pb->pwm, 0, pb->period);
		pwm_disable(pb->pwm);
	} else {
		//pwm_config(pb->pwm, brightness * pb->period / max, pb->period);
		pwm_config(pb->pwm, duty, pb->period);
		pwm_enable(pb->pwm);
	}
	//printk("<0>#####period=%d#####\n",pb->period);
	//printk("<0>#####max=%d#####\n",max);
	printk("<0>#####brightness=%d#####\n",brightness);
	return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int pwm_backlight_check_fb(struct fb_info *info)
{
	char *id = info->fix.id;
	if (!strcmp(id, "DISP3 BG"))
	    return 1;
	else
	return 0;
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_update_status,
	.get_brightness	= pwm_backlight_get_brightness,
	.check_fb = pwm_backlight_check_fb,
};

#if 0
static void backlight_generate_event(struct backlight_device *bd,
				     enum backlight_update_reason reason)
{
	char *envp[2];

	switch (reason) {
	case BACKLIGHT_UPDATE_SYSFS:
		envp[0] = "SOURCE=sysfs";
		break;
	case BACKLIGHT_UPDATE_HOTKEY:
		envp[0] = "SOURCE=hotkey";
		break;
	default:
		envp[0] = "SOURCE=unknown";
		break;
	}
	envp[1] = NULL;
	kobject_uevent_env(&bd->dev.kobj, KOBJ_CHANGE, envp);
	sysfs_notify(&bd->dev.kobj, NULL, "actual_brightness");
}

static ssize_t backlight_show_brightness(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int rc = -ENXIO;
	struct backlight_device *bd = to_backlight_device(dev);

	mutex_lock(&bd->ops_lock);
	if (bd->ops && bd->ops->get_brightness)
		rc = sprintf(buf, "%d\n", bd->ops->get_brightness(bd));
	mutex_unlock(&bd->ops_lock);

	return rc;
}

static ssize_t backlight_store_brightness(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct backlight_device *bd = to_backlight_device(dev);
	unsigned long brightness;

	rc = strict_strtoul(buf, 0, &brightness);
	if (rc)
		return rc;

	rc = -ENXIO;

	mutex_lock(&bd->ops_lock);
	if (bd->ops) {
		if (brightness > bd->props.max_brightness)
			rc = -EINVAL;
		else {
			pr_debug("backlight: set brightness to %lu\n",
				 brightness);
			bd->props.brightness = brightness;
			backlight_update_status(bd);
			rc = count;
		}
	}
	mutex_unlock(&bd->ops_lock);

	backlight_generate_event(bd, BACKLIGHT_UPDATE_SYSFS);

	return rc;
}

static ssize_t backlight_switch(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	unsigned long lightswitch;
	rc = strict_strtoul(buf, 0, &lightswitch);
	switch (lightswitch){
	case 1:
	LIGHT_POWER_ON;
	break;
	case 0:
	LIGHT_POWER_OFF;
	break;
	default:
	sprintf(buf, "%s\n", "Input Error!");
	}
	return 0;
}
#endif

static int pwm_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	int ret;

	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = kzalloc(sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	pb->period = data->pwm_period_ns;
	pb->notify = data->notify;
	pb->dev = &pdev->dev;

	pb->pwm = pwm_request(data->pwm_id, "frontlight");
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM for frontlight\n");
		ret = PTR_ERR(pb->pwm);
		goto err_pwm;
	} else
		dev_dbg(&pdev->dev, "got pwm for frontlight\n");

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = data->max_brightness;
	props.brightness = data->dft_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
//	printk("<0>#######bl register#######\n");
	if (IS_ERR(bl)) {
		//printk(&pdev->dev, "failed to register frontlight\n"); // ? WTF
		printk("failed to register frontlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}
	
//	backlight_update_status(bl);
//	LIGHT_POWER_ON;

	printk("<0>######update status######\n");
	
	platform_set_drvdata(pdev, bl);
	return 0;

err_bl:
	pwm_free(pb->pwm);
err_pwm:
	kfree(pb);
err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	pwm_free(pb->pwm);
	kfree(pb);
	if (data->exit)
		data->exit(&pdev->dev);
	LIGHT_POWER_OFF;
	return 0;
}
static int pm_state;
#ifdef CONFIG_PM
static int pwm_backlight_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

        hw_get_deepsuspend(&pm_state);
        if(pm_state == PM_SUSPEND_MEM)
        {
	if (pb->notify)
		pb->notify(pb->dev, 0);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	LIGHT_POWER_OFF;
        }
	return 0;
}

static int pwm_backlight_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

        if(pm_state == PM_SUSPEND_MEM)
        {
		backlight_update_status(bl);
		gpio_request(FL_EN, "pwm-power-on");
//		LIGHT_POWER_ON;
        }
	return 0;
}
#else
#define pwm_backlight_suspend	NULL
#define pwm_backlight_resume	NULL
#endif

static struct platform_driver pwm_backlight_driver = {
	.driver		= {
		.name	= "pwm-frontlight",
		.owner	= THIS_MODULE,
	},
	.probe		= pwm_backlight_probe,
	.remove		= pwm_backlight_remove,
	.suspend	= pwm_backlight_suspend,
	.resume		= pwm_backlight_resume,
};



static int __init pwm_backlight_init(void)
{
	return platform_driver_register(&pwm_backlight_driver);
}
module_init(pwm_backlight_init);

static void __exit pwm_backlight_exit(void)
{
	platform_driver_unregister(&pwm_backlight_driver);
}
module_exit(pwm_backlight_exit);

MODULE_DESCRIPTION("PWM based Frontlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-frontlight");

