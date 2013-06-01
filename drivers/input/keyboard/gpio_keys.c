/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/cdev.h>

#include <linux/device.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#include <linux/hw_ops.h>
#include <linux/suspend.h>

#define GPIOKEY_STATE_IDLE		0
#define GPIOKEY_STATE_DEBOUNCE		1
#define GPIOKEY_STATE_PRESSED		2
#define KEY_MAJOR			170

#define MENU_KEY			99
#define HOME_KEY			100
#define BACKWARD_KEY			102
#define FORWARD_KEY			97 

#define CM_KEYSTATE			107
#define CM_KEY				106

static int device_major = KEY_MAJOR;
struct input_dev *input_report;
static struct class *gpiokeys_class;
int global_disable = 0;
static int state;

struct key_cdev
{
	struct cdev cdev;
	unsigned char value;
};

struct key_cdev *gpiokey_cdev;

struct gpio_button_data {
	struct gpio_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
	int pressed_time;
	bool disabled;
	int gk_state;
	struct device_attribute key_status;
};

struct gpio_button_data *pwrkey_bdata;

struct gpio_keys_drvdata {
	struct input_dev *input;
	struct mutex disable_lock;
	unsigned int n_buttons;
	struct gpio_button_data data[0];
};

int set_key_lock(int lock)
{
	global_disable = lock;
	printk("the global_disable is : %d\n", global_disable);
	return 0;
}

/* sometimes power key event lost when exiting sleep mode,
 * (two up events was sent that was not recognized as keypress if keypress time is short)
 */

static int last_pwr_key_state = 0;
static int last_pwr_key_upup = 0;

/***********add power button to event0*********************/
int pwrkey_event0_report(int state)
{
	struct gpio_keys_button *button = pwrkey_bdata->button;
	struct input_dev *input = pwrkey_bdata->input;
	unsigned int type = button->type ?:EV_KEY;

	int upup = last_pwr_key_upup;

	printk(KERN_NOTICE "Last pwr key state %d upup %d\n", last_pwr_key_state, last_pwr_key_upup);

	last_pwr_key_upup = 0;

	if (upup && !state) {
		printk(KERN_NOTICE "consume UP\n");
		return 0;
	}

	if (!last_pwr_key_state && !state) {
		// up when last state up - send down prior sending up further
		input_event(input, type, button->code, 1);
		input_sync(input);
		printk(KERN_NOTICE "button->code:%d,state:%d\n",button->code, 1);
		last_pwr_key_upup = 1; // mark the state
	}

	input_event(input, type, button->code, state);	
	input_sync(input);
//	printk("button->code:%d,state:%d\n",button->code, state);
	printk(KERN_NOTICE "button->code:%d,state:%d\n",button->code, state);
	last_pwr_key_state = state;

	return 0;
}
/*********************************************************/

static void gpio_keys_report_event(struct gpio_button_data *bdata)
{
	struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state = (gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low;
	switch (bdata->gk_state)
	{
	case GPIOKEY_STATE_IDLE :
		break;

	case GPIOKEY_STATE_DEBOUNCE :
		if(state)
		{
			input_event(input, type, button->code, !!state);
			//printk("%s:the key sate : %d\n", __func__, state);
			input_sync(input);
			bdata->pressed_time = 0;
			bdata->gk_state = GPIOKEY_STATE_PRESSED;
			mod_timer(&bdata->timer, jiffies + msecs_to_jiffies(button->timer_interval));
		}
		else
		{
			bdata->gk_state = GPIOKEY_STATE_IDLE;
		}
		break;
	case GPIOKEY_STATE_PRESSED :
		if(state)
		{
			if (bdata->pressed_time >= button->longpress_time)
			{
				input_event(input, type, button->code, 2);
				//printk("%s:the key sate : %d\n", __func__, state);
				input_sync(input);
				bdata->pressed_time = 0;
			}
			else
			{
				bdata->pressed_time += button->timer_interval;
			}
			mod_timer(&bdata->timer, jiffies + msecs_to_jiffies(button->timer_interval));
		}
		else
		{
			input_event(input, type, button->code, !!state);
			//printk("%s:the key sate : %d\n", __func__, state);
			input_sync(input);
			
			bdata->pressed_time = 0;
			bdata->gk_state = GPIOKEY_STATE_IDLE;
		}
		break;
	default :
		break;
	}
}

static void gpio_keys_work_func(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work);

	gpio_keys_report_event(bdata);
}

static void gpio_keys_timer(unsigned long _data)
{
	struct gpio_button_data *data = (struct gpio_button_data *)_data;

	schedule_work(&data->work);
}

static irqreturn_t gpio_keys_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	struct gpio_keys_button *button = bdata->button;
	int state = 0;
	int ret;

	if(!global_disable)
	{	BUG_ON(irq != gpio_to_irq(button->gpio));

		if (button->debounce_interval)
		{
			if (!gpio_get_value(button->gpio))
			{
				state = gpio_get_value(MENU_KEY) << 0;
				state |= gpio_get_value(HOME_KEY) << 1; 
				state |= gpio_get_value(BACKWARD_KEY) << 2;
				state |= gpio_get_value(FORWARD_KEY) << 3;
				//printk("get key value is 0x%x\n", state);
				ret = hw_set_key(state);
				if(ret != 0)	
				{
					printk("hw_set_key failed\n");
				}
				return IRQ_HANDLED;
			}
			else
			{
				state = gpio_get_value(MENU_KEY) << 0;
				state |= gpio_get_value(HOME_KEY) << 1; 
				state |= gpio_get_value(BACKWARD_KEY) << 2;
				state |= gpio_get_value(FORWARD_KEY) << 3;
				//printk("get key value is 0x%x\n", state);
				ret = hw_set_key(state);
				if(ret != 0)	
				{
					printk("hw_set_key failed\n");
				}
				bdata->gk_state = GPIOKEY_STATE_DEBOUNCE;
				mod_timer(&bdata->timer,
					jiffies + msecs_to_jiffies(button->debounce_interval));
			}
		}
		else
			schedule_work(&bdata->work);
	}
	return IRQ_HANDLED;
}

static ssize_t gpio_keys_show_status(struct device *dev, struct device_attribute *attr, char *buffer)
{
	struct gpio_keys_button *button = container_of(attr->attr.name, struct gpio_keys_button, key_status[0]);
	return snprintf(buffer, PAGE_SIZE, "%d\n", !!((gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low));
}

static ssize_t gpio_keys_store_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        struct gpio_keys_button *button = container_of(attr->attr.name, struct gpio_keys_button, key_status[0]);
        unsigned int type = button->type ?: EV_KEY;

        input_event(input_report, type, button->code, 1);
        input_sync(input_report);
        input_event(input_report, type, button->code, 0);
        input_sync(input_report);

        return count;

}

static int __devinit gpio_keys_setup_key(struct platform_device *pdev,
					 struct gpio_button_data *bdata,
					 struct gpio_keys_button *button)
{
	char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	unsigned long irqflags;
	int irq, error;

	setup_timer(&bdata->timer, gpio_keys_timer, (unsigned long)bdata);
	INIT_WORK(&bdata->work, gpio_keys_work_func);

	error = gpio_request(button->gpio, desc);
	if (error < 0) {
		dev_err(dev, "failed to request GPIO %d, error %d\n",
			button->gpio, error);
		goto fail2;
	}

	error = gpio_direction_input(button->gpio);
	if (error < 0) {
		dev_err(dev, "failed to configure"
			" direction for GPIO %d, error %d\n",
			button->gpio, error);
		goto fail3;
	}

	irq = gpio_to_irq(button->gpio);
	if (irq < 0) {
		error = irq;
		dev_err(dev, "Unable to get irq number for GPIO %d, error %d\n",
			button->gpio, error);
		goto fail3;
	}

	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = request_irq(irq, gpio_keys_isr, irqflags, desc, bdata);
	if (error) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			irq, error);
		goto fail3;
	}

	return 0;

fail3:
	gpio_free(button->gpio);
fail2:
	return error;
}

extern int report_pwrkey(int (*pwrkey_event0_report)(int));

static int __devinit gpio_keys_probe(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata;
	struct device *dev = &pdev->dev;
	struct input_dev *input;
	int i, error;
	int ret = 0;
	int wakeup = 1;

	ret = hw_set_key(0);
	ret = hw_set_keylockctl(set_key_lock);
	ret = report_pwrkey(pwrkey_event0_report);
	if (ret != 0)
	{
		printk("hw_set_keylockctl failed\n");
	}
	ddata = kzalloc(sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		dev_err(dev, "failed to allocate state\n");
		error = -ENOMEM;
		goto fail1;
	}

	ddata->input = input;
	ddata->n_buttons = pdata->nbuttons;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);

	input->name = pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	pwrkey_bdata = &ddata->data[4];

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];
		unsigned int type = button->type ?: EV_KEY;

		bdata->input = input;
		bdata->button = button;
		
		if(button->key_status != NULL)
		{
			bdata->key_status.attr.name = button->key_status;
			bdata->key_status.attr.owner = THIS_MODULE;
			bdata->key_status.attr.mode = 0666;
			bdata->key_status.show = gpio_keys_show_status;
			bdata->key_status.store = gpio_keys_store_status;
		}
		ret = device_create_file(&(pdev->dev), &(bdata->key_status));
		if(ret < 0)
		{
			printk(KERN_WARNING "gpio-keys: failed to add devices for %s\n", button->key_status);
		}
		
		if(button->code != 5)
		{
			error = gpio_keys_setup_key(pdev, bdata, button);
			if (error)
				goto fail2;
		}

		if (button->wakeup)
			wakeup = 1;

		input_set_capability(input, type, button->code);
	}

	if (error) {
		dev_err(dev, "Unable to export keys/switches, error: %d\n",
			error);
		goto fail2;
	}

	input_report = input;
	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto fail2;
	}

	/* get current state of buttons */
	for (i = 0; i < pdata->nbuttons; i++)
		gpio_keys_report_event(&ddata->data[i]);
	input_sync(input);

	device_init_wakeup(&pdev->dev, wakeup);

	return 0;

 fail2:
	while (--i >= 0) {
		free_irq(gpio_to_irq(pdata->buttons[i].gpio), &ddata->data[i]);
		if (pdata->buttons[i].debounce_interval)
			del_timer_sync(&ddata->data[i].timer);
		cancel_work_sync(&ddata->data[i].work);
		gpio_free(pdata->buttons[i].gpio);
	}

	platform_set_drvdata(pdev, NULL);
 fail1:
	input_free_device(input);
	kfree(ddata);

	return error;
}

static int __devexit gpio_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < pdata->nbuttons; i++) {
		int irq = gpio_to_irq(pdata->buttons[i].gpio);
		free_irq(irq, &ddata->data[i]);
		if (pdata->buttons[i].debounce_interval)
			del_timer_sync(&ddata->data[i].timer);
		cancel_work_sync(&ddata->data[i].work);
		gpio_free(pdata->buttons[i].gpio);
	}

	input_unregister_device(input);

	return 0;
}


#ifdef CONFIG_PM
static int gpio_keys_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	int i;
	
	hw_get_deepsuspend(&state);
	if(state == PM_SUSPEND_STANDBY)
	{
		if (device_may_wakeup(&pdev->dev)) {
			for (i = 0; i < pdata->nbuttons; i++) {
				struct gpio_keys_button *button = &pdata->buttons[i];
				if (button->wakeup) {
					int irq = gpio_to_irq(button->gpio);
					enable_irq_wake(irq);
				}
			}
		}
		
	}
/*
	if (device_may_wakeup(&pdev->dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct gpio_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) {
				int irq = gpio_to_irq(button->gpio);
				enable_irq_wake(irq);
			}
		}
	}
*/

	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	int i;

	if(state == PM_SUSPEND_STANDBY)
	{
		for (i = 0; i < pdata->nbuttons; i++) {

			struct gpio_keys_button *button = &pdata->buttons[i];
			if (button->wakeup && device_may_wakeup(&pdev->dev)) {
				int irq = gpio_to_irq(button->gpio);
				disable_irq_wake(irq);
			}
		}
	}
	input_sync(ddata->input);

	return 0;
}

static const struct dev_pm_ops gpio_keys_pm_ops = {
	.suspend	= gpio_keys_suspend,
	.resume		= gpio_keys_resume,
};
#endif

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= __devexit_p(gpio_keys_remove),
	.driver		= {
		.name	= "gpio-keys",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &gpio_keys_pm_ops,
#endif
	}
};

int gpiokey_open(struct inode *inode, struct file *filep)
{
	return 0;
}

static int gpiokey_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned char keystate = 0x00;
	switch(cmd)
	{
		case CM_KEYSTATE:
			keystate |= gpio_get_value(MENU_KEY) << 0;
			keystate |= gpio_get_value(HOME_KEY) << 1;
			keystate |= gpio_get_value(BACKWARD_KEY) << 2;
			keystate |= gpio_get_value(FORWARD_KEY) << 3;
			put_user(keystate, (int *)arg);
			printk("the value of keystate : %d\n", keystate);
			break;
		default : 
			break;
			
			
	}
	return 0;
}

static struct file_operations gpiokey_fops = 
{
	.owner 	= THIS_MODULE,
	.open  	= gpiokey_open,
	.ioctl	= gpiokey_ioctl,
};

static void setup_cdev(struct key_cdev *dev, int index)
{
	int err, ret;
	dev_t devno = MKDEV(device_major, index);
	
	cdev_init(&dev->cdev, &gpiokey_fops);

	dev->cdev.owner = THIS_MODULE;

	err = cdev_add(&dev->cdev, devno, 1);
	if(err != 0)
	{
		printk(KERN_NOTICE "Err %d adding gpio-keys cdev\n", err);
	}

	gpiokeys_class = class_create(THIS_MODULE, "gpio-keys");
	if(IS_ERR(gpiokeys_class))
	{
		printk("create class error\n");
	}

	ret = device_create(gpiokeys_class, NULL, devno, NULL, "gpio-keys");
	if(!ret)	
	{
		printk("Unable to create class device\n");
	}
}
static int __init gpio_keys_init(void)
{
	int ret = 0;
	dev_t devno = MKDEV(device_major, 0);
	printk("%s", __func__);
	
	gpiokey_cdev = kmalloc(sizeof(struct cdev), GFP_KERNEL); 
	if(!gpiokey_cdev)
	{
		printk("key_cdev kmalloc mem failed\n");
	}
	//dev_t devno = MKDEV(device_major, 0);
	if(device_major)
	{
		ret = register_chrdev_region(devno, 1, "gpio-keys");
	}
	else
	{
		ret = alloc_chrdev_region(&devno, 0, 1, "gpio-keys");
		device_major = MAJOR(devno);
	}
	if(ret < 0)
	{
		printk("Unable to register char device gpio-keys\n");
	}
	setup_cdev(gpiokey_cdev, 0);

	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

module_init(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for CPU GPIOs");
MODULE_ALIAS("platform:gpio-keys");
