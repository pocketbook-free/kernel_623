
#include <linux/module.h>

#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/bcd.h>
#include <linux/slab.h>
#include <mach/hardware.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/mach/time.h>

#include <linux/device.h>
#include <linux/err.h> 
#include <linux/delay.h>
#include <linux/cdev.h>
#include <asm/system.h>

#include <linux/io.h>
#include <linux/sched.h>
#include <linux/hw_ops.h>

#include "hw_state.h"
#include "hw_ioctl.h"


#define EPIO_NAME  "asd"


static int epio_major = 246;
dev_t devno_a;

struct hardware_state_dev hw_state_dev;

struct epio_dev_t
{
	struct cdev cdev; 
};
struct epio_dev_t epio_dev;

enum epio_cmd
{
	
	CM_UPDATE_FW,

	CMD_END
};

int epio_open(struct inode *inode, struct file *filep)
{
     //filep->private_data = gp_ioc->client ;

     filep->private_data = & epio_dev.cdev ;
     return 0;
}

int epio_release(struct inode *inode, struct file *filep)
{
      return 0 ;
}



static int epio_ioctl(struct inode *inode, struct file *filp, \
					unsigned int cmd, unsigned long *arg)
{
	int temp,cp_size;
	char *platform_name = NULL;
	unsigned long long hwconfig;
	int ret = 0;
	switch(cmd)
	{
        case CM_SETLED:	
//			printk("the arg value is %d\n",arg);
			cp_size = copy_from_user(&temp,(int *)arg,sizeof(int));
//			printk("the temp value is %d\n",temp);
//			hw_led_onoff(temp);
			hw_led_set_usermode(temp);
			break ;
/*		case CM_KEY:
			hw_get_key(&temp);
//			printk("the key temp is %d\n",temp);
			cp_size = copy_to_user((int*)arg, &temp, sizeof(int));
			break ;
		case CM_USB_STATUS:
			hw_get_usb(&temp);
			cp_size = copy_to_user((int*)arg, &temp, sizeof(int));
			break ;
		case CM_POWER_BTN:
			hw_get_pwbtn(&temp);
			cp_size = copy_to_user((int*)arg, &temp, sizeof(int));
			break ;*/
		case CM_SD_STATUS:
			hw_get_sd(&temp);
			cp_size = copy_to_user((int*)arg, &temp, sizeof(int));
			break ;
/*		case CM_AC_IN:
			hw_get_ac(&temp);
			cp_size = copy_to_user((int*)arg, &temp, sizeof(int));
			break ;
		case CM_DEEP_SUSPEND:
			cp_size = copy_from_user(&temp, (int *)arg, sizeof(int));
			hw_set_deepsuspend_level(temp);
			break ;*/
		case CM_KEY_LOCK:
			cp_size = copy_from_user(&temp,(int *)arg,sizeof(int));
//			printk("the temp value is %d\n",temp);
			hw_key_lock(temp);
			hw_ts_lock(temp);
			break ;
/*		case CM_BATTERY:
			hw_get_batt(&temp);
			cp_size = copy_to_user((int*)arg, &temp, sizeof(int));
			break ;
		case CM_ALARM_WAKEUP:
			cp_size = copy_from_user(&temp,(int*)arg, sizeof(int));
			hw_set_alrwkup(temp);
			break ;*/
		case CM_PLATFORM:
			platform_name = (char *)kzalloc(4 * sizeof(char), GFP_KERNEL);
			hw_set_platform("ep7a");
			hw_get_platform(platform_name);
			cp_size = copy_to_user((char *)arg, platform_name, 4 * sizeof(char));
			if (cp_size < 0)
				return cp_size;
			break;
		case CM_HWCONFIG:
			hw_get_config(&hwconfig);
			cp_size = copy_to_user((int*)arg, &hwconfig, sizeof(long long));
			if (cp_size < 0)
				return cp_size;
			break;
/*		case CM_HEADPHONE_STATUS:
			hw_get_hp(&temp);
			cp_size = copy_to_user((int*)arg, &temp, sizeof(int));
			break;
		case CM_WIFI_STATUS:
			hw_get_wfpw(&temp);
			cp_size = copy_to_user((int*)arg, &temp, sizeof(int));
			break;
		case CM_POWEROFF:
			printk("device is poweroff...\n");
			hw_poweroff();
			ret = -1;
			return ret;
			break;
		case CM_REBOOT:
			printk("device is reboot...\n");
			hw_reboot();
			ret = -1;
			return ret;
			break;
		case CM_WATCHDOG:
			cp_size = copy_from_user(&temp,(int *)arg,sizeof(int));
			ret = hw_set_wdt(temp);
            if(ret<0)
				return -1;
			break;
		case CM_LOW_BATT:
			hw_get_low_batt(&temp);
			cp_size = copy_to_user((int*)arg, &temp, sizeof(int));
			break;*/
		default:
			return -EINVAL;
	}

	return ret;
}





static const struct file_operations epio_fops = {
	.owner 		= THIS_MODULE,
	.open		  = epio_open,
	.release	= epio_release,
	.ioctl		= epio_ioctl,
};

static int __init epio_init(void)
{
//	int h_cfg ;
	int res;
	struct class *asd_class;	
	printk("%s\n", __FUNCTION__);	
	

	
	if(epio_major)
	{
		printk("epio_major = %d *************\n", epio_major) ;
		
		devno_a = MKDEV(epio_major, 0) ;
		res = register_chrdev_region(devno_a, 1, EPIO_NAME);
	}
	else
	{
		res = alloc_chrdev_region(&devno_a, 0, 1, EPIO_NAME);
		epio_major = MAJOR(devno_a);
	}

	if(res)
	{
		printk("register chrdev region fail.\n") ;
		goto out_unreg_chrdev_region ;
	}

	cdev_init(&epio_dev.cdev, &epio_fops);
	
	res = cdev_add(&epio_dev.cdev, devno_a, 1);
        printk("epio_driver:cdev_add:res=%d\n",res); 
	if (res)
	{
		printk("cdev_add fail.\n") ;
		//goto out ;
	}
	
	asd_class = class_create(THIS_MODULE,EPIO_NAME);

	if(IS_ERR(asd_class))
	{
		printk("create the asd_class failed\n");
		return -1;
	}

	device_create(asd_class, NULL, devno_a, NULL, EPIO_NAME);
	printk("register the asd device OK\n");
	hw_state_dev.led_usermode=1;

out_unreg_chrdev_region: 
		unregister_chrdev_region(devno_a, 1) ;

	return 0 ;

	
}

static void __exit epio_exit(void)
{
	printk("%s\n", __FUNCTION__);
	
	unregister_chrdev_region(devno_a, 1) ;
	cdev_del(&epio_dev.cdev);
}

//module_init(epio_init);
fs_initcall(epio_init);//for the pmic_battery.c file use the hw_get_usb_DpDn() func
module_exit(epio_exit);

MODULE_AUTHOR("Jacky Zhuo<TMSBG/CEN/FOXCONN>");
MODULE_DESCRIPTION("epio Driver");
MODULE_LICENSE("GPL");
