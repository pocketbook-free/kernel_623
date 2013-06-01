/* 
   drivers/mxc/pmic/mc13892/pmic_leds.c 
*/
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <asm/mach-types.h>
#include <linux/pmic_external.h>
#include <linux/workqueue.h>
#include <linux/hw_ops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <mach/iomux-mx50.h>
#include <mach/iomux-v3.h>

#define DEBUG
#include <linux/platform_device.h>
#include <linux/pmic_status.h>

MODULE_LICENSE("Dual BSD/GPL");

#define LED_ON	1
#define LED_OFF	0
#define LED_MAJOR	0
#define LED_NAME	"pmic_leds"
//#define CHRG_Indicator	(5*32 + 13)	/* GPIO_6_13 EP7*/
#define CHRG_Indicator	(5*32 + 11)	/* GPIO_6_11 EP7A*/
#define LEDKP_LSH	9
#define LEDKP_WID	3
#define LEDKPDC_LSH	3
#define LEDKPDC_WID	6

#define WIFI_CMD	(4*32 + 7)
#define WIFI_CLK	(4*32 + 6)
#define WIFI_DAT0	(4*32 + 8)
#define WIFI_DAT1	(4*32 + 9)
#define WIFI_DAT2	(4*32 + 10)
#define WIFI_DAT3	(4*32 + 11)
#define WIFI_EN	(3*32 + 5)	/*GPIO_4_5*/

static unsigned int value;
static unsigned int reg_value = 0;
int chgled_en = 0;
int mask;
int state = LED_ON;
static int led_major;
struct workqueue_struct *led_wq;
struct delayed_work led_work;
struct delayed_work idle_work;
int user_setting;

int system_idle(void)
{
	printk("%s\n",__func__);
	hw_set_deepsuspend_level(1);
//	hw_set_deepsuspend(1);
//	schedule_delayed_work(&idle_work, 2*HZ);
}
	

int pmic_set_led_on(void)
{
	user_setting = hw_led_get_usermode();
	if(user_setting == 0)
	{
  		return 0;
  	}
 	reg_value = BITFVAL(LEDKP, 2);
 	mask = BITFMASK(LEDKP);
 	CHECK_ERROR(pmic_write_reg(REG_LED_CTL1, reg_value, mask));
 	pmic_read_reg(REG_LED_CTL1, &value, BITFMASK(LEDKP));
  	chgled_en = BITFEXT(value, LEDKP);
    
  	reg_value = BITFVAL(LEDKPDC, 32);
	mask = BITFMASK(LEDKPDC);
 	CHECK_ERROR(pmic_write_reg(REG_LED_CTL1, reg_value, mask));
   	pmic_read_reg(REG_LED_CTL1, &value, BITFMASK(LEDKPDC));
 	chgled_en = BITFEXT(value, LEDKPDC);
	//printk("%s\n",__FUNCTION__);
	return 0;
}
    
int pmic_set_led_off(void)
{
  	reg_value = BITFVAL(LEDKP, 2);
	mask = BITFMASK(LEDKP);
	CHECK_ERROR(pmic_write_reg(REG_LED_CTL1, reg_value, mask));
  	pmic_read_reg(REG_LED_CTL1, &value, BITFMASK(LEDKP));
	chgled_en = BITFEXT(value, LEDKP);
     
  	reg_value = BITFVAL(LEDKPDC, 0);
	mask = BITFMASK(LEDKPDC);
	CHECK_ERROR(pmic_write_reg(REG_LED_CTL1, reg_value, mask));
  	pmic_read_reg(REG_LED_CTL1, &value, BITFMASK(LEDKPDC));
	chgled_en = BITFEXT(value, LEDKPDC);
	//printk("%s\n",__FUNCTION__);
	return 0;
}
    
int pmic_set_led_flash(void)
{
	//printk("%s\n",__FUNCTION__);
	reg_value = BITFVAL(LEDKP, 2);
	mask = BITFMASK(LEDKP);
	CHECK_ERROR(pmic_write_reg(REG_LED_CTL1, reg_value, mask));
	pmic_read_reg(REG_LED_CTL1, &value, BITFMASK(LEDKP));
	chgled_en = BITFEXT(value, LEDKP);

	reg_value = BITFVAL(LEDKPDC, 16);
	mask = BITFMASK(LEDKPDC);
	CHECK_ERROR(pmic_write_reg(REG_LED_CTL1, reg_value, mask));
	pmic_read_reg(REG_LED_CTL1, &value, BITFMASK(LEDKPDC));
	chgled_en = BITFEXT(value, LEDKPDC);
	return 0;
}

int gpio_set_led_on(void)
{
	//printk("%s\n",__FUNCTION__);
	gpio_request(CHRG_Indicator, "chrg_indiator");
	gpio_direction_output(CHRG_Indicator, 0);
	return 0;
}

int gpio_set_led_off(void)
{
	printk("%s\n",__func__);
	gpio_request(CHRG_Indicator, "chrg_indiator");
	gpio_direction_output(CHRG_Indicator, 1);
	return 0;
}
		
int led_on(void)
{
	//printk("%s\n",__FUNCTION__);
	user_setting = hw_led_get_usermode();
	if(user_setting)
	{
		pmic_set_led_on();
	}
	return 0;
}

int led_off(void)
{
	//printk("%s\n",__FUNCTION__);
	pmic_set_led_off();
	return 0;
}

int led_flash(void)
{
	//printk("%s\n",__FUNCTION__);
	INIT_DELAYED_WORK(&led_work, led_flash);
	led_wq = create_singlethread_workqueue("pmic_leds");	
	switch(state){
	case LED_ON:
		led_on();
		state = LED_OFF;
		break;
	case LED_OFF:
		led_off();
		state = LED_ON;
		break;
	}
	queue_delayed_work(led_wq, &led_work, HZ);
	
	return 0;
}

int led_close_flash(void)
{
	//printk("%s\n",__FUNCTION__);
	cancel_delayed_work(&led_work);		
	return 0;
}

/*!
 * This function implements the open method on a LED device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @return       This function returns 0.
*/
static int led_open(struct inode *inode, struct file *file)
{
	return 0;
}

/*!
 * This function implements IOCTL controls on a LED device. 
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @param        cmd         the command
 * @param        arg         the parameter
 * @return       This function returns 0 if successful.
 */
static int led_ioctl(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg)
{
   switch(cmd)
   { 
	case LED_ON:
		led_on();
		break;
	case LED_OFF:
		led_off();
		break;	            	            
	default:
		printk("The cmd = %d is wrong, cmd = 0 or 1",cmd);            
   }
	return 0;
}

/* add led_io for interface control led state*/
int led_io(int cmd)
{
   //printk("%s\n",__FUNCTION__);
   switch(cmd)
   { 
	case LED_ON:
		led_on();
		break;
	case LED_OFF:
		led_off();
		break;
	default:
		printk("The cmd = %d is wrong, cmd = 0 or 1",cmd);                        
   }
	return 0;
}

/*!
 * This structure defines file operations for a LED device.
 */
static struct file_operations led_fops = {
	
	.owner = THIS_MODULE,
	.ioctl = led_ioctl,
	.open = led_open,
};

static int led_suspend(struct platform_device *dev, pm_message_t state)
{
	int temp;
	hw_get_ac(&temp);
	if(temp == 0)
	pmic_set_led_off();

	/* close wifi power */
	gpio_direction_output(WIFI_EN, 0);
	mxc_iomux_v3_setup_pad(MX50_PAD_SD2_CMD__GPIO_5_7 | PAD_CTL_PUS_100K_UP);
	gpio_request(WIFI_CMD, "wifi cmd");
	gpio_direction_output(WIFI_CMD,0);
	mxc_iomux_v3_setup_pad(MX50_PAD_SD2_CLK__GPIO_5_6 | PAD_CTL_PUS_100K_UP);
	gpio_request(WIFI_CLK, "wifi clk");
	gpio_direction_output(WIFI_CLK,0);
	mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D0__GPIO_5_8 | PAD_CTL_PUS_100K_UP);
	gpio_request(WIFI_DAT0, "wifi dat0");
	gpio_direction_output(WIFI_DAT0,0);
	mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D1__GPIO_5_9 | PAD_CTL_PUS_100K_UP);
	gpio_request(WIFI_DAT1, "wifi dat1");
	gpio_direction_output(WIFI_DAT1,0);
	mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D2__GPIO_5_10 | PAD_CTL_PUS_100K_UP);
	gpio_request(WIFI_DAT2, "wifi dat2");
	gpio_direction_output(WIFI_DAT2,0);
	mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D3__GPIO_5_11 | PAD_CTL_PUS_100K_UP);
	gpio_request(WIFI_DAT3, "wifi dat3");
	gpio_direction_output(WIFI_DAT3,0);

	return 0;
};

static int led_resume(struct platform_device *pdev)
{
	pmic_set_led_on();

	/* open wifi power */
	gpio_free(WIFI_CMD);
	gpio_free(WIFI_CLK);
	gpio_free(WIFI_DAT0);
	gpio_free(WIFI_DAT1);
	gpio_free(WIFI_DAT2);
	gpio_free(WIFI_DAT3);
	mxc_iomux_v3_setup_pad(MX50_PAD_SD2_CMD__SD2_CMD);
	mxc_iomux_v3_setup_pad(MX50_PAD_SD2_CLK__SD2_CLK);
	mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D0__SD2_D0);
	mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D1__SD2_D1);
	mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D2__SD2_D2);
	mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D3__SD2_D3);
	gpio_direction_output(WIFI_EN, 1);

	return 0;
};

static int led_remove(struct platform_device *pdev)
{
	return 0;
}

static int led_probe(struct platform_device *pdev)
{
	int ret;
	int led_device;
	static struct class *led_class;
	dev_t devno;

	gpio_set_led_off();
	pmic_set_led_on();

	led_major = register_chrdev(LED_MAJOR, LED_NAME, &led_fops);
	if (led_major < 0) 
	{
		printk(KERN_ERR "unable to get a major for led\n");
		return led_major;
	}
	
	led_class = class_create(THIS_MODULE, LED_NAME);
	if (IS_ERR(led_class)) 
	{
		printk(KERN_ERR "Error creating led class.\n");
	}
	//dev_t devno = MKDEV(led_major, 0);  
	devno = MKDEV(led_major, 0);  
	
	led_device = device_create(led_class, NULL, devno, NULL, "pmic_leds");
	if(!led_device)
	{
		printk("Unable to create class device\n");
	}
		
	ret = hw_set_ledctl(led_io);
	if(ret != 0)
			printk("hw_set_ledctl failed\n");
	INIT_DELAYED_WORK(&idle_work, system_idle);
//	schedule_delayed_work(&idle_work, 20*HZ);	
	printk(KERN_INFO "Led Character device: successfully loaded\n");
	return 0;
}

static struct platform_driver led_driver_ldm = {
	.driver = {
		   .name = "pmic_leds",
		  },
	.suspend = led_suspend,
	.resume = led_resume,
	.probe = led_probe,
	.remove = led_remove,
};

static int __init led_init(void)
{	
	pr_debug("PMIC Light driver loading...\n");
	return platform_driver_register(&led_driver_ldm);
}
static void __exit led_exit(void)
{
	unregister_chrdev(led_major, LED_NAME);	
	platform_driver_unregister(&led_driver_ldm);
	pr_debug(KERN_INFO "LED Character device: successfully unloaded\n");	
}

module_init(led_init);
module_exit(led_exit);

MODULE_AUTHOR("Sunny Shang <249898491@qq.com>");
