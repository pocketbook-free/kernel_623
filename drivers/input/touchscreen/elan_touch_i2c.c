/*
 *  ELAN touchscreen driver
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
      
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>

#include <linux/hw_ops.h>
#include "ektf2k.h"
#include <linux/suspend.h>

static const char ELAN_TS_NAME[]	= "elan-touch";

#define ELAN_TS_X_MAX 		1088
#define ELAN_TS_Y_MAX 		768
#define IDX_PACKET_SIZE		8
#define PROC_CMD_LEN		20
#define I2C_MAJOR			117		

#define ELAN_DEBUG	0	

#define TOUCH_POWER_OFF			gpio_direction_output(145, 0)
#define TOUCH_POWER_ON			gpio_direction_output(145, 1)
#define RESETPIN_SET0			gpio_direction_output(175, 0)
#define RESETPIN_SET1			gpio_direction_output(175, 1)
#define RESETPIN_SET_INPUT		gpio_direction_input(175)

//modify for update firmware
#define PACKET_SIZE	9
#define NORMAL_PKT	0x5A

#define CMD_S_PKT		0x52
#define CMD_R_PKT		0x53
#define CMD_W_PKT		0x54
#define HELLO_PKT		0x55

#define IOCTL_I2C_SLAVE			1
#define IOCTL_MAJOR_FW_VER		2
#define IOCTL_MINOR_FW_VER		3
#define IOCTL_RESET			4
#define IOCTL_IAP_MODE_LOCK		5	
#define IOCTL_CHECK_RECOVERY_MODE	6	
#define IOCTL_FW_VER			7	
#define IOCTL_X_RESOLUTION		8	
#define IOCTL_Y_RESOLUTION		9	
#define IOCTL_FW_ID			10
#define IOCTL_IAP_MODE_UNLOCK		11	
#define IOCTL_I2C_INT			12	
#define RESOLUTION_X		1024
#define RESOLUTION_Y		758

uint16_t checksum_err = 0;
uint8_t button_state = 0;
uint8_t RECOVERY = 0x00;
int work_lock = 0x00;
int FW_VERSION = 0x00;
int X_RESOLUTION = 0x00;
int Y_RESOLUTION = 0x00;
int FW_ID = 0x00;

static int state;
static unsigned char Wrbuf_normal[4] 	= {0x54,0x58,0x0a,0x01};
static unsigned char Wrbuf_idle[4] 		= {0x54,0x54,0x00,0x01};
static unsigned char Wrbuf_deepsleep[4] 	= {0x54,0x50,0x00,0x01};

enum {
	hello_packet 		= 0x55,
	idx_coordinate_packet 	= 0x5a,
};

enum {
	idx_finger_state = 7,
};

static struct workqueue_struct *elan_wq;
static struct class *elan_touch_class;

struct elan_data
{
	struct delayed_work work;
	struct i2c_client *client;
	struct input_dev *input;
	int intr_gpio;
	int irq;
	int if_irq;
//Firmware Information
	int fw_ver;
	int fw_id;
	int x_resolution;
	int y_resolution;
//For firmware updata
	struct miscdevice firmware;
};

static struct elan_data *tsdata;
static int locked = 0;
int set_ts_lock(int lock)
{
	int ret = 0;
	if(1 == lock && 0 == locked)
	{
		ret = cancel_delayed_work_sync(&tsdata->work);
		printk("ret = %d\n", ret);
		if(ret == 1)
		{
			enable_irq(tsdata->irq);
		}
		ret = i2c_master_send(tsdata->client, Wrbuf_deepsleep, sizeof(Wrbuf_deepsleep));
		if(ret != 4)
		{
			printk("Unable to write to touchscreen \n");
			return -1;
		}
			disable_irq_nosync(tsdata->irq);
			locked = 1;
	}
	else
		if(0 == lock && 1 == locked)
		{
			ret = i2c_master_send(tsdata->client, Wrbuf_normal, sizeof(Wrbuf_normal));
			if(ret != 4)
			{
				printk("Unable to write to touchscreen \n");
				return -1;
			}		
			enable_irq(tsdata->irq);
			locked = 0;
		}	
	return 0;
}

static int elan_get_version(char *buf)
{
	unsigned char Wrbuf[4] = {0x53, 0x00, 0x00, 0x01};
	unsigned char Rdbuf[4] = {0};
	int ret = 0;
	int x = 0, y = 0;
	
	ret = i2c_master_send(tsdata->client, Wrbuf, sizeof(Wrbuf));
	if(ret != 4)
	{
		printk("elan get version sending command fail\n");
		return -1;
	}
	
	ret = i2c_master_recv(tsdata->client, Rdbuf, sizeof(Rdbuf));
	if(ret != 4)
	{
		printk("elan get version reveive data fail\n");
		return -1;
	}
	//printk("version buf[0x%x 0x%x 0x%x 0x%x]\n",Rdbuf[0], Rdbuf[1], Rdbuf[2], Rdbuf[3]);
	
	y = (Rdbuf[3] & 0xf0) >> 4;
	x = Rdbuf[2] & 0x0f;

	//printk("touch firmware version is %d%d\n", x, y );
	return sprintf(buf, "%d%d\n", x, y );
	
	//return 0;
}

static int elan_set_version(unsigned int version)
{
	return 0;
}

static int elan_get_power_status(char *buf)
{
	int ret;
	unsigned char Wrbuf[4] = {0x53, 0x50, 0x00, 0x01};
	unsigned char Rdbuf[4] = {0};
	int status;
	
	ret = i2c_master_send(tsdata->client, Wrbuf, sizeof(Wrbuf));
	if(ret != 4)
	{
		printk("elan get power_status sending command fail\n");
		return -1;
	}

	ret = i2c_master_recv(tsdata->client, Rdbuf, sizeof(Rdbuf));
	if(ret != 4)
	{
		printk("elan get power_status receive data fail\n");
		return -1;
	}
	printk("power_state buf[0x%x 0x%x 0x%x 0x%x]\n",Rdbuf[0], Rdbuf[1], Rdbuf[2], Rdbuf[3]);

	status = Rdbuf[1] & 0x0c;
	switch(status)
	{
		case 0x08 :
			printk("touchscreen is running in Normal mode\n");
			break;
		case 0x04 :
			printk("touchscreen is running in Idle mode\n");
			break;
		case 0x00 :
			printk("touchscreen is running in Deep sleep mode\n");
			break;
		default :
			break;
	}

	return 0;	
}

static int elan_set_power_status(unsigned int status)
{
	int ret;
	
	printk("the status:%d\n", status);
	switch (status)
	{
		case 0x00 :	//deep sleep mode
			printk("status == 0");
			ret = i2c_master_send(tsdata->client, Wrbuf_deepsleep, sizeof(Wrbuf_deepsleep));
			if(ret != 4)	
			{
				printk("Unable to write to touchscreen \n");
				return -1;
			}
			break;
		case 0x02 :	//idle mode
			ret = i2c_master_send(tsdata->client, Wrbuf_idle, sizeof(Wrbuf_deepsleep));
			if(ret != 4)
			{
				printk("Unable to write to touchscreen\n");
				return -1;
			}
			break;
		case 0x01 :	//normal mode
			ret = i2c_master_send(tsdata->client, Wrbuf_normal, sizeof(Wrbuf_normal));
			if(ret != 4)
			{
				printk("Unable to write to touchscreen\n");
				return -1;
			}
			break;
		default	:
			break;
	}
	
	return 0;
}

static int elan_get_calibration_status(char *buf)
{
	int ret;
	unsigned char Wrbuf[4] = {0x53, 0x20, 0x00, 0x01};
	unsigned char Rdbuf[4] = {0};

	ret = i2c_master_send(tsdata->client, Wrbuf, sizeof(Wrbuf));
	if(ret != 4)
	{
		printk("elan get calibration status sending command fail\n");
		return -1;
	}

	ret = i2c_master_recv(tsdata->client, Rdbuf, sizeof(Rdbuf));
	if(ret != 4)
	{
		printk("elan get calibration status receive data fial\n");
		return -1;
	}
	printk("elan calibration status buf[0x%x 0x%x 0x%x 0x%x]\n", Rdbuf[0], Rdbuf[1], Rdbuf[2], Rdbuf[3]);

	return 0;
}

static int elan_set_calibration_status(unsigned int state)
{
	return 0;
}

static int elan_get_palm_rejection_status(char *buf)
{
	int ret;
	unsigned char Wrbuf[4] = {0x53, 0x31, 0x00, 0x01};
	unsigned char Rdbuf[4] = {0};

	ret = i2c_master_send(tsdata->client, Wrbuf, sizeof(Wrbuf));
	if(ret != 4)
	{
		printk("elan get palm_rejection status sending command fail\n");
		return -1;
	}

	ret = i2c_master_recv(tsdata->client, Rdbuf, sizeof(Rdbuf));
	if(ret != 4)
	{
		printk("elan get palm_rejection status receive data fial\n");
		return -1;
	}
	printk("elan palm_rejection status buf[0x%x 0x%x 0x%x 0x%x]\n", Rdbuf[0], Rdbuf[1], Rdbuf[2], Rdbuf[3]);

	return 0;
}

static int elan_set_palm_rejection_status(unsigned int state)
{
	return 0;
}

static int elan_get_sensitivity_settings(char *buf)
{
	
	unsigned char Wrbuf[4] = {0x53, 0x40, 0x00, 0x01};
	unsigned char Rdbuf[4] = {0};
	int ret = 0;

	ret = i2c_master_send(tsdata->client, Wrbuf, sizeof(Wrbuf));
	if(ret != 4)
	{
		printk("elan get sensitivity_settings sending comman fail\n");
		return -1;
	}

	ret = i2c_master_recv(tsdata->client, Rdbuf, sizeof(Rdbuf));
	if(ret != 4)
	{
		printk("elan get sensitivity_settings receive data fail\n");
		return -1;
	}
	printk("Sensitivity settings buf[0x%x 0x%x 0x%x 0x%x]\n",Rdbuf[0], Rdbuf[1], Rdbuf[2], Rdbuf[3]);

	return 0;
}

static int elan_set_sensitivity_settings(unsigned int state)
{
	int ret;
	unsigned char Wrbuf0[4] = {0x54, 0x40, 0x03, 0x11};
	unsigned char Wrbuf1[4] = {0x54, 0x40, 0x7A, 0x11};
	switch (state)
	{
		case 0:
			ret = i2c_master_send(tsdata->client, Wrbuf0, sizeof(Wrbuf0));
			if(ret != 4)
			{
				printk("elan get sensitivity_settings sending comman fail\n");
				return -1;
			}
			break;
		case 1:
			ret = i2c_master_send(tsdata->client, Wrbuf1, sizeof(Wrbuf1));
			if(ret != 4)
			{
				printk("elan get sensitivity_settings sending comman fail\n");
				return -1;
			}
			break;
		default:
			break;
	}

	return 0;
}

static int elan_get_firmware_id(char *buf)
{
	unsigned char Wrbuf[4] = {0x53, 0xF0, 0x00, 0x01};
	unsigned char Rdbuf[4] = {0};
	int ret = 0;

	ret = i2c_master_send(tsdata->client, Wrbuf, sizeof(Wrbuf));
	if(ret != 4)
	{
		printk("elan get firmware_id sending comman fail\n");
		return -1;
	}

	ret = i2c_master_recv(tsdata->client, Rdbuf, sizeof(Rdbuf));
	if(ret != 4)
	{
		printk("elan get firmware_id receive data fail\n");
		return -1;
	}
	printk("firmware_id buf[0x%x 0x%x 0x%x 0x%x]\n",Rdbuf[0], Rdbuf[1], Rdbuf[2], Rdbuf[3]);
	return 0;
}

static int elan_set_firmware_id(unsigned int state)
{
	return 0;
}

static int elan_get_reset(char *buf)
{
	return 0;
}

static int elan_touch_reset(unsigned int state)
{
	printk("reset the touch!!\n");
	RESETPIN_SET0;
	udelay(5);
	RESETPIN_SET1;
}

struct elan_proc
{
	char *module_name;
	int (*elan_get) (char *);
	int (*elan_set) (unsigned int);
};

static const struct elan_proc elan_modules[] =
{
	{"elan_version", elan_get_version, elan_set_version},
	{"elan_power_status", elan_get_power_status, elan_set_power_status},
	{"elan_calibration_status", elan_get_calibration_status, elan_set_calibration_status},
	{"elan_palm_rejection_status", elan_get_palm_rejection_status, elan_set_palm_rejection_status},
	{"elan_sensitivity_settings", elan_get_sensitivity_settings, elan_set_sensitivity_settings},
	{"elan_firmware_id", elan_get_firmware_id, elan_set_firmware_id},
	{"elan_touch_reset", elan_get_reset, elan_touch_reset},
};

ssize_t elan_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	ssize_t len;
	char *p = page;
	const struct elan_proc *proc = data;
	
	p += proc->elan_get(p);
	
	len = p - page;
	
	*eof = 1;
	
	return len;
}

size_t elan_proc_write(struct file *file, const char __user * buf, unsigned long len, void *data)
{
	char command[PROC_CMD_LEN];
	const struct elan_proc *proc = data;
	
	if(!buf || len > PAGE_SIZE - 1)
	{
		return -EINVAL;
	}
	
	if(len > PROC_CMD_LEN)	
	{
		printk("command so long\n");
		return -ENOSPC;
	}
	
	if(copy_from_user(command, buf, len))
	{
		return -EFAULT;
	}
	
	if(len < 1)
	{
		return -1;
	}
	
	if(strnicmp(command, "on", 2) == 0 || strnicmp(command, "1", 1) == 0)
	{
		proc->elan_set(1);
	} 
	else if (strnicmp(command, "off", 3) == 0 || strnicmp(command, "0", 1) == 0)
	{
		proc->elan_set(0);
	} 
	else 
	{
		return -EINVAL;
	}

	return len;

}

static int proc_node_num = (sizeof(elan_modules) / sizeof(*elan_modules));
static struct proc_dir_entry *update_proc_root;
static struct proc_dir_entry *proc[5];

static int elan_proc_init(void)
{
	unsigned int i;
	
	update_proc_root = proc_mkdir("elan_touch", NULL);
	if(!update_proc_root)
	{
		printk("Create elan_touch directory failed\n");
		return -1;
	}
	
	for(i = 0; i < proc_node_num; i++)
	{
		mode_t mode;
		
		mode = 0;
		if(elan_modules[i].elan_set)
		{
			mode |= S_IRUGO;
		}
		if(elan_modules[i].elan_get)
		{
			mode |= S_IWUGO;
		}
		
		proc[i] = create_proc_entry(elan_modules[i].module_name, mode, update_proc_root);
		if(proc[i])
		{
			proc[i]->data = (void *)(&elan_modules[i]);
			proc[i]->read_proc = elan_proc_read;
			proc[i]->write_proc = elan_proc_write;
		}
	}
	return 0;
}

static void elan_proc_remove(void)
{
	remove_proc_entry("elan_version", update_proc_root);
	remove_proc_entry("elan_power_status", update_proc_root);
	remove_proc_entry("elan_touch", NULL);
}

/*--------------------------------------------------------------*/
static int elan_touch_detect_int_level(void)
{
	int v = 0;
	v = gpio_get_value(tsdata->intr_gpio);
//	printk("the value of intr_gpio: %d\n", v);
	return v;
}

static int elan_touch_poll(struct i2c_client *client)
{
	int status = 0, retry = 20;
	do
	{
		status = elan_touch_detect_int_level();
		retry--;
		mdelay(20);
	} while(status == 1 && retry > 0);

	return (status == 0 ? 0 : -ETIMEDOUT);
}


static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	unsigned char buf_recv[4] = { 0 };
	unsigned char buf_recv1[4] = { 0 };
//	printk("###### hello packet handler #######\n");

	rc = elan_touch_poll(client);
	if (rc < 0) 
	{
		printk("failed to elan_touch_poll \n");
		return -EINVAL;
	}

	rc = i2c_master_recv(client, buf_recv, 4);
	printk("elan %s:hello packet %2x:%2x:%2x:%2x\n", __func__,buf_recv[0],buf_recv[1],buf_recv[2],buf_recv[3]);
	if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x80 && buf_recv[3]==0x80)
	{
		rc = elan_touch_poll(client);
		if(rc < 0)
		{
			dev_err(&client->dev,"elan %s:failed\n",__func__);
			return -EINVAL;
		}
		rc = i2c_master_recv(client, buf_recv1, 4);
		printk("elan %s: recovery hello packet %2x:%2x:%2x:%2x", __func__,buf_recv1[0],
				buf_recv1[1], buf_recv1[2], buf_recv1[3]);
		RECOVERY = 0x80;
		return RECOVERY;

	}

	return 0;
}

static void get_elan_touch_version(void)
{
	unsigned char Wrbuf[4] = {0};
	unsigned char Rdbuf[4] = {0};
	int ret = 0;
	
	Wrbuf[0] = 0x53;
	Wrbuf[1] = 0x00;
	Wrbuf[2] = 0x00;
	Wrbuf[3] = 0x01;

	ret = i2c_master_send(tsdata->client, Wrbuf, sizeof(Wrbuf));
	if(ret != 4)
	{
		printk("Unable to write to i2c touchscreen\n");
	}
	
	ret = i2c_master_recv(tsdata->client, Rdbuf, sizeof(Rdbuf));
	if(ret != 4)
	{
		printk("Unable to read from touch IC\n");
	}
	printk("firmware version packet: [0x%x 0x%x 0x%x 0x%x]\n",Rdbuf[0], Rdbuf[1], Rdbuf[2], Rdbuf[3]);
	
}

static void get_elan_touch_powerstate(void)
{
	unsigned char Wrbuf[4] = {0};
	unsigned char Rdbuf[4] = {0};
	int ret = 0;

	Wrbuf[0] = 0x53;
	Wrbuf[1] = 0x50;
	Wrbuf[2] = 0x00;
	Wrbuf[3] = 0x01;

	ret = i2c_master_send(tsdata->client, Wrbuf, sizeof(Wrbuf));
	if (ret != 4)
	{
		printk("send the elan touch powerstate command failed\n");
	}

	ret = i2c_master_recv(tsdata->client, Rdbuf, sizeof(Rdbuf));
	if (ret != 4)
	{
		printk("get elan touch powerstate failed\n");
	}
	printk("powersate buf[0x%x 0x%x 0x%x 0x%x]", Rdbuf[0], Rdbuf[1], Rdbuf[2], Rdbuf[3]);
	
}

static void force_elan_recalibrate(void)
{
	unsigned char Wrbuf[4] = {0x54, 0x20, 0x01, 0x01};
	int ret = 0;

	ret = i2c_master_send(tsdata->client, Wrbuf, sizeof(Wrbuf));
	if(ret != 4)
		printk(KERN_ERR "%s: elan force recalibrate sending comman fail\n", __FUNCTION__);
	else
		printk(KERN_INFO "%s: touch recalibrated\n", __FUNCTION__);
}

static int __elan_touch_init(struct i2c_client *client)
{
	int rc;
	rc = __hello_packet_handler(client);
	if (rc < 0)
	{
		printk(KERN_ERR "%s: can't get hello packet\n", __FUNCTION__);
		goto hand_shake_failed;
	}		
	get_elan_touch_version();
	get_elan_touch_powerstate();
//	set_elan_touch_sensitivity();
	force_elan_recalibrate();
hand_shake_failed:
	return rc;
}

static int elan_touch_recv_data(struct i2c_client *client, uint8_t *buf)
{
	int rc, bytes_to_recv = IDX_PACKET_SIZE;

	if (buf == NULL)
		return -EINVAL;

	memset(buf, 0, bytes_to_recv);
	rc = i2c_master_recv(client, buf, bytes_to_recv);
	printk("buf[0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x]\n",buf[0], buf[1], buf[2],
			buf[3], buf[4], buf[5], buf[6],buf[7]);
	if (rc != bytes_to_recv) 
	{
		dev_err(&client->dev,"[elan] %s:i2c_master_recv error! \n", __func__);
		return -EINVAL;
	}

	return rc;
}

static void elan_touch_report_data(struct i2c_client *client, uint8_t *buf)
{
	uint8_t finger_stat;
	uint16_t x1 = 0, y1 = 0, x2 = 0, y2 = 0;

//	printk("buf[0]=%x",buf[0]);
	if(buf[0] != 0x5a)
	{
		printk("the packet error!!\n");
		goto out;
	}
	
	x1 = (((buf[1] & 0xf0) << 4) | buf[2]);
	y1 = (((buf[1] & 0x0f) << 8) | buf[3]);
	x2 = (((buf[4] & 0xf0) << 4) | buf[5]);
	y2 = (((buf[4] & 0x0f) << 8) | buf[6]);
//	printk("buf:[0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x]\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);

	/*convert*/
	x1 = (x1*RESOLUTION_X/1088);
	y1 = (y1*RESOLUTION_Y/768);
	x2 = (x2*RESOLUTION_X/1088);
	y2 = (y2*RESOLUTION_Y/768);

	finger_stat = (buf[idx_finger_state] & 0x06) >> 1;

	if (finger_stat == 0) 
	{
		input_report_key(tsdata->input, BTN_TOUCH, 0);
		input_report_key(tsdata->input, BTN_2, 0);
		input_report_abs(tsdata->input, ABS_MT_POSITION_X, 0);
		input_report_abs(tsdata->input, ABS_MT_POSITION_Y, 0);
//		printk("touch release !!\n");
	}
	else if (finger_stat) 
	{
		input_report_key(tsdata->input, BTN_TOUCH, 1);
		input_report_abs(tsdata->input, ABS_MT_POSITION_X, RESOLUTION_X - x1);
		input_report_abs(tsdata->input, ABS_MT_POSITION_Y, RESOLUTION_Y - y1);
		input_mt_sync(tsdata->input);

		if(finger_stat == 2)
		{
			input_report_abs(tsdata->input, ABS_MT_POSITION_X, RESOLUTION_X - x2);
			input_report_abs(tsdata->input, ABS_MT_POSITION_Y, RESOLUTION_Y - y2);
			input_mt_sync(tsdata->input);
		}
#if ELAN_DEBUG
		printk("x1=%d,y1=%d; x2=%d,y2=%d\n", x1,y1,x2,y2);		
#endif
	} 
	input_sync(tsdata->input);
	
out:
	enable_irq(tsdata->irq);
}

static void elan_touch_work_func(struct work_struct *work)
{
//	printk("%s\n", __func__);
	int rc;
	uint8_t buf[IDX_PACKET_SIZE] = { 0 };
	struct i2c_client *client = tsdata->client;
	static const uint8_t hello_pkt[4] = {0x55, 0x55, 0x55, 0x55};

	if (elan_touch_detect_int_level())
	{
		printk("the value of int gpio is true , here return\n");	
		enable_irq(tsdata->irq);
		return;
	}

	rc = elan_touch_recv_data(client, buf);
	if (rc != 8)
	{
		printk("rc != 8 , here return\n");
		enable_irq(tsdata->irq);
		return;
	}

	if (!memcmp(buf, hello_pkt, 4)) {
		// handle hello packet
		printk("elan %s: got hello packet %02hhx:%02hhx:%02hhx:%02hhx, recalibrate\n", __func__, buf[0], buf[1], buf[2], buf[3]);
		force_elan_recalibrate();
		enable_irq(tsdata->irq);
	} else		
		elan_touch_report_data(client, buf);
}

static irqreturn_t elan_touch_ts_interrupt(int irq, void *dev_id)
{
//	printk("%s\n", __func__);
	disable_irq_nosync(tsdata->irq);
	queue_work(elan_wq, &tsdata->work.work);

	return IRQ_HANDLED;
}

static int elan_touch_register_interrupt(struct i2c_client *client)
{
	int err = 0;

	if (client->irq) {
		tsdata->if_irq = 1;
		err = request_irq(client->irq, elan_touch_ts_interrupt, IRQF_TRIGGER_FALLING,
				  ELAN_TS_NAME, &tsdata->irq);
		if (err < 0) {
			printk("%s(%s): Can't allocate irq %d\n", __FILE__, __func__, client->irq);
			tsdata->irq = 0;
		}
	}

	printk("elan ts starts in %s mode.\n",	tsdata->if_irq == 1 ? "interrupt":"polling");
	
	return err;
}

static int elan_touch_get_data(struct i2c_client *client, uint8_t *cmd,
				uint8_t *buf, size_t size)
{
	int rc;
	if (buf == NULL)
		return -EINVAL;
	if(i2c_master_send(client, cmd, 4) != 4)
	{
		dev_err(&client->dev,"ELAN %s: i2c_master_send_fail\n", __func__);
		return -EINVAL;
	}

	rc = elan_touch_poll(client);
	if(rc < 0)
		return -EINVAL;
	else
	{
		if (i2c_master_recv(client, buf, size) != size || buf[0] != CMD_S_PKT)
			return -EINVAL;
	}

	return 0;

}

static int __fw_packet_handler(struct i2c_client *client)
{
	int rc;
	int major,minor;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};
	uint8_t cmd_x[] = {0x53, 0x60, 0x00, 0x00}; //get x resolution
	uint8_t cmd_y[] = {0x53, 0x63, 0x00, 0x00}; //get y resolution
	uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01}; //get firmware ID
	uint8_t buf_recv[4] = {0};
// Firmware version
      rc = elan_touch_get_data(client, cmd, buf_recv, 4);
      if (rc < 0)
		return rc;

	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	tsdata->fw_ver = major << 8 | minor;
	FW_VERSION = tsdata->fw_ver;
// X Resolution
	rc = elan_touch_get_data(client, cmd_x, buf_recv, 4);
	if (rc < 0)
		return rc;

	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
      tsdata->x_resolution =minor;
	X_RESOLUTION = tsdata->x_resolution;

// Y Resolution         
	rc = elan_touch_get_data(client, cmd_y, buf_recv, 4);
	if (rc < 0)
		return rc;

	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
      tsdata->y_resolution =minor;
	Y_RESOLUTION = tsdata->y_resolution;
// Firmware ID
      rc = elan_touch_get_data(client, cmd_id, buf_recv, 4);
	if (rc < 0)
	return rc;

	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
      tsdata->fw_id = major << 8 | minor;
	FW_ID = tsdata->fw_id;

      printk(KERN_INFO "[elan] %s: firmware version: 0x%4.4x\n",
			__func__, tsdata->fw_ver);
	printk(KERN_INFO "[elan] %s: firmware ID: 0x%4.4x\n",
			__func__, tsdata->fw_id);

      printk(KERN_INFO "[elan] %s: x resolution: %d, y resolution: %d\n",
			__func__, tsdata->x_resolution, tsdata->y_resolution);
	return 0;
}

int elan_iap_open(struct inode *inode, struct file *filp)
{
	return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp)
{
	return 0;
}

ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp)
{
	int ret;
	char *tmp;

	if(count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);
	if(tmp == NULL)
		return -ENOMEM;

	if (copy_from_user(tmp, buff, count))
		return -EFAULT;

	ret = i2c_master_send(tsdata->client, tmp, count);
	if(ret != count )
		printk("ELAN i2c_master_send fail, ret = %d\n", ret);

	kfree(tmp);
	return ret;
}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
	char *tmp;
	int ret;
	long rc;

	//printk("ELAN into elan_iap_read\n");

	if(count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);
	if(tmp == NULL)
		return -ENOMEM;

	ret = i2c_master_recv(tsdata->client, tmp, count);
	if(ret >= 0)
		rc = copy_to_user(buff, tmp, count);

	kfree(tmp);

	return ret;
}

int elan_iap_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	int __user *ip = (int __user *)arg;
//	printk("ELAN into elan_ioctl_iap\n");

	switch(cmd)
	{
		case IOCTL_I2C_SLAVE:
			tsdata->client->addr = (int __user *)arg;
			break;
		case IOCTL_MAJOR_FW_VER:
			break;
		case IOCTL_MINOR_FW_VER:
			break;
		case IOCTL_RESET:
			break;
		case IOCTL_IAP_MODE_LOCK:
			work_lock = 1;
			break;
		case IOCTL_IAP_MODE_UNLOCK:
			work_lock = 0;
			if (gpio_get_value(tsdata->intr_gpio))
			{
				enable_irq(tsdata->irq);
			}
			break;
		case IOCTL_CHECK_RECOVERY_MODE:
			return RECOVERY;
			break;
		case IOCTL_FW_VER:
			__fw_packet_handler(tsdata->client);
			return FW_VERSION;
			break;
		case IOCTL_X_RESOLUTION:
			__fw_packet_handler(tsdata->client);
			return X_RESOLUTION;
			break;
		case IOCTL_Y_RESOLUTION:
			__fw_packet_handler(tsdata->client);
			return Y_RESOLUTION;
			break;
		case IOCTL_FW_ID:
			__fw_packet_handler(tsdata->client);
			return FW_ID;
			break;
		case IOCTL_I2C_INT:
			put_user(gpio_get_value(tsdata->intr_gpio), ip);
			break;
		default:
			break;
	}
	return 0;
}

struct file_operations elan_touch_fops = {
		.owner = THIS_MODULE,
		.open	= elan_iap_open,
		.write = elan_iap_write,
		.read = elan_iap_read,
		.release = elan_iap_release,
		.ioctl = elan_iap_ioctl,
};

static int elan_touch_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0 , ret = 0;
	struct input_dev *input;

	struct elan_ktf2k_i2c_platform_data *pdata;

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_ERR "[elan]%s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ret = hw_set_tslockctl(set_ts_lock);
	if(ret != 0)
	{
		printk("hw_set_tslockctl failed\n");
	}
	elan_wq = create_singlethread_workqueue("elan_wq");
	if (!elan_wq) 
	{
		err = -ENOMEM;
		goto fail;
	}
	tsdata = kzalloc(sizeof(*tsdata), GFP_KERNEL);
	if(!tsdata)
	{
		printk("failed to allocate driver data\n");
		err = -ENOMEM;
		return err;
	}
	
	tsdata->client = client;
	strlcpy(client->name, ELAN_TS_NAME, I2C_NAME_SIZE);

	INIT_DELAYED_WORK(&tsdata->work, elan_touch_work_func);
	
	input = input_allocate_device();
	if (input == NULL)
	{
		err = -ENOMEM;
		printk("failed to allocate input device\n");
		goto fail;
	}

	err = __elan_touch_init(client);
	if (err < 0) {
	    printk("Read Hello Packet Fail\n");
	    goto fail;
	}
	printk("%s:err=%d\n", __func__, err);
// Firmware update
	if(err == 0x80)
	{
		tsdata->firmware.minor = MISC_DYNAMIC_MINOR;
		tsdata->firmware.name = "elan-iap";
		tsdata->firmware.fops = &elan_touch_fops;
		tsdata->firmware.mode = S_IRWXUGO;

		if(misc_register(&tsdata->firmware) < 0)
			printk("elan misc_register fail!!\n");
		else
			printk("elan misc_register finish!!\n");
		return 0;
	}
// End Firmware update

	input->name = ELAN_TS_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
	tsdata->client = client;
	tsdata->input = input;
	tsdata->irq = client->irq;
	tsdata->intr_gpio = 174;
	input_set_drvdata(input, tsdata);
	pdata = client->dev.platform_data;
	
	set_bit(BTN_2, input->evbit);
	set_bit(BTN_TOUCH, input->evbit);
	set_bit(EV_SYN, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(EV_ABS, input->evbit);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, ELAN_TS_X_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ELAN_TS_Y_MAX, 0, 0);

	err = input_register_device(input);
	if (err < 0) 
	{
		printk("fail to register input device\n");
		goto fail;
	}

	elan_touch_register_interrupt(tsdata->client);
	
	elan_proc_init();

// MISC
	tsdata->firmware.minor = MISC_DYNAMIC_MINOR;
	tsdata->firmware.name = "elan-iap";
	tsdata->firmware.fops = &elan_touch_fops;
	tsdata->firmware.mode = S_IRWXUGO;

	if(misc_register(&tsdata->firmware) < 0)
		printk("elan misc_register fail!!\n");
	else
		printk("elan misc_register finish!!\n");

	return 0;

fail:
	input_free_device(tsdata->input);
	if (elan_wq)
		destroy_workqueue(elan_wq);
	return err;
err_check_functionality_failed:
	return err;
}

static int elan_touch_suspend(struct i2c_client *client)
{
	int ret = 0;
	printk("%s\n", __func__);
	ret = cancel_delayed_work_sync(&tsdata->work);
//	printk("ret = %d\n", ret);
	if(ret == 1)
	{
		enable_irq(tsdata->irq);
	}
	
	hw_get_deepsuspend(&state);
	if (state == PM_SUSPEND_STANDBY)
	{
 		 enable_irq_wake(gpio_to_irq(174));
	}

	if(state == PM_SUSPEND_MEM)
	{
		RESETPIN_SET0;
		TOUCH_POWER_OFF;
	}
/*
	if(state == PM_SUSPEND_MEM)
	{
		disable_irq_nosync(tsdata->irq); 	
		ret = i2c_master_send(tsdata->client, Wrbuf_deepsleep, sizeof(Wrbuf_deepsleep));
		if(ret != 4)
		{
			printk("Unable to write to touchscreen \n");
			return -1;
		}
	}
*/
	return 0;
}

static int elan_touch_resume(struct i2c_client *client)
{
	printk("%s\n", __func__);
	if (state == PM_SUSPEND_STANDBY)
  		disable_irq_wake(gpio_to_irq(174));

	if(state == PM_SUSPEND_MEM)
	{
		TOUCH_POWER_ON;
		mdelay(10);
		RESETPIN_SET1;
		mdelay(100);
	}
/*
	if(state == PM_SUSPEND_MEM)
	{
		enable_irq(tsdata->irq);  
		ret = i2c_master_send(tsdata->client, Wrbuf_normal, sizeof(Wrbuf_normal));
		if(ret != 4)
		{
			printk("Unable to write to touchscreen \n");
			return -1;
		}
	}
*/
	return 0;

}
static int elan_touch_remove(struct i2c_client *client)
{
	if (elan_wq)
		destroy_workqueue(elan_wq);

	input_unregister_device(tsdata->input);

	if (tsdata->irq)
	{
		free_irq(client->irq, client);
	}
	
	elan_proc_remove();

	RESETPIN_SET0;
	mdelay(10);
	TOUCH_POWER_OFF;
	printk("the value of gpio_145:%d\n", gpio_get_value(145));
	return 0;
}

static int elan_touch_shutdown(struct i2c_client *client) {
	printk("<0> elan touch shutdown\n");
	// lock TS, disabling IRQ and cancelling work
	set_ts_lock(1);
	// turn off touch
	RESETPIN_SET0;
	mdelay(10);
	TOUCH_POWER_OFF;
	return 0;
}


/* -------------------------------------------------------------------- */
static const struct i2c_device_id elan_touch_id[] = {
    {"elan-touch", 0 },
	{ }
};

static struct i2c_driver elan_touch_driver = {
	.probe		= elan_touch_probe,
	.remove		= elan_touch_remove,
	.suspend		= elan_touch_suspend,
	.resume		= elan_touch_resume,
	.shutdown	= elan_touch_shutdown,
	.id_table		= elan_touch_id,
	.driver		= {
		.name = "elan-touch",
		.owner = THIS_MODULE,
	},
};

int elan_open(struct inode *inode, struct file *file)
{
	return 0;
}
static int elan_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static ssize_t elan_read(struct file * file, unsigned int __user *buf, size_t count,loff_t *offset)
{
	return 0;
}

static struct file_operations elan_i2c_ts_ops = {
	.owner = THIS_MODULE,
	.open  = elan_open,
	.read  = elan_read,
	.ioctl = elan_ioctl,
};

static int __init elan_touch_init(void)
{
	int ret = 0;
	ret = register_chrdev(I2C_MAJOR, "elan-touch", &elan_i2c_ts_ops);
	if(ret)
	{
		printk(KERN_ERR "Can't register major number, register chadev failed\n");
	}

	elan_touch_class = class_create(THIS_MODULE, "elan-touch");
	if(IS_ERR(elan_touch_class))
	{
		printk("create elan-touch class failed\n");
	}	

	ret = device_create(elan_touch_class, NULL, MKDEV(I2C_MAJOR, 0), NULL, "elan-touch");
	if(!ret)
	{
		printk("Unable to create class device\n");
	}
	ret = i2c_add_driver(&elan_touch_driver);
	printk("ret = %d\n", ret);
	return ret;
}

static void __exit elan_touch_exit(void)
{
	i2c_del_driver(&elan_touch_driver);
}

module_init(elan_touch_init);
module_exit(elan_touch_exit);

MODULE_AUTHOR("Stanley Zeng <stanley.zeng@emc.com.tw>");
MODULE_DESCRIPTION("ELAN Touch Screen driver");
MODULE_LICENSE("GPL");
