/*
 *  max17040_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/max17040_battery.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
//#include <linux/hw_ops.h>
#include <linux/ep7a/charge_core.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#define MAX17040_VCELL_MSB	0x02
#define MAX17040_VCELL_LSB	0x03
#define MAX17040_SOC_MSB	0x04
#define MAX17040_SOC_LSB	0x05
#define MAX17040_MODE_MSB	0x06
#define MAX17040_MODE_LSB	0x07
#define MAX17040_VER_MSB	0x08
#define MAX17040_VER_LSB	0x09
#define MAX17040_RCOMP_MSB	0x0C
#define MAX17040_RCOMP_LSB	0x0D
#define MAX17040_CMD_MSB	0xFE
#define MAX17040_CMD_LSB	0xFF

#define MAX17040_DELAY		2000
#define MAX17040_BATTERY_FULL	95

extern int charger_fault;

//#define MAX17043_DEBUG
#ifdef MAX17043_DEBUG
#define pr_max_info(fmt, ...) \
        printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_max_info(fmt, ...) \
        0
#endif
static int max17044_write_code(u8 reg,int wt_value,
                         struct i2c_client *client);
static int max17044_read_code(u8 reg, int *rt_value,
			struct i2c_client *client);

static int max17044_read(u8 reg, int *rt_value,
                                struct i2c_client *client);
static int max17044_write(u8 reg, int wt_value,
                                struct i2c_client *client);

static struct workqueue_struct *bat_wq;

struct max17040_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct delayed_work		lwork;

	struct power_supply		battery;
	struct max17040_platform_data	*pdata;

	/* State Of Connect */
	int online;
	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
	/* State Of Charge */
	int status;
	/* State of capacity level*/
	int cap_level;
	/* Battery health status*/
	int health;
	
	int irq;

        /* read/write mutex lock */
        struct mutex rwlock;
};
static struct max17040_chip *chip; 

static int max17040_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17040_chip *chip = container_of(psy,
				struct max17040_chip, battery);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		//val->intval = chip->online;
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->soc;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = chip->cap_level;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->health;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int max17040_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	mutex_lock(&chip->rwlock);
	ret = i2c_smbus_write_byte_data(client, reg, value);
	mutex_unlock(&chip->rwlock);

	if (ret < 0){
		//dev_err(&client->dev, "%s: err %d\n", __func__, ret);
                dev_err(&client->dev, "Unable to write max17043 register via I2C\n");
                printk("Unable to write max17043 register via I2C\n");
                return ret;
        }

	return ret;
}

static int max17040_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	mutex_lock(&chip->rwlock);
	ret = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->rwlock);

	if (ret < 0){
		//dev_err(&client->dev, "%s: err %d\n", __func__, ret);
                dev_err(&client->dev, "Unable to read max17043 register via I2C\n");
                printk("Unable to read max17043 register via I2C\n");
                return ret;
        }

	return ret;
}

static void max17040_reset(struct i2c_client *client)
{
	printk("max17040_reset\n");
	max17040_write_reg(client, MAX17040_CMD_MSB, 0x40);
	max17040_write_reg(client, MAX17040_CMD_LSB, 0x00);
}

static void max17040_get_vcell(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	/*
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(client, MAX17040_VCELL_MSB);
	lsb = max17040_read_reg(client, MAX17040_VCELL_LSB);

	//chip->vcell = (msb << 4) + (lsb >> 4);
	
	chip->vcell = ((msb << 4) + (lsb >> 4))*125;
	*/
	int volt = 0;
	max17044_read(MAX17040_VCELL_MSB,&volt,client);
	
	volt = ((volt >> 4) & 0xFFF)*1250;
	chip->vcell=volt;
}
//base on customer BSP spec
//0---2%-15%
//1---16%-29%
//2---30%-43%
//3---44%-57%
//4---58%-71%
//5---72%-85%
//6---86%-100%
static void max17040_get_soc(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	/*
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(client, MAX17040_SOC_MSB);
	lsb = max17040_read_reg(client, MAX17040_SOC_LSB);

	chip->soc = msb/2;
	*/
	int value;
	int rsoc=0;
	int step=0;
	int counter;

	max17044_read(MAX17040_SOC_MSB,&rsoc,client);
	rsoc = ((rsoc>>8) & 0xff)/2;

	counter = 10;
	while((rsoc == 0) && (chip->vcell > 3600000))
	{
		printk("%d:abnormal detect capacity:rsoc=%d, vcell=%d\n", counter, rsoc, chip->vcell);
		max17044_read(MAX17040_SOC_MSB,&rsoc,client);
		rsoc = ((rsoc>>8) & 0xff)/2;
		if(! --counter)
			break;
	}	 

	counter = 10;
	while((rsoc == 100) && (chip->vcell < 4080000))
	{
		printk("%d:abnormal detect capacity:rsoc=%d, vcell=%d\n", counter, rsoc, chip->vcell);
		max17044_read(MAX17040_SOC_MSB,&rsoc,client);
		rsoc = ((rsoc>>8) & 0xff)/2;
		if(! --counter)
			break;
	}	 


#if 1
	if(rsoc>10)
	{
		max17044_read(0x0C,&value,client);
		value &= 0xffe0;
		value |= 0x000c;
		max17044_write(0x0C,value,client);
		max17044_read(0x0C,&value,chip->client);
	}

	if(rsoc<10 && rsoc>5)
	{
		max17044_read(0x0C,&value,client);
		value &= 0xffe0;
		value |= 0x0016;
		max17044_write(0x0C,value,client);
		max17044_read(0x0C,&value,chip->client);
	}
	
	if(rsoc<5)
	{
		max17044_read(0x0C,&value,client);
		value &= 0xffe0;
		value |= 0x001a;
		max17044_write(0x0C,value,client);
		max17044_read(0x0C,&value,chip->client);
	}	
#endif	
	if(rsoc >= 100)
		rsoc = 100;
	
	if(get_bat_charge_status() == BAT_CHARGEFULL)
		rsoc = 100;
	
	if(get_bat_charge_status() == BAT_CRITICALLOW)
	{
		rsoc = 0;
		printk("battery critical low, set capacity to 0.\n");
	}

	chip->soc = rsoc;
	
	if(rsoc>=16)
	step=1;
	
	if(rsoc>=30)
	step=2;
		
	if(rsoc>=44)
	step=3;
	
	if(rsoc>=58)
	step=4;
	
	if(rsoc>=72)
	step=5;

	if(rsoc>=86)
	step=6;

	//hw_set_batt(step);	
//	hw_set_batt(rsoc);	
	//printk("%s:capacity:%d\n", __func__, rsoc);
}

//static void max17040_get_version(struct i2c_client *client)
static int max17040_get_version(struct i2c_client *client)
{
/*
	u8 msb;
	u8 lsb;
	printk("max17040_get_version\n");
	msb = max17040_read_reg(client, MAX17040_VER_MSB);
	lsb = max17040_read_reg(client, MAX17040_VER_LSB);

	dev_info(&client->dev, "MAX17040 Fuel-Gauge Ver %d%d\n", msb, lsb);
*/
	int version=-1;
	u8 version_msb;
	u8 version_lsb;
	max17044_read(MAX17040_VER_MSB,&version,client);
	printk("version = %d\n",version);
	version_msb= (version>>8) & 0xff;
	version_lsb= version & 0xff;
	
	dev_info(&client->dev, "MAX17040 Fuel-Gauge Ver %d %d\n", version_msb, version_lsb);
	return version;
}

static void max17040_get_online(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	if (chip->pdata->battery_online)
		chip->online = chip->pdata->battery_online();
	else
		chip->online = 1;
}

static void max17040_get_status(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
		
	if (!chip->pdata->charger_online || !chip->pdata->charger_full) {
		chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
		return;
	}
	
	if (chip->pdata->charger_online()) {
//		if (chip->pdata->charger_full()) 
//		{
//			chip->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
//			chip->status = POWER_SUPPLY_STATUS_FULL;
//		}
//		else
		chip->status = POWER_SUPPLY_STATUS_CHARGING;
	} else 
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;	
  
	dev_dbg(chip->battery.dev, "bat status: %d\n",
		chip->status);
}

static void max17040_get_capacity_level(struct i2c_client *client)
{ 
  struct max17040_chip *chip = i2c_get_clientdata(client);
  
	if((chip->soc < 0) || (chip->soc > 100))
		chip->cap_level = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
	else if((chip->soc <= 100) && (chip->soc > 10)){
//	if(chip->soc > MAX17040_BATTERY_FULL)	   
		if(chip->soc == 100)	
			chip->cap_level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else
			chip->cap_level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;}
//	else if((chip->soc <= 10) && (chip->soc > 5) && (chip->status == POWER_SUPPLY_STATUS_DISCHARGING))
	else if((chip->soc <= 10) && (chip->soc > 5))
			chip->cap_level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
//	else if((chip->soc <= 5) && (chip->soc >=0) && (chip->status == POWER_SUPPLY_STATUS_DISCHARGING))
	else if((chip->soc <= 5) && (chip->soc >=0))
			chip->cap_level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;	 
}

static void max17040_get_health(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	if (charger_fault == 0)
		chip->health = POWER_SUPPLY_HEALTH_GOOD;
	else if(charger_fault == 1)
		chip->health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else if(charger_fault == 2)
		chip->health = POWER_SUPPLY_HEALTH_DEAD;
	else if(charger_fault == 3)
		chip->health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else 
		chip->health = POWER_SUPPLY_HEALTH_UNKNOWN;
}

static void max17040_work(struct work_struct *work)
{
	int old_status;
	int old_cap_level;
	int old_health;
	
	struct max17040_chip *chip;
	chip = container_of(work, struct max17040_chip, work.work);
	
	old_status = chip->status;
	old_cap_level = chip->cap_level;
	old_health = chip->health;
  
	max17040_get_vcell(chip->client);
	max17040_get_soc(chip->client);
	max17040_get_online(chip->client);
	max17040_get_status(chip->client);
	max17040_get_capacity_level(chip->client);
	max17040_get_health(chip->client);
	
	if ((old_status != POWER_SUPPLY_STATUS_UNKNOWN && chip->status != old_status) \
		  || (old_cap_level != POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN && chip->cap_level != old_cap_level) \
		  || (old_health != POWER_SUPPLY_HEALTH_UNKNOWN && chip->health != old_health))
	power_supply_changed(&chip->battery);
	
	schedule_delayed_work(&chip->work, msecs_to_jiffies(MAX17040_DELAY));
}

static enum power_supply_property max17040_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_HEALTH,
};

static int max17044_write(u8 reg, int wt_value,
                                struct i2c_client *client)
{
        int ret;
        ret = max17044_write_code(reg,wt_value,client);
        return ret;
}

static int max17044_read(u8 reg, int *rt_value,
                                struct i2c_client *client)
{
        int ret;
        ret = max17044_read_code(reg,rt_value,client);
        return ret;
}
/*For normal data writing*/
static int max17044_write_code(u8 reg,int wt_value,
                         struct i2c_client *client)
{
        struct i2c_msg msg[1];
        unsigned char data[3];
        int err;

        if (!client->adapter)
                return -ENODEV;

        msg->addr = client->addr;
        msg->flags = 0;
        msg->len = 3;
        msg->buf = data;

        data[0] = reg;
        data[1] = wt_value >> 8;
        data[2] = wt_value & 0xFF;
        err = i2c_transfer(client->adapter, msg, 1);
        return err;
}
/*For normal data reading*/
static int max17044_read_code(u8 reg, int *rt_value,
			struct i2c_client *client)
{
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		msg->len = 2;
		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		pr_max_info("data[0] is %x and data[1] is %x \n",data[0],data[1]);
		if (err >= 0) {		
			*rt_value = get_unaligned_be16(data);
		pr_max_info(" get_unaligned_be16(data) is %x \n",*rt_value);
			return 0;
		}
	}
	return err;
}


/*
 ************************ Max17044 Loading a Custom Model********************************
 */

/*1.Unlock Model Access*/
static int max17044_ulock_model(struct i2c_client *client)
{
	int ret ;
	/*max17040_write_reg(client, 0x3e, 0x4a);
	max17040_write_reg(client, 0x3f, 0x57);*/
	ret = max17044_write(0x3E,0x4A57,client);
	/*
	if (ret) {
		pr_max_info( "writing unlock  Register,and ret is %d\n",ret);
		return ret;
	}*/
	return ret;
}

/*For mode burn reading*/
static int max17044_read_code_burn(u8 reg, char *rt_value,
			struct i2c_client *client)
{
//	struct i2c_client *client = di->client;
	struct i2c_msg msg[1];
	unsigned char data[4];
	int err;
	int i;
	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		msg->len = 4;
		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		pr_max_info("data[0] is %x and data[1] is %x,and data[2] is %x, and data[3] is %x, \n",data[0],data[1],data[2],data[3]);
		if (err >= 0) {		
		//	*rt_value = get_unaligned_be16(data);
		//	pr_max_info(" get_unaligned_be16(data) is %x \n",*rt_value);
			for(i = 0;i<4;i++)
			{
				rt_value[i]=data[i];
			}
		
			return 0;
		}
	}
	return err;
}
/*2.Read Original RCOMP and OCV Register*/
static int max17044_read_rccmp_ocv(struct i2c_client *client,char *rt_value)
{
	int ret;
	char rsoc[4];
	int i;
	ret = max17044_read_code_burn(0x0C,rsoc,client);
	if (ret) {
		pr_max_info( "reading rccmp and ocv  Register\n");
		return ret;
	}
	for(i=0;i<4;i++)
	{
		rt_value[i] = rsoc[i];
	}
	pr_max_info("rt_value[0] is %x and rt_value[1] is %x,and rt_value[2] is %x, and rt_value[3] is %x, \n",\
							    rt_value[0],rt_value[1],rt_value[2],rt_value[3]);
	return ret;
}

/*3.Write OCV Register*/
static int max17044_write_ocv(struct i2c_client *client)
{	
	int ret ;
	//max17040_write_reg(client, 0x0e, 0xda);
	//max17040_write_reg(client, 0x0f, 0x30);
	
//	ret = max17044_write(0x0E,0xDA00,client);
	ret = max17044_write(0x0E,0xDA10,client); // 0109
	if (ret) {
		pr_max_info( "writing OCV Register,and ret is %d\n",ret);
		return ret;
	}
	
	return ret;
	
}

/*4.Write RCCOMP Register to a Maximum value of 0xFF00h*/
static int max17044_write_rccomp(struct i2c_client *client, int data)
{	
	int ret;
	/*
	u8 msb;
	u8 lsb;
	msb=(data>>8)&0xff;
	lsb=data&0xff;	
	max17040_write_reg(client, 0x0c, msb);
	max17040_write_reg(client, 0x0d, lsb);
	*/
	
	ret = max17044_write(0x0C,data,client);
	if (ret) {
		pr_max_info( "writing rccomp Register,and ret is %d\n",ret);
		return ret;
	}
	return ret;
}
/*For mode burn writing*/
static int max17044_write_code_burn(u8 reg,char *wt_value,
			 struct i2c_client *client)
{
	struct i2c_msg msg[1];
	unsigned char data[17];
	int err;
	int i;
	
	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 17;
	msg->buf = data;

	data[0] = reg;
	for(i=1;i<17;i++)
	{
		data[i]=wt_value[i-1];
	}
#if 0	
	for(i=0;i<17;i++)
	{	
		pr_max_info( "*********get into max17044 write code burn*************");
		pr_max_info( "data[%d] is %x\n",i,data[i]);
	}
#endif
	err = i2c_transfer(client->adapter, msg, 1);
	return err;
}

/*5.Write the Model,once the model is unlocked,
 *the host software must write the 64 byte model to the max17044.
 *the model is located between memory locations 0x40h and 0x7Fh.
 */
static int max17044_write_model(struct i2c_client *client)
{	
	int ret;
	//int i = 0;
#if 0	// import new capacity table	0605 EP8
/*****************set 64 byte mode ******************************/ 
	char data0[] = {0xAF,0x00,0xB7,0x60,0xB9,0xC0,0xBA,0xF0,
			  0xBC,0x30,0xBC,0x70,0xBC,0xC0,0xBD,0x10};

	char data1[] = {0xBD,0x60,0xBE,0x30,0xBF,0x00,0xC0,0xA0,
			  0xC4,0x40,0xC7,0xF0,0xCB,0xC0,0xCF,0xA0};

	char data2[] = {0x03,0x00,0x1E,0x00,0x1E,0x00,0x1C,0x80,
			  0x96,0x90,0x78,0x70,0x78,0x70,0x78,0x70};

	char data3[] = {0x2C,0x40,0x2C,0x40,0x2C,0x40,0x1B,0x40,
			  0x1A,0xC0,0x12,0x30,0x11,0xF0,0x11,0xF0};

#endif

#if 1   // import new capacity table    0109
/*****************set 64 byte mode ******************************/ 
        char data0[] = {0xAF,0x00,0xB6,0xB0,0xB9,0x20,0xBA,0x70,
                          0xBB,0xC0,0xBC,0x20,0xBC,0x90,0xBC,0xE0};

        char data1[] = {0xBD,0x30,0xBE,0x80,0xC0,0xE0,0xC3,0x30,
                          0xC5,0xF0,0xC8,0xB0,0xCC,0x60,0xD0,0x10};

        char data2[] = {0x03,0x40,0x20,0xC0,0x15,0x70,0x15,0x70,
                          0x82,0xD0,0x70,0x20,0x7C,0xF0,0x7C,0xF0};

        char data3[] = {0x31,0xF0,0x1D,0xD0,0x1B,0x00,0x19,0x60,
                          0x19,0x60,0x11,0x00,0x11,0x00,0x11,0x00};

#endif


#if 0
	for(i=0;i<16;i++)
	{	
		pr_max_info( "*********get into max17044 write model*************");
		pr_max_info( "data0[%d] is %x\n",i,data0[i]);
	}
#endif			
	ret = max17044_write_code_burn(0x40,data0,client);
	if (ret) {
		pr_max_info( "data0 writing mode,and ret is %d\n",ret);

	}

	ret = max17044_write_code_burn(0x50,data1,client);
	if (ret) {
		pr_max_info( "data1 writing mode,and ret is %d\n",ret);

	}

	ret = max17044_write_code_burn(0x60,data2,client);
	if (ret) {
		pr_max_info( "data2 writing mode,and ret is %d\n",ret);

	}

	ret = max17044_write_code_burn(0x70,data3,client);
	if (ret) {
		pr_max_info( "data3 writing mode,and ret is %d\n",ret);
	}

	return ret;
}

/*For mode burn restore*/
static int max17044_write_code_restore(u8 reg,char *wt_value,
			 struct i2c_client *client)
{
	struct i2c_msg msg[1];
	unsigned char data[5];
	int err;
	int i;
	
	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 5;
	msg->buf = data;

	data[0] = reg;
	for(i=1;i<5;i++)
	{
		data[i]=wt_value[i-1];
		pr_max_info( "*****get into write code restore and data[%d] is %x\n",i,data[i]);
	}	
	err = i2c_transfer(client->adapter, msg, 1);
	return err;
}

/*10.Restore RCOMP and OCV */
static int max17044_restore(struct i2c_client *client, char *val)
{	
	int ret;
	int i;
	char data[4] ;
	for(i=0;i<4;i++)
	{
		data[i] = val[i];
		pr_max_info( "restore data[%d] is %x\n",i,data[i]);
	}
	data[0] = 0x58;	 	// set temperture register to 0x93H 	
	data[1] = 0x00;
	ret = max17044_write_code_restore(0x0C,data,client);
	if (ret) {
		pr_max_info( "restore writing mode,and ret is %d\n",ret);
		return ret;
	}

	return ret;
}

/*lock Model Access*/
static int max17044_lock_model(struct i2c_client *client)
{
	int ret ;
	//max17040_write_reg(client, 0x3e, 0x0);
	//max17040_write_reg(client, 0x3f, 0x0);
	
	ret = max17044_write(0x3E,0x0000,client);
	/*
	if (ret) {
		pr_max_info( "writing lock  Register,and ret is %d\n",ret);
		return ret;
	} */
	return ret;
}
/*burn mode sequence */
static void max17044_burn(struct i2c_client *client)
{
        char rt_value[4] = { 0,0,0,0};
        printk("max17044_ulock_model\n");
	max17044_ulock_model(client);                   //1.unlock model access
        printk("max17044_read_rccmp_ocv\n");
        max17044_read_rccmp_ocv(client, rt_value);      //2.read rcomp and ocv register
        printk("max17044_write_ocv\n");
        max17044_write_ocv(client);                     //3.write OCV register
        printk("max17044_write_rccompl\n");
        max17044_write_rccomp(client, 0x4200);          //4.write rcomp to a maximum value of 0xff00
//        max17044_write_rccomp(client, 0xFF00);
        printk("max17044_write_model\n");
        max17044_write_model(client);                   //5.write the model
        mdelay(200);                                    //6.delay at least 200ms
        printk("max17044_write_ocv\n");
        max17044_write_ocv(client);                     //7.write ocv register
        mdelay(200);                                    //8. this delay must between 150ms and 600ms
        /*9.read soc register and compare to expected result
         *if SOC1 >= 0xF9 and SOC1 <= 0xFB then 
         *mode was loaded successful else was not loaded successful
         */
        max17044_restore(client, rt_value);             //10.restore rcomp and ocv
        max17044_lock_model(client);                    //11.lock model access
        max17044_write_rccomp(client, 0x580c);          //4.write rcomp to a  value of 0x930c, as define lower battery thread is 10%
}

static void max17043_lbirq_work(struct work_struct *work)
{
	struct max17040_chip *chip;
	int lbatt;
	printk("%s\n", __func__);	

	chip = container_of(work, struct max17040_chip, lwork.work);

	max17044_read(0x0C,&lbatt,chip->client);
	printk("ATHD:%x\n",lbatt);
	lbatt &= 0xFFDF;
	max17044_write(0x0C,lbatt,chip->client);
	max17044_read(0x0C,&lbatt,chip->client);
	printk("..........................ATHD:%x\n",lbatt);
	enable_irq(chip->irq);
//	max17044_write(0x06,0x4000,chip->client);
//	max17044_burn(chip->client);

	//cancel_delayed_work(&chip->work);
}
static irqreturn_t max17043_lowbatt_callback(int irq, void *dev_id)
{
	printk("%s\n", __func__);
//	struct i2c_client *client = (struct i2c_client *)dev_id;
	disable_irq_nosync(irq);
	schedule_delayed_work(&chip->lwork, 0);	
	return IRQ_HANDLED;
}

static int max17043_lowbatt_interrupt(struct i2c_client *client)
{
	int err = 0;
	printk("%s\n", __func__);

	if (client->irq) {
		err = request_irq(client->irq, max17043_lowbatt_callback, IRQF_TRIGGER_LOW,
				  "battery", client);
		if (err < 0) {
			printk("%s(%s): Can't allocate irq %d\n", __FILE__, __func__, client->irq);
		}
	}
	return 0;
}

static int __devinit max17040_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;
	ret = max17040_get_version(client);
        if(ret < 0) {
                  dev_err(&client->dev, "Can't detect max17043 \n");
                  return -EIO;
        }

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->irq = client->irq;
	chip->pdata = client->dev.platform_data;

	i2c_set_clientdata(client, chip);
	mutex_init(&chip->rwlock);
	chip->battery.name		= "battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max17040_get_property;
	chip->battery.properties	= max17040_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(max17040_battery_props);
  
	chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
	chip->cap_level = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
	chip->health = POWER_SUPPLY_HEALTH_UNKNOWN;
  
	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		kfree(chip);
		return ret;
	}

//	max17040_reset(client);
//	max17040_get_version(client);
	max17044_burn(client); // burn a new customer mode

#if 0
	INIT_DELAYED_WORK_DEFERRABLE(&chip->lwork, max17043_lbirq_work);
	max17043_lowbatt_interrupt(client);
	//INIT_DELAYED_WORK_DEFERRABLE(&chip->lwork, max17043_lbirq_work);
	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, max17040_work);	
	schedule_delayed_work(&chip->work, msecs_to_jiffies(MAX17040_DELAY));
#endif

	INIT_DELAYED_WORK_DEFERRABLE(&chip->lwork, max17043_lbirq_work);
	max17043_lowbatt_interrupt(client);
	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, max17040_work);
//	schedule_delayed_work(&chip->work, msecs_to_jiffies(CW2015_DELAY));	
	bat_wq = create_singlethread_workqueue("bat_wq");
	if (!bat_wq) {
		printk("can't create thread bat_wq in %s\n",__func__);
	}
	queue_delayed_work(bat_wq, &chip->work, 2*HZ);
	return 0;
}

static int __devexit max17040_remove(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->battery);
	cancel_delayed_work(&chip->work);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static int max17040_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	enable_irq_wake(gpio_to_irq(103));
	cancel_delayed_work_sync(&chip->work);
	return 0;
}

/* sometimes when device wakes up for couple of seconds and then sleep capacity and voltage readings was not updated,
 * that causes in some situations device to discharge to 3.2V battery voltage without application know about it.
 * so we update chip work 1 msecs after resume.
 */
static int max17040_resume(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	disable_irq_wake(gpio_to_irq(103));
	//schedule_delayed_work(&chip->work, msecs_to_jiffies(MAX17040_DELAY));
//	schedule_delayed_work(&chip->work, msecs_to_jiffies(1));
	queue_delayed_work(bat_wq, &chip->work, msecs_to_jiffies(1));
	return 0;
}

#else

#define max17040_suspend NULL
#define max17040_resume NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id max17040_id[] = {
	{ "max17043", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17040_id);

static struct i2c_driver max17040_i2c_driver = {
	.driver	= {
		.name	= "max17043",
	},
	.probe		= max17040_probe,
	.remove		= __devexit_p(max17040_remove),
	.suspend	= max17040_suspend,
	.resume		= max17040_resume,
	.id_table	= max17040_id,
};

static int __init max17040_init(void)
{
	int ret =0;
	ret = i2c_add_driver(&max17040_i2c_driver);
	if(ret){
		printk(KERN_INFO "max17043 gas gauge :could not add i2c driver\n");
	  return ret;}
	  
	return 0;
}
module_init(max17040_init);

static void __exit max17040_exit(void)
{
	i2c_del_driver(&max17040_i2c_driver);
}
module_exit(max17040_exit);

MODULE_AUTHOR("Victor Liu <liuduogc@gmail.com>");
MODULE_DESCRIPTION("MAX17040 Fuel Gauge");
MODULE_LICENSE("GPL");
