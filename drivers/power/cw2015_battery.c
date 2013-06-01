/*
 *  cw2015_battery.c
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
#include <linux/cw2015_battery.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/hw_ops.h>
#include <linux/ep7a/charge_core.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>

/*----- driver defines -----*/
#define MYDRIVER "cw2015"


#define REG_VERSION     0x00
#define REG_VCELL       0x02
#define REG_SOC         0x04
#define REG_RRT_ALERT   0x06
#define REG_CONFIG      0x08
#define REG_MODE        0x0A
#define REG_BATINFO     0x10

#define SIZE_BATINFO    64 

#define MODE_SLEEP_MASK (0x03<<6)
#define MODE_SLEEP      (0x03<<6)
#define MODE_NORMAL     (0x00<<6)
#define MODE_QUICK_START (0x03<<4)
#define MODE_RESTART    (0x0f<<0)

#define CONFIG_UPDATE_FLG (0x01<<1)
#define ATHD (0x0a<<3)   //ATHD =10%

#define CW2015_DELAY	2000


/*----- config part for battery information -----*/
#if 0
#undef dev_info
#define dev_info dev_err
#endif

#define FORCE_WAKEUP_CHIP 1

/* battery info: GS, 3.7v, 1600mAh */
static char cw2015_bat_config_info[SIZE_BATINFO] = {
	0x15,	0x95,	0x68,	0x5C,	0x57,	0x55,	0x53,	0x4D,
	0x4A,	0x46,	0x45,	0x45,	0x47,	0x4F,	0x40,	0x2B,	
	0x20,	0x1A,	0x17,	0x0F,	0x16,	0x2F,	0x49,	0x58,	
	0x1C,	0x3D,	0x0B,	0x85,	0x40,	0x61,	0x62,	0x78,	
	0x81,	0x71,	0x70,	0x6F,	0x3B,	0x19,	0x50,	0x3B,	
	0x0B,	0x60,	0x2A,	0x5C,	0x84,	0x96,	0x9B,	0x13,	
	0x50,	0x7B,	0x99,	0xBE,	0xAF,	0xA1,	0xCA,	0xCB,	
	0x2F,	0x7D,	0x72,	0xA5,	0xB5,	0xC1,	0x46,	0xAE
};


extern int charger_fault;

//#define MAX17043_DEBUG
#ifdef MAX17043_DEBUG
#define pr_max_info(fmt, ...) \
        printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_max_info(fmt, ...) \
        0
#endif
static int cw2015_write_code(u8 reg,int wt_value,
                         struct i2c_client *client);
static int cw2015_read_code(u8 reg, int *rt_value,
			struct i2c_client *client);

static int cw2015_read(u8 reg, int *rt_value,
                                struct i2c_client *client);
static int cw2015_write(u8 reg, int wt_value,
                                struct i2c_client *client);

static struct workqueue_struct *bat_wq;

struct cw2015_chip {
	struct i2c_client		*client;
	struct device      		*dev;
	
	struct delayed_work		work;
	struct delayed_work		lwork;

	struct power_supply		battery;
	struct cw2015_platform_data	*pdata;

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
static struct cw2015_chip *chip; 

static int cw2015_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct cw2015_chip *chip = container_of(psy,
				struct cw2015_chip, battery);

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

static int cw2015_write_reg(struct i2c_client *client, int reg, u8 value)
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

static int cw2015_read_reg(struct i2c_client *client, int reg)
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

/*static void cw2015_reset(struct i2c_client *client)
{
	printk("cw2015_reset\n");
	cw2015_write_reg(client, MAX17040_CMD_MSB, 0x40);
	cw2015_write_reg(client, MAX17040_CMD_LSB, 0x00);
}
*/

static void cw2015_get_vcell(struct i2c_client *client)
{
	struct cw2015_chip *chip = i2c_get_clientdata(client);
	/*
	u8 msb;
	u8 lsb;

	msb = cw2015_read_reg(client, MAX17040_VCELL_MSB);
	lsb = cw2015_read_reg(client, MAX17040_VCELL_LSB);

	//chip->vcell = (msb << 4) + (lsb >> 4);
	
	chip->vcell = ((msb << 4) + (lsb >> 4))*125;
	*/
	int volt = 0;
	cw2015_read(REG_VCELL,&volt,client);
//	printk("%s:%x\n", __func__, volt);	
	//volt = (volt & 0x3FFF)*312 / 1024;
	volt = (volt & 0x3FFF)*305 ;
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
static void cw2015_get_soc(struct i2c_client *client)
{
	struct cw2015_chip *chip = i2c_get_clientdata(client);
	/*
	u8 msb;
	u8 lsb;

	msb = cw2015_read_reg(client, MAX17040_SOC_MSB);
	lsb = cw2015_read_reg(client, MAX17040_SOC_LSB);

	chip->soc = msb/2;
	*/
	int value;
	int rsoc=0;
	int step=0;
	int counter;
	cw2015_read(REG_SOC,&rsoc,client);
	//printk("%s:%x\n", __func__, rsoc);	
	rsoc = ((rsoc>>8) & 0xff);

	counter = 10;
	while(( rsoc == 0) && ( chip->vcell > 3600000))
	{
		printk("abnormal detect capacity:rsoc=%d, vcell=%d\n", rsoc, chip->vcell);
		cw2015_read(REG_SOC,&rsoc,client);
		rsoc = ((rsoc>>8) & 0xff)/2;
		if(! --counter)
			break;
	}	 

	counter = 10;
	while(( rsoc == 100) && ( chip->vcell < 4080000))
	{
		printk("abnormal detect capacity:rsoc=%d, vcell=%d\n", rsoc, chip->vcell);
		cw2015_read(REG_SOC,&rsoc,client);
		rsoc = ((rsoc>>8) & 0xff)/2;
		if(! --counter)
			break;
	}	 


#if 1
	if(rsoc>=10)
	{
		cw2015_read(REG_CONFIG,&value,client);
		value &= 0x07FF;
		value |= 0x5000;
		cw2015_write(REG_CONFIG,value,client);
		cw2015_read(REG_CONFIG,&value,chip->client);
	}

	if(rsoc<10 && rsoc>=5)
	{
		cw2015_read(REG_CONFIG,&value,client);
		value &= 0x07FF;
		value |= 0x2800;
		cw2015_write(REG_CONFIG,value,client);
		cw2015_read(REG_CONFIG,&value,chip->client);
	}
	
	if(rsoc<5)
	{
		cw2015_read(REG_CONFIG,&value,client);
		value &= 0x07FF;
		value |= 0x1800;
		cw2015_write(REG_CONFIG,value,client);
		cw2015_read(REG_CONFIG,&value,chip->client);
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
	//hw_set_batt(rsoc);	
	//printk("%s:capacity:%d\n", __func__, rsoc);
}

static int cw2015_get_version(struct i2c_client *client)
{
	/*int version=0;
	u8 version_msb;
	u8 version_lsb;
	cw2015_read(REG_VERSION,&version,client);
	printk("version = %d\n",version);
	version_msb= (version>>8) & 0xff;
	version_lsb= version & 0xff;
	
	dev_info(&client->dev, "MAX17040 Fuel-Gauge Ver %d %d\n", version_msb, version_lsb);
	*/
	int ret = 0;
	ret = i2c_smbus_read_byte_data(client, REG_VERSION);
  	if(ret < 0) {
		  dev_err(&client->dev, "Error to read REG_VERSION\n");
		  return ret;
  	}
	printk("%s:%d\n",__func__,ret);
}

static void cw2015_get_online(struct i2c_client *client)
{
	struct cw2015_chip *chip = i2c_get_clientdata(client);

	if (chip->pdata->battery_online)
		chip->online = chip->pdata->battery_online();
	else
		chip->online = 1;
}

static void cw2015_get_status(struct i2c_client *client)
{
	struct cw2015_chip *chip = i2c_get_clientdata(client);
		
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

static void cw2015_get_capacity_level(struct i2c_client *client)
{ 
  struct cw2015_chip *chip = i2c_get_clientdata(client);
  
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

static void cw2015_get_health(struct i2c_client *client)
{
	struct cw2015_chip *chip = i2c_get_clientdata(client);

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

static void cw2015_work(struct work_struct *work)
{
	int old_status;
	int old_cap_level;
	int old_health;
	
	struct cw2015_chip *chip;
	chip = container_of(work, struct cw2015_chip, work.work);
	
	old_status = chip->status;
	old_cap_level = chip->cap_level;
	old_health = chip->health;
  
	cw2015_get_vcell(chip->client);
	cw2015_get_soc(chip->client);
	cw2015_get_online(chip->client);
	cw2015_get_status(chip->client);
	cw2015_get_capacity_level(chip->client);
	cw2015_get_health(chip->client);
	
	if ((old_status != POWER_SUPPLY_STATUS_UNKNOWN && chip->status != old_status) \
		  || (old_cap_level != POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN && chip->cap_level != old_cap_level) \
		  || (old_health != POWER_SUPPLY_HEALTH_UNKNOWN && chip->health != old_health))
	power_supply_changed(&chip->battery);
	
//	schedule_delayed_work(&chip->work, msecs_to_jiffies(CW2015_DELAY));
 	queue_delayed_work(bat_wq, &chip->work, 2*HZ);
}

static enum power_supply_property cw2015_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_HEALTH,
};

static int cw2015_write(u8 reg, int wt_value,
                                struct i2c_client *client)
{
        int ret;
        ret = cw2015_write_code(reg,wt_value,client);
        return ret;
}

static int cw2015_read(u8 reg, int *rt_value,
                                struct i2c_client *client)
{
        int ret;
        ret = cw2015_read_code(reg,rt_value,client);
        return ret;
}
/*For normal data writing*/
static int cw2015_write_code(u8 reg,int wt_value,
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
static int cw2015_read_code(u8 reg, int *rt_value,
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

static int cw2015_verify_update_battery_info(void)
{
	int ret = 0;
	int i;
	char value = 0;
	short value16 = 0;
	char buffer[SIZE_BATINFO*2];
	struct i2c_client *client = NULL;
	char reg_mode_value = 0;


	if(NULL == chip)
		return -1;

	client = to_i2c_client(chip->dev);

	/* make sure not in sleep mode */
	ret = i2c_smbus_read_byte_data(client, REG_MODE);
	if(ret < 0) {
		dev_err(&client->dev, "Error read mode\n");
		return ret;
	}

	value = ret;
	reg_mode_value = value; /* save MODE value for later use */
	if((value & MODE_SLEEP_MASK) == MODE_SLEEP) {
		dev_err(&client->dev, "Error, device in sleep mode, cannot update battery info\n");
		return -1;
	}

	/* update new battery info */
	for(i=0; i<SIZE_BATINFO; i++) {
		ret = i2c_smbus_write_byte_data(client, REG_BATINFO+i, cw2015_bat_config_info[i]);
		if(ret < 0) {
			dev_err(&client->dev, "Error update battery info @ offset %d, ret = 0x%x\n", i, ret);
			return ret;
		}
	}

	/* readback & check */
	for(i=0; i<SIZE_BATINFO; i++) {
		ret = i2c_smbus_read_byte_data(client, REG_BATINFO+i);
		if(ret < 0) {
			dev_err(&client->dev, "Error read origin battery info @ offset %d, ret = 0x%x\n", i, ret);
			return ret;
		}

		buffer[i] = ret;
	}

	if(0 != memcmp(buffer, cw2015_bat_config_info, SIZE_BATINFO)) {
		dev_info(&client->dev, "battery info NOT matched, after readback.\n");
		return -1;
	} else {
		dev_info(&client->dev, "battery info matched, after readback.\n");
	}

	/* set 2015 to use new battery info */
	ret = i2c_smbus_read_byte_data(client, REG_CONFIG);
	if(ret < 0) {
		dev_err(&client->dev, "Error to read CONFIG\n");
		return ret;
	}
	value = ret;
	  
	value |= CONFIG_UPDATE_FLG;/* set UPDATE_FLAG */
	value &= 0x7;  /* clear ATHD */
	value |= ATHD; /* set ATHD */
	
	ret = i2c_smbus_write_byte_data(client, REG_CONFIG, value);
	if(ret < 0) {
		dev_err(&client->dev, "Error to update flag for new battery info\n");
		return ret;
	}
	
	/* check 2015 for ATHD&update_flag */
	ret = i2c_smbus_read_byte_data(client, REG_CONFIG);
	if(ret < 0) {
		dev_err(&client->dev, "Error to read CONFIG\n");
		return ret;
	}
	value = ret;
	
	if (!(value & CONFIG_UPDATE_FLG)) {
	  dev_info(&client->dev, "update flag for new battery info have not set\n");
	}
	if ((value & 0xf8) != ATHD) {
	  dev_info(&client->dev, "the new ATHD have not set\n");
	}	  

	reg_mode_value &= ~(MODE_RESTART);  /* RSTART */
	ret = i2c_smbus_write_byte_data(client, REG_MODE, reg_mode_value|MODE_RESTART);
	if(ret < 0) {
		dev_err(&client->dev, "Error to restart battery info1\n");
		return ret;
	}
	ret = i2c_smbus_write_byte_data(client, REG_MODE, reg_mode_value|0);
	if(ret < 0) {
		dev_err(&client->dev, "Error to restart battery info2\n");
		return ret;
	}
	return 0;
}

static int cw2015_init_charger(void)
{
  int cnt = 0;
  	int i = 0;
	int ret = 0;
	char value = 0;
	char buffer[SIZE_BATINFO*2];
	short value16 = 0;
	struct i2c_client *client = NULL;

	if(NULL == chip)
		return -1;

	client = to_i2c_client(chip->dev);

#if FORCE_WAKEUP_CHIP
	value = MODE_SLEEP;
#else
	/* check if sleep mode, bring up */
	ret = i2c_smbus_read_byte_data(client, REG_MODE);
	if(ret < 0) {
		dev_err(&client->dev, "read mode\n");
		return ret;
	}

	value = ret;
#endif

	if((value & MODE_SLEEP_MASK) == MODE_SLEEP) {

		/* do wakeup cw2015 */
		ret = i2c_smbus_write_byte_data(client, REG_MODE, MODE_NORMAL);
		if(ret < 0) {
			dev_err(&client->dev, "Error update mode\n");
			return ret;
		}

    /* check 2015 if not set ATHD */
  	ret = i2c_smbus_read_byte_data(client, REG_CONFIG);
  	if(ret < 0) {
		  dev_err(&client->dev, "Error to read CONFIG\n");
		  return ret;
  	}
  	value = ret;
	printk("%s:%d\n",__func__,ret);

	  if ((value & 0xf8) != ATHD) {
	    dev_info(&client->dev, "the new ATHD have not set\n");
  	 	value &= 0x7;  /* clear ATHD */
	    value |= ATHD; 
	  /* set ATHD */
	    ret = i2c_smbus_write_byte_data(client, REG_CONFIG, value);
	    if(ret < 0) {
		    dev_err(&client->dev, "Error to set new ATHD\n");
	  	  return ret;
	    }
	  }
	  
	  /* check 2015 for update_flag */
  	ret = i2c_smbus_read_byte_data(client, REG_CONFIG);
	  if(ret < 0) {
	  	dev_err(&client->dev, "Error to read CONFIG\n");
	  	return ret;
	  }
  	value = ret;  	    	 
    /* not set UPDATE_FLAG,do update_battery_info  */
	  if (!(value & CONFIG_UPDATE_FLG)) {
	    dev_info(&client->dev, "update flag for new battery info have not set\n");
	    cw2015_verify_update_battery_info();
	  }
	
  	/* read origin info */
	  for(i=0; i<SIZE_BATINFO; i++) {
		ret = i2c_smbus_read_byte_data(client, REG_BATINFO+i);
		if(ret < 0) {
			dev_err(&client->dev, "Error read origin battery info @ offset %d, ret = 0x%x\n", i, ret);
			return ret;
		}

		buffer[i] = ret;
	  }

  	if(0 != memcmp(buffer, cw2015_bat_config_info, SIZE_BATINFO)) {
	  	dev_info(&client->dev, "battery info NOT matched.\n");
	 /* battery info not matched,do update_battery_info  */
	  	cw2015_verify_update_battery_info();
	  } else {
	  	dev_info(&client->dev, "battery info matched.\n");
	  }


		/* do wait valide SOC, if the first time wakeup after poweron */
		ret = i2c_smbus_read_word_data(client, REG_SOC);
		if(ret < 0) {
			dev_err(&client->dev, "Error read init SOC\n");
			return ret;
		}   
		value16 = ret;
		printk("value:%x",value16);

/*		while ((value16 == 0xff)||(cnt < 10000)) {     //SOC value is not valide or time is not over 3 seconds
		  ret = i2c_smbus_read_word_data(client, REG_SOC);
		  if(ret < 0) {
		  	dev_err(&client->dev, "Error read init SOC\n");
		  	return ret;
		  }   
		  value16 = ret;
		  cnt++;
		}
*/
/*		
		if(value16 == 0) {     //do QUICK_START
			ret = i2c_smbus_write_byte_data(client, REG_MODE, MODE_QUICK_START|MODE_NORMAL);
			if(ret < 0) {
				dev_err(&client->dev, "Error quick start1\n");
				return ret;
			}

			ret = i2c_smbus_write_byte_data(client, REG_MODE, MODE_NORMAL);
			if(ret < 0) {
				dev_err(&client->dev, "Error quick start2\n");
				return ret;
			}
		}
*/
	}

	return 0;
}

static void cw2015_lbirq_work(struct work_struct *work)
{
	struct cw2015_chip *chip;
	int lbatt;
	printk("%s\n", __func__);	

	chip = container_of(work, struct cw2015_chip, lwork.work);

	cw2015_read(REG_RRT_ALERT,&lbatt,chip->client);
	printk("ATHD:%x\n",lbatt);
	lbatt &= 0x7FFF;
	cw2015_write(REG_RRT_ALERT,lbatt,chip->client);
	cw2015_read(REG_RRT_ALERT,&lbatt,chip->client);
	printk("..........................ATHD:%x\n",lbatt);
	enable_irq(chip->irq);
//	cw2015_write(0x06,0x4000,chip->client);
//	max17044_burn(chip->client);

	//cancel_delayed_work(&chip->work);
}
static irqreturn_t cw2015_lowbatt_callback(int irq, void *dev_id)
{
	printk("%s\n", __func__);	
//	struct i2c_client *client = (struct i2c_client *)dev_id;
	disable_irq_nosync(irq);
	schedule_delayed_work(&chip->lwork, 0);	
	return IRQ_HANDLED;
}

static int cw2015_lowbatt_interrupt(struct i2c_client *client)
{
	int err = 0;
	printk("%s\n", __func__);

	if (client->irq) {
		err = request_irq(client->irq, cw2015_lowbatt_callback, IRQF_TRIGGER_LOW,
				  "battery", client);
		if (err < 0) {
			printk("%s(%s): Can't allocate irq %d\n", __FILE__, __func__, client->irq);
		}
	}
	return 0;
}

static int __devinit cw2015_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	ret = cw2015_get_version(client);
	if(ret < 0) {
		  dev_err(&client->dev, "can't detect cw2015\n");
		  return -EIO;
  	}

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->dev = &client->dev;
	chip->irq = client->irq;
	chip->pdata = client->dev.platform_data;

	i2c_set_clientdata(client, chip);
	mutex_init(&chip->rwlock);
	chip->battery.name		= "battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= cw2015_get_property;
	chip->battery.properties	= cw2015_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(cw2015_battery_props);
  
	chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
	chip->cap_level = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
	chip->health = POWER_SUPPLY_HEALTH_UNKNOWN;
  
	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		kfree(chip);
		return ret;
	}
	
//	cw2015_get_version(client);
//	cw2015  burn a new customer mode
	ret = cw2015_init_charger();
	if(ret < 0)
		return ret;

	INIT_DELAYED_WORK_DEFERRABLE(&chip->lwork, cw2015_lbirq_work);
	cw2015_lowbatt_interrupt(client);
	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, cw2015_work);
//	schedule_delayed_work(&chip->work, msecs_to_jiffies(CW2015_DELAY));	
	bat_wq = create_singlethread_workqueue("bat_wq");
	if (!bat_wq) {
		printk("can't create thread bat_wq in %s\n",__func__);
	}
	queue_delayed_work(bat_wq, &chip->work, 2*HZ);

	return 0;
}

static int __devexit cw2015_remove(struct i2c_client *client)
{
	struct cw2015_chip *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->battery);
	cancel_delayed_work(&chip->work);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static int cw2015_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct cw2015_chip *chip = i2c_get_clientdata(client);
	enable_irq_wake(gpio_to_irq(103));
	cancel_delayed_work_sync(&chip->work);
	return 0;
}

/* sometimes when device wakes up for couple of seconds and then sleep capacity and voltage readings was not updated,
 * that causes in some situations device to discharge to 3.2V battery voltage without application know about it.
 * so we update chip work 1 msecs after resume.
 */
static int cw2015_resume(struct i2c_client *client)
{
	struct cw2015_chip *chip = i2c_get_clientdata(client);
	disable_irq_wake(gpio_to_irq(103));
	//schedule_delayed_work(&chip->work, msecs_to_jiffies(CW2015_DELAY));
//	schedule_delayed_work(&chip->work, msecs_to_jiffies(1));
	queue_delayed_work(bat_wq, &chip->work, msecs_to_jiffies(1));
	return 0;
}

#else

#define cw2015_suspend NULL
#define cw2015_resume NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id cw2015_id[] = {
	{ "cw2015", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cw2015_id);

static struct i2c_driver cw2015_i2c_driver = {
	.driver	= {
		.name	= "cw2015",
	},
	.probe		= cw2015_probe,
	.remove		= __devexit_p(cw2015_remove),
	.suspend	= cw2015_suspend,
	.resume		= cw2015_resume,
	.id_table	= cw2015_id,
};

static int __init cw2015_init(void)
{
	int ret =0;
	ret = i2c_add_driver(&cw2015_i2c_driver);
	if(ret){
		printk(KERN_INFO "max17043 gas gauge :could not add i2c driver\n");
	  return ret;}
	  
	return 0;
}
module_init(cw2015_init);

static void __exit cw2015_exit(void)
{
	i2c_del_driver(&cw2015_i2c_driver);
}
module_exit(cw2015_exit);

MODULE_AUTHOR("Victor Liu <liuduogc@gmail.com>");
MODULE_DESCRIPTION("CW2015 Fuel Gauge");
MODULE_LICENSE("GPL");
