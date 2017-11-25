/*
 * Copyright(c) 2016, LG Electronics. All rights reserved.
 *
 * e-pack i2c device driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define pr_fmt(fmt)	"[E-PACK] %s : " fmt, __func__

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/firmware.h>

#include <linux/input/epack_core.h>
#include <linux/input/epack_audio.h>

#include "cmd.h"
#include "I2CMain.h"

#define DEFAULT_MONITOR_PERIOD_MS 30000
#define EPACK_DEFAULT_POLLING_TIME 1000
#define EPACK_I2C_NAME "epack"
#define AUTO_UPDATE_FLAG  0

static struct epack_dev_data *the_epack;
static bool has_already_flag = 0;

static int ep_id_status = 0;
static int ep_pwr_status = 0;

enum irq_type{
	IRQ_INIT,
	IRQ_EP_ID,
	IRQ_EP_PWR,
	IRQ_USB_PWR,
};
char *irq_str[4] = {"INIT","EP_ID","EP_PWR","USB_PWR"};


int epack_firmware_update(struct i2c_client *client, struct device *dev,int flag, char* fwpath)
{
	struct epack_dev_data *epack= dev_get_drvdata(dev);
	int result =0, retry_cnt=1;

	epack_log("%s start \n", __func__);

	result = get_epack_status();
	if(result!=EPACK_POWER_OK){
			epack_log("%s can't start update. result %d \n", __func__ , result);
			return  result;
	}

	while(retry_cnt <= MAX_RETRY_COUNT){
		result = cmd_run_cmd(client, CMD_GET_FLASHMODE, &(epack->slavemode));   //Slave Mode check
		if(result<=0) {
			epack_log("cmd_run_cmd(CMD_GET_FLASHMODE) is failed. return code %d  retry_cnt %d\n", result, retry_cnt++);
		} else {
			epack_log("slave mode is %d retry_cnt %d\n", epack->slavemode, retry_cnt);
			break;
		}
		msleep(50);
	}

	if(retry_cnt > MAX_RETRY_COUNT){
		return result;
	}
	retry_cnt = 1;

	if (epack->slavemode!= APROM_MODE)
		goto update;

	while(retry_cnt <= MAX_RETRY_COUNT){
		result = cmd_fw_version(client,true, &(epack->fwver)); 	//FW version check
		if(result == 0) {
			epack_log("cmd_fw_version is failed. return code %d retry_cnt %d\n", result, retry_cnt++);
		} else {
			epack_log("slave fw_version is %x retry_cnt %d \n", epack->fwver, retry_cnt);
			break;
		}
		msleep(50);
	}

	if(retry_cnt > MAX_RETRY_COUNT){
		return result;
	}
	retry_cnt = 1;

	while(retry_cnt <= MAX_RETRY_COUNT){
		result = cmd_hw_version(client,true, &(epack->hwver)); 	//HW version check
		if(result == 0) {
			epack_log("cmd_hw_version is failed. return code %d retry_cnt %d\n", result, retry_cnt++);
		} else {
			epack_log("slave hw_version is %x retry_cnt %d \n", epack->hwver, retry_cnt);
			break;
		}
		msleep(50);
	}

	if(retry_cnt > MAX_RETRY_COUNT){
		retry_cnt = 1;
		return result;
	}

	result = check_fw_update(epack->fwver,epack->hwver);   //check need update
	if(result==0 && the_epack->force_update == 0){
			epack_log("%s not need update. FW version is same \n", __func__);
			return  0;
	}

	result = 	cmd_sync_packno(client);                            // Sync Check
	if(result==0){
		epack_log("cmd_sync_packno is failed. return code %d\n", result);
		return result;
	}

	cmd_run_cmd(client, CMD_RUN_LDROM, NULL); 	// set LDROM_MODE

	result = cmd_run_cmd(client, CMD_GET_FLASHMODE, &(epack->slavemode));   //Slave Mode check
	if (epack->slavemode!= LDROM_MODE){
		epack_log("slave mode is %d. exit update.\n", epack->slavemode);
		cmd_run_cmd(client, CMD_RUN_APROM, NULL); // set APROM_MODE
		return 0;
	}

/*
	result = cmd_get_deviceID(client, 0, &(epack->devid));   //Slave Device ID read
	if(result<=0){
		epack_log("cmd_get_deviceID is failed. return code %d\n", result);
		goto out;
	} else {
		epack_log("deviceID is %x\n", epack->devid);
	}

	result = cmd_get_config(client, 0,  epack->devconfig);
	if(result<=0) {
		epack_log("cmd_get_config is failed. return code %d\n", result);
		goto out;
	} else {
		epack_log("config[0] is %x. config[1] is %x\n", epack->devconfig[0], epack->devconfig[1]);
	}
*/

update :

	if(flag)
		{
			result = cmd_update_aprom(client,1,fwpath,dev,0x0);
		}
	else
		{
			result = cmd_update_aprom(client,0,NULL,dev,epack->hwver);
		}

	//cmd_run_cmd(client, CMD_RUN_APROM, NULL); // set APROM_MODE

	if(result==0){
		epack_log("cmd_update_aprom is failed. return code %d\n", result);
		return result;
	}

	msleep(1000); //wait for MCU reboot delay

	result = cmd_fw_version(client,true, &(epack->fwver)); 	//FW version check
	if(result == 0) {
		epack_log("cmd_fw_version is failed. return code %d \n", result);
	} else {
		epack_log("slave fw_version is %x\n", epack->fwver);
	}

	return result;
}

static ssize_t show_fw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct epack_dev_data *epack= dev_get_drvdata(dev);
	struct i2c_client *client = epack->client;
	int result = 0;
	int ret =0;

	epack_log("%s start \n",__func__);
	result = cmd_fw_version(client,true, &(epack->fwver)); 	//FW version check
	result = cmd_hw_version(client,true, &(epack->hwver)); 	//HW version check
	if(result <= 0) {
		epack_log("cmd_fw_version is failed. return code %d\n", result);
		return sprintf(buf, "%s is failed.\n",__func__);
	}
	 else {
		epack_log("%s : hw_version is %x, fw_version is %x\n", __func__,  epack->hwver,epack->fwver);
		ret += sprintf(buf+ret, "=================\n");
		ret += sprintf(buf+ret, "hw_version : %x \n", epack->hwver);
		ret += sprintf(buf+ret, "fw_version : %x \n", epack->fwver);
		ret += sprintf(buf+ret, "=================\n");
		return ret;
	}
}
static ssize_t store_fw_version(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epack_dev_data *epack = dev_get_drvdata(dev);
	struct i2c_client *client = epack->client;

	int cmd,result = 0;
	char fwpath[MAX_FWPATH_SIZE] = {0};
	sscanf(buf,"%d %255s", &cmd,fwpath);
	the_epack->force_update = 1;

	epack_log("%s start \n",__func__);

	switch(cmd)
	{
		case 1 :
			epack_log("%s update from boot.img", __func__);
			mutex_lock(&epack->i2c_lock);
			result = epack_firmware_update(client, dev, 0, NULL);
			mutex_unlock(&epack->i2c_lock);
			break;

		case 2 :
			epack_log("%s FW path :  %s\n",__func__ ,fwpath);
			mutex_lock(&epack->i2c_lock);
		  result = epack_firmware_update(client, dev, 1, fwpath);
			mutex_unlock(&epack->i2c_lock);
			break;

		default :
			break;
	}
	the_epack->force_update = 0;
	return count;
}

static DEVICE_ATTR(fw_version, S_IRUGO | S_IWUSR | S_IWGRP, show_fw_version, store_fw_version);


static ssize_t show_cmd_test(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct epack_dev_data *epack = dev_get_drvdata(dev);
	struct i2c_client *client = epack->client;
	int ret =0,fail =0,version_fault=0;
	int result = 0;
/*
	ret += sprintf(buf+ret, "Usage : \n");
	ret += sprintf(buf+ret, "cat fw_version : FW version check \n");
	ret += sprintf(buf+ret, "echo 2 > cmd_test : Sync Check\n");
	ret += sprintf(buf+ret, "echo 3 > cmd_test : Slave Mode check \n");
	ret += sprintf(buf+ret, "echo 4 > cmd_test : Slave Device ID read \n");
	ret += sprintf(buf+ret, "echo 5 > cmd_test : Slave config read \n");
	ret += sprintf(buf+ret, "echo 6 > cmd_test : set LDROM_MODE\n");
	ret += sprintf(buf+ret, "echo 7 > cmd_test : set APROM_MODE\n");
	ret += sprintf(buf+ret, "echo 8 > cmd_test : set APROM_MODE\n");
	ret += sprintf(buf+ret, "echo 1 go > fw_version : Quick FW update \n");
*/
	while(1)
	{
			result = cmd_fw_version(client,true, &(epack->fwver)); 					//FW version check
			if(result<=0)
				{
					epack_log("%s : cmd_fw_version is failed. return code %d\n", __func__, result);
					fail++;
				}
			else
				{
					epack_log("%s : fw_version is %x\n", __func__, epack->fwver);
				}
			if(epack->fwver != 0x02)
				{
					epack_log("%s : fw_version is fault.\n", __func__);
					version_fault++;
				}
			ret++;
			epack_log("[I2C stability test] test count : %d,  i2c success : %d, fail : %d, version_fault : %d\n",ret,(ret-fail-version_fault),fail, version_fault);
			msleep(1000);
	}

	return ret;
}
static ssize_t store_cmd_test(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epack_dev_data *epack = dev_get_drvdata(dev);
	struct i2c_client *client = epack->client;
	int temp = simple_strtoul(buf, NULL, 10);
	int result = 0;
	int ret=0;
	char buf2[256] ={0,};

	epack_log("%s CMD : %d\n",__func__ ,temp);
	switch(temp)
	{
		case 1:
			result = cmd_fw_version(client,true, &(epack->fwver)); 					//FW version check
			if(result<=0)
				{
					epack_log("%s : cmd_fw_version is failed. return code %d\n", __func__, result);
					ret += sprintf(buf2+ret, "%s : cmd_fw_version is failed.",__func__);
				}
			else
				{
					epack_log("%s : fw_version is %x\n", __func__, epack->fwver);
					ret += sprintf(buf2+ret, "fw_version : %x \n", epack->fwver);
				}
			break;

		case 2:
			result = 	cmd_sync_packno(client);                            // Sync Check
			if(result<=0)
				{
					epack_log("%s : cmd_sync_packno is failed. return code %d\n", __func__, result);
					ret = sprintf(buf2, "%s : cmd_sync_packno is failed.",__func__);
				}
			break;

		case 3:
			result = cmd_run_cmd(client, CMD_GET_FLASHMODE, &(epack->slavemode));   //Slave Mode check
			if(result<=0)
				{
					epack_log("%s : cmd_run_cmd(CMD_GET_FLASHMODE) is failed. return code %d\n", __func__, result);
					ret = sprintf(buf2, "%s : cmd_run_cmd(CMD_GET_FLASHMODE) is failed. return code %d\n", __func__, result);
				}
			else
				{
					epack_log("%s : slave mode is %d\n", __func__, epack->slavemode);
					ret = sprintf(buf2, "%s : slave mode is %d\n", __func__, epack->slavemode);
				}
			break;

		case 4:
			result = cmd_get_deviceID(client, 0, &(epack->devid));                           //Slave Device ID read
			if(result<=0)
				{
					epack_log("%s : cmd_get_deviceID is failed. return code %d\n", __func__, result);
					ret += sprintf(buf2+ret, "%s : cmd_get_deviceID is failed. return code %d\n", __func__, result);
				}
			else
				{
					epack_log("%s : deviceID is %x\n", __func__, epack->devid);
					ret += sprintf(buf2+ret, "%s : deviceID is %x\n", __func__, epack->devid);
				}
			break;

		case 5:
			result = cmd_get_config(client, 0,  epack->devconfig);	     //Slave config read
			if(result<=0)
				{
					epack_log("%s : cmd_get_config is failed. return code %d\n", __func__, result);
					ret += sprintf(buf2+ret, "%s : cmd_get_config is failed. return code %d\n", __func__, result);
				}
			else
				{
					epack_log("%s : config[0] is %x. config[1] is %x\n", __func__, epack->devconfig[0], epack->devconfig[1]);
					ret += sprintf(buf2+ret, "%s : config[0] is %x. config[1] is %x\n", __func__, epack->devconfig[0], epack->devconfig[1]);
				}
			break;

		case 6:
			result = cmd_run_cmd(client, CMD_RUN_LDROM, NULL); // set LDROM_MODE
			if(result<=0)
				{
					epack_log("%s : cmd_run_cmd(CMD_RUN_LDROM) is failed. return code %d\n", __func__, result);
					ret += sprintf(buf2+ret, "%s : cmd_run_cmd(CMD_RUN_LDROM) is failed. return code %d\n", __func__, result);
				}
			break;

		case 7:
			result = cmd_run_cmd(client, CMD_RUN_APROM, NULL); // set CMD_RUN_APROM
			if(result<=0)
				{
					epack_log("%s : cmd_run_cmd(CMD_RUN_APROM) is failed. return code %d\n", __func__, result);
					ret += sprintf(buf2+ret, "%s : cmd_run_cmd(CMD_RUN_APROM) is failed. return code %d\n", __func__, result);
				}
			break;

		case 8:
			result = cmd_hw_version(client,true, &(epack->hwver)); 	//HW version check
			if(result == 0) {
				epack_log("cmd_hw_version is failed. return code %d \n", result);
				} else {
				epack_log("slave hw_version is %x\n", epack->hwver);
				break;
				}

			result = cmd_update_aprom(client,0,NULL,dev,epack->hwver);    //update APROM
			if(result<=0)
				{
					epack_log("%s : cmd_update_aprom is failed. return code %d\n", __func__, result);
					ret += sprintf(buf2+ret, "%s : cmd_update_aprom is failed. return code %d\n", __func__, result);
				}
				break;

		case 9:
			result = cmd_hw_version(client,true, &(epack->hwver)); 					//HW version check
			if(result<=0)
				{
					epack_log("%s : cmd_hw_version is failed. return code %d\n", __func__, result);
					ret += sprintf(buf2+ret, "%s : cmd_hw_version is failed.",__func__);
				}
			else
				{
					epack_log("%s : hw_version is %x\n", __func__, epack->hwver);
					ret += sprintf(buf2+ret, "hw_version : %x \n", epack->hwver);
				}
			break;

		default:
			break;
	}

	return count;
}

static DEVICE_ATTR(cmd_test, S_IRUGO | S_IWUSR | S_IWGRP, show_cmd_test, store_cmd_test);

static ssize_t show_detect_pin_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct epack_dev_data *epack= dev_get_drvdata(dev);
	int pin_status=-1;

	epack_log("%s\n", __func__);

	if (!gpio_is_valid(epack->id_gpio))
		{
			epack_log("%s gpio_get fail\n", __func__);
	 		return 0;
		}

	pin_status = gpio_get_value(epack->id_gpio);
	epack_log("detect pin(gpio %d) status : %d\n",epack->id_gpio,  pin_status);

	return sprintf(buf, "detect pin(gpio %d) status : %d\n", epack->id_gpio,pin_status);
}

static DEVICE_ATTR(detect_pin_status, S_IRUGO, show_detect_pin_status, NULL);

static ssize_t show_polling_time_snd(struct device *dev
		, struct device_attribute *attr, char *buf)
{
	struct epack_dev_data *epack= dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", epack->polling_time_snd);
}

static ssize_t store_polling_time_snd(struct device *dev
		,struct device_attribute *attr, const char *buf, size_t count)
{
	struct epack_dev_data *epack = dev_get_drvdata(dev);
	int temp = simple_strtoul(buf, NULL, 10);

	if (temp)
		epack->polling_time_snd = temp;
	else
		epack->polling_time_snd = EPACK_DEFAULT_POLLING_TIME;

	pr_info("epack->polling_time_snd: (%d)\n", temp);
	return count;
}

static DEVICE_ATTR(polling_time_snd, S_IRUGO | S_IWUSR | S_IWGRP, show_polling_time_snd, store_polling_time_snd);

static ssize_t show_polling_time_pwr (struct device *dev
				,struct device_attribute *attr, char *buf)
{
	struct epack_dev_data *epack= dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", epack->polling_time_pwr);
}
static ssize_t store_polling_time_pwr(struct device *dev
				,struct device_attribute *attr, const char *buf, size_t count)
{
	struct epack_dev_data *epack = dev_get_drvdata(dev);
	int temp = simple_strtoul(buf, NULL, 10);
	pr_info("\n");
	if (temp)
		epack->polling_time_pwr = temp;

	pr_info("epack->polling_time_pwr: (%d)\n", temp);
	return count;
}
static DEVICE_ATTR(polling_time_pwr, S_IRUGO | S_IWUSR | S_IWGRP, show_polling_time_pwr, store_polling_time_pwr);

int epack_sysfs_init(struct epack_dev_data *epack)
{
	int err = 0;
	struct i2c_client *client = epack->client;

	err = device_create_file(&client->dev, &dev_attr_polling_time_snd);
	if (err)
		epack_log("failed create file polling_time_snd\n");

	err = device_create_file(&client->dev, &dev_attr_polling_time_pwr);
	if (err)
		epack_log("failed create file polling_time_pwr\n");

	err = device_create_file(&client->dev, &dev_attr_detect_pin_status);
	if (err)
		epack_log("failed create file detect_pin_status\n");

	err = device_create_file(&client->dev, &dev_attr_fw_version);
	if (err)
		epack_log("failed create file fw_version\n");

	err = device_create_file(&client->dev, &dev_attr_cmd_test);
	if (err)
		epack_log("failed create file cmd_test\n");

	return err;
}
/*
static void epack_sysfs_deinit(struct epack_dev_data *epack)
{
	struct i2c_client *client = epack->client;

	device_remove_file(&client->dev, &dev_attr_polling_time_snd);
	device_remove_file(&client->dev, &dev_attr_polling_time_pwr);
}
*/

bool check_fw_update(unsigned int fwver,unsigned int hwver)
{
	if(hwver == 0x04)
		{
			epack_log("%s : slave_fw_ver :%x master_fw_ver: %x\n", __func__, fwver, the_epack->bin_fwver[0]);
			return (fwver!= the_epack->bin_fwver[0]);
		}
	else if(hwver == 0x05)
		{
			epack_log("%s : slave_fw_ver :%x master_fw_ver: %x\n", __func__, fwver, the_epack->bin_fwver[1]);
			return (fwver!= the_epack->bin_fwver[1]);
		}
	else
		{
			epack_log("%s : slave_fw_ver :%x unknown hw_ver: %x\n", __func__, fwver, hwver);
			return 0;
		}

}

int get_fw_ver_from_file(char* fwpath, struct device *dev)
{
  int result,fwver,hwver;
  const struct firmware *fw;

  result = request_firmware(&fw, fwpath, dev);
  if (result < 0)
    {
      epack_log("%s fail to request_firmware fwpath: %s (result:%d)\n",__func__, fwpath, result);
      return result;
    }
  fwver = fw->data[(fw->size)-1];
  hwver = fw->data[(fw->size)-2];

  epack_log("%s fw version: %x, hw version : %x, fw size: %d, data_pointer: %p, fwpath: %s\n", __func__, fwver,hwver, fw->size, fw->data, fwpath);
  release_firmware(fw);

  return fwver;
}

static int check_vbus_source(struct epack_dev_data *epack)
{
	if (!gpio_get_value(epack->usb_power_gpio))
		return FROM_USB_PORT;
	if (!gpio_get_value(epack->pack_power_gpio))
		return FROM_EPACK;
	return NONE;
}

int get_vbus_source(void)
{
	if (!the_epack) {
		epack_log("%s : EPACK is not initialized yet \n",__func__);
		return NONE;
	}
	return check_vbus_source(the_epack);
}

static void epack_notify_vbus_src(struct work_struct *work)
{
	epack_log("vbus source is chagned\n");
}

char *vs_str[3] = {"NONE","from_PACK","from_USB"};
static int update_vbus_source(struct epack_dev_data *epack,int update_reason)
{
	int vbus_src;

	vbus_src = check_vbus_source(epack);
	epack_log("irq %-7s vbus_src  : %s -> %s \n"
		,irq_str[update_reason],vs_str[epack->vbus_src],vs_str[vbus_src]);

	if (vbus_src != epack->vbus_src) {
		epack->vbus_src = vbus_src;
		//schedule_delayed_work(&epack->notify_vbus_src_work,
		//		msecs_to_jiffies(0));
	}
	return vbus_src;
}

static int check_epack_status(struct epack_dev_data *epack)
{

	ep_id_status = gpio_get_value(epack->id_gpio);
	ep_pwr_status = !gpio_get_value(epack->pack_power_gpio);

	if (!ep_id_status)
		return EPACK_ABSENT;
	if (ep_pwr_status)
		return EPACK_POWER_OK;

	return EPACK_POWER_LOW;
}
module_param(ep_id_status, int, 0444);
module_param(ep_pwr_status, int, 0444);

int get_epack_status(void)
{
	if (!the_epack) {
		epack_log("%s : EPACK is not initialized yet \n",__func__);
		return EPACK_ABSENT;
	}
	return check_epack_status(the_epack);
}

static void notify_epack_ready(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct epack_dev_data *epack = container_of(dwork,
		struct epack_dev_data, notify_epack_ready_work);

	epack_log("%s: epack power ok, set usb_psy epack *******************\n", __func__);
	// notify_to_audio;
	audio_input_set_sdev_name(epack, 1);
	// notify_to_usb;
	power_supply_set_usb_epack(epack->usb_psy, 1);
  if(AUTO_UPDATE_FLAG) {
    epack_log("%s: Epack power ok, start firmware update\n", __func__);
    if(&(epack->client->dev) == NULL)
      {
				epack_log("%s: dev is null\n", __func__);
        return;
      }
    if(has_already_flag ==0){
			mutex_lock(&epack->i2c_lock);
 			epack_firmware_update(epack->client, &(epack->client->dev), 0, NULL);
			mutex_unlock(&epack->i2c_lock);
 		}
		else
			epack_log("%s: has_already_flag %d don't update\n", __func__, has_already_flag);
    has_already_flag = 1;
    epack_log("%s: has_already_flag %d\n", __func__, has_already_flag);
  }
}

static void notify_epack_unready(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct epack_dev_data *epack = container_of(dwork,
		struct epack_dev_data, notify_epack_unready_work);

	epack_log("%s: epack power low or absent ************************\n",__func__);
	// notify_to_audio;
	audio_input_set_sdev_name(epack, 0);
	// notify_to_usb;
	has_already_flag = 0;
	epack_log("%s: has_already_flag %d\n", __func__, has_already_flag);
	power_supply_set_usb_epack(epack->usb_psy, 0);
}

char *es_str[3] = {"ABSENT","POWER_LOW","POWER_OK"};

static void update_epack_status(struct epack_dev_data *epack,int update_reason)
{
	int status;

	status = check_epack_status(epack);

	epack_log("irq %-7s epack_sts : %s -> %s \n"
		,irq_str[update_reason],es_str[epack->last_status],es_str[status]);
	if (status == epack->last_status)
		return;

	if(status == EPACK_ABSENT) {
		cancel_delayed_work(&epack->pwr_monitor_work);
	} else {
		if (epack->last_status == EPACK_ABSENT) {
			cancel_delayed_work(&epack->pwr_monitor_work);
			schedule_delayed_work(&epack->pwr_monitor_work
									, msecs_to_jiffies(5000));
		}
	}

	if (epack->last_status == EPACK_POWER_OK) {
		epack->last_status = status;
		schedule_delayed_work(&epack->notify_epack_unready_work,
				msecs_to_jiffies(0));
	} else {
		epack->last_status = status;
		if (status == EPACK_POWER_OK)
			schedule_delayed_work(&epack->notify_epack_ready_work,
				msecs_to_jiffies(0));
	}

	return;
}

static irqreturn_t epack_detect_handler(int irq, void *data)
{
	struct epack_dev_data *epack = (struct epack_dev_data *)data;
	epack_log("irq %-7s occurs!\n",irq_str[IRQ_EP_ID]);

	/* i2c register update takes time, 30msec sleep required as per HPG */
	msleep(30);

	mutex_lock(&epack->irq_lock);
	update_epack_status(epack,IRQ_EP_ID);
	mutex_unlock(&epack->irq_lock);

	return IRQ_HANDLED;
}

static irqreturn_t epack_power_change_handler(int irq, void *data)
{
	struct epack_dev_data *epack = (struct epack_dev_data *)data;
	epack_log("irq %-7s occurs!\n",irq_str[IRQ_EP_PWR]);

	mutex_lock(&epack->irq_lock);
	update_epack_status(epack,IRQ_EP_PWR);
	update_vbus_source(epack,IRQ_EP_PWR);
	mutex_unlock(&epack->irq_lock);

	return IRQ_HANDLED;
}

static irqreturn_t usb_power_change_handler(int irq, void *data)
{
	struct epack_dev_data *epack = (struct epack_dev_data *)data;
	epack_log("irq %-7s occurs!\n",irq_str[IRQ_USB_PWR]);

	mutex_lock(&epack->irq_lock);
	update_vbus_source(epack,IRQ_USB_PWR);
	mutex_unlock(&epack->irq_lock);

	return IRQ_HANDLED;
}

static void epack_parse_dt(struct epack_dev_data *epack)
{
	struct device *cdev = &epack->client->dev;
	struct device_node *np = cdev->of_node;
	int rc = 0;

	if (cdev == NULL || np == NULL) {
		epack_log( "can't get client info. select default value.\n");
		epack->polling_time_pwr = DEFAULT_MONITOR_PERIOD_MS;
		epack->polling_time_snd = 1000;
		epack->id_gpio = 62;
		epack->pack_power_gpio = 35;
		epack->usb_power_gpio = 34;
		epack->ovp_sw_pcon_gpio = 50;
		return;
	}

	rc = of_property_read_u32(np, "polling_time_pwr",&epack->polling_time_pwr);
	if (rc < 0) {
		epack_log( "polling time for power is not available\n");
		epack->polling_time_pwr = DEFAULT_MONITOR_PERIOD_MS; // 30sec
	}

	rc = of_property_read_u32(np, "polling_time_snd",&epack->polling_time_snd);
	if (rc < 0) {
		epack_log( "polling time for sound is not available\n");
		epack->polling_time_snd = 1000; // 1sec
	}

	epack->id_gpio  = of_get_named_gpio_flags(np, "id-gpio", 0, NULL);
	if (rc < 0) {
		epack_log( "irq_gpio is not available\n");
		epack->id_gpio = 62;
	}

	epack->pack_power_gpio = of_get_named_gpio_flags(np, "pack-power-gpio", 0, NULL);
	if (rc < 0) {
		epack_log( "irq_gpio is not available\n");
		epack->pack_power_gpio = 35;
	}

	epack->usb_power_gpio = of_get_named_gpio(np, "usb-power-gpio", 0);
	if (rc < 0) {
		epack_log( "irq_gpio is not available\n");
		epack->usb_power_gpio = 34;
	}

	epack->ovp_sw_pcon_gpio  = of_get_named_gpio(np, "ovp-sw-pcon-gpio", 0);
	if (rc < 0) {
		epack_log( "irq_gpio is not available\n");
		epack->ovp_sw_pcon_gpio = 50;
	}

	return;
}

static int epack_gpio_init(struct epack_dev_data *epack)
{
	int irq, ret;

	/* EP_ID */
	if (gpio_is_valid(epack->id_gpio)) {
		ret = gpio_request(epack->id_gpio, "epack_detect");
		if (ret) {
			epack_log("id_gpio request failed, ret=%d", ret);
			goto err_detect_irq;
		}
		ret = gpio_direction_input(epack->id_gpio);
		if (ret) {
			epack_log("set_direction for id_gpio failed\n");
			goto err_detect_irq;
		}
		irq = gpio_to_irq(epack->id_gpio);
		if (irq <0) {
			epack_log("Invalid id_gpio irq = %d\n", irq);
			goto err_detect_irq;
		}
		ret = devm_request_threaded_irq(&(epack->client)->dev,irq
				, NULL, epack_detect_handler,
				IRQF_ONESHOT | IRQF_TRIGGER_FALLING| IRQF_TRIGGER_RISING,
				"epack_detect_irq", epack);
		if (ret){
			epack_log("Failed request_irq irq=%d, gpio=%d ret=%d\n",
					irq,epack->id_gpio,ret);
			goto err_detect_irq;
		}
		enable_irq_wake(irq);
	} else {
		epack_log("Invalid id-gpio\n");
		goto err_detect_irq;
	}

	/* APPS_IN_N (INB_OK) */
	if (gpio_is_valid(epack->pack_power_gpio)) {
		ret = gpio_request(epack->pack_power_gpio, "epack_power_ok");
		if (ret) {
			epack_log("pack_power_gpio request failed, ret=%d", ret);
			goto err_pack_power_irq;
		}
		ret = gpio_direction_input(epack->pack_power_gpio);
		if (ret) {
			epack_log("set_direction for pack_power_gpio failed\n");
			goto err_pack_power_irq;
		}
		irq = gpio_to_irq(epack->pack_power_gpio);
		if (irq <0) {
			epack_log("Invalid pack_power_gpio irq = %d\n", irq);
			goto err_pack_power_irq;
		}
		ret = devm_request_threaded_irq(&(epack->client)->dev,irq
				, NULL, epack_power_change_handler,
				IRQF_ONESHOT | IRQF_TRIGGER_FALLING| IRQF_TRIGGER_RISING,
				"epack_power_chnage_irq", epack);
		if (ret){
			epack_log("Failed request_irq irq=%d, gpio=%d ret=%d\n",
					irq,epack->pack_power_gpio,ret);
			goto err_pack_power_irq;
		}
		enable_irq_wake(irq);
	} else {
		epack_log("Invalid pack-power-gpio\n");
		goto err_pack_power_irq;
	}

	/* CHARGER_IN_N (INA_OK) */
	if (gpio_is_valid(epack->usb_power_gpio)) {
		ret = gpio_request(epack->usb_power_gpio,"usb_power_ok");
		if (ret) {
			epack_log("usb_power_gpio request failed, ret=%d", ret);
			goto err_usb_power_gpio;
		}
		ret = gpio_direction_input(epack->usb_power_gpio);
		if (ret) {
			epack_log("set_direction for usb_power_gpio failed\n");
			goto err_usb_power_gpio;
		}
		irq = gpio_to_irq(epack->usb_power_gpio);
		if (irq <0) {
			epack_log("Invalid usb_power_gpio irq = %d\n", irq);
			goto err_usb_power_gpio;
		}
		ret = devm_request_threaded_irq(&(epack->client)->dev,irq
				, NULL, usb_power_change_handler,
				IRQF_ONESHOT | IRQF_TRIGGER_FALLING| IRQF_TRIGGER_RISING,
				"usb_power_chnage_irq", epack);
		if (ret){
			epack_log("Failed request_irq irq=%d, gpio=%d ret=%d\n",
					irq,epack->usb_power_gpio,ret);
			goto err_usb_power_gpio;
		}
		enable_irq_wake(irq);
	}

	/* OVP_SW_PCON*/
	if (gpio_is_valid(epack->ovp_sw_pcon_gpio)) {
		ret = gpio_request(epack->ovp_sw_pcon_gpio, "ovp_sw_pcontrol");
		if (ret) {
			epack_log("ovp_sw_pcontrol request failed, ret=%d", ret);
			goto err_ovp_sw_pcon_gpio;
		}
		ret = gpio_direction_output(epack->ovp_sw_pcon_gpio,0);
		if (ret) {
			epack_log("set_direction for ovp_sw_pcontrol failed\n");
			goto err_ovp_sw_pcon_gpio;
		}
	}

	return 0;

err_ovp_sw_pcon_gpio:
	if(gpio_is_valid(epack->ovp_sw_pcon_gpio))
		gpio_free(epack->ovp_sw_pcon_gpio);
err_usb_power_gpio:
	if(gpio_is_valid(epack->usb_power_gpio))
		gpio_free(epack->usb_power_gpio);
err_pack_power_irq:
	if(gpio_is_valid(epack->pack_power_gpio))
		gpio_free(epack->pack_power_gpio);
err_detect_irq:
	if(gpio_is_valid(epack->id_gpio))
		gpio_free(epack->id_gpio);

	return -1;
}

static int epack_init_workqueue(struct epack_dev_data *epack)
{
	epack->wq = create_singlethread_workqueue("epack_wq");
	epack_log("create_singlethread_workqueue\n");

	if (!epack->wq) {
		epack_log("failed to create workqueue\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&epack->audio_work, epack_audio_work_func);
	INIT_DELAYED_WORK(&epack->pwr_monitor_work, epack_pwr_monitor_work);
	INIT_DELAYED_WORK(&epack->notify_epack_ready_work, notify_epack_ready);
	INIT_DELAYED_WORK(&epack->notify_epack_unready_work, notify_epack_unready);
	INIT_DELAYED_WORK(&epack->notify_vbus_src_work, epack_notify_vbus_src);

	//schedule_delayed_work(&epack->audio_work, 0);
	//schedule_delayed_work(&epack->power_work, 0);

	return 0;
}

static int epack_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct epack_dev_data *epack;
	struct device *cdev = &client->dev;
	struct power_supply *usb_psy;

	int ret = 0;

	epack_log("probe start\n");
	//dump_stack();

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("usb supply not found, deferring probe\n");
		return -EPROBE_DEFER;
	}

	if (!i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_BYTE_DATA |I2C_FUNC_SMBUS_WORD_DATA)) {
		epack_log( "smbus data not supported!\n");
		return -EIO;
	}

	epack = devm_kzalloc(cdev, sizeof(struct epack_dev_data), GFP_KERNEL);
	if (!epack) {
		epack_log( "can't alloc epack_dev_data\n");
		return -ENOMEM;
	}

	audio_input_init(client , epack);
	epack->client = client;
	i2c_set_clientdata(client, epack);

	epack_parse_dt(epack);

	if (epack_gpio_init(epack)) {
		epack_log("epack_gpio_init is failed-%d\n", ret);
		return -EIO;
	}

	epack_sysfs_init(epack);

	mutex_init(&epack->irq_lock);
	mutex_init(&epack->i2c_lock);

	//epack_detect_handler(epack->id_gpio, epack);

	ret = epack_init_workqueue(epack);
	if (ret)
		epack_log("epack_init_workqueue is failed-%d\n", ret);

	epack->usb_psy = usb_psy;
	the_epack = epack;
	the_epack->force_update = 0;

	/* get bin FW version*/
	the_epack->bin_fwver[0] = get_fw_ver_from_file(DEFAULT_FW_PATH_REV_04, cdev);
	the_epack->bin_fwver[1] = get_fw_ver_from_file(DEFAULT_FW_PATH_REV_05, cdev);

	update_epack_status(epack,IRQ_INIT);
	update_vbus_source(epack,IRQ_INIT);

	epack_log("probe done\n");

	return ret;
}

static int epack_remove(struct i2c_client *client)
{
	struct epack_dev_data *epack = i2c_get_clientdata(client);
	struct device *cdev = &client->dev;

	if (!epack) {
		epack_log("epack is null\n");
		return -ENODEV;
	}

	if(gpio_is_valid(epack->id_gpio))
		gpio_free(epack->id_gpio);
	if(gpio_is_valid(epack->pack_power_gpio))
		gpio_free(epack->pack_power_gpio);
	if(gpio_is_valid(epack->usb_power_gpio))
		gpio_free(epack->usb_power_gpio);
	if(gpio_is_valid(epack->ovp_sw_pcon_gpio))
		gpio_free(epack->ovp_sw_pcon_gpio);

	i2c_set_clientdata(client, NULL);
	devm_kfree(cdev, epack);
	return 0;
}

static void epack_shutdown(struct i2c_client *client)
{
	return;
}

#ifdef CONFIG_PM
static int epack_suspend(struct device *dev)
{
	return 0;
}

static int epack_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops epack_dev_pm_ops = {
	.suspend = epack_suspend,
	.resume  = epack_resume,
};
#endif

static const struct i2c_device_id epack_id_table[] = {
	{EPACK_I2C_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, epack_id_table);

#ifdef CONFIG_OF
static struct of_device_id epack_match_table[] = {
	{ .compatible = "em-tech,epack",},
	{ },
};
#else
#define epack_match_table NULL
#endif


static struct i2c_driver epack_i2c_driver = {
	.driver = {
		.name = EPACK_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = epack_match_table,
#ifdef CONFIG_PM
		.pm = &epack_dev_pm_ops,
#endif
	},
	.probe = epack_probe,
	.remove = epack_remove,
	.shutdown = epack_shutdown,
	.id_table = epack_id_table,
};

static __init int epack_i2c_init(void)
{
	return i2c_add_driver(&epack_i2c_driver);
}
module_init(epack_i2c_init);

static __exit void epack_i2c_exit(void)
{
	i2c_del_driver(&epack_i2c_driver);
}
module_exit(epack_i2c_exit);

MODULE_DESCRIPTION("I2C bus driver for E-Pack");
MODULE_AUTHOR("Sungho Ji <sungho.ji@lge.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.4");


