/*
 *  Copyright (C) 2012 rockchip Electronics
 *  zyw <zyw@rock-chips.com>
 * 
 */

#include <common.h>
#include <power/pmic.h>
#include <power/battery.h>
#include <errno.h>
#include <asm/unaligned.h>
#include <power/rockchip_power.h>
#include <asm/arch/rkplat.h>
#include <asm/gpio.h>

DECLARE_GLOBAL_DATA_PTR;

#define TEMP_K              2731
#define MIN_CHARGE_TEMPERATURE       0
#define MAX_CHARGE_TEMPERATURE       450
#define BQ40z50_REG_TEMP		0x08
#define BQ40z50_REG_VOLT		0x09
#define BQ40z50_REG_AI			0x0A
#define BQ40z50_REG_STATUS		0x16 //BatteryStatus
#define BQ40z50_REG_TTE			0x06
#define BQ40z50_REG_TTF			0x05
#define BQ40z50_REG_TTECP		0x12 //AverageTimeToEmpty
#define BQ40z50_REG_RSOC		0x0D /* Relative State-of-Charge */
#define BQ40z50_REG_ECC			0x2F /* Authenticate */
#define BQ40z50_REG_CELLVOLT1	0x3F
#define BQ40z50_REG_CELLVOLT2	0x3E

#define CONFIG_BQ40Z50_I2C_ADDR      0x0b

#define BATTERY_IN 0
#define BATTERY_OUT 1
#define msleep(a) udelay(a * 1000)

struct fdt_gpio_state bat_det_gpio;
struct fdt_gpio_state charge_gpio;

int cw201x_charge_parse_dt(const void *blob)
{
	int node;
	node = fdt_node_offset_by_compatible(blob,0, "cw201x");
	if (node < 0) {
		debug("can't find dts node for ext battery\n");
		return -ENODEV;
	}
	if (!fdt_device_is_available(blob,node)) {
		debug("device battery is disabled\n");
		return -EPERM;
	}
	fdtdec_decode_gpio(blob, node, "charge_gpio", &charge_gpio);
	charge_gpio.flags = !(charge_gpio.flags  & OF_GPIO_ACTIVE_LOW);
	gpio_request(charge_gpio.gpio, "cw201x-charge-gpio");
	gpio_direction_output(charge_gpio.gpio, !charge_gpio.flags);
	return 0;
}

/*
return 0: bat exist
return 1: bat no exit
*/
int bq40z50_battery_parse_dt(const void *blob)
{
	int node;
	node = fdt_node_offset_by_compatible(blob,0, "bq40z50");
	if (node < 0) {
		debug("can't find dts node for ext battery\n");
		return -ENODEV;
	}
	if (!fdt_device_is_available(blob,node)) {
		debug("device battery is disabled\n");
		return -EPERM;
	}
	fdtdec_decode_gpio(blob, node, "irq-gpio", &bat_det_gpio);
	gpio_request(bat_det_gpio.gpio, "ext-battery-gpio");

	return 0;
}
/*
return 1: bat exist
return 0: bat no exit
*/
int bq40z50_battery_is_exist(void)
{
	int i,level;

	if (bat_det_gpio.gpio < 0)
		return 0;
	
	for(i=0; i<3; i++)
	{
		level = gpio_get_value(bat_det_gpio.gpio);
		if(level < 0)
		{
			printf("%s:get pin level again,pin=%d,i=%d\n",__FUNCTION__,bat_det_gpio.gpio,i);
			msleep(1);
			continue;
		}
		else
			break;
	}
	if(level < 0)
	{
		printf("%s:get pin level  err!\n",__FUNCTION__);
		return 0;
	}
	
	return (level == BATTERY_OUT ? 0 : 1);

}

int bq40z50_battery_init(void)
{
	bat_det_gpio.gpio = -1;
	charge_gpio.gpio = -1;
	bq40z50_battery_parse_dt(gd->fdt_blob);
	cw201x_charge_parse_dt(gd->fdt_blob);
	return 0;
}

static int bq40z50_battery_temperature(void)
{
	int ret = 0;
	int temp = 0;
	u8 buf[2] ={0};

	ret = i2c_read(CONFIG_BQ40Z50_I2C_ADDR,BQ40z50_REG_TEMP,1,buf,2);
	if (ret<0) {
		//printf("err:%s --ret = %d\n",__FUNCTION__,ret);
		return 125;
	}
	temp = get_unaligned_le16(buf);
	temp = temp - TEMP_K;  //K

	return temp;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq40z50_battery_rsoc(void)
{
	int ret;
	int rsoc = 0;
	u8 buf[2];

	ret = i2c_read(CONFIG_BQ40Z50_I2C_ADDR,BQ40z50_REG_RSOC, 1,buf, 2);
	if (ret<0) {
		//printf("err:%s --ret = %d\n",__FUNCTION__,ret);
		return 10;
	}
	rsoc = get_unaligned_le16(buf);

	return rsoc;
}

/*
get battery status, contain capacity, voltage, status
struct battery *batt_status:
voltage_uV. battery voltage
capacity.   battery capacity
state_of_chrg: 0. no charger; 1. usb charging; 2. AC charging
*/
int bq40z50_battery_get_status(struct battery *batt_status)
{
    int temp, rsoc;
    i2c_set_bus_num(3);
    i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_BQ40Z50_I2C_ADDR);
    i2c_set_bus_speed(CONFIG_SYS_I2C_SPEED);

    temp = bq40z50_battery_temperature();
	rsoc = bq40z50_battery_rsoc();
	printf("%s --temp = %d rsoc = %d\n",__FUNCTION__,temp, rsoc);
#if defined(CONFIG_DJI_ZS600A)
	if (temp >= 0 && rsoc > 0)
		batt_status->voltage_uV = 1;
	else if (temp >= -100 && rsoc >= 10)
		batt_status->voltage_uV = 1;
	else if (temp >= -200 && rsoc >= 40)
		batt_status->voltage_uV = 1;
	else
		batt_status->voltage_uV = -1;
#else
	if (temp >= 0 && rsoc > 0)
		batt_status->voltage_uV = 1;
	else if (temp >= -100 && rsoc >= 40)
		batt_status->voltage_uV = 1;
	else if (temp >= -200 && rsoc >= 70)
		batt_status->voltage_uV = 1;
	else
		batt_status->voltage_uV = -1;
#endif

    batt_status->state_of_chrg = 0;//no charging
    batt_status->isexistbat = bq40z50_battery_is_exist();
	return 0;
}
