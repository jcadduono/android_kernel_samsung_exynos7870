/*
 * leds-s2mu005-rgb.c - Service notification LED driver based on s2mu005 RGB LED
 *
 * Copyright (C) 2016, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/mfd/samsung/s2mu005.h>
#include <linux/mfd/samsung/s2mu005-private.h>
#include <linux/leds-s2mu005.h>
#include <linux/platform_device.h>
#include <linux/sec_batt.h>
#include <linux/sec_sysfs.h>

#include "leds-s2mu005-rgb.h"

#define S2MU005_RGB_LED_DRIVER_NAME 	"leds-s2mu005-rgb"

#define SEC_LED_SPECIFIC
//#define S2MU005_ENABLE_INDIVIDUAL_CURRENT

#ifdef SEC_LED_SPECIFIC
/**
* s2mu005_pattern - enum constants for different fixed displayable LED pattern 
**/
enum s2mu005_pattern {
	PATTERN_OFF,
	CHARGING,
	CHARGING_ERR,
	MISSED_NOTI,
	LOW_BATTERY,
	FULLY_CHARGED,
	POWERING,
};

enum s2mu005_LED {
	LED_B,
	LED_G,
	LED_R,
	LED_MAX
};
#endif // SEC_LED_SPECIFIC

/**
* s2mu005_rgb_led - structure to hold all service led related driver data 
* @led_dynamic_current - holds max permissible (tuned) LED current 
*          in the present configuration
* @led_max_current - holds max current of each LED for normal mode
* @led_low_current - holds max current of each LED for lowpower mode
* @led_lowpower_mode - holds whether led is in low power mode
* @type - TBD
* @color - holds presently displaying color on SVC LED.
* @led_on_time - holds blink on time
* @led_off_time - holds blink off time
* @mode - current SEC LED pattern displaying on the phone
* @i2c - holds the parent driver's (PMIC's struct s2mu005_dev)'s I2C client 
*         reference
* @blink_work - work queue to handle the request from user space
* @led_dev - pointer to hold the device attribute to communicate through sysfs
**/
struct s2mu005_rgb_led{
	u8 led_dynamic_current[LED_MAX];
	u8 led_max_current[LED_MAX];
	u8 led_low_current[LED_MAX];  
	u8 led_lowpower_mode;		
	u32	color;
	u32 led_on_time;
    u32 led_off_time;
	enum s2mu005_pattern mode;
	struct i2c_client *i2c;
	struct work_struct blink_work;
	struct device *led_dev;
	//struct s2mu005_dev   s2mu005_dev;
};

/**
* g_s2mu005_rgb_led_data - Holds all service led related driver data  
**/
struct s2mu005_rgb_led *g_s2mu005_rgb_led_data;


/**
* s2mu005_rgb_write_reg - writes LED data to s2mu005 RGB LED registers.
* @reg - register offset
* @val -> data to be written on to the register
**/
static int __inline s2mu005_rgb_write_reg(u8 reg, u8 val)
{
	return s2mu005_write_reg(g_s2mu005_rgb_led_data->i2c, reg, val);
}

/**
* s2mu005_rgb_write_reg - Holds all service led related driver data 
**/
static u8 __inline s2mu005_rgb_dynamic_current(u32 color, u8 dynamic_current)
{
	return ((color * dynamic_current) / S2MU005_LEDx_MAX_CURRENT);
}

/**
* s2mu005_rgb_leds_write_all - writes to s2mu005's rgb registers with user
*        requested color and blink timing. also applies current tuning
* @color - color in 0xRRGGBB format
* @on_time -> blink on time in ms
* @off_time -> blink off time in ms
**/
static int s2mu005_rgb_leds_write_all(u32 color, long int on_time, long int off_time)
{
    u32 ramp_up_time;
    u32 ramp_down_time;
    u32 stable_on_time;
    u32 temp;
#ifdef S2MU005_ENABLE_INDIVIDUAL_CURRENT		
    u8 mode = 0;
    u8 curr;
#endif	

	//  Write RESET to ENABEL register to initialize register writes
	s2mu005_rgb_write_reg(S2MU005_LED_ENABLE_REG, S2MU005_LED_RESET_WRITE);

	// Switch off the LED
	//if(!color || !on_time ) return 0;
	if(!color) return 0;  // ON time DONT CARE


#ifdef S2MU005_ENABLE_INDIVIDUAL_CURRENT
	//  Write LED3 current
	curr = s2mu005_rgb_dynamic_current((u32)(color>>16 & 0xFF), \
		g_s2mu005_rgb_led_data->led_dynamic_current[LED_R]);
	if(curr) {
		s2mu005_rgb_write_reg(S2MU005_LED_R_CURRENT_REG, curr);
		mode = 0b11;
	}
    //  Write LED2 current
	curr = s2mu005_rgb_dynamic_current((u32)(color>>8 & 0xFF), \
		g_s2mu005_rgb_led_data->led_dynamic_current[LED_G]);
	if(curr) {
		s2mu005_rgb_write_reg(S2MU005_LED_G_CURRENT_REG, curr);
		mode |= 0b1100;
	}
    //  Write LED1 current
	curr = s2mu005_rgb_dynamic_current((u32)(color & 0xFF), \
		g_s2mu005_rgb_led_data->led_dynamic_current[LED_B]);
	if(curr) {
		s2mu005_rgb_write_reg(S2MU005_LED_B_CURRENT_REG, curr);
		mode |= 0b110000;
	}
	
#else
	//  Write LED3 current
    s2mu005_rgb_write_reg(S2MU005_LED_R_CURRENT_REG, \
			s2mu005_rgb_dynamic_current((u32)(color>>16 & 0xFF), \
				g_s2mu005_rgb_led_data->led_dynamic_current[LED_R]));
	//  Write LED2 current
    s2mu005_rgb_write_reg(S2MU005_LED_G_CURRENT_REG, \
			s2mu005_rgb_dynamic_current((u32)(color>>8 & 0xFF),
				g_s2mu005_rgb_led_data->led_dynamic_current[LED_G]));
	//  Write LED1 current
    s2mu005_rgb_write_reg(S2MU005_LED_B_CURRENT_REG, \
			s2mu005_rgb_dynamic_current((u32)(color & 0xFF), 
				g_s2mu005_rgb_led_data->led_dynamic_current[LED_B]));
#endif			

	if(off_time == 0) //constant glow
	{
		//constant mode in enable reg
#ifdef S2MU005_ENABLE_INDIVIDUAL_CURRENT		
		s2mu005_rgb_write_reg(S2MU005_LED_ENABLE_REG, (mode & S2MU005_LED_CONST_CURR));
#else		
		s2mu005_rgb_write_reg(S2MU005_LED_ENABLE_REG, S2MU005_LED_CONST_CURR);
#endif	
		return 0;
	}

	if(off_time <= 5000)
		off_time /= 500;
	else if (off_time <= 8000)
		off_time = ((off_time - 5000)/1000) + 10 ;
	else if (off_time <= 12000)
		off_time = ((off_time - 8000)/2000) + 13 ;
	else
		off_time = 0xF;

	if(on_time < 300)
	{
		ramp_up_time = ramp_down_time=0;
		stable_on_time = on_time/100;
	}
	else
	{
		if(on_time > 7650)
			on_time = 7650;

		temp = on_time/3;

		if(temp > 2200)
			temp = 2200;

		if(temp <= 800)
			ramp_up_time /= 100;
		else if(temp <= 2200)
			ramp_up_time = ((temp - 800)/200) + 8;
		else
			ramp_up_time = 0xF;

		ramp_down_time = ramp_up_time;
		stable_on_time = on_time - (temp << 2);
	}

	temp  = (ramp_up_time << 4) | ramp_down_time;

	// Write LED1 Ramp up and down
	s2mu005_rgb_write_reg(S2MU005_LED1_RAMP_REG, temp);
	// Write LED2 Ramp up and down
	s2mu005_rgb_write_reg(S2MU005_LED2_RAMP_REG, temp);
	// Write LED3 Ramp up and down
	s2mu005_rgb_write_reg(S2MU005_LED3_RAMP_REG, temp);

	temp  = (stable_on_time<<4)|off_time ;

	// Write LED1 Duration
	s2mu005_rgb_write_reg(S2MU005_LED1_DUR_REG, temp);
	// Write LED2 Duration
	s2mu005_rgb_write_reg(S2MU005_LED2_DUR_REG, temp);
	// Write LED3 Duration
	s2mu005_rgb_write_reg(S2MU005_LED3_DUR_REG, temp);

	//Write LED on to Enable reg
#ifdef S2MU005_ENABLE_INDIVIDUAL_CURRENT	
    s2mu005_rgb_write_reg(S2MU005_LED_ENABLE_REG, (mode & S2MU005_LED_SLOPE_CURR));
#else
	s2mu005_rgb_write_reg(S2MU005_LED_ENABLE_REG, S2MU005_LED_SLOPE_CURR);
#endif	

    return 0;
}

#if 0
/**
* s2mu005_rgb_led_parse_dt - parses hw details from dts
**/
static int s2mu005_rgb_led_parse_dt(struct platform_device *dev, struct s2mu005_rgb_led *data)
{

	//TBD : yet to register led class
	return 0;

}
#endif


#ifdef SEC_LED_SPECIFIC
/**
* s2mu005_start_led_pattern - Displays specific service LED patterns
*       such as charging, missed notification, etc.,
* @mode -> denotes LED pattern to be displayed
* @type -> TBD
**/
static void s2mu005_start_led_pattern(int mode)
{
	switch (mode) {
	case PATTERN_OFF:
		pr_info("[SVC LED] Pattern off\n");
		g_s2mu005_rgb_led_data->color = 0;
	break;

	case CHARGING:
		pr_info("[SVC LED] Battery Charging Pattern on\n");
		g_s2mu005_rgb_led_data->color = 0xFF0000;  // Red Color
		g_s2mu005_rgb_led_data->led_on_time = 100;
		g_s2mu005_rgb_led_data->led_off_time = 0;

		break;

	case CHARGING_ERR:
		pr_info("[SVC LED] Battery Charging error Pattern on\n");
		g_s2mu005_rgb_led_data->color = 0xFF0000;  // Red Color
		g_s2mu005_rgb_led_data->led_on_time = 500;
		g_s2mu005_rgb_led_data->led_off_time = 500;
		break;

	case MISSED_NOTI:
		pr_info("[SVC LED] Missed Notifications Pattern on\n");
		g_s2mu005_rgb_led_data->color = 0xFF;  // Blue Color
		g_s2mu005_rgb_led_data->led_on_time = 500;
		g_s2mu005_rgb_led_data->led_off_time = 5000;
		break;

	case LOW_BATTERY:
		pr_info("[SVC LED] Low Battery Pattern on\n");
		g_s2mu005_rgb_led_data->color = 0xFF0000;  // Red Color
		g_s2mu005_rgb_led_data->led_on_time = 500;
		g_s2mu005_rgb_led_data->led_off_time = 5000;
		break;

	case FULLY_CHARGED:
		pr_info("[SVC LED] full Charged battery Pattern on\n");
		g_s2mu005_rgb_led_data->color = 0xFF00;  // Green Color
		g_s2mu005_rgb_led_data->led_on_time = 100;
		g_s2mu005_rgb_led_data->led_off_time = 0;
		break;

	case POWERING:
		pr_info("[SVC LED] Powering Pattern on\n");
		g_s2mu005_rgb_led_data->color = 0xFF;  // Blue Color
		g_s2mu005_rgb_led_data->led_on_time = 1000;
		g_s2mu005_rgb_led_data->led_off_time = 1000;
		break;

	default:
		pr_info("[SVC LED] Wrong Pattern\n");
		return;
	}

	schedule_work(&g_s2mu005_rgb_led_data->blink_work);
}
#endif // SEC_LED_SPECIFIC


/**
* s2mu005_rgb_led_blink_work - work handler to execute HW access for blink works
**/
static void s2mu005_rgb_led_blink_work(struct work_struct *work)
{
	pr_info("[SVC LED] %s %d", __func__, __LINE__);

	s2mu005_rgb_leds_write_all(g_s2mu005_rgb_led_data->color, \
		g_s2mu005_rgb_led_data->led_on_time, g_s2mu005_rgb_led_data->led_off_time);
}

/**
* s2mu005_rgb_led_blink_store - sysfs write interface to get the LED 
*               color data in the format 
*				"<COLOR in 0xRRGGBB> <ON TIME in ms> <OFF TIME in ms>" 
**/
static ssize_t s2mu005_rgb_led_blink_store(struct device *dev, \
		struct device_attribute *attr,const char *buf, size_t len)
{
	int retval;
	u32 color, delay_on_time, delay_off_time;

	dev_err(dev, "%s %s\n", __func__, buf);

	retval = sscanf(buf, "0x%x %u %u", &color, &delay_on_time, &delay_off_time); // brightness from here??
    if (unlikely(retval == 0)) {
		dev_err(dev, "%s fail to get led_blink value.\n", __func__);
	}

    g_s2mu005_rgb_led_data->color = color;
    g_s2mu005_rgb_led_data->led_on_time = delay_on_time;
    g_s2mu005_rgb_led_data->led_off_time = delay_off_time;

	schedule_work(&g_s2mu005_rgb_led_data->blink_work);

	return len;
}

/**
* s2mu005_rgb_led_r_store - internal call from sysfs interface to read
*			brightness value from Userspace.
* @colorshift -> specifies R or G or B's color info.	
**/
void s2mu005_rgb_led_get_brightness(struct device *dev, \
		const char *buf, int colorshift)
{
	u8 brightness;
	
	if(likely(!kstrtou8(buf, 0, &brightness)))
	{
		g_s2mu005_rgb_led_data->color = brightness << colorshift;
		g_s2mu005_rgb_led_data->led_on_time = 1000;
		g_s2mu005_rgb_led_data->led_off_time = 0;

		schedule_work(&g_s2mu005_rgb_led_data->blink_work);
	}
	else
	{
		dev_err(dev, "[SVC LED] Error in getting brightness!\n");
	}
}

/**
* s2mu005_rgb_led_r_store - sysfs write interface to get the R LED brightness.
**/
static ssize_t s2mu005_rgb_led_r_store(struct device *dev, \
		struct device_attribute *attr,const char *buf, size_t len)
{
	dev_info(dev, "%s %s\n", __func__, buf);

	s2mu005_rgb_led_get_brightness(dev, buf, LED_R_SHIFT);	

	return len;
}

/**
* s2mu005_rgb_led_g_store - sysfs write interface to get the G LED brightness.
**/
static ssize_t s2mu005_rgb_led_g_store(struct device *dev, \
		struct device_attribute *attr,const char *buf, size_t len)
{
	dev_info(dev, "%s %s\n", __func__, buf);

	s2mu005_rgb_led_get_brightness(dev, buf, LED_G_SHIFT);	
	
	return len;
}

/**
* s2mu005_rgb_led_b_store - sysfs write interface to get the B LED brightness.
**/
static ssize_t s2mu005_rgb_led_b_store(struct device *dev, \
		struct device_attribute *attr,const char *buf, size_t len)
{
	dev_info(dev, "%s %s\n", __func__, buf);

	s2mu005_rgb_led_get_brightness(dev, buf, LED_B_SHIFT);	
	
	return len;
}

/**
* s2mu005_rgb_led_store_lowpower - sysfs write interface to enable/disable 
*                  lowpower mode. 0 - normal mode, 1 - lowpower mode.
**/
static ssize_t s2mu005_rgb_led_store_lowpower(struct device *dev, \
		struct device_attribute *attr, const char *buf, size_t len)
{
	u8 led_lowpower;
	
	if(likely(!kstrtou8(buf, 0, &led_lowpower)))
	{
		g_s2mu005_rgb_led_data->led_lowpower_mode = led_lowpower;
		
		if(led_lowpower)
		{
			g_s2mu005_rgb_led_data->led_dynamic_current[LED_R] = g_s2mu005_rgb_led_data->led_low_current[LED_R];
			g_s2mu005_rgb_led_data->led_dynamic_current[LED_G] = g_s2mu005_rgb_led_data->led_low_current[LED_G];
			g_s2mu005_rgb_led_data->led_dynamic_current[LED_B] = g_s2mu005_rgb_led_data->led_low_current[LED_B];
		}
		else		
		{
			g_s2mu005_rgb_led_data->led_dynamic_current[LED_R] = g_s2mu005_rgb_led_data->led_max_current[LED_R];
			g_s2mu005_rgb_led_data->led_dynamic_current[LED_G] = g_s2mu005_rgb_led_data->led_max_current[LED_G];
			g_s2mu005_rgb_led_data->led_dynamic_current[LED_B] = g_s2mu005_rgb_led_data->led_max_current[LED_B];
		}
		
		// Schedule the work, so that the low power can be updated.
		schedule_work(&g_s2mu005_rgb_led_data->blink_work);
	}
	else
	{
		dev_err(dev, "[SVC LED] Wrong low power mode\n");
	}
	
	return len;
}

/**
* s2mu005_rgb_led_store_led_pattern - sysfs write interface to get the LED 
*               Pattern to be displayed
**/
static ssize_t s2mu005_rgb_led_store_led_pattern(struct device *dev, \
		struct device_attribute *attr, const char *buf, size_t len)
{
	int retval;
	unsigned int mode = 0;

	retval = sscanf(buf, "%d", &mode);

	dev_err(dev, "[SVC LED] Store Pattern: %d\n", mode);

	if (unlikely(retval == 0)) {
		dev_err(dev, "[SVC LED] fail to get led_pattern mode.\n");
	}
	else
	{
		g_s2mu005_rgb_led_data->mode = mode;
		s2mu005_start_led_pattern(mode);
	}

	return len;
}

#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP
/**
* s2mu005_rgb_led_show_lowpower - sysfs read interface to show the low power mode
*     0 - normal mode, 1 - lowpower mode.
*     Accessible only on non-ship binaries.
**/
static ssize_t s2mu005_rgb_led_show_lowpower(struct device *dev, \
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d", g_s2mu005_rgb_led_data->led_lowpower_mode);
}

/**
* s2mu005_rgb_led_blink_show - sysfs read interface to show the LED 
*      configuration (color, blink on time, blink off time) currently 
*      being displayed.
*      Accessible only on non-ship binaries.
**/
static ssize_t s2mu005_rgb_led_blink_show(struct device *dev, \
		struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%x %u %u", g_s2mu005_rgb_led_data->color, \
		g_s2mu005_rgb_led_data->led_on_time, \
		g_s2mu005_rgb_led_data->led_off_time);
}

/**
* s2mu005_rgb_led_r_show - sysfs read interface to show the R LED 
*      brightness value.
*      Accessible only on non-ship binaries.
**/
static ssize_t s2mu005_rgb_led_r_show(struct device *dev, \
		struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%d", g_s2mu005_rgb_led_data->color >> LED_R_SHIFT);
}

/**
* s2mu005_rgb_led_g_show - sysfs read interface to show the G LED 
*      brightness value.
*      Accessible only on non-ship binaries.
**/
static ssize_t s2mu005_rgb_led_g_show(struct device *dev, \
		struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%d", 
	(g_s2mu005_rgb_led_data->color >> LED_G_SHIFT) & LED_BRIGHTNESS_MASK);
}

/**
* s2mu005_rgb_led_b_show - sysfs read interface to show the B LED 
*      brightness value.
*      Accessible only on non-ship binaries.
**/
static ssize_t s2mu005_rgb_led_b_show(struct device *dev, \
		struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%d", 
		g_s2mu005_rgb_led_data->color & LED_BRIGHTNESS_MASK);
}

/**
* s2mu005_rgb_led_show_led_pattern - sysfs read interface to show the  
*      LED pattern currently being displayed.
*      Accessible only on non-ship binaries.
**/
static ssize_t s2mu005_rgb_led_show_led_pattern(struct device *dev, \
		struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%d", g_s2mu005_rgb_led_data->mode);
}

/**
* s2mu005_rgb_led_show_reg - debug sysfs read interface to show   
*      all the RGB LED register values of s2mu005.
*      Accessible only on non-ship binaries.
**/
static ssize_t s2mu005_rgb_led_show_reg(struct device *dev, \
		struct device_attribute *attr, char *buf)
{
	int i, len = 0;
	u8 data[S2MU005_LED_MAX_REG_COUNT];
	
	s2mu005_bulk_read(g_s2mu005_rgb_led_data->i2c, S2MU005_LED_BASE_REG, \
		S2MU005_LED_MAX_REG_COUNT, data);
	
	for(i = 0; i < S2MU005_LED_MAX_REG_COUNT; i++) {
		len = sprintf((buf + len), "%X ", data[i]);
	}

	return len;
}

/**
* s2mu005_rgb_led_show_reg - debug sysfs write interface to write   
*      specified 8 bit value on to the specified register of s2mu005's 
*      RGB LED register values of .
*      Accessible only on non-ship binaries.
**/
static ssize_t s2mu005_rgb_led_store_reg(struct device *dev, \
		struct device_attribute *attr, const char *buf, size_t len)
{
	int reg, val;
	
	sscanf(buf, "%x %x", &reg, &val);
	
	s2mu005_rgb_write_reg(reg, val);
	
	return len;
}

/**
* s2mu005_rgb_led_show_dynamic_current - debug sysfs read interface to show   
*      max current for normal mode and low power mode of each LEDs (R, G, B)
*      Accessible only on non-ship binaries.
**/
static ssize_t s2mu005_rgb_led_show_dynamic_current(struct device *dev, \
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x %x %x %x %x %x", \
		g_s2mu005_rgb_led_data->led_max_current[LED_R],
		g_s2mu005_rgb_led_data->led_max_current[LED_G],
		g_s2mu005_rgb_led_data->led_max_current[LED_B],
		g_s2mu005_rgb_led_data->led_low_current[LED_R],
		g_s2mu005_rgb_led_data->led_low_current[LED_G],
		g_s2mu005_rgb_led_data->led_low_current[LED_B]);		
}

/**
* s2mu005_rgb_led_show_dynamic_current - debug sysfs write interface to update   
*      max current for normal mode and low power mode of each LEDs (R, G, B)
*      Accessible only on non-ship binaries.
**/
static ssize_t s2mu005_rgb_led_store_dynamic_current(struct device *dev, \
		struct device_attribute *attr, const char *buf, size_t len)
{	
	int max_current[LED_MAX], low_current[LED_MAX];
	
	sscanf(buf, "%x %x %x %x %x %x", \
			&max_current[0], &max_current[1], &max_current[2],
			&low_current[0], &low_current[1], &low_current[2]);
	
	g_s2mu005_rgb_led_data->led_max_current[LED_R] = max_current[0];
	g_s2mu005_rgb_led_data->led_max_current[LED_G] = max_current[1];
	g_s2mu005_rgb_led_data->led_max_current[LED_B] = max_current[2];
	g_s2mu005_rgb_led_data->led_low_current[LED_R] = low_current[0];
	g_s2mu005_rgb_led_data->led_low_current[LED_G] = low_current[1];
	g_s2mu005_rgb_led_data->led_low_current[LED_B] = low_current[2];
	
	return len;
}


/* List of nodes, created for non ship binary */
static DEVICE_ATTR(reg, 0664, s2mu005_rgb_led_show_reg, s2mu005_rgb_led_store_reg);
static DEVICE_ATTR(dynamic_current, 0664, s2mu005_rgb_led_show_dynamic_current, s2mu005_rgb_led_store_dynamic_current);
static DEVICE_ATTR(led_lowpower, 0664, s2mu005_rgb_led_show_lowpower, s2mu005_rgb_led_store_lowpower);
static DEVICE_ATTR(led_pattern, 0664, s2mu005_rgb_led_show_led_pattern, s2mu005_rgb_led_store_led_pattern);
static DEVICE_ATTR(led_blink, 0644, s2mu005_rgb_led_blink_show, s2mu005_rgb_led_blink_store);
static DEVICE_ATTR(led_r, 0644, s2mu005_rgb_led_r_show, s2mu005_rgb_led_r_store);
static DEVICE_ATTR(led_g, 0644, s2mu005_rgb_led_g_show, s2mu005_rgb_led_g_store);
static DEVICE_ATTR(led_b, 0644, s2mu005_rgb_led_b_show, s2mu005_rgb_led_b_store);
#else
	
/* List of nodes, created for ship binary */
static DEVICE_ATTR(led_lowpower, 0664, NULL, s2mu005_rgb_led_store_lowpower);
static DEVICE_ATTR(led_pattern, 0664, NULL, s2mu005_rgb_led_store_led_pattern);	
static DEVICE_ATTR(led_blink, 0644, NULL, s2mu005_rgb_led_blink_store);
/* the followig nodes are used in HW module test */
static DEVICE_ATTR(led_r, 0644, NULL, s2mu005_rgb_led_r_store);
static DEVICE_ATTR(led_g, 0644, NULL, s2mu005_rgb_led_g_store);
static DEVICE_ATTR(led_b, 0644, NULL, s2mu005_rgb_led_b_store);
#endif //CONFIG_SAMSUNG_PRODUCT_SHIP

#ifdef SEC_LED_SPECIFIC
static struct attribute *sec_led_attributes[] = {
	&dev_attr_led_pattern.attr,
	&dev_attr_led_blink.attr,
	&dev_attr_led_r.attr,
	&dev_attr_led_g.attr,
	&dev_attr_led_b.attr,
	&dev_attr_led_lowpower.attr,
#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP
	&dev_attr_reg.attr,
	&dev_attr_dynamic_current.attr,
#endif //CONFIG_SAMSUNG_PRODUCT_SHIP
	NULL,
};

static struct attribute_group sec_led_attr_group = {
	.attrs = sec_led_attributes,
};
#endif //SEC_LED_SPECIFIC
 

/**
* s2mu005_rgb_led_probe - Enumerates service LED resources.
**/
static int s2mu005_rgb_led_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct s2mu005_dev *s2mu005 = dev_get_drvdata(pdev->dev.parent);

	dev_info(&pdev->dev, "start! %s %d\n", __func__, __LINE__);

    if(unlikely(!(g_s2mu005_rgb_led_data = devm_kzalloc(s2mu005->dev,
			sizeof(struct s2mu005_rgb_led), GFP_KERNEL)))) {
		dev_err(&pdev->dev, "failed to allocate driver data.\n");
		return -ENOMEM;
	}

	g_s2mu005_rgb_led_data->i2c = s2mu005->i2c;
	platform_set_drvdata(pdev, g_s2mu005_rgb_led_data); //TBD


#if 0
	ret =  s2mu005_rgb_led_parse_dt(pdev, g_s2mu005_rgb_led_data);
	if (ret) {
		dev_err(&pdev->dev, "[%s] s2mu005_rgb_led parse dt failed\n", __func__);
		goto exit;
	}
#endif
	INIT_WORK(&(g_s2mu005_rgb_led_data->blink_work),
				 s2mu005_rgb_led_blink_work);

#ifdef SEC_LED_SPECIFIC
	g_s2mu005_rgb_led_data->led_dev = sec_device_create(g_s2mu005_rgb_led_data, "led");
	if (unlikely(IS_ERR(g_s2mu005_rgb_led_data->led_dev))) {
		dev_err(&pdev->dev,
			"Failed to create device for Samsung specific led\n");
		ret = -ENODEV;
		goto exit;
	}
	
	ret = sysfs_create_group(&g_s2mu005_rgb_led_data->led_dev->kobj, &sec_led_attr_group);
	if (unlikely(ret)) {
		dev_err(&pdev->dev,
			"Failed to create sysfs group for Samsung specific led\n");
		goto exit;
	}
#endif

	g_s2mu005_rgb_led_data->led_dynamic_current[LED_R] = S2MU005_LED_R_MAX_CURRENT;
	g_s2mu005_rgb_led_data->led_dynamic_current[LED_G] = S2MU005_LED_G_MAX_CURRENT;
	g_s2mu005_rgb_led_data->led_dynamic_current[LED_B] = S2MU005_LED_B_MAX_CURRENT;
	g_s2mu005_rgb_led_data->led_max_current[LED_R] = S2MU005_LED_R_MAX_CURRENT;
	g_s2mu005_rgb_led_data->led_max_current[LED_G] = S2MU005_LED_G_MAX_CURRENT;
	g_s2mu005_rgb_led_data->led_max_current[LED_B] = S2MU005_LED_B_MAX_CURRENT;
	g_s2mu005_rgb_led_data->led_low_current[LED_R] = S2MU005_LED_R_LOW_CURRENT;
	g_s2mu005_rgb_led_data->led_low_current[LED_G] = S2MU005_LED_G_LOW_CURRENT;
	g_s2mu005_rgb_led_data->led_low_current[LED_B] = S2MU005_LED_B_LOW_CURRENT;

	return 0;
exit:
	dev_err(&pdev->dev, "[SVC LED] err: %s %d\n", __func__, __LINE__);
	kfree(g_s2mu005_rgb_led_data);
	g_s2mu005_rgb_led_data = NULL;
	return ret;
}

/**
* s2mu005_rgb_led_shutdown - Things to be done on shutdown call.
**/
static void s2mu005_rgb_led_shutdown(struct device *dev)
{
	s2mu005_rgb_leds_write_all(0, 0, 0);
}

/**
* s2mu005_rgb_led_shutdown - Things to be done on device removal.
**/
static int s2mu005_rgb_led_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "Device Remove\n");
	
	s2mu005_rgb_leds_write_all(0, 0, 0);
	
	return 0;
}

#if 1
static struct of_device_id s2mu005_rgb_leds_id[] = {
	{ .compatible = S2MU005_RGB_LED_DRIVER_NAME, },
	{ },
};
#else
static const struct platform_device_id s2mu005_rgb_leds_id[] = {
	{S2MU005_RGB_LED_DRIVER_NAME, 0},
	{ },
};
#endif


/**
* s2mu005_rgb_led_driver - Holds driver mapping of LED platform driver
*        for Service notification LED.
**/
static struct platform_driver s2mu005_rgb_led_driver = {
	.probe = s2mu005_rgb_led_probe,
	//.id_table = s2mu005_rgb_leds_id,
	.remove = s2mu005_rgb_led_remove,
	.driver = {
		.name =  S2MU005_RGB_LED_DRIVER_NAME,
		.owner = THIS_MODULE,
		.shutdown = s2mu005_rgb_led_shutdown,
		.of_match_table = s2mu005_rgb_leds_id,
	},
};

module_platform_driver(s2mu005_rgb_led_driver);

MODULE_DESCRIPTION("S2MU005 RGB LED driver");
MODULE_AUTHOR("Architha Chitukuri <a.chitukuri@samsung.com>");
MODULE_LICENSE("GPL");



