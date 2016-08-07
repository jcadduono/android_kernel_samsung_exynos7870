/*
 * drivers/video/decon_7870/panels/ltl101al06_lcd_ctrl.c
 *
 * Samsung SoC MIPI LCD CONTROL functions
 *
 * Copyright (c) 2015 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/lcd.h>
#include <linux/backlight.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <video/mipi_display.h>
#include <linux/i2c.h>
#include <linux/pwm.h>
#include "../dsim.h"
#include "dsim_panel.h"

#include "ltl101al06_param.h"

#define POWER_IS_ON(pwr)			(pwr <= FB_BLANK_NORMAL)
#define LEVEL_IS_HBM(level)			(level >= 6)
#define LEVEL_IS_ACL_OFF(brightness)		(brightness >= 255 && brightness <= 281)

struct i2c_client *tc358764_client;

unsigned int			panel_power_gpio;
unsigned int			panel_pwm_gpio;

struct lcd_info {
	unsigned int			bl;
	unsigned int			brightness;
	unsigned int			auto_brightness;
	unsigned int			acl_enable;
	unsigned int			siop_enable;
	unsigned int			current_acl;
	unsigned int			current_bl;
	unsigned int			current_hbm;
	unsigned int			ldi_enable;


	struct lcd_device		*ld;
	struct backlight_device		*bd;

	int				temperature;
	unsigned int			temperature_index;

	unsigned char			dump_info[3];
	unsigned int			weakness_hbm_comp;

	struct dsim_device		*dsim;

	unsigned int			state;
	struct mutex			lock;

	struct pinctrl			*pins;
	struct pinctrl_state		*pins_state[2];
	struct pinctrl			*pins_pwm;
	struct pinctrl_state		*pins_state_pwm[2];

	struct pwm_device *pwm;
	unsigned int			pwm_period;
	unsigned int			pwm_min;
	unsigned int			pwm_max;
	unsigned int			pwm_outdoor;
};

static int pwm_pinctrl_enable(struct lcd_info *lcd, int enable)
{
	struct device *dev = &lcd->ld->dev;
	int ret = 0;

	if (!IS_ERR_OR_NULL(lcd->pins_state_pwm[enable])) {
		ret = pinctrl_select_state(lcd->pins_pwm, lcd->pins_state_pwm[enable]);
		if (ret) {
			dev_err(dev, "%s: pwm_pinctrl_select_state for %s\n", __func__, enable ? "on" : "off");
			return ret;
		}
	}

	return ret;
}

static int tc358764_array_write(u16 addr, u32 w_data)
{
	int ret = 0;
	char buf[6] = {0, };

	//I2C Format
	//addr [15:8]:addr [7:0]:data[7:0]:data[15:8]:data[23:16]:data[31:24]

	//addr [7:0] addr[15:8]
	buf[0] = (u8)(addr >> 8) & 0xff;
	buf[1] = (u8)addr & 0xff;

	//data
	buf[2] = w_data & 0xff;
	buf[3] = (w_data >> 8) & 0xff;
	buf[4] = (w_data >> 16) & 0xff;
	buf[5] = (w_data >> 24) & 0xff;

	ret = i2c_smbus_write_i2c_block_data(tc358764_client, buf[0], 5, &buf[1]);
	if (ret < 0)
		dsim_err(":%s error : setting fail : %d\n", __func__, ret);

	return 0;
}

static int pinctrl_enable(struct lcd_info *lcd, int enable)
{
	struct device *dev = &lcd->ld->dev;
	int ret = 0;

	if (!IS_ERR_OR_NULL(lcd->pins_state[enable])) {
		ret = pinctrl_select_state(lcd->pins, lcd->pins_state[enable]);
		if (ret) {
			dev_err(dev, "%s: pinctrl_select_state for %s\n", __func__, enable ? "on" : "off");
			return ret;
		}
	}

	return ret;
}

static int dsim_panel_set_brightness(struct dsim_device *dsim, int force)
{
	int ret = 0;
	struct lcd_info *lcd = dsim->priv.par;
	unsigned int duty = 0;

	mutex_lock(&lcd->lock);

	lcd->brightness = lcd->bd->props.brightness;

	lcd->bl = lcd->brightness;

	if (!force && lcd->state != PANEL_STATE_RESUMED) {
		dev_info(&lcd->ld->dev, "%s: panel is not active state..\n", __func__);
		goto exit;
	}
	if (LEVEL_IS_HBM(lcd->auto_brightness))
		duty = lcd->pwm_outdoor;
	else
		duty = lcd->bl * (lcd->pwm_max - lcd->pwm_min) / 255 + lcd->pwm_min;

	if (duty <= lcd->pwm_min)
		duty = 0;	// duty must set over 0.7 percent.

	pwm_config(lcd->pwm, duty, lcd->pwm_period);

	dsim_info("%s: bl: %d duty: %d/%d\n", __func__, lcd->bl, duty, lcd->pwm_period);

	lcd->current_bl = lcd->bl;
exit:
	mutex_unlock(&lcd->lock);

	return ret;
}

static int panel_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

static int panel_set_brightness(struct backlight_device *bd)
{
	int ret = 0;
	int brightness = bd->props.brightness;
	struct panel_private *priv = bl_get_data(bd);
	struct lcd_info *lcd = priv->par;
	struct dsim_device *dsim;

	dsim = container_of(priv, struct dsim_device, priv);

	if (brightness < UI_MIN_BRIGHTNESS || brightness > UI_MAX_BRIGHTNESS) {
		pr_alert("Brightness should be in the range of 0 ~ 255\n");
		ret = -EINVAL;
		goto exit;
	}

	if (lcd->state == PANEL_STATE_RESUMED) {
		ret = dsim_panel_set_brightness(dsim, 0);
		if (ret) {
			dev_err(&lcd->ld->dev, "%s: fail to set brightness\n", __func__);
			goto exit;
		}
	}

exit:
	return ret;
}

static const struct backlight_ops panel_backlight_ops = {
	.get_brightness = panel_get_brightness,
	.update_status = panel_set_brightness,
};

static ssize_t lcd_type_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	sprintf(buf, "SMD_LTL101AL06\n");

	return strlen(buf);
}

static ssize_t auto_brightness_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_private *priv = dev_get_drvdata(dev);
	struct lcd_info *lcd = priv->par;

	sprintf(buf, "%u\n", lcd->auto_brightness);

	return strlen(buf);
}

static ssize_t auto_brightness_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct dsim_device *dsim;
	struct panel_private *priv = dev_get_drvdata(dev);
	int value;
	int rc;
	struct lcd_info *lcd = priv->par;

	dsim = container_of(priv, struct dsim_device, priv);

	rc = kstrtoul(buf, (unsigned int)0, (unsigned long *)&value);
	if (rc < 0)
		return rc;
	else {
		if (lcd->auto_brightness != value) {
			dev_info(&lcd->ld->dev, "%s: %d, %d\n", __func__, lcd->auto_brightness, value);
			mutex_lock(&lcd->lock);
			lcd->auto_brightness = value;
			mutex_unlock(&lcd->lock);
			if (lcd->state == PANEL_STATE_RESUMED)
				dsim_panel_set_brightness(dsim, 0);
		}
	}
	return size;
}

static ssize_t siop_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_private *priv = dev_get_drvdata(dev);
	struct lcd_info *lcd = priv->par;

	sprintf(buf, "%u\n", lcd->siop_enable);

	return strlen(buf);
}

static ssize_t siop_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct dsim_device *dsim;
	struct panel_private *priv = dev_get_drvdata(dev);
	int value;
	int rc;
	struct lcd_info *lcd = priv->par;

	dsim = container_of(priv, struct dsim_device, priv);

	rc = kstrtoul(buf, (unsigned int)0, (unsigned long *)&value);
	if (rc < 0)
		return rc;
	else {
		if (lcd->siop_enable != value) {
			dev_info(dev, "%s: %d, %d\n", __func__, lcd->siop_enable, value);
			mutex_lock(&lcd->lock);
			lcd->siop_enable = value;
			mutex_unlock(&lcd->lock);
			if (lcd->state == PANEL_STATE_RESUMED)
				dsim_panel_set_brightness(dsim, 1);
		}
	}
	return size;
}

static ssize_t power_reduce_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_private *priv = dev_get_drvdata(dev);
	struct lcd_info *lcd = priv->par;

	sprintf(buf, "%u\n", lcd->acl_enable);

	return strlen(buf);
}

static ssize_t power_reduce_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct dsim_device *dsim;
	struct panel_private *priv = dev_get_drvdata(dev);
	int value;
	int rc;
	struct lcd_info *lcd = priv->par;

	dsim = container_of(priv, struct dsim_device, priv);
	rc = kstrtoul(buf, (unsigned int)0, (unsigned long *)&value);

	if (rc < 0)
		return rc;
	else {
		if (lcd->acl_enable != value) {
			dev_info(dev, "%s: %d, %d\n", __func__, lcd->acl_enable, value);
			mutex_lock(&lcd->lock);
			lcd->acl_enable = value;
			mutex_unlock(&lcd->lock);
			if (lcd->state == PANEL_STATE_RESUMED)
				dsim_panel_set_brightness(dsim, 1);
		}
	}
	return size;
}

static ssize_t temperature_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char temp[] = "-20, -19, 0, 1\n";

	strcat(buf, temp);
	return strlen(buf);
}

static ssize_t temperature_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct dsim_device *dsim;
	struct panel_private *priv = dev_get_drvdata(dev);
	int value, rc = 0;

	dsim = container_of(priv, struct dsim_device, priv);

	rc = kstrtoint(buf, 10, &value);

	return size;
}

static ssize_t weakness_hbm_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_private *priv = dev_get_drvdata(dev);
	struct lcd_info *lcd = priv->par;

	sprintf(buf, "%d\n", lcd->weakness_hbm_comp);

	return strlen(buf);
}

static ssize_t weakness_hbm_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int rc, value;
	struct dsim_device *dsim;
	struct panel_private *priv = dev_get_drvdata(dev);
	struct lcd_info *lcd = priv->par;

	dsim = container_of(priv, struct dsim_device, priv);

	rc = kstrtouint(buf, (unsigned int)0, &value);

	if (rc < 0)
		return rc;
	else {
		if (lcd->weakness_hbm_comp != value) {
			if ((lcd->weakness_hbm_comp == 1) && (value == 2)) {
				dev_info(&lcd->ld->dev, "%s: don't support hbm interpolation\n", __func__);
				return size;
			}
			if ((lcd->weakness_hbm_comp == 2) || (value == 2)) {
				lcd->weakness_hbm_comp = value;
				if (lcd->state == PANEL_STATE_RESUMED)
					dsim_panel_set_brightness(dsim, 1);
				dev_info(&lcd->ld->dev, "%s: %d, %d\n", __func__, lcd->weakness_hbm_comp, value);
			}
		}
	}
	return size;
}

static DEVICE_ATTR(lcd_type, 0444, lcd_type_show, NULL);
static DEVICE_ATTR(auto_brightness, 0644, auto_brightness_show, auto_brightness_store);
static DEVICE_ATTR(siop_enable, 0664, siop_enable_show, siop_enable_store);
static DEVICE_ATTR(power_reduce, 0664, power_reduce_show, power_reduce_store);
static DEVICE_ATTR(temperature, 0664, temperature_show, temperature_store);
static DEVICE_ATTR(weakness_hbm_comp, 0664, weakness_hbm_show, weakness_hbm_store);

static struct attribute *lcd_sysfs_attributes[] = {
	&dev_attr_lcd_type.attr,
	&dev_attr_siop_enable.attr,
	&dev_attr_power_reduce.attr,
	&dev_attr_temperature.attr,
	NULL,
};

static struct attribute *backlight_sysfs_attributes[] = {
	&dev_attr_auto_brightness.attr,
	&dev_attr_weakness_hbm_comp.attr,
	NULL,
};

static const struct attribute_group lcd_sysfs_attr_group = {
	.attrs = lcd_sysfs_attributes,
};

static const struct attribute_group backlight_sysfs_attr_group = {
	.attrs = backlight_sysfs_attributes,
};

static void lcd_init_sysfs(struct dsim_device *dsim)
{
	struct lcd_info *lcd = dsim->priv.par;
	int ret = 0;

	ret = sysfs_create_group(&lcd->ld->dev.kobj, &lcd_sysfs_attr_group);
	if (ret < 0)
		dev_err(&lcd->ld->dev, "failed to add lcd sysfs\n");

	ret = sysfs_create_group(&lcd->bd->dev.kobj, &backlight_sysfs_attr_group);
	if (ret < 0)
		dev_err(&lcd->bd->dev, "failed to add backlight sysfs\n");
}

static int dsim_panel_early_probe(struct dsim_device *dsim)
{
	int ret = 0;
	struct panel_private *priv = &dsim->priv;

	priv->ops = dsim_panel_get_priv_ops(dsim);

	if (priv->ops->early_probe)
		ret = priv->ops->early_probe(dsim);

	return ret;
}

static int dsim_panel_probe(struct dsim_device *dsim)
{
	int ret = 0;
	struct panel_private *priv = &dsim->priv;
	struct lcd_info *lcd = dsim->priv.par;

	priv->par = lcd = kzalloc(sizeof(struct lcd_info), GFP_KERNEL);
	if (!lcd) {
		pr_err("failed to allocate for lcd\n");
		ret = -ENOMEM;
		goto probe_err;
	}

	dsim->lcd = lcd->ld = lcd_device_register("panel", dsim->dev, &dsim->priv, NULL);
	if (IS_ERR(lcd->ld)) {
		pr_err("%s: faield to register lcd device\n", __func__);
		ret = PTR_ERR(lcd->ld);
		goto probe_err;
	}

	lcd->bd = backlight_device_register("panel", dsim->dev, &dsim->priv, &panel_backlight_ops, NULL);
	if (IS_ERR(lcd->bd)) {
		pr_err("%s: failed register backlight device\n", __func__);
		ret = PTR_ERR(lcd->bd);
		goto probe_err;
	}

	priv->lcdConnected = PANEL_CONNECTED;
	lcd->state = PANEL_STATE_RESUMED;

	mutex_init(&lcd->lock);

	if (priv->ops->probe) {
		ret = priv->ops->probe(dsim);
		if (ret) {
			dev_err(&lcd->ld->dev, "%s: failed to probe panel\n", __func__);
			goto probe_err;
		}
	}

#if defined(CONFIG_EXYNOS_DECON_LCD_SYSFS)
	lcd_init_sysfs(dsim);
#endif

	dev_info(&lcd->ld->dev, "%s probe done\n", __FILE__);
probe_err:
	return ret;
}

static int dsim_panel_displayon(struct dsim_device *dsim)
{
	int ret = 0;
	struct panel_private *priv = &dsim->priv;
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "+%s\n", __func__);

	if (lcd->state == PANEL_STATE_SUSPENED) {
		if (priv->ops->init) {
			ret = priv->ops->init(dsim);
			if (ret) {
				dev_info(&lcd->ld->dev, "%s: failed to panel init\n", __func__);
				goto displayon_err;
			}
		}
	}

	if (priv->ops->displayon) {
		ret = priv->ops->displayon(dsim);
		if (ret) {
			dev_info(&lcd->ld->dev, "%s: failed to panel display on\n", __func__);
			goto displayon_err;
		}
	}

displayon_err:
	mutex_lock(&lcd->lock);
	lcd->state = PANEL_STATE_RESUMED;
	mutex_unlock(&lcd->lock);

	dev_info(&lcd->ld->dev, "-%s: %d\n", __func__, priv->lcdConnected);

	pinctrl_enable(lcd, 1);

	return ret;
}

static int dsim_panel_suspend(struct dsim_device *dsim)
{
	int ret = 0;
	struct panel_private *priv = &dsim->priv;
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "+%s\n", __func__);

	if (lcd->state == PANEL_STATE_SUSPENED)
		goto suspend_err;

	lcd->state = PANEL_STATE_SUSPENDING;

	if (priv->ops->exit) {
		ret = priv->ops->exit(dsim);
		if (ret) {
			dev_info(&lcd->ld->dev, "%s: failed to panel exit\n", __func__);
			goto suspend_err;
		}
	}

suspend_err:
	mutex_lock(&lcd->lock);
	lcd->state = PANEL_STATE_SUSPENED;
	mutex_unlock(&lcd->lock);

	pinctrl_enable(lcd, 0);

	dev_info(&lcd->ld->dev, "-%s: %d\n", __func__, priv->lcdConnected);

	return ret;
}

struct mipi_dsim_lcd_driver ltl101al06_mipi_lcd_driver = {
	.early_probe = dsim_panel_early_probe,
	.probe		= dsim_panel_probe,
	.displayon	= dsim_panel_displayon,
	.suspend	= dsim_panel_suspend,
};

static int ltl101al06_exit(struct dsim_device *dsim)
{
	int ret = 0;
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	pwm_disable(lcd->pwm);
	pwm_pinctrl_enable(lcd, 0);

	return ret;
}

static int ltl101al06_displayon(struct dsim_device *dsim)
{
	int ret = 0;
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "%s\n", __func__);
	/*Before pwm_enable, pwm_config should be called*/
	dsim_panel_set_brightness(dsim, 1);
	pwm_enable(lcd->pwm);
	msleep(200);
	pwm_pinctrl_enable(lcd, 1);

	return ret;
}

static int ltl101al06_init(struct dsim_device *dsim)
{
	int ret = 0;
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

//	msleep(300); //?? internal PLL boosting time...

	ret = gpio_request_one(panel_power_gpio, GPIOF_OUT_INIT_HIGH, "BLIC_ON");
	gpio_free(panel_power_gpio);

	//TC358764_65XBG_Tv12p_ParameterSetting_SS_1280x800_noMSF_SEC_151211.xls

	//TC358764/65XBG DSI Basic Parameters.  Following 10 setting should be pefromed in LP mode
	tc358764_array_write(0x013C,	0x00050006);
	tc358764_array_write(0x0114,	0x00000004);
	tc358764_array_write(0x0164,	0x00000004);
	tc358764_array_write(0x0168,	0x00000004);
	tc358764_array_write(0x016C,	0x00000004);
	tc358764_array_write(0x0170,	0x00000004);
	tc358764_array_write(0x0134,	0x0000001F);
	tc358764_array_write(0x0210,	0x0000001F);
	tc358764_array_write(0x0104,	0x00000001);
	tc358764_array_write(0x0204,	0x00000001);

	//TC358764/65XBG Timing and mode setting (LP or HS)
	tc358764_array_write(0x0450,	0x03F00120);
	tc358764_array_write(0x0454,	0x00340032);
	tc358764_array_write(0x0458,	0x00340500);
	tc358764_array_write(0x045C,	0x00420006);
	tc358764_array_write(0x0460,	0x00440320);
	tc358764_array_write(0x0464,	0x00000001);
	tc358764_array_write(0x04A0,	0x00448006);
	usleep_range(1000, 1100); 	//More than 100us
	tc358764_array_write(0x04A0,	0x00048006);
	tc358764_array_write(0x0504,	0x00000004);

	//TC358764/65XBG LVDS Color mapping setting (LP or HS)
	tc358764_array_write(0x0480,	0x03020100);
	tc358764_array_write(0x0484,	0x08050704);
	tc358764_array_write(0x0488,	0x0F0E0A09);
	tc358764_array_write(0x048C,	0x100D0C0B);
	tc358764_array_write(0x0490,	0x12111716);
	tc358764_array_write(0x0494,	0x1B151413);
	tc358764_array_write(0x0498,	0x061A1918);

	//TC358764/65XBG LVDS enable (LP or HS)
	tc358764_array_write(0x049C,	0x00000001);

	return ret;
}

static int tc358764_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : fail.\n", __func__);
		ret = -ENODEV;
		goto err_i2c;
	}

	tc358764_client = client;
	panel_power_gpio = of_get_gpio(client->dev.of_node, 0);
	panel_pwm_gpio = of_get_gpio(client->dev.of_node, 1);

err_i2c:
	return ret;
}

static struct i2c_device_id tc358764_id[] = {
	{"tc358764", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, tc358764_id);

static struct of_device_id tc358764_i2c_dt_ids[] = {
	{ .compatible = "tc358764,i2c" },
	{ }
};

MODULE_DEVICE_TABLE(of, tc358764_i2c_dt_ids);

static struct i2c_driver tc358764_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tc358764",
		.of_match_table	= of_match_ptr(tc358764_i2c_dt_ids),
	},
	.id_table = tc358764_id,
	.probe = tc358764_probe,
};

static int pwm_probe(struct dsim_device *dsim, struct device_node *parent)
{
	struct device_node *np = NULL;
	struct lcd_info *lcd = dsim->priv.par;
	u32 pwm_id = -1;
	int ret = 0;

	np = of_parse_phandle(parent, "pwm_info", 0);

	if (!np) {
		pr_err("%s: %s node does not exist!!!\n", __func__, "pwm_info");
		ret = -ENODEV;
	}

	of_property_read_u32(np, "pwm_id", &pwm_id);
	of_property_read_u32(np, "duty_period", &lcd->pwm_period);
	of_property_read_u32(np, "duty_min", &lcd->pwm_min);
	of_property_read_u32(np, "duty_max", &lcd->pwm_max);
	of_property_read_u32(np, "duty_outdoor", &lcd->pwm_outdoor);

	dsim_info("%s id: %d duty_period: %d duty_min: %d max: %d outdoor: %d\n",
		__func__, pwm_id, lcd->pwm_period, lcd->pwm_min, lcd->pwm_max, lcd->pwm_outdoor);

	lcd->pwm = pwm_request(pwm_id, "lcd_pwm");
	if (IS_ERR(lcd->pwm)) {
		dsim_err(":%s error : setting fail : %d\n", __func__, ret);
		ret = -EFAULT;
	}

	/*Before pwm_enable, pwm_config should be called*/
	dsim_panel_set_brightness(dsim, 1);
	pwm_enable(lcd->pwm);

	return 0;
}

static int ltl101al06_probe(struct dsim_device *dsim)
{
	int ret = 0;
	struct panel_private *priv = &dsim->priv;
	struct lcd_info *lcd = dsim->priv.par;
	struct device_node *np;
	struct platform_device *pdev;

	pr_info("%s: was called\n", __func__);

	if (lcdtype == 0) {
		priv->lcdConnected = PANEL_DISCONNEDTED;
		dsim_err("dsim : %s lcd was not connected\n", __func__);
		goto exit;
	} else
		priv->lcdConnected = PANEL_CONNECTED;

	lcd->bd->props.max_brightness = UI_MAX_BRIGHTNESS;
	lcd->bd->props.brightness = UI_DEFAULT_BRIGHTNESS;

	lcd->temperature = NORMAL_TEMPERATURE;
	lcd->acl_enable = 0;
	lcd->current_acl = 0;
	lcd->auto_brightness = 0;
	lcd->siop_enable = 0;
	lcd->current_hbm = 0;

	np = of_find_node_with_property(NULL, "lcd_info");
	np = of_parse_phandle(np, "lcd_info", 0);
	pdev = of_platform_device_create(np, NULL, dsim->dev);

	lcd->pins_pwm = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(lcd->pins_pwm)) {
		pr_err("%s: devm_pinctrl_get fail\n", __func__);
		goto exit;
	}

	lcd->pins_state_pwm[0] = pinctrl_lookup_state(lcd->pins_pwm, "pwm_off");
	lcd->pins_state_pwm[1] = pinctrl_lookup_state(lcd->pins_pwm, "pwm_on");
	if (IS_ERR_OR_NULL(lcd->pins_state_pwm[0]) || IS_ERR_OR_NULL(lcd->pins_state_pwm[1])) {
		pr_err("%s: pinctrl_lookup_state fail\n", __func__);
		goto exit;
	}

	lcd->pins_pwm = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(lcd->pins_pwm)) {
		pr_err("%s: devm_pinctrl_get fail\n", __func__);
		goto exit;
	}

	ret = i2c_add_driver(&tc358764_i2c_driver);
	if (ret) {
		pr_err("%s : add_i2c_driver fail.\n", __func__);
		goto exit;
	}

	ret = pwm_probe(dsim, np);
	if (ret) {
		pr_err("%s : add_PWM_driver fail.\n", __func__);
		goto exit;
	}
	dev_info(&lcd->ld->dev, "%s: done\n", __func__);
exit:
	return ret;
}

struct dsim_panel_ops ltl101al06_panel_ops = {
	.probe		= ltl101al06_probe,
	.displayon	= ltl101al06_displayon,
	.exit		= ltl101al06_exit,
	.init		= ltl101al06_init,
};
