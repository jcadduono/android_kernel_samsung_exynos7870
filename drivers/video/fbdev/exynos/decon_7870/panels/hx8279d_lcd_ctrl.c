/*
 * drivers/video/decon_7870/panels/hx8279d_lcd_ctrl.c
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
#include "../dsim.h"
#include "dsim_panel.h"

#include "hx8279d_param.h"

#define POWER_IS_ON(pwr)			(pwr <= FB_BLANK_NORMAL)
#define LEVEL_IS_HBM(brightness)		(brightness == UI_MAX_BRIGHTNESS)

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

	unsigned char			id[3];
	unsigned char			dump_info[3];
	unsigned int			weakness_hbm_comp;

	struct dsim_device		*dsim;

	unsigned int			state;
	struct mutex			lock;

	struct pinctrl			*pins;
	struct pinctrl_state		*pins_state[2];

	/* temporary sysfs to panel parameter tuning */
	unsigned int			write_disable;
};

int _dsim_write_hl_data(struct dsim_device *dsim, const u8 *cmd, u32 cmdSize)
{
	int ret;
	int retry;
	struct panel_private *panel = &dsim->priv;
	struct lcd_info *lcd = dsim->priv.par;

	if (panel->lcdConnected == PANEL_DISCONNEDTED)
		return cmdSize;

	if (lcd->state == PANEL_STATE_RESUMED && lcd->write_disable)
		return ret;

	retry = 5;

try_write:
	if (cmdSize == 1)
		ret = dsim_write_data(dsim, MIPI_DSI_DCS_SHORT_WRITE, cmd[0], 0);
	else if (cmdSize == 2)
		ret = dsim_write_data(dsim, MIPI_DSI_DCS_SHORT_WRITE_PARAM, cmd[0], cmd[1]);
	else
		ret = dsim_write_data(dsim, MIPI_DSI_DCS_LONG_WRITE, (unsigned long)cmd, cmdSize);

	if (ret != 0) {
		if (--retry)
			goto try_write;
		else
			dsim_err("dsim write failed,  cmd : %x\n", cmd[0]);
	}

	return ret;
}

static int dsim_panel_set_brightness(struct dsim_device *dsim, int force)
{
	int ret = 0;
	struct lcd_info *lcd = dsim->priv.par;
	unsigned char bl_reg[2] = {0xB8, 0x00};

	mutex_lock(&lcd->lock);

	lcd->brightness = lcd->bd->props.brightness;

	lcd->bl = lcd->brightness;

	if (!force && lcd->state != PANEL_STATE_RESUMED) {
		dev_info(&lcd->ld->dev, "%s: panel is not active state..\n", __func__);
		goto exit;
	}


	bl_reg[1] = lcd->bl * HX8279D_PWM_DUTY_MAX / 0xFF;

	ret = _dsim_write_hl_data(dsim, SEQ_TABLE_4, ARRAY_SIZE(SEQ_TABLE_4));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_TABLE_04\n", __func__);
		goto exit;
	}

	ret = _dsim_write_hl_data(dsim, bl_reg, ARRAY_SIZE(bl_reg));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : bl_reg\n", __func__);
		goto exit;
	}

	dsim_info("%s : %02d [0x%02x]\n", __func__, lcd->bl, bl_reg[1]);

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
	struct panel_private *priv = dev_get_drvdata(dev);
	struct lcd_info *lcd = priv->par;

	sprintf(buf, "BOE_%02X%02X%02X\n", lcd->id[0], lcd->id[1], lcd->id[2]);

	return strlen(buf);
}

static ssize_t window_type_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_private *priv = dev_get_drvdata(dev);
	struct lcd_info *lcd = priv->par;

	sprintf(buf, "%x %x %x\n", lcd->id[0], lcd->id[1], lcd->id[2]);

	return strlen(buf);
}

static ssize_t dump_register_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dsim_device *dsim;
	struct panel_private *priv = dev_get_drvdata(dev);
	struct lcd_info *lcd = priv->par;
	char *pos = buf;
	u8 reg, len, table;
	int ret, i;
	u8 *dump = NULL;

	dsim = container_of(priv, struct dsim_device, priv);

	reg = lcd->dump_info[0];
	len = lcd->dump_info[1];
	table = lcd->dump_info[2];

	if (!reg || !len || reg > 0xff || len > 255 || table > 255)
		goto exit;

	dump = kzalloc(len * sizeof(u8), GFP_KERNEL);

	if (lcd->state == PANEL_STATE_RESUMED) {
		switch(table) {
			case 1:
				ret = _dsim_write_hl_data(dsim, SEQ_TABLE_1, ARRAY_SIZE(SEQ_TABLE_1));
				break;
			case 2:
				ret = _dsim_write_hl_data(dsim, SEQ_TABLE_2, ARRAY_SIZE(SEQ_TABLE_2));
				break;
			case 3:
				ret = _dsim_write_hl_data(dsim, SEQ_TABLE_3, ARRAY_SIZE(SEQ_TABLE_3));
				break;
			case 4:
				ret = _dsim_write_hl_data(dsim, SEQ_TABLE_4, ARRAY_SIZE(SEQ_TABLE_4));
				break;
			case 5:
				ret = _dsim_write_hl_data(dsim, SEQ_TABLE_5, ARRAY_SIZE(SEQ_TABLE_5));
				break;
			case 6:
				ret = _dsim_write_hl_data(dsim, SEQ_TABLE_6, ARRAY_SIZE(SEQ_TABLE_6));
				break;
			case 7:
				ret = _dsim_write_hl_data(dsim, SEQ_TABLE_7, ARRAY_SIZE(SEQ_TABLE_7));
				break;
			default:
				ret = _dsim_write_hl_data(dsim, SEQ_TABLE_0, ARRAY_SIZE(SEQ_TABLE_0));
				break;
			}

		ret = dsim_read_hl_data(dsim, reg, len, dump);

	}

	pos += sprintf(pos, "+ [%02X]\n", reg);
	for (i = 0; i < len; i++)
		pos += sprintf(pos, "%2d: %02x\n", i + 1, dump[i]);
	pos += sprintf(pos, "- [%02X]\n", reg);

	dev_info(&lcd->ld->dev, "+ [%02X]\n", reg);
	for (i = 0; i < len; i++)
		dev_info(&lcd->ld->dev, "%2d: %02x\n", i + 1, dump[i]);
	dev_info(&lcd->ld->dev, "- [%02X]\n", reg);

	kfree(dump);
exit:
	return pos - buf;
}

static ssize_t dump_register_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct panel_private *priv = dev_get_drvdata(dev);
	unsigned int reg, len, offset;
	int ret;
	struct lcd_info *lcd = priv->par;

	ret = sscanf(buf, "%x %d %d", &reg, &len, &offset);

	if (ret == 2)
		offset = 0;

	dev_info(dev, "%s: %x %d %d\n", __func__, reg, len, offset);

	if (ret < 0)
		return ret;
	else {
		if (!reg || !len || reg > 0xff || len > 255 || offset > 255)
			return -EINVAL;

		lcd->dump_info[0] = reg;
		lcd->dump_info[1] = len;
		lcd->dump_info[2] = offset;
	}

	return size;
}

static ssize_t write_register_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct dsim_device *dsim;
	struct panel_private *priv = dev_get_drvdata(dev);
	struct lcd_info *lcd = priv->par;
	int ret, i, data, count = 0;
	unsigned char seqbuf[255] = {0,};
	unsigned char *printbuf = NULL;
	char *pos, *token;

	dsim = container_of(priv, struct dsim_device, priv);

	pos = (char *)buf;
	while ((token = strsep(&pos, " ")) != NULL) {
		ret = sscanf(token, "%x", &data);
		if (ret) {
			seqbuf[count] = data;
			count++;
		}
		if (count == ARRAY_SIZE(seqbuf))
			break;
	}

	pos = printbuf = kzalloc(size * sizeof(u8), GFP_KERNEL);
	for (i = 0; i < count; i++) {
		pos += sprintf(pos, "%02x ", seqbuf[i]);
	}
	pos += sprintf(pos, "\n");

	if (count <= 1) {
		dev_info(&lcd->ld->dev, "%s: invalid input, %s\n", __func__, printbuf);
		return -EINVAL;
	} else
		dev_info(&lcd->ld->dev, "%s: %s\n", __func__, printbuf);

	if (lcd->state == PANEL_STATE_RESUMED)
		ret = dsim_write_hl_data(dsim, seqbuf, count);

	kfree(printbuf);

	return size;
}

static ssize_t write_disable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_private *priv = dev_get_drvdata(dev);
	struct lcd_info *lcd = priv->par;

	sprintf(buf, "%u\n", lcd->write_disable);

	return strlen(buf);
}

static ssize_t write_disable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct panel_private *priv = dev_get_drvdata(dev);
	struct lcd_info *lcd = priv->par;
	int value;
	int rc;

	rc = kstrtoul(buf, (unsigned int)0, (unsigned long *)&value);
	if (rc < 0)
		return rc;
	else {
		if (lcd->write_disable != value) {
			dev_info(&lcd->ld->dev, "%s: %d, %d\n", __func__, lcd->write_disable, value);
			mutex_lock(&lcd->lock);
			lcd->write_disable = value;
			mutex_unlock(&lcd->lock);
		}
	}
	return size;
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
static DEVICE_ATTR(window_type, 0444, window_type_show, NULL);
static DEVICE_ATTR(auto_brightness, 0644, auto_brightness_show, auto_brightness_store);
static DEVICE_ATTR(siop_enable, 0664, siop_enable_show, siop_enable_store);
static DEVICE_ATTR(power_reduce, 0664, power_reduce_show, power_reduce_store);
static DEVICE_ATTR(temperature, 0664, temperature_show, temperature_store);
static DEVICE_ATTR(weakness_hbm_comp, 0664, weakness_hbm_show, weakness_hbm_store);
static DEVICE_ATTR(dump_register, 0644, dump_register_show, dump_register_store);
static DEVICE_ATTR(write_register, 0220, NULL, write_register_store);
static DEVICE_ATTR(write_disable, 0644, write_disable_show, write_disable_store);

static struct attribute *lcd_sysfs_attributes[] = {
	&dev_attr_lcd_type.attr,
	&dev_attr_window_type.attr,
	&dev_attr_siop_enable.attr,
	&dev_attr_power_reduce.attr,
	&dev_attr_temperature.attr,
	&dev_attr_dump_register.attr,
	&dev_attr_write_register.attr,
	&dev_attr_write_disable.attr,
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

	dev_info(&lcd->ld->dev, "-%s: %d\n", __func__, priv->lcdConnected);

	return ret;
}

struct mipi_dsim_lcd_driver hx8279d_mipi_lcd_driver = {
	.early_probe = dsim_panel_early_probe,
	.probe		= dsim_panel_probe,
	.displayon	= dsim_panel_displayon,
	.suspend	= dsim_panel_suspend,
};

static int hx8279d_exit(struct dsim_device *dsim)
{
	int ret = 0;
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	ret = _dsim_write_hl_data(dsim, SEQ_TABLE_0, ARRAY_SIZE(SEQ_TABLE_0));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_TABLE_0\n", __func__);
		goto exit_err;
	}

	ret = _dsim_write_hl_data(dsim, SEQ_DISPLAY_OFF, ARRAY_SIZE(SEQ_DISPLAY_OFF));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_DISPLAY_OFF\n", __func__);
		goto exit_err;
	}
	msleep(50);

	ret = _dsim_write_hl_data(dsim, SEQ_SLEEP_IN, ARRAY_SIZE(SEQ_SLEEP_IN));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_SLEEP_IN\n", __func__);
		goto exit_err;
	}
	msleep(120);

exit_err:
	return ret;
}

static int dsi_write_table(struct dsim_device *dsim, const struct mipi_cmd *table, int size)
{
	int i, table_size, ret = 0;
	const struct mipi_cmd *table_ptr;

	table_ptr = table;
	table_size = size;

	for (i = 0; i < table_size; i++) {
		ret = _dsim_write_hl_data(dsim, table_ptr[i].cmd, ARRAY_SIZE(table_ptr[i].cmd));

		if (ret < 0) {
			dsim_err("%s : fail to write CMD : 0x%02x, 0x%02x\n", __func__, table_ptr[i].cmd[0], table_ptr[i].cmd[1]);
			goto write_exit;
		}
	}

write_exit:
	return ret;
}

static int hx8279d_read_id(struct dsim_device *dsim)
{
	int i = 0;
	struct panel_private *panel = &dsim->priv;
	struct lcd_info *lcd = dsim->priv.par;

	dsim_info("MDD : %s was called\n", __func__);

	if (lcdtype == 0) {
		panel->lcdConnected = PANEL_DISCONNEDTED;
		goto read_exit;
	}

	lcd->id[0] = (lcdtype & 0xFF0000) >> 16;
	lcd->id[1] = (lcdtype & 0x00FF00) >> 8;
	lcd->id[2] = (lcdtype & 0x0000FF) >> 0;

	dsim_info("READ ID : ");
	for (i = 0; i < 3; i++)
		dsim_info("%02x, ", lcd->id[i]);
	dsim_info("\n");

read_exit:
	return 0;
}

static int hx8279d_displayon(struct dsim_device *dsim)
{
	int ret = 0;
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	ret = _dsim_write_hl_data(dsim, SEQ_TABLE_0, ARRAY_SIZE(SEQ_TABLE_0));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_TABLE_0\n", __func__);
		goto display_err;
	}

	ret = _dsim_write_hl_data(dsim, SEQ_DISPLAY_ON, ARRAY_SIZE(SEQ_DISPLAY_ON));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_DISPLAY_ON\n", __func__);
		goto display_err;
	}


	dsim_panel_set_brightness(dsim, 1);

display_err:
	return ret;
}

static int hx8279d_init(struct dsim_device *dsim)
{
	int ret = 0;
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	ret = _dsim_write_hl_data(dsim, SEQ_TABLE_5, ARRAY_SIZE(SEQ_TABLE_5));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_TABLE_5\n", __func__);
		goto init_err;
	}

	ret = _dsim_write_hl_data(dsim, SEQ_EOTP_DISABLE, ARRAY_SIZE(SEQ_EOTP_DISABLE));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_EOTP_DISABLE\n", __func__);
		goto init_err;
	}

	ret = _dsim_write_hl_data(dsim, SEQ_TLPX_80NS, ARRAY_SIZE(SEQ_TLPX_80NS));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_TLPX_80NS\n", __func__);
		goto init_err;
	}

	ret = _dsim_write_hl_data(dsim, SEQ_TABLE_0, ARRAY_SIZE(SEQ_TABLE_0));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_TABLE_5\n", __func__);
		goto init_err;
	}

	ret = _dsim_write_hl_data(dsim, SEQ_DISPLAY_OFF, ARRAY_SIZE(SEQ_DISPLAY_OFF));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_DISPLAY_OFF\n", __func__);
		goto init_err;
	}

	msleep(100);

	ret = dsi_write_table(dsim, SEQ_CMD_TABLE, ARRAY_SIZE(SEQ_CMD_TABLE));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_CMD_TABLE\n", __func__);
		goto init_err;
	}

	ret = dsi_write_table(dsim, SEQ_CMD_TABLE_BL, ARRAY_SIZE(SEQ_CMD_TABLE_BL));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SEQ_CMD_TABLE_BL\n", __func__);
		goto init_err;
	}

	msleep(15);

init_err:
	return ret;
}

static int hx8279d_probe(struct dsim_device *dsim)
{
	int ret = 0;
	struct panel_private *priv = &dsim->priv;
	struct lcd_info *lcd = dsim->priv.par;

	pr_info("%s: was called\n", __func__);

	priv->lcdConnected = PANEL_CONNECTED;

	lcd->bd->props.max_brightness = UI_MAX_BRIGHTNESS;
	lcd->bd->props.brightness = UI_DEFAULT_BRIGHTNESS;

	lcd->temperature = NORMAL_TEMPERATURE;
	lcd->acl_enable = 0;
	lcd->current_acl = 0;
	lcd->auto_brightness = 0;
	lcd->siop_enable = 0;
	lcd->current_hbm = 0;

	hx8279d_read_id(dsim);

	return ret;
}

struct dsim_panel_ops hx8279d_panel_ops = {
	.probe		= hx8279d_probe,
	.displayon	= hx8279d_displayon,
	.exit		= hx8279d_exit,
	.init		= hx8279d_init,
};
