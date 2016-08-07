/* linux/drivers/video/backlight/s6d7aa0_degas2_mipi_lcd.c
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
#include <linux/of_device.h>
#include <video/mipi_display.h>
#include "../dsim.h"
#include "dsim_panel.h"

#include "s6d7aa0_degas2_param.h"

#if defined(CONFIG_EXYNOS_DECON_MDNIE_LITE)
#include "mdnie.h"
#include "mdnie_lite_table_degas2.h"
#endif

#define S6D7AA0_ID_REG			0xD8	/* LCD ID1,ID2,ID3 */
#define S6D7AA0_ID_LEN			3
#define BRIGHTNESS_REG 0x51

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
	struct pinctrl			*pins_pwm;
	struct pinctrl_state		*pins_state_pwm[2];

	struct pwm_device *pwm;
	unsigned int			pwm_period;
	unsigned int			pwm_min;
	unsigned int			pwm_max;
};

static int dsim_panel_set_brightness(struct dsim_device *dsim, int force)
{
	int ret = 0;
	unsigned char bl_reg[2];
	struct lcd_info *lcd = dsim->priv.par;

	mutex_lock(&lcd->lock);

	lcd->brightness = lcd->bd->props.brightness;

	lcd->bl = lcd->brightness;

	if (!force && lcd->state != PANEL_STATE_RESUMED) {
		dev_info(&lcd->ld->dev, "%s: panel is not active state..\n", __func__);
		goto exit;
	}
	bl_reg[0] = BRIGHTNESS_REG;
	if (lcd->bl >= 3)
		bl_reg[1] = lcd->bl ;
	else
		bl_reg[1] = 0x00;

	dsim_info("%s: platform BL : %d panel BL reg : %d\n", __func__, lcd->bd->props.brightness, bl_reg[1]);
	if (dsim_write_hl_data(dsim, bl_reg, ARRAY_SIZE(bl_reg)) < 0)
		dsim_err("%s : fail to write brightness cmd.\n", __func__);
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

	sprintf(buf, "SDC_%02X%02X%02X\n", lcd->id[0], lcd->id[1], lcd->id[2]);

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
#if 0
static ssize_t color_coordinate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_private *priv = dev_get_drvdata(dev);

	sprintf(buf, "%u, %u\n", priv->coordinate[0], priv->coordinate[1]);
	return strlen(buf);
}

static ssize_t manufacture_date_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_private *priv = dev_get_drvdata(dev);
	u16 year;
	u8 month, day, hour, min;

	year = ((priv->date[0] & 0xF0) >> 4) + 2011;
	month = priv->date[0] & 0xF;
	day = priv->date[1] & 0x1F;
	hour = priv->date[2] & 0x1F;
	min = priv->date[3] & 0x3F;

	sprintf(buf, "%d, %d, %d, %d:%d\n", year, month, day, hour, min);
	return strlen(buf);
}

static ssize_t manufacture_code_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct panel_private *priv = dev_get_drvdata(dev);

	sprintf(buf, "%02X%02X%02X%02X%02X\n",
		priv->code[0], priv->code[1], priv->code[2], priv->code[3], priv->code[4]);

	return strlen(buf);
}

static ssize_t dump_register_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dsim_device *dsim;
	struct panel_private *priv = dev_get_drvdata(dev);
	struct lcd_info *lcd = priv->par;
	char *pos = buf;
	u8 reg, len, offset;
	int ret, i;
	u8 *dump = NULL;
	unsigned char read_reg[] = {
		S6D7AA0_READ_TX_REG,
		0x00,
	};

	dsim = container_of(priv, struct dsim_device, priv);

	reg = lcd->dump_info[0];
	len = lcd->dump_info[1];
	offset = lcd->dump_info[2];

	read_reg[1] = reg;

	if (!reg || !len || reg > 0xff || len > 255 || offset > 255)
		goto exit;

	dump = kzalloc(len * sizeof(u8), GFP_KERNEL);

	if (lcd->state == PANEL_STATE_RESUMED) {
		if (dsim_write_hl_data(dsim, SEQ_PASSWD1, ARRAY_SIZE(SEQ_PASSWD1)) < 0)
			dsim_err("fail to write test on f0 command.\n");

		dsim_write_hl_data(dsim, read_reg, ARRAY_SIZE(read_reg));
		ret = dsim_read_hl_data(dsim, S6D7AA0_READ_RX_REG, len, dump);

		if (dsim_write_hl_data(dsim, SEQ_PASSWD1_LOCK, ARRAY_SIZE(SEQ_PASSWD1_LOCK)) < 0)
			dsim_err("fail to write test off f0 command.\n");
	}

	pos += sprintf(pos, "+ [%02X]\n", reg);
	for (i = 0; i < len; i++)
		pos += sprintf(pos, "%2d: %02x\n", i + offset + 1, dump[i]);
	pos += sprintf(pos, "- [%02X]\n", reg);

	dev_info(&lcd->ld->dev, "+ [%02X]\n", reg);
	for (i = 0; i < len; i++)
		dev_info(&lcd->ld->dev, "%2d: %02x\n", i + offset + 1, dump[i]);
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
#endif

static DEVICE_ATTR(lcd_type, 0444, lcd_type_show, NULL);
static DEVICE_ATTR(window_type, 0444, window_type_show, NULL);
//static DEVICE_ATTR(manufacture_code, 0444, manufacture_code_show, NULL);

static DEVICE_ATTR(auto_brightness, 0644, auto_brightness_show, auto_brightness_store);
static DEVICE_ATTR(siop_enable, 0664, siop_enable_show, siop_enable_store);
static DEVICE_ATTR(power_reduce, 0664, power_reduce_show, power_reduce_store);
static DEVICE_ATTR(temperature, 0664, temperature_show, temperature_store);
#if 0
static DEVICE_ATTR(color_coordinate, 0444, color_coordinate_show, NULL);
static DEVICE_ATTR(manufacture_date, 0444, manufacture_date_show, NULL);
static DEVICE_ATTR(dump_register, 0644, dump_register_show, dump_register_store);
#endif
static struct attribute *lcd_sysfs_attributes[] = {
	&dev_attr_siop_enable.attr,
	&dev_attr_power_reduce.attr,
	&dev_attr_temperature.attr,
	&dev_attr_lcd_type.attr,
	&dev_attr_window_type.attr,
	NULL,
};

static struct attribute *backlight_sysfs_attributes[] = {
	&dev_attr_auto_brightness.attr,
	//&dev_attr_weakness_hbm_comp.attr,
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


#if defined(CONFIG_EXYNOS_DECON_MDNIE_LITE)
static int mdnie_lite_write_set(struct dsim_device *dsim, struct lcd_seq_info *seq, u32 num)
{
	int ret = 0, i;

	for (i = 0; i < num; i++) {
		if (seq[i].cmd) {
			ret = dsim_write_hl_data(dsim, seq[i].cmd, seq[i].len);
			if (ret != 0) {
				dsim_err("%s failed.\n", __func__);
				return ret;
			}
		}
		if (seq[i].sleep)
			usleep_range(seq[i].sleep * 1000 , seq[i].sleep * 1000);
	}
	return ret;
}

static int mdnie_lite_send_seq(struct dsim_device *dsim, struct lcd_seq_info *seq, u32 num)
{
	int ret = 0;
	struct panel_private *panel = &dsim->priv;

	if (panel->state != PANEL_STATE_RESUMED) {
		dsim_info("%s : panel is not active\n", __func__);
		return -EIO;
	}

	mutex_lock(&panel->lock);
	ret = mdnie_lite_write_set(dsim, seq, num);

	mutex_unlock(&panel->lock);

	return ret;
}

static int mdnie_lite_read(struct dsim_device *dsim, u8 addr, u8 *buf, u32 size)
{
	int ret = 0;
	struct panel_private *panel = &dsim->priv;

	if (panel->state != PANEL_STATE_RESUMED) {
		dsim_info("%s : panel is not active\n", __func__);
		return -EIO;
	}
	mutex_lock(&panel->lock);
	ret = dsim_read_hl_data(dsim, addr, size, buf);

	mutex_unlock(&panel->lock);

	return ret;
}
#endif


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

struct mipi_dsim_lcd_driver s6d7aa0_mipi_lcd_driver = {
	.early_probe = dsim_panel_early_probe,
	.probe		= dsim_panel_probe,
	.displayon	= dsim_panel_displayon,
	.suspend	= dsim_panel_suspend,
};

static int s6d7aa0_read_init_info(struct dsim_device *dsim)
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
	for (i = 0; i < S6D7AA0_ID_LEN; i++)
		dsim_info("%02x, ", lcd->id[i]);
	dsim_info("\n");

read_exit:
	return 0;

}

static int s6d7aa0_displayon(struct dsim_device *dsim)
{
	int ret = 0;
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	ret = dsim_write_hl_data(dsim, SEQ_DISPLAY_ON, ARRAY_SIZE(SEQ_DISPLAY_ON));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : DISPLAY_ON\n", __func__);
		goto displayon_err;
	}

	dsim_panel_set_brightness(dsim, 1);


displayon_err:
	return ret;

}

static int s6d7aa0_exit(struct dsim_device *dsim)
{
	int ret = 0;
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "%s\n", __func__);
	ret = dsim_write_hl_data(dsim, SEQ_DISPLAY_OFF, ARRAY_SIZE(SEQ_DISPLAY_OFF));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : DISPLAY_OFF\n", __func__);
		goto exit_err;
	}

	msleep(35);

	ret = dsim_write_hl_data(dsim, SEQ_SLEEP_IN, ARRAY_SIZE(SEQ_SLEEP_IN));
	if (ret < 0) {
		dsim_err("%s : fail to write CMD : SLEEP_IN\n", __func__);
		goto exit_err;
	}

	msleep(150);

exit_err:
	return ret;
}

static int s6d7aa0_init(struct dsim_device *dsim)
{
	int ret = 0;
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	ret = dsim_write_hl_data(dsim, SEQ_PASSWD1, ARRAY_SIZE(SEQ_PASSWD1));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_PASSWD1\n", __func__);
		goto init_exit;
	}
	ret = dsim_write_hl_data(dsim, SEQ_PASSWD2, ARRAY_SIZE(SEQ_PASSWD2));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_PASSWD2\n", __func__);
		goto init_exit;
	}
	ret = dsim_write_hl_data(dsim, SEQ_PASSWD3, ARRAY_SIZE(SEQ_PASSWD3));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_PASSWD3\n", __func__);
		goto init_exit;
	}
	ret = dsim_write_hl_data(dsim, SEQ_OTP_RELOAD, ARRAY_SIZE(SEQ_OTP_RELOAD));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_OTP_RELOAD\n", __func__);
		goto init_exit;
	}
	ret = dsim_write_hl_data(dsim, SEQ_BL_ON_CTL, ARRAY_SIZE(SEQ_BL_ON_CTL));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_BL_ON_CTL\n", __func__);
		goto init_exit;
	}
	ret = dsim_write_hl_data(dsim, SEQ_BC_PARAM_MDNIE, ARRAY_SIZE(SEQ_BC_PARAM_MDNIE));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_BC_PARAM_MDNIE\n", __func__);
		goto init_exit;
	}
	ret = dsim_write_hl_data(dsim, SEQ_FD_PARAM_MDNIE, ARRAY_SIZE(SEQ_FD_PARAM_MDNIE));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_FD_PARAM_MDNIE\n", __func__);
		goto init_exit;
	}
	ret = dsim_write_hl_data(dsim, SEQ_FE_PARAM_MDNIE, ARRAY_SIZE(SEQ_FE_PARAM_MDNIE));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_FE_PARAM_MDNIE\n", __func__);
		goto init_exit;
	}
	ret = dsim_write_hl_data(dsim, SEQ_B3_PARAM, ARRAY_SIZE(SEQ_B3_PARAM));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_B3_PARAM\n", __func__);
		goto init_exit;
	}
	ret = dsim_write_hl_data(dsim, SEQ_BACKLIGHT_CTL, ARRAY_SIZE(SEQ_BACKLIGHT_CTL));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_BACKLIGHT_CTL\n", __func__);
		goto init_exit;
	}
	ret = dsim_write_hl_data(dsim, SEQ_PORCH_CTL, ARRAY_SIZE(SEQ_PORCH_CTL));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_PORCH_CTL\n", __func__);
		goto init_exit;
	}
	msleep(10);
	ret = dsim_write_hl_data(dsim, SEQ_SLEEP_OUT, ARRAY_SIZE(SEQ_SLEEP_OUT));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_SLEEP_OUT\n", __func__);
		goto init_exit;
	}
	msleep(120);
		ret = dsim_write_hl_data(dsim, SEQ_PASSWD1_LOCK, ARRAY_SIZE(SEQ_PASSWD1_LOCK));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_PASSWD1_LOCK\n", __func__);
		goto init_exit;
	}
	ret = dsim_write_hl_data(dsim, SEQ_PASSWD2_LOCK, ARRAY_SIZE(SEQ_PASSWD2_LOCK));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_PASSWD2_LOCK\n", __func__);
		goto init_exit;
	}
	ret = dsim_write_hl_data(dsim, SEQ_PASSWD3_LOCK, ARRAY_SIZE(SEQ_PASSWD3_LOCK));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_PASSWD3_LOCK\n", __func__);
		goto init_exit;
	}
	ret = dsim_write_hl_data(dsim, SEQ_TEON_CTL, ARRAY_SIZE(SEQ_TEON_CTL));
	if (ret < 0) {
		dsim_err(":%s fail to write CMD : SEQ_TEON_CTL\n", __func__);
		goto init_exit;
	}

	init_exit:
	return ret;
}

static int s6d7aa0_probe(struct dsim_device *dsim)
{
	int ret = 0;
	struct panel_private *priv = &dsim->priv;
	struct lcd_info *lcd = dsim->priv.par;
	struct device_node *np;
	struct platform_device *pdev;

	pr_info("%s: was called\n", __func__);

	priv->lcdConnected = PANEL_CONNECTED;

	s6d7aa0_read_init_info(dsim);
	if (priv->lcdConnected == PANEL_DISCONNEDTED) {
		dsim_err("dsim : %s lcd was not connected\n", __func__);
		goto exit;
	}
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

	dev_info(&lcd->ld->dev, "%s: done\n", __func__);
exit:
	return ret;

}

struct dsim_panel_ops s6d7aa0_panel_ops = {
	.probe			= s6d7aa0_probe,
	.displayon		= s6d7aa0_displayon,
	.exit			= s6d7aa0_exit,
	.init			= s6d7aa0_init,
};

