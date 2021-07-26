/*
 * Copyright (C) 2013, NVIDIA Corporation.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_panel.h>

#include <video/display_timing.h>
#include <video/mipi_display.h>
#include <linux/of_device.h>
#include <video/of_display_timing.h>
#include <linux/of_graph.h>
#include <video/videomode.h>

#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/delay.h>

enum PANEL_TYPE {
	PANEL_BDC = 0,
	PANEL_XENARC,
	PANEL_DRMC_28,
	PANEL_DRMC_32,
	PANEL_DRMC_48
};

struct duragon_panel_desc {
	enum PANEL_TYPE type;
	const struct drm_display_mode *modes;
	unsigned int num_modes;
	const struct display_timing *timings;
	unsigned int num_timings;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	/**
	 * @reset: the time (in milliseconds) indicates the delay time
	 *         after the panel to operate reset gpio
	 * @init: the time (in milliseconds) that it takes for the panel to
	 *           power on and dsi host can send command to panel
	 * @prepare: the time (in milliseconds) that it takes for the panel to
	 *           become ready and start receiving video data
	 * @enable: the time (in milliseconds) that it takes for the panel to
	 *          display the first valid frame after starting to receive
	 *          video data
	 * @disable: the time (in milliseconds) that it takes for the panel to
	 *           turn the display off (no content is visible)
	 * @unprepare: the time (in milliseconds) that it takes for the panel
	 *             to power itself down completely
	 */
	struct {
		unsigned int reset;
		unsigned int init;
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;

	u32 bus_format;
};

struct ptn3460 {
	u8 address;
};

struct ds90ub927q {
	u8 address;
	u8 deserializer_id;
	bool link;
};

struct ds90ub928q {
	u8 address;
	u8 serializer_id;
};

struct pca9534a {
	u8 address;
};

struct duragon_panel {
	struct drm_panel base;
	bool prepared;
	bool enabled;
	bool power_invert;

	struct device *dev;
	const struct duragon_panel_desc *desc;

	struct backlight_device *backlight;
	
	struct regulator *supply;

	struct i2c_adapter *edid_bus;
	struct i2c_adapter *control_bus;

	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;

	struct ptn3460 *edp_to_lvds;
	struct ds90ub927q *fpd_link_ser;
	struct ds90ub928q *fpd_link_deser;
	struct pca9534a *panel_io;

	struct mutex lock;
	struct task_struct *thread;
};

static int probe_defer = 0;

static int control_bus_read_reg(struct duragon_panel *panel, u8 slave, u8 reg, u8 *val)
{
	int ret;
	struct i2c_msg msg[2];

	mutex_lock(&panel->lock);
	msg[0].addr = slave;
	msg[0].buf = &reg;
	msg[0].len = 1;
	msg[0].flags = 0;

	msg[1].addr = slave;
	msg[1].buf = val;
	msg[1].len = 1;
	msg[1].flags = I2C_M_RD | I2C_M_NOSTART;

	ret = i2c_transfer(panel->control_bus, msg, 2);
	if(ret < 0)
		dev_err(panel->dev, "slave: 0x%02x reg: 0x%02x read failed, %d", slave, reg, ret);
	else 
		dev_dbg(panel->dev, "slave: 0x%02x reg: 0x%02x value: 0x%02x", slave, reg, *val);

	mutex_unlock(&panel->lock);

	return ret;
}

static int control_bus_write_reg(struct duragon_panel *panel, u8 slave, u8 reg, u8 val)
{
	int ret;
	struct i2c_msg msg;
	u8 buf[2] = { 0x00, 0x00 };

	mutex_lock(&panel->lock);
	buf[0] = reg;
	buf[1] = val;
	msg.addr = slave;
	msg.buf = buf;
	msg.len = 2;
	msg.flags = 0;

	ret = i2c_transfer(panel->control_bus, &msg, 1);
	if(ret < 0)
		dev_err(panel->dev, "slave: 0x%02x reg: 0x%02x val: 0x%02x write failed, %d", slave, reg, val, ret);

	if(ret > 0)
		dev_err(panel->dev, "slave: 0x%02x reg: 0x%02x val: 0x%02x writen", slave, reg, val);

	mutex_unlock(&panel->lock);

	return ret;
}

static int control_bus_read_bit(struct duragon_panel *panel, u8 slave, u8 reg, u8 bit, bool *value)
{
	int ret;
	u8 val;

	ret = control_bus_read_reg(panel, slave, reg, &val);
	if(ret < 0)
		goto error;

	if(val & (1 << bit))
		*value = true;
	else
		*value = false;
	
error:
	return ret;
}

static int control_bus_write_bit(struct duragon_panel *panel, u8 slave, u8 reg, u8 bit, bool value)
{
	int ret;
	u8 val;

	ret = control_bus_read_reg(panel, slave, reg, &val);
	if(ret < 0)
		goto error;

	if(value)
		val |= (1 << bit);
	else
		val &= ~(1 << bit);

	ret = control_bus_write_reg(panel, slave, reg, val);

error:
	return ret;
}

static int control_bus_check_and_update_reg(struct duragon_panel *panel, u8 slave, u8 reg, u8 match)
{
	int ret;
	u8 val;

	ret = control_bus_read_reg(panel, slave, reg, &val);
	if(ret < 0)
		goto error;

	if(val == match)
		ret = 0;
	else
		ret = 1;

	if(ret)
		ret = control_bus_write_reg(panel, slave, reg, match);

error:
	return ret;
}

static int control_bus_check_and_update_bit(struct duragon_panel *panel, u8 slave, u8 reg, u8 bit, bool match)
{
	int ret;
	bool val;

	ret = control_bus_read_bit(panel, slave, reg, bit, &val);

	if(ret < 0)
		goto error;
	
	if(val == match)
		ret = 0;
	else 
		ret = 1;

	if(ret)
		ret = control_bus_write_bit(panel, slave, reg, bit, match);

error:
	return ret;
}

static void duragon_panel_sleep(unsigned int msec)
{
	if (msec > 20)
		msleep(msec);
	else
		usleep_range(msec * 1000, (msec + 1) * 1000);
}

static inline struct duragon_panel *to_duragon_panel(struct drm_panel *panel)
{
	return container_of(panel, struct duragon_panel, base);
}

static int duragon_panel_get_fixed_modes(struct duragon_panel *panel)
{
	struct drm_connector *connector = panel->base.connector;
	struct drm_device *drm = panel->base.drm;
	struct drm_display_mode *mode;
	unsigned int i, num = 0;

	if (!panel->desc)
		return 0;

	for (i = 0; i < panel->desc->num_timings; i++) {
		const struct display_timing *dt = &panel->desc->timings[i];
		struct videomode vm;
		videomode_from_timing(dt, &vm);
		mode = drm_mode_create(drm);
		if (!mode) {
			dev_err(drm->dev, "failed to add mode %ux%u\n",
				dt->hactive.typ, dt->vactive.typ);
			continue;
		}

		drm_display_mode_from_videomode(&vm, mode);
		drm_mode_set_name(mode);

		drm_mode_probed_add(connector, mode);
		num++;
	}

	for (i = 0; i < panel->desc->num_modes; i++) {
		const struct drm_display_mode *m = &panel->desc->modes[i];
		mode = drm_mode_duplicate(drm, m);
		if (!mode) {
			dev_err(drm->dev, "failed to add mode %ux%u@%u\n",
				m->hdisplay, m->vdisplay, m->vrefresh);
			continue;
		}

		drm_mode_set_name(mode);

		drm_mode_probed_add(connector, mode);
		num++;
	}

	connector->display_info.bpc = panel->desc->bpc;
	connector->display_info.width_mm = panel->desc->size.width;
	connector->display_info.height_mm = panel->desc->size.height;
	if (panel->desc->bus_format)
		drm_display_info_set_bus_formats(&connector->display_info,
						 &panel->desc->bus_format, 1);

	return num;
}

static int duragon_panel_of_get_native_mode(struct duragon_panel *panel)
{
	struct drm_connector *connector = panel->base.connector;
	struct drm_device *drm = panel->base.drm;
	struct drm_display_mode *mode;
	struct device_node *timings_np;
	int ret;

	timings_np = of_get_child_by_name(panel->dev->of_node,
					  "display-timings");
	if (!timings_np) {
		dev_warn(panel->dev, "OF display-timings not available\n");
		return 0;
	}

	of_node_put(timings_np);
	mode = drm_mode_create(drm);
	if (!mode)
		return 0;

	ret = of_get_drm_display_mode(panel->dev->of_node, mode,
				      OF_USE_NATIVE_MODE);
	if (ret) {
		dev_warn(panel->dev, "failed to find dts display timings\n");
		drm_mode_destroy(drm, mode);
		return 0;
	}

	drm_mode_set_name(mode);
	mode->type |= DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static int duragon_panel_regulator_enable(struct drm_panel *panel)
{
	struct duragon_panel *p = to_duragon_panel(panel);
	int err = 0;

	if (p->power_invert) {
		if (regulator_is_enabled(p->supply) > 0)
			regulator_disable(p->supply);
	} else {
		err = regulator_enable(p->supply);
		if (err < 0) {
			dev_err(panel->dev, "failed to enable supply: %d\n",
				err);
			return err;
		}
	}

	return err;
}

static int duragon_panel_regulator_disable(struct drm_panel *panel)
{
	struct duragon_panel *p = to_duragon_panel(panel);
	int err = 0;

	if (p->power_invert) {
		if (!regulator_is_enabled(p->supply)) {
			err = regulator_enable(p->supply);
			if (err < 0) {
				dev_err(panel->dev, "failed to enable supply: %d\n",
					err);
				return err;
			}
		}
	} else {
		regulator_disable(p->supply);
	}

	return err;
}

static int duragon_panel_loader_protect(struct drm_panel *panel, bool on)
{
	struct duragon_panel *p = to_duragon_panel(panel);
	int err;

	dev_info(panel->dev, "%s -> %s", __func__, on ? "ON" : "OFF");
	if (on) {
		err = duragon_panel_regulator_enable(panel);
		if (err < 0) {
			dev_err(panel->dev, "failed to enable supply: %d\n", err);
			return err;
		}

		p->prepared = true;
		p->enabled = true;
	} else {
		/* do nothing */
	}

	return 0;
}

static int duragon_panel_disable(struct drm_panel *panel)
{
	struct duragon_panel *p = to_duragon_panel(panel);

	dev_info(panel->dev, "%s", __func__);
	if (!p->enabled)
		return 0;

	backlight_disable(p->backlight);

	if (p->desc && p->desc->delay.disable)
		duragon_panel_sleep(p->desc->delay.disable);

	p->enabled = false;

	return 0;
}

static int duragon_panel_unprepare(struct drm_panel *panel)
{
	struct duragon_panel *p = to_duragon_panel(panel);

	dev_info(panel->dev, "%s", __func__);
	if (!p->prepared)
		return 0;

	if (p->reset_gpio)
		gpiod_direction_output(p->reset_gpio, 1);

	if (p->enable_gpio)
		gpiod_direction_output(p->enable_gpio, 0);

	duragon_panel_regulator_disable(panel);

	if (p->desc && p->desc->delay.unprepare)
		duragon_panel_sleep(p->desc->delay.unprepare);

	p->prepared = false;

	return 0;
}

static int duragon_panel_prepare(struct drm_panel *panel)
{
	struct duragon_panel *p = to_duragon_panel(panel);
	int err;

	dev_info(panel->dev, "%s", __func__);
	if (p->prepared)
		return 0;

	err = duragon_panel_regulator_enable(panel);
	if (err < 0) {
		dev_err(panel->dev, "failed to enable supply: %d\n", err);
		return err;
	}

	if (p->enable_gpio)
		gpiod_direction_output(p->enable_gpio, 1);

	if (p->desc && p->desc->delay.prepare)
		duragon_panel_sleep(p->desc->delay.prepare);

	if (p->reset_gpio)
		gpiod_direction_output(p->reset_gpio, 0);

	if (p->desc && p->desc->delay.reset)
		duragon_panel_sleep(p->desc->delay.reset);

	if (p->desc && p->desc->delay.init)
		duragon_panel_sleep(p->desc->delay.init);

	p->prepared = true;

	return 0;
}

static int duragon_panel_enable(struct drm_panel *panel)
{
	struct duragon_panel *p = to_duragon_panel(panel);

	dev_info(panel->dev, "%s", __func__);
	if (p->enabled)
		return 0;

	if (p->desc && p->desc->delay.enable)
		duragon_panel_sleep(p->desc->delay.enable);

	backlight_enable(p->backlight);

	p->enabled = true;

	return 0;
}

static int duragon_panel_get_modes(struct drm_panel *panel)
{
	struct duragon_panel *p = to_duragon_panel(panel);
	int num = 0;

	dev_info(panel->dev, "%s", __func__);
	/* add device node plane modes */
	num += duragon_panel_of_get_native_mode(p);

	/* add hard-coded panel modes */
	num += duragon_panel_get_fixed_modes(p);

	/* probe EDID if a DDC bus is available */
	if (p->edid_bus) {
		struct edid *edid = drm_get_edid(panel->connector, p->edid_bus);
		drm_mode_connector_update_edid_property(panel->connector, edid);
		if (edid) {
			num += drm_add_edid_modes(panel->connector, edid);
			kfree(edid);
		}
	}

	return num;
}

static int duragon_panel_get_timings(struct drm_panel *panel,
				    unsigned int num_timings,
				    struct display_timing *timings)
{
	struct duragon_panel *p = to_duragon_panel(panel);
	unsigned int i;

	dev_info(panel->dev, "%s", __func__);
	if (!p->desc)
		return 0;

	if (p->desc->num_timings < num_timings)
		num_timings = p->desc->num_timings;

	if (timings)
		for (i = 0; i < num_timings; i++)
			timings[i] = p->desc->timings[i];

	return p->desc->num_timings;
}

static const struct drm_panel_funcs duragon_panel_funcs = {
	.loader_protect = duragon_panel_loader_protect,
	.disable = duragon_panel_disable,
	.unprepare = duragon_panel_unprepare,
	.prepare = duragon_panel_prepare,
	.enable = duragon_panel_enable,
	.get_modes = duragon_panel_get_modes,
	.get_timings = duragon_panel_get_timings,
};

static int bl_update_status(struct backlight_device *bl)
{
	struct duragon_panel *p = bl_get_data(bl);

	if (!p->prepared)
		return 0;

	return 0;
}

static int bl_get_brightness(struct backlight_device *bl)
{
	struct duragon_panel *p = bl_get_data(bl);
	u16 brightness = bl->props.brightness;

	if (!p->prepared)
		return 0;

	return brightness & 0xff;
}

static const struct backlight_ops dcs_bl_ops = {
	.update_status = bl_update_status,
	.get_brightness = bl_get_brightness,
};

static int of_property_read_hex_u8(const struct device_node *node, const char *property, u8 *val)
{
	int err = 0;
	const __be32 *addr_be;
	u32 addr;
	int len;

	addr_be = of_get_property(node, property, &len);
	if (!addr_be || (len < sizeof(*addr_be))) {
		pr_err("%s: invalid %s\n", node->full_name, property);
		err = -ENXIO;
		goto error;
	}
	
	addr = be32_to_cpup(addr_be);

	*val = addr;

error:
	return err;
}

static int ptn3460_probe(struct duragon_panel *panel)
{
	struct device_node *edp_lvds_node;
	struct device *dev = panel->dev;
	u8 address;
	int err;
	u8 val;

	edp_lvds_node = of_get_child_by_name(dev->of_node, "ptn3460");
	if(!edp_lvds_node) {
		err = -ENXIO;
		goto error;
	}
	
	err = of_property_read_hex_u8(edp_lvds_node, "address", &address);
	if(err)
		goto error;

	panel->edp_to_lvds = devm_kzalloc(dev, sizeof(*panel->edp_to_lvds), GFP_KERNEL);
	if(!panel->edp_to_lvds) {
		err = -ENOMEM;
		goto error;
	}

	panel->edp_to_lvds->address = address;
	dev_info(dev, "eDP -> LVDS[0x%02x]", address);

	/**
	 * Required setting already taken care by u-boot 
	 * So here we make sure
	 **/
	err = control_bus_read_reg(panel, panel->edp_to_lvds->address, 0x81, &val);

	if(err > 0) {
		if(val != 0x10 && (panel->desc->bpc == 8))	/* Should be 24bit JEIDA */
			err = control_bus_write_reg(panel, panel->edp_to_lvds->address, 0x81, 0x10);	
		if(val != 0x20 && (panel->desc->bpc == 6))	/* Should be 18bit VESA and JEIDA */
			err = control_bus_write_reg(panel, panel->edp_to_lvds->address, 0x81, 0x20);	

		
		err = control_bus_read_reg(panel, panel->edp_to_lvds->address, 0x81, &val);
		if(err > 0)
			dev_info(dev, "Color and Data Format: %s\n", (((val & 0x30) >> 4) > 2) ? "RESERVED" : 
											(((val & 0x30) >> 4) > 1) ? "VESA and JEIDA 18bpp" : 
											(((val & 0x30) >> 4) > 0) ? "JEIDA 24bpp" : " VESA 24bpp");
	}

error:
	return err;
}

static int fpd_link_iii_status(struct duragon_panel *panel, bool *status) 
{
	int err;
	u8 val;

	err = control_bus_read_reg(panel, panel->fpd_link_ser->address, 0x0c, &val);
	if(err > 0) {
		if(!(val & (1 << 0)))
			*status = false;
		else
			*status = true;
			
		if(!(val & (1 << 2)))
			dev_info(panel->dev, "PCLK not detected");
	}

	return err;
}

static int ds90ub927q_probe(struct duragon_panel *panel)
{
	struct device_node *ds90ub927q_node;
	struct device *dev = panel->dev;
	u8 address;
	int err;
	u8 val;

	ds90ub927q_node = of_get_child_by_name(dev->of_node, "ds90ub927q");
	if(!ds90ub927q_node) {
		err = -ENXIO;
		goto error;
	}
	
	err = of_property_read_hex_u8(ds90ub927q_node, "address", &address);
	if(err)
		goto error;

	panel->fpd_link_ser = devm_kzalloc(dev, sizeof(*panel->fpd_link_ser), GFP_KERNEL);
	if(!panel->fpd_link_ser) {
		err = -ENOMEM;
		goto error;
	}

	panel->fpd_link_ser->address = address;
	dev_info(dev, "LVDS -> FPDLink III[0x%02x]", address);

	fpd_link_iii_status(panel, &panel->fpd_link_ser->link);
	if(!panel->fpd_link_ser->link)
		dev_info(panel->dev, "Cable link not detected");

	err = control_bus_read_reg(panel, panel->fpd_link_ser->address, 0x06, &val);
	if(err > 0)
		panel->fpd_link_ser->deserializer_id = (val >> 1);

error:
	err = -ENXIO;
	return err;
}

static int ds90ub928q_probe(struct duragon_panel *panel)
{
	struct device_node *ds90ub928q_node;
	struct device *dev = panel->dev;
	u8 address;
	int err;
	u8 val;

	ds90ub928q_node = of_get_child_by_name(dev->of_node, "ds90ub928q");
	if(!ds90ub928q_node) {
		err = -ENXIO;
		goto error;
	}
	
	err = of_property_read_hex_u8(ds90ub928q_node, "address", &address);
	if(err)
		goto error;

	panel->fpd_link_deser = devm_kzalloc(dev, sizeof(*panel->fpd_link_deser), GFP_KERNEL);
	if(!panel->fpd_link_deser) {
		err = -ENOMEM;
		goto error;
	}

	panel->fpd_link_deser->address = address;
	dev_info(dev, "FPDLink III -> LVDS[0x%02x]", address);

	err = control_bus_read_reg(panel, panel->fpd_link_deser->address, 0x07, &val);
	if(err > 0)
		panel->fpd_link_deser->serializer_id = (val >> 1);

error:

	return err;
}

static int pca9534a_probe(struct duragon_panel *panel)
{
	struct device_node *pca9534a_node;
	struct device *dev = panel->dev;
	u8 address;
	int err;

	pca9534a_node = of_get_child_by_name(dev->of_node, "ds90ub928q");
	if(!pca9534a_node) {
		err = -ENXIO;
		goto error;
	}
	
	err = of_property_read_hex_u8(pca9534a_node, "address", &address);
	if(err)
		goto error;

	panel->panel_io = devm_kzalloc(dev, sizeof(*panel->panel_io), GFP_KERNEL);
	if(!panel->panel_io) {
		err = -ENOMEM;
		goto error;
	}

	panel->panel_io->address = address;
	dev_info(dev, "PANEL IO[0x%02x]", address);

error:

	return err;
}

static int duragon_panel_thread(void *pv)
{
	struct duragon_panel *panel = (struct duragon_panel *)pv;

	int err;
	u8 val;
	bool alert = false;

	/* This will monitor panel hot plug/remove every second*/
	while (!kthread_should_stop())
	{
		msleep(1000);

		/* Check FPD Link III Status */
		fpd_link_iii_status(panel, &panel->fpd_link_ser->link);
		while(!panel->fpd_link_ser->link) {
			msleep(1000);
			err = fpd_link_iii_status(panel, &panel->fpd_link_ser->link);
			if(!alert) {
				dev_info(panel->dev, "FPDLink III Link lost");
				alert = true;
			}
		}

		if(alert) {
			alert = false;
			dev_info(panel->dev, "FPDLink III Link detected");
		}

		/* I2C Pass all */
		err = control_bus_check_and_update_reg(panel, panel->fpd_link_ser->address, 0x17, 0x9e);
		if(err > 0)
			dev_crit(panel->dev, "Serializer I2C Control register currupted\n");

		/* Serializer GPIO 1 */
		err = control_bus_check_and_update_reg(panel, panel->fpd_link_ser->address, 0x0e, 0x05);
		if(err > 0)
			dev_crit(panel->dev, "Serializer GPIO1 register currupted\n");


		err = control_bus_check_and_update_reg(panel, panel->fpd_link_deser->address, 0x1e, 0x03);
		if(err > 0)
			dev_crit(panel->dev, "DeSerializer GPIO1 register currupted\n");

		err = control_bus_check_and_update_bit(panel, panel->fpd_link_ser->address, 0xc6, 5, true);
		if(err > 0) {
			dev_crit(panel->dev, "Serializer ICR register currupted\n");
			/* To activate ICR currpted reg fix we need to do dummy read */
			err = control_bus_read_reg(panel, panel->fpd_link_ser->address, 0xc7, &val);
		}

		if(panel->enabled) {
			/* We use pca9534a module driver */
			if(panel->enable_gpio)
				gpiod_direction_output(panel->enable_gpio, 1);

			if(panel->reset_gpio)
				gpiod_direction_output(panel->reset_gpio, 0);
		}

	}
	
	return 0;
}

static int duragon_panel_probe(struct device *dev, const struct duragon_panel_desc *desc)
{
	struct device_node *backlight_node;
	struct device_node *edid_bus_node;
	struct device_node *control_bus_node;
	struct duragon_panel *panel;
	struct duragon_panel_desc *of_desc;
	u32 u32_val;
	int err;
	const char *compatible_str;

	err = of_property_read_string(dev->of_node, "compatible", &compatible_str);
	if(!err)
		dev_info(dev, "Panel: %s", compatible_str);

	panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
	if (!panel) {
		err = -ENOMEM;
		goto free_nothing;
	}

	if (!desc) {
		of_desc = devm_kzalloc(dev, sizeof(*of_desc), GFP_KERNEL);

		if(!of_desc) {
			err = -ENOMEM;
			goto free_panel;
		}
			
		if (!of_property_read_u32(dev->of_node, "bus-format", &u32_val))
			of_desc->bus_format = u32_val;
		if (!of_property_read_u32(dev->of_node, "bpc", &u32_val))
			of_desc->bpc = u32_val;
		if (!of_property_read_u32(dev->of_node, "prepare-delay-ms", &u32_val))
			of_desc->delay.prepare = u32_val;
		if (!of_property_read_u32(dev->of_node, "enable-delay-ms", &u32_val))
			of_desc->delay.enable = u32_val;
		if (!of_property_read_u32(dev->of_node, "disable-delay-ms", &u32_val))
			of_desc->delay.disable = u32_val;
		if (!of_property_read_u32(dev->of_node, "unprepare-delay-ms", &u32_val))
			of_desc->delay.unprepare = u32_val;
		if (!of_property_read_u32(dev->of_node, "reset-delay-ms", &u32_val))
			of_desc->delay.reset = u32_val;
		if (!of_property_read_u32(dev->of_node, "init-delay-ms", &u32_val))
			of_desc->delay.init = u32_val;
		if (!of_property_read_u32(dev->of_node, "width-mm", &u32_val))
			of_desc->size.width = u32_val;
		if (!of_property_read_u32(dev->of_node, "height-mm", &u32_val))
			of_desc->size.height = u32_val;
	}
	else
		of_desc = devm_kmemdup(dev, desc, sizeof(*of_desc), GFP_KERNEL);

	panel->enabled = false;
	panel->prepared = false;
	panel->desc = of_desc;
	panel->dev = dev;

	panel->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(panel->supply)) {
		err = PTR_ERR(panel->supply);
		dev_err(dev, "failed to get power regulator, %d\n", err);
		goto free_panel;
	}

	panel->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(panel->enable_gpio)) {
		err = PTR_ERR(panel->enable_gpio);
		if(err == -EPROBE_DEFER) {
			if(probe_defer != -EPROBE_DEFER) {
				dev_err(dev, "enable-gpio defered\n");		
				probe_defer = -EPROBE_DEFER;
				goto free_panel;
			}
			else {
				/* We tried */
				err = 0;
			}
		}
		dev_err(dev, "failed to request enable-gpio, %d\n", err);		
	}
	else {
		probe_defer = 0;
	}

	panel->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(panel->reset_gpio)) {
		err = PTR_ERR(panel->reset_gpio);
		if(err == -EPROBE_DEFER) {
			if(probe_defer != -EPROBE_DEFER) {
				dev_err(dev, "reset-gpio defered\n");
				probe_defer = -EPROBE_DEFER;
				goto free_panel;
			}
			else {
				/* We tried */
				err = 0;
			}
		}
		dev_err(dev, "failed to request reset-gpio, %d\n", err);
	}
	else {
		probe_defer = 0;
	}

	panel->power_invert = of_property_read_bool(dev->of_node, "power-invert");

	backlight_node = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight_node) {
		panel->backlight = of_find_backlight_by_node(backlight_node);
		of_node_put(backlight_node);
		if (!panel->backlight) {
			err = -EPROBE_DEFER;
			dev_err(dev, "failed to find backlight, %d\n", err);
			goto free_panel;
		}
	}

	edid_bus_node = of_parse_phandle(dev->of_node, "edid-i2c-bus", 0);
	if (edid_bus_node) {
		panel->edid_bus = of_find_i2c_adapter_by_node(edid_bus_node);
		of_node_put(edid_bus_node);

		if (!panel->edid_bus) {
			err = -EPROBE_DEFER;
			dev_err(dev, "failed to find edid-i2c-bus: %d\n", err);
			goto free_backlight;
		}
	}

	control_bus_node = of_parse_phandle(dev->of_node, "control-i2c-bus", 0);
	if (control_bus_node) {
		panel->control_bus = of_find_i2c_adapter_by_node(control_bus_node);
		of_node_put(control_bus_node);

		if (!panel->control_bus) {
			err = -EPROBE_DEFER;
			dev_err(dev, "failed to find control-i2c-bus: %d\n", err);
			goto free_edid;
		}
	}	

	mutex_init(&panel->lock);
	ptn3460_probe(panel);
	ds90ub927q_probe(panel);
	ds90ub928q_probe(panel);
	pca9534a_probe(panel);
	if(err == -EPROBE_DEFER)
		goto free_resources;

	drm_panel_init(&panel->base);
	panel->base.dev = dev;
	panel->base.funcs = &duragon_panel_funcs;

	err = drm_panel_add(&panel->base);
	if (err < 0)
		goto free_resources;

	dev_set_drvdata(dev, panel);

	panel->thread = kthread_run(duragon_panel_thread, panel, "DURAGON PANEL");
	if(panel->thread)
		err = 0;
	else
		dev_err(panel->dev, "cannot create thread\n");

	goto free_nothing;

free_resources:

	if(panel->panel_io)
		devm_kfree(dev, panel->panel_io);

	if(panel->fpd_link_deser)
		devm_kfree(dev, panel->fpd_link_deser);

	if(panel->fpd_link_ser)
		devm_kfree(dev, panel->fpd_link_ser);

	if(panel->edp_to_lvds)
		devm_kfree(dev, panel->edp_to_lvds);

	if (panel->control_bus) 
		put_device(&panel->edid_bus->dev);

	mutex_destroy(&panel->lock);

free_edid:
	if (panel->edid_bus)
		put_device(&panel->edid_bus->dev);
    
free_backlight:
	if (panel->backlight)
		put_device(&panel->backlight->dev);

free_panel:
	if(panel->desc)
		devm_kfree(dev, (void *)panel->desc);

	if(panel)
		devm_kfree(dev, (void *)panel);

free_nothing:

	return err;
}

static int duragon_panel_remove(struct device *dev)
{
	struct duragon_panel *panel = dev_get_drvdata(dev);

	dev_info(panel->dev, "%s", __func__);
	drm_panel_detach(&panel->base);
	drm_panel_remove(&panel->base);

	duragon_panel_disable(&panel->base);
	duragon_panel_unprepare(&panel->base);

	if(panel->thread)
		kthread_stop(panel->thread);

	if(panel->panel_io)
		devm_kfree(dev, panel->panel_io);

	if(panel->fpd_link_deser)
		devm_kfree(dev, panel->fpd_link_deser);

	if(panel->fpd_link_ser)
		devm_kfree(dev, panel->fpd_link_ser);

	if(panel->edp_to_lvds)
		devm_kfree(dev, panel->edp_to_lvds);

	mutex_destroy(&panel->lock);

	if (panel->control_bus) 
		put_device(&panel->edid_bus->dev);

	if (panel->edid_bus)
		put_device(&panel->edid_bus->dev);

	if (panel->backlight)
		put_device(&panel->backlight->dev);

	return 0;
}

static void duragon_panel_shutdown(struct device *dev)
{
	struct duragon_panel *panel = dev_get_drvdata(dev);

	dev_info(panel->dev, "%s", __func__);
	duragon_panel_disable(&panel->base);

	if (panel->prepared) {
		if (panel->reset_gpio)
			gpiod_direction_output(panel->reset_gpio, 1);

		if (panel->enable_gpio)
			gpiod_direction_output(panel->enable_gpio, 0);

		duragon_panel_regulator_disable(&panel->base);
	}
}


static const struct drm_display_mode xenarc_wvga_display_mode = {
	.clock = 29500,
	.hdisplay = 800,
	.hsync_start = 800 + 91,
	.hsync_end = 800 + 91 + 90,
	.htotal = 800 + 91 + 90 + 100,
	.vdisplay = 480,
	.vsync_start = 480 + 9,
	.vsync_end = 480 + 9 + 8,
	.vtotal = 480 + 9 + 8 + 10,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

static const struct display_timing xenarc_wvga_display_timing = {
	.pixelclock = { 28000000, 29500000, 32000000 },
	.hactive = { 800, 800, 800 },
	.hfront_porch = { 61, 91, 141 },
	.hback_porch = {60, 90, 140 },
	.hsync_len = { 12, 12, 12 },
	.vactive = { 480, 480, 480},
	.vfront_porch = { 4, 9, 30 },
	.vback_porch = { 4, 8, 28 },
	.vsync_len = { 2, 2, 2 },
	.flags = DISPLAY_FLAGS_PIXDATA_POSEDGE | DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH | DISPLAY_FLAGS_DE_HIGH,
};

static const struct duragon_panel_desc xenarc_wvga_panel_desc = {
	.type = PANEL_XENARC,
	.timings = &xenarc_wvga_display_timing,
	.num_timings = 1,
	.modes = &xenarc_wvga_display_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 152,
		.height = 91,
	},
	.delay = {
		.prepare = 10,
		.enable = 100,
		.disable = 100,
		.unprepare = 800,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

static const struct of_device_id platform_of_match[] = {
	{
		.compatible = "duragon, bdc-fixed",
		.data = NULL,
	}, {
		.compatible = "duragon, xenarc-wvga",
		.data = &xenarc_wvga_panel_desc,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, platform_of_match);

static int platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_node(platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return duragon_panel_probe(&pdev->dev, id->data);
}

static int platform_remove(struct platform_device *pdev)
{
	return duragon_panel_remove(&pdev->dev);
}

static void platform_shutdown(struct platform_device *pdev)
{
	duragon_panel_shutdown(&pdev->dev);
}

static struct platform_driver duragon_panel_platform_driver = {
	.driver = {
		.name = "duragon-panel",
		.of_match_table = platform_of_match,
	},
	.probe = platform_probe,
	.remove = platform_remove,
	.shutdown = platform_shutdown,
};

static int __init duragon_panel_init(void)
{
	int err;

	err = platform_driver_register(&duragon_panel_platform_driver);
	if (err < 0)
		return err;

	return 0;
}
module_init(duragon_panel_init);

static void __exit duragon_panel_exit(void)
{
	platform_driver_unregister(&duragon_panel_platform_driver);
}
module_exit(duragon_panel_exit);

MODULE_AUTHOR("Ponmadasamy Muthuraj<ponmadasamy@live.com>");
MODULE_DESCRIPTION("DRM Panel Driver for Duragon BDC");
MODULE_LICENSE("GPL and additional rights");
