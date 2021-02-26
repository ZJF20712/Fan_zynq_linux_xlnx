/*
 * pl_hdmi_encoder.c - DRM slave encoder for Video-out on PL HDMI out
 *
 * Copyright (C) 2020 Fanserasery
 * Author: Fanserasery <ZJF207122@gmail.com>
 *
 * Based on sun4i_hdmi_enc.c  Copyright (C) 2016 Maxime Ripard.
 * Also based on tda998x_drv.c, Copyright (C) 2012 Texas Instruments.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <drm/drmP.h>
#include <drm/drm_edid.h>
#include <drm/drm_crtc.h>
#include <drm/drm_encoder.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include <linux/component.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

static uint link_freq_Mhz = 100;
module_param_named(link_freq, link_freq_Mhz, uint, 0444);
MODULE_PARM_DESC(link_freq, "HDMI Connector Max Frequency in MHZ(default 100Mhz)");

/*
 * Default frame maximums/prefs; can be set in devicetree
 */
#define PL_HDMI_ENC_MAX_FREQ 100000  //KHz
#define PL_HDMI_ENC_MAX_H 1920
#define PL_HDMI_ENC_MAX_V 1080
#define PL_HDMI_ENC_PREF_H 1280
#define PL_HDMI_ENC_PREF_V 720

struct pl_hdmi {
	struct drm_encoder encoder;
	struct drm_connector connector;
	struct i2c_adapter *i2c_bus;
	struct device *dev;
   	struct gpio_desc *HPD_IO;
	struct gpio_desc *HDMI_OE;
	bool hdmi_enabled;
   	u32 fmax;
   	u32 hmax;
   	u32 vmax;
   	u32 hpref;
   	u32 vpref;
};

static inline struct pl_hdmi *to_pl_hdmi_encoder(struct drm_encoder *encoder)
{
	return container_of(encoder, struct pl_hdmi, encoder);
}

static inline struct pl_hdmi *to_pl_hdmi_connector(struct drm_connector *
connector)
{
	return container_of(connector, struct pl_hdmi, connector);
}

//Connector Helper Functions
static int pl_hdmi_connector_get_modes(struct drm_connector *connector)//okay
{
   	struct pl_hdmi *hdmi = to_pl_hdmi_connector(connector);
	struct edid *edid;
   	int num_modes = 0, ret;
   
    if(hdmi->i2c_bus){
        edid = drm_get_edid(connector, hdmi->i2c_bus);
        if (edid) {
            ret = drm_detect_hdmi_monitor(edid);
            if(ret){
                /* Old version: before 4.19 */
                //drm_mode_connector_update_edid_property(connector, edid);
                drm_connector_update_edid_property(connector, edid);
                num_modes = drm_add_edid_modes(connector, edid);
                DRM_DEBUG("Get HDMI EDID Successfully\n");  
                kfree(edid);  
            }
        }
   	}else{
    	num_modes = drm_add_modes_noedid(connector, hdmi->hmax, hdmi->vmax);
    	drm_set_preferred_mode(connector, hdmi->hpref, hdmi->vpref);
		DRM_DEBUG("No EDID I2C port, use default display argument\n"); 
	}  
	return num_modes;
}

static enum drm_mode_status 
pl_hdmi_connector_mode_valid(struct drm_connector *connector,
				 struct drm_display_mode *mode)
{
/*
	u32 vic = drm_match_cea_mode(mode);

	if (vic > 1)
		return MODE_OK;
	else
		return MODE_BAD;
*/

	struct pl_hdmi *hdmi = to_pl_hdmi_connector(connector);
   	if (mode && 
      	!(mode->flags & ((DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK) | 
			DRM_MODE_FLAG_3D_MASK)) &&
      		(mode->clock <= hdmi->fmax) &&
      		(mode->hdisplay <= hdmi->hmax) && 
      		(mode->vdisplay <= hdmi->vmax)) 
        return MODE_OK;
   	return MODE_BAD;
}

static struct drm_encoder *
pl_hdmi_connector_best_encoder(struct drm_connector *connector)
{
	struct pl_hdmi *hdmi = to_pl_hdmi_connector(connector);

	return &hdmi->encoder;
}

//Connector Functions
static enum drm_connector_status pl_hdmi_connector_detect(struct 
drm_connector *connector, 
					bool force)
{
   struct pl_hdmi *hdmi = to_pl_hdmi_connector(connector);
	int connect = 0;

	if(IS_ERR(hdmi->HPD_IO)){
        if (hdmi->i2c_bus)
   		{	
      		if (drm_probe_ddc(hdmi->i2c_bus))
         		return connector_status_connected;
      		return connector_status_disconnected;
  	 	}
   		else
      		return connector_status_unknown; 
	}else{
        connect = gpiod_get_value(hdmi->HPD_IO);
        if(connect){
            dev_dbg(hdmi->dev, "HDMI Attached with HPD High\n");
            return connector_status_connected;
        }else
            return connector_status_disconnected;
	}
}

static void pl_hdmi_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static const struct drm_connector_helper_funcs pl_hdmi_connector_helper_funcs = {
	.get_modes	= pl_hdmi_connector_get_modes,//okay
	.mode_valid = pl_hdmi_connector_mode_valid,//okay
	.best_encoder = pl_hdmi_connector_best_encoder,//okay
};

static const struct drm_connector_funcs pl_hdmi_connector_funcs = {
	.detect			= pl_hdmi_connector_detect,//To check
	//.dpms 			= drm_atomic_helper_connector_dpms,
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.destroy		= pl_hdmi_connector_destroy,//okay
	.reset			= drm_atomic_helper_connector_reset,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
};

//Encoder helper functions
static void pl_hdmi_encoder_enable(struct drm_encoder *encoder)
{
	struct pl_hdmi *hdmi = to_pl_hdmi_encoder(encoder);
	hdmi->hdmi_enabled = 1;
}

static void pl_hdmi_encoder_disable(struct drm_encoder *encoder)
{
	struct pl_hdmi *hdmi = to_pl_hdmi_encoder(encoder);
	hdmi->hdmi_enabled = 0;
}

static bool pl_hdmi_mode_fixup(struct drm_encoder *encoder,
			   const struct drm_display_mode *mode,
			   struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void pl_hdmi_encoder_mode_set(struct drm_encoder *encoder,
				 struct drm_display_mode *mode,
				 struct drm_display_mode *adjusted_mode)
{
}

static int pl_hdmi_atomic_check(struct drm_encoder *encoder,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *conn_state)
{
	struct drm_display_mode *mode = &crtc_state->mode;

	if (mode->flags & DRM_MODE_FLAG_DBLCLK)
		return -EINVAL;

	return 0;
}

static void pl_hdmi_encoder_dpms(struct drm_encoder *encoder, int mode)
{
}

static enum drm_mode_status pl_hdmi_encoder_mode_valid(struct drm_encoder *
encoder,
				    const struct drm_display_mode *mode)
{
   struct pl_hdmi *hdmi = to_pl_hdmi_encoder(encoder);
   if (mode && 
      !(mode->flags & ((DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK) | 
DRM_MODE_FLAG_3D_MASK)) &&
      (mode->clock <= hdmi->fmax) &&
      (mode->hdisplay <= hdmi->hmax) && 
      (mode->vdisplay <= hdmi->vmax)) 
         return MODE_OK;
   return MODE_BAD;
}

static void pl_hdmi_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_helper_funcs pl_hdmi_encoder_helper_funcs = {
	.enable     	= pl_hdmi_encoder_enable,
	.disable    	= pl_hdmi_encoder_disable,
	.mode_fixup		= pl_hdmi_mode_fixup,
	.atomic_check	= pl_hdmi_atomic_check,
	.mode_set		= pl_hdmi_encoder_mode_set,
	.mode_valid		= pl_hdmi_encoder_mode_valid,
	.dpms			= pl_hdmi_encoder_dpms,//set power state
};

static const struct drm_encoder_funcs pl_hdmi_encoder_funcs = {
	.destroy = pl_hdmi_encoder_destroy,
};


static irqreturn_t pl_hdmi_hardirq(int irq, void *dev_id)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t pl_hdmi_irq(int irq, void *dev_id)
{
	struct pl_hdmi *hdmi = dev_id;

	drm_helper_hpd_irq_event(hdmi->connector.dev);

	return IRQ_HANDLED;
}


static void pl_hdmi_of_parse(struct device *dev)
{
	struct device_node *i2c_node;
	struct pl_hdmi *hdmi = dev_get_drvdata(dev);
    int ret;

	//hdmi->fmax = PL_HDMI_ENC_MAX_FREQ;
	hdmi->fmax = link_freq_Mhz * 1000;
	hdmi->hmax = PL_HDMI_ENC_MAX_H;
	hdmi->vmax = PL_HDMI_ENC_MAX_V;
	hdmi->hpref = PL_HDMI_ENC_PREF_H;
	hdmi->vpref = PL_HDMI_ENC_PREF_V;

	hdmi->HPD_IO = devm_gpiod_get(dev, "hdmi-hpd", GPIOD_IN);
	if (IS_ERR(hdmi->HPD_IO)){
		dev_warn(dev, "Failed to get hdmi-hpd pin\n");
	}else{
		gpiod_direction_input(hdmi->HPD_IO);
	}

	hdmi->HDMI_OE = devm_gpiod_get(dev, "hdmi-oe", GPIOD_OUT_HIGH);
	if (IS_ERR(hdmi->HDMI_OE)){
		dev_warn(dev, "Failed to get hdmi-oe pin\n");
	}else{
		gpiod_set_value_cansleep(hdmi->HDMI_OE, 1);
	}

	i2c_node = of_parse_phandle(dev->of_node, "pl_hdmi,edid-i2c", 0);
	if (i2c_node) 
   	{
        dev_info(dev, "pl hdmi get remote i2c node\n");
        hdmi->i2c_bus = of_find_i2c_adapter_by_node(i2c_node);
	   	of_node_put(i2c_node);
   	}
   	if (hdmi->i2c_bus){
        dev_info(dev, "pl hdmi get edid i2c adapter\n");
    }else{
        //For No edid support
        dev_warn(dev, "failed to get the edid i2c adapter, using default modes\n");

        ret = of_property_read_u32(dev->of_node, "pl_hdmi,fmax", &hdmi->fmax);
        if (ret < 0) {            
            dev_warn(dev, "No max frequency in DT, using default %dKHz\n", 
                    	hdmi->fmax);
        }

        ret = of_property_read_u32(dev->of_node, "pl_hdmi,hmax", &hdmi->hmax);
        if (ret < 0) {
            dev_warn(dev, "No max horizontal width in DT, using default %d\n", 
                        hdmi->hmax);
        }

        ret = of_property_read_u32(dev->of_node, "pl_hdmi,vmax", &hdmi->vmax);
        if (ret < 0) {
            dev_warn(dev, "No max vertical height in DT, using default %d\n", 
                        hdmi->vmax);
        }
        
        ret = of_property_read_u32(dev->of_node, "pl_hdmi,hpref", &hdmi->hpref);
        if (ret < 0) {
            dev_warn(dev, "No pref horizontal width in DT, using default %d\n", 
                        hdmi->hpref);
        }

        ret = of_property_read_u32(dev->of_node, "pl_hdmi,vpref", &hdmi->vpref);
        if (ret < 0) {
            dev_warn(dev, "No pref horizontal width in DT, using default %d\n", 
                        hdmi->vpref);
        }
    }
}

static int pl_hdmi_bind(struct device *dev, struct device *master,
			void *data)
{
	struct pl_hdmi *hdmi = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct drm_encoder *encoder = &hdmi->encoder;
	struct drm_connector *connector = &hdmi->connector;
	int ret;
    
	pl_hdmi_of_parse(dev);

	//DRM add encoder
	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm_dev, dev->of_node);
	if (encoder->possible_crtcs == 0){
		DRM_INFO("Possible CRTC is 0, use default value: 1\n");
        encoder->possible_crtcs = 1;
        //return -EPROBE_DEFER;
    }

	ret = drm_encoder_init(drm_dev, encoder, &pl_hdmi_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS, NULL);	
	if (ret) {
		dev_err(dev, "Failed to initialize encoder with drm\n");
		goto err_encoder;
	}
	drm_encoder_helper_add(encoder, &pl_hdmi_encoder_helper_funcs);
	
	//DRM add connector
	//interlace_allowed: can this connector handle interlaced modes?
	//doublescan_allowed: can this connector handle doublescan?
	connector->interlace_allowed = false;
	connector->doublescan_allowed = false;
	//connector->port = dev->of_node;
	
	if(IS_ERR(hdmi->HPD_IO)){		
        /* There is no HPD interrupt, so we need to poll the controller */
		connector->polled = DRM_CONNECTOR_POLL_CONNECT |
						DRM_CONNECTOR_POLL_DISCONNECT;
	}else{
		connector->polled = DRM_CONNECTOR_POLL_HPD;
		ret = devm_request_threaded_irq(dev, gpiod_to_irq(hdmi->HPD_IO), pl_hdmi_hardirq, 
					pl_hdmi_irq, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, dev_name(dev), 
					hdmi);
	}
	
	ret = drm_connector_init(drm_dev, connector,
				 &pl_hdmi_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);
	if (ret) {
		dev_err(dev, "Failed to initialize connector with drm\n");
		goto err_connector;
	}

	drm_connector_helper_add(connector, &pl_hdmi_connector_helper_funcs);
	drm_connector_register(connector);
	drm_connector_attach_encoder(connector, encoder);
    
    return 0;
				
err_connector:
	drm_encoder_cleanup(encoder);
err_encoder:
	return ret;
}

static void pl_hdmi_unbind(struct device *dev, struct device *master,
			   void *data)
{
	struct pl_hdmi *hdmi = dev_get_drvdata(dev);

	drm_connector_cleanup(&hdmi->connector);
	drm_encoder_cleanup(&hdmi->encoder);
}

static const struct component_ops pl_hdmi_ops = {
	.bind 	= pl_hdmi_bind,
	.unbind = pl_hdmi_unbind,
};


static int pl_hdmi_probe(struct platform_device *pdev)
{
	struct pl_hdmi *hdmi;

	dev_info(&pdev->dev, "pl hdmi probe started\n");

	hdmi = devm_kzalloc(&pdev->dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;
	
	platform_set_drvdata(pdev, hdmi);
	hdmi->dev = &pdev->dev;

	return component_add(&pdev->dev, &pl_hdmi_ops);
}

static int pl_hdmi_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &pl_hdmi_ops);
	return 0;
}

static const struct of_device_id pl_hdmi_of_match[] = {
	{ .compatible = "fan,pl-hdmi", },
	{ /* end of table */ },
};
MODULE_DEVICE_TABLE(of, pl_hdmi_of_match);

static struct platform_driver pl_hdmi_driver = {
	.probe			= pl_hdmi_probe,
	.remove			= pl_hdmi_remove,
	.driver			= {
		.owner		= THIS_MODULE,
		.name		= "pl-hdmi",
		.of_match_table	= pl_hdmi_of_match,
	},
};

static int __init pl_hdmi_init(void)
{
	return platform_driver_register(&pl_hdmi_driver);
}

static void __exit pl_hdmi_exit(void)
{
	platform_driver_unregister(&pl_hdmi_driver);
}

//module_init(pl_hdmi_init);
late_initcall(pl_hdmi_init);
module_exit(pl_hdmi_exit);

MODULE_AUTHOR("Fanserasery.");
MODULE_DESCRIPTION("HDMI DRM encoder & connector for Video-out");
MODULE_LICENSE("GPL v2");
