/*
 * linux/arch/arm/mach-omap2/board-sbc8100_plus-camera.c
 *
 * Copyright (C) 2009 Texas Instruments Inc.
 * Sergio Aguirre <saaguirre@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef CONFIG_TWL4030_CORE

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mm.h>

#include <linux/i2c/twl.h>

#include <asm/io.h>

#include <mach/gpio.h>

static int cam_inited;
#include <media/v4l2-int-device.h>
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>

#define LDPCAM_USE_XCLKA	0
#define LDPCAM_USE_XCLKB	1

#define VAUX_1_8_V		0x05
#define VAUX_DEV_GRP_P1		0x20
#define VAUX_DEV_GRP_NONE	0x00

#if defined(CONFIG_VIDEO_TVP514X) || defined(CONFIG_VIDEO_TVP514X_MODULE)
#include <media/tvp514x-int.h>
#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
static struct omap34xxcam_hw_config decoder_hwc = {  // 解码器硬件配置
        .dev_index              = 0,
        .dev_minor              = 0,
        .dev_type               = OMAP34XXCAM_SLAVE_SENSOR,
        .u.sensor.sensor_isp    = 1,
        .u.sensor.capture_mem   = PAGE_ALIGN(720*525*2*4),
};
// tvp5146配置
// 
static struct isp_interface_config tvp5146_if_config = {
        .ccdc_par_ser           = ISP_PARLL_YUV_BT,  // ccd  
        .dataline_shift         = 0x1,  //      1  camext13-2  对应 cam11-0
        .hsvs_syncdetect        = ISPCTRL_SYNC_DETECT_VSRISE,
        .strobe                 = 0x0,
        .prestrobe              = 0x0,
        .shutter                = 0x0,
        .wait_hs_vs             = 2,
        .u.par.par_bridge       = 0x0,
        .u.par.par_clk_pol      = 0x0,
};
#endif

static struct v4l2_ifparm ifparm = {
        .if_type = V4L2_IF_TYPE_BT656,  // 接口类型
        .u       = {
                .bt656 = {
                        .frame_start_on_rising_vs = 1,
                        .bt_sync_correct = 0,
                        .swap           = 0,
                        .latch_clk_inv  = 0,
                        .nobt_hs_inv    = 0,    /* active high */
                        .nobt_vs_inv    = 0,    /* active high */
                        .mode           = V4L2_IF_TYPE_BT656_MODE_BT_8BIT,  // 模式
                        .clock_min      = TVP514X_XCLK_BT656,
                        .clock_max      = TVP514X_XCLK_BT656,
                },
        },
};

/**
 * @brief tvp5146_ifparm - Returns the TVP5146 decoder interface parameters
 *
 * @param p - pointer to v4l2_ifparm structure
 *
 * @return result of operation - 0 is success
 */
 // tvp5146接口参数 返回tvp5146接口参数
 // 输入参数 p 指向一个v4l2的ifparm结构体
 // 返回操作结果  如果成功返回0
static int tvp5146_ifparm(struct v4l2_ifparm *p)
{
        if (p == NULL)
                return -EINVAL;

        *p = ifparm;  // 直接返回全局结构体
        return 0;
}

/**
 * @brief tvp5146_set_prv_data - Returns tvp5146 omap34xx driver private data
 *
 * @param priv - pointer to omap34xxcam_hw_config structure
 *
 * @return result of operation - 0 is success
 */
 // tvp5146私有数据结构体  返回tvp5146 omap34xx驱动私有数据
 // 参数 priv 指向omap34xxcam hw config 结构体
 // 返回操作结果
static int tvp5146_set_prv_data(struct v4l2_int_device *s, void *priv)
{
#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
        struct omap34xxcam_hw_config *hwc = priv;  // 获取输入的结构体

        if (priv == NULL)
                return -EINVAL;
		// 返回实际参数的值  就是将decoder_hwc中的值复制到输入的参数结构体中
        hwc->u.sensor.sensor_isp = decoder_hwc.u.sensor.sensor_isp;
        hwc->u.sensor.capture_mem = decoder_hwc.u.sensor.capture_mem;
        hwc->dev_index = decoder_hwc.dev_index;
        hwc->dev_minor = decoder_hwc.dev_minor;
        hwc->dev_type = decoder_hwc.dev_type;
        return 0;
#else
        return -EINVAL;
#endif
}

/**
 * @brief tvp5146_power_set - Power-on or power-off TVP5146 device
 *
 * @param power - enum, Power on/off, resume/standby
 *
 * @return result of operation - 0 is success
 */
 // tvp5146设定电源 power on和power off
 // 参数 power  开 关 恢复 标准
 // 返回操作结果，如果成功返回0
static int tvp5146_power_set(struct v4l2_int_device *s, enum v4l2_power power)
{
        struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;

        switch (power) {
        case V4L2_POWER_OFF:
                break;

        case V4L2_POWER_STANDBY:
                break;

        case V4L2_POWER_ON:  // power on 上电
#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
				// 配置isp接口
                isp_configure_interface(vdev->cam->isp, &tvp5146_if_config);
#endif
                break;

        default:
                return -ENODEV;
                break;
        }
        return 0;
}

// tvp5146摄像头的结构体
struct tvp514x_platform_data tvp5146_pdata = {
        .master         = "omap34xxcam",
        .power_set      = tvp5146_power_set,  // 电源设定
        .priv_data_set  = tvp5146_set_prv_data, // 设定私有参数
        .ifparm         = tvp5146_ifparm,  // 接口参数设定
        /* Some interface dependent params */  // 一些接口的独立参数 
        .clk_polarity   = 0, /* data clocked out on falling edge */  // clk下降沿
        .hs_polarity    = 1, /* 0 - Active low, 1- Active high */  // 高有效
        .vs_polarity    = 1, /* 0 - Active low, 1- Active high */  // 高有效
};
#endif                          /* #ifdef CONFIG_VIDEO_TVP514X */


// 下面讲到的是ov2556 数字摄像头
#if defined(CONFIG_VIDEO_OV2656) || defined(CONFIG_VIDEO_OV2656_MODULE)
#define GPIO_CAM_PDN            167  // cam的power on 引脚
#define GPIO_CAM_RST            126  // cam的复位引脚
#include <media/ov2656.h>
// ov2656最大的帧字节大小
#define OV2656_BIGGEST_FRAME_BYTE_SIZE	PAGE_ALIGN(2048 * 1536 * 2)
// omap34xxcam传感器配置
static struct omap34xxcam_sensor_config ov2656_hwc = {
	.sensor_isp = 1,
	.capture_mem = OV2656_BIGGEST_FRAME_BYTE_SIZE * 2,
	.ival_default	= { 1, 15 },
};
// isp接口配置
static struct isp_interface_config ov2656_if_config = {
	.ccdc_par_ser		= ISP_PARLL,  // 并行
	.dataline_shift 	= 0x1,  // 数据偏移
	.hsvs_syncdetect 	= ISPCTRL_SYNC_DETECT_VSRISE,// vs上升沿检测
	.strobe 		= 0x0,// strobe
	.prestrobe 		= 0x0,
	.shutter 		= 0x0,
    	.wenlog 		= ISPCCDC_CFG_WENLOG_AND,  // wenlog and
	.wait_hs_vs             = 2,
	.u.par.par_bridge       = 0x3,
	.u.par.par_clk_pol      = 0x0,
};
// ov2656传感器 设定私有数据函数，设定到输入的参数priv结构体中返回
static int ov2656_sensor_set_prv_data(struct v4l2_int_device *s, void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.sensor_isp = ov2656_hwc.sensor_isp;  // 获取信息
	hwc->u.sensor.capture_mem = ov2656_hwc.capture_mem;
	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	return 0;
}
// ov2656摄像头电源设定  比如power on
static int ov2656_sensor_power_set(struct v4l2_int_device *s,
				   enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	// 静态变量
	static enum v4l2_power previous_power = V4L2_POWER_OFF;

	// 如果没有初始化过cam，则返回
	if (!cam_inited) {
		printk(KERN_ERR "OV2656: Unable to control board GPIOs!\n");
		return -EFAULT;
	}

	// 根据输入的参数power
	switch (power) {
	case V4L2_POWER_ON:// 开启电源
		// isp配置
		isp_configure_interface(vdev->cam->isp, &ov2656_if_config);

		// 如果之前的电源是关闭的
		if (previous_power == V4L2_POWER_OFF) {
			/* turn on analog power */
			// 打开模拟电源
			// 1.8v
			twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_1_8_V, TWL4030_VAUX4_DEDICATED);
			twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_DEV_GRP_P1, TWL4030_VAUX4_DEV_GRP);
			// gpio方向输出 poweron引脚和rst引脚
			gpio_direction_output(GPIO_CAM_PDN, 0);
			gpio_direction_output(GPIO_CAM_RST, 0);
			mdelay(1);
			// 开始复位
			gpio_direction_output(GPIO_CAM_RST, 1);
			mdelay(1);
		}
		break;
		// 关闭电源
	case V4L2_POWER_OFF:
		/* Power Down Sequence */
		// 掉电顺序
		twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				VAUX_DEV_GRP_NONE, TWL4030_VAUX4_DEV_GRP);
		break;
		// 标准电源
	case V4L2_POWER_STANDBY:
		break;
	}
	previous_power = power;

	return 0;
}

// ov2656传感器设定xclk
static u32 ov2656_sensor_set_xclk(struct v4l2_int_device *s, u32 xclkfreq)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	// isp设定xclk
	return isp_set_xclk(vdev->cam->isp, xclkfreq, LDPCAM_USE_XCLKA);
}

// ov2656平台数据
struct ov2656_platform_data sbc8100_plus_ov2656_platform_data = {
	.power_set	 = ov2656_sensor_power_set,// 设定power函数
	.priv_data_set	 = ov2656_sensor_set_prv_data,// 设定私有数据函数
	.set_xclk	 = ov2656_sensor_set_xclk,// 设定xclk函数
};

#endif//CONFIG_VIDEO_OV2656

// cam初始化
void __init sbc8100_plus_cam_init(void)
{
#if defined(CONFIG_VIDEO_OV2656) || defined(CONFIG_VIDEO_OV2656_MODULE)
	cam_inited = 0;

		// 请求power on这个gpio口
        if (gpio_request(GPIO_CAM_PDN, "OV2656 PND") < 0) {
                printk("Can't get GPIO %d\n", GPIO_CAM_PDN);
                return;
        }

		// 请求cam的rst这个gpio口
        if (gpio_request(GPIO_CAM_RST, "OV2656 RST") < 0) {
                printk("Can't get GPIO %d\n", GPIO_CAM_RST);
                return;
        }

		// gpio口输出
        gpio_direction_output(GPIO_CAM_PDN, 0);
        gpio_direction_output(GPIO_CAM_RST, 0);
        mdelay(1);
        gpio_direction_output(GPIO_CAM_RST, 1);
        mdelay(1);
#endif
	// 表示cam初始化完成
	cam_inited = 1;
}
#else//CONFIG_TWL4030_CORE
void __init sbc8100_plus_cam_init(void)
{
}
#endif
