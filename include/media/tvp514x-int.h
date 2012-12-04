/*
 * drivers/media/video/tvp514x.h
 *
 * Copyright (C) 2008 Texas Instruments Inc
 * Author: Vaibhav Hiremath <hvaibhav@ti.com>
 *
 * Contributors:
 *     Sivaraj R <sivaraj@ti.com>
 *     Brijesh R Jadav <brijesh.j@ti.com>
 *     Hardik Shah <hardik.shah@ti.com>
 *     Manjunath Hadli <mrh@ti.com>
 *     Karicheri Muralidharan <m-karicheri2@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef _TVP514X_H
#define _TVP514X_H

/*
 * Other macros
 */
#define TVP514X_MODULE_NAME		"tvp514x"  // 设备名称

#define TVP514X_XCLK_BT656		(27000000)  // 

/* Number of pixels and number of lines per frame for different standards */
#define NTSC_NUM_ACTIVE_PIXELS		(720)   // ntsc 像素点
#define NTSC_NUM_ACTIVE_LINES		(480)   //  ntsc 线输入
#define PAL_NUM_ACTIVE_PIXELS		(720)  // 
#define PAL_NUM_ACTIVE_LINES		(576)

/**
 * enum tvp514x_input - enum for different decoder input pin
 *		configuration.
 */
 // tvp514x输入枚举  不同的解码器的输入引脚配置
enum tvp514x_input {
	/*
	 * CVBS input selection
	 */
	 // cvbs输入选择
	INPUT_CVBS_VI1A = 0x0,
	INPUT_CVBS_VI1B,
	INPUT_CVBS_VI1C,
	INPUT_CVBS_VI2A = 0x04,
	INPUT_CVBS_VI2B,
	INPUT_CVBS_VI2C,
	INPUT_CVBS_VI3A = 0x08,
	INPUT_CVBS_VI3B,
	INPUT_CVBS_VI3C,
	INPUT_CVBS_VI4A = 0x0C,
	/*
	 * S-Video input selection
	 */
	 // s video输入选择
	INPUT_SVIDEO_VI2A_VI1A = 0x44,
	INPUT_SVIDEO_VI2B_VI1B,
	INPUT_SVIDEO_VI2C_VI1C,
	INPUT_SVIDEO_VI2A_VI3A = 0x54,
	INPUT_SVIDEO_VI2B_VI3B,
	INPUT_SVIDEO_VI2C_VI3C,
	INPUT_SVIDEO_VI4A_VI1A = 0x4C,
	INPUT_SVIDEO_VI4A_VI1B,
	INPUT_SVIDEO_VI4A_VI1C,
	INPUT_SVIDEO_VI4A_VI3A = 0x5C,
	INPUT_SVIDEO_VI4A_VI3B,
	INPUT_SVIDEO_VI4A_VI3C,

	/* Need to add entries for
	 * RGB, YPbPr and SCART.
	 */
	 // 增加选项在这里
	INPUT_INVALID
};

/**
 * enum tvp514x_output - enum for output format
 *			supported.
 *
 */
 // tvp514x输出格式支持 枚举
enum tvp514x_output {
	OUTPUT_10BIT_422_EMBEDDED_SYNC = 0,
	OUTPUT_20BIT_422_SEPERATE_SYNC,
	OUTPUT_10BIT_422_SEPERATE_SYNC = 3,
	OUTPUT_INVALID
};

/**
 * struct tvp514x_platform_data - Platform data values and access functions.
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @ifparm: Interface parameters access function.
 * @priv_data_set: Device private data (pointer) access function.
 * @clk_polarity: Clock polarity of the current interface.
 * @ hs_polarity: HSYNC Polarity configuration for current interface.
 * @ vs_polarity: VSYNC Polarity configuration for current interface.
 */
 // tvp514x摄像头的平台数据结构体  用于保存操作的函数
 // power_set 用于电源设定的操作函数
 // ifparm 接口参数操作函数
 // priv_data_set 设备私有数据指针操作函数
 // clk_polarity 当前接口的clk极性配置
 // hs_polarity  当前接口hsync极性配置
 // vs_polarity  当前接口vsync极性配置
struct tvp514x_platform_data {
	char *master;
	int (*ifparm)(struct v4l2_ifparm *p);// 接口参数设定
	int (*power_set)(struct v4l2_int_device *s, enum v4l2_power power);// 电源设定
	int (*priv_data_set)(struct v4l2_int_device *s, void *priv);// 私有数据设定
	/* Interface control params */
	bool clk_polarity;  // clk极性
	bool hs_polarity;  // hs极性
	bool vs_polarity;  // vs极性
};


#endif				/* ifndef _TVP514X_H */
