/*
* dw9714.h
*
* Copyright (c) 2011, NVIDIA, All Rights Reserved.
*
* This file is licensed under the terms of the GNU General Public License
* version 2. This program is licensed "as is" without any warranty of any
* kind, whether express or implied.
*/

#ifndef __DW9714_H__
#define __DW9714_H__

#include <linux/ioctl.h>  /* For IOCTL macros */
#include <linux/i2c.h>

#define DW9714_IOCTL_GET_CONFIG		  _IOR('o', 10, struct dw9714_config)
#define DW9714_IOCTL_SET_POSITION	  _IOW('o', 11, __u32)
#define DW9714_IOCTL_SET_MODE		    _IOW('o', 12, __u32)

struct dw9714_config {
	__u32 settle_time;
	__u32 actuator_range;
	__u32 pos_low;
	__u32 pos_high;
	float focal_length;
	float fnumber;
	float max_aperture;
};
struct dw9714_info {
	struct i2c_client *i2c_client;
	struct dw9714_config config;
};

#endif /* __DW9714_H__ */
