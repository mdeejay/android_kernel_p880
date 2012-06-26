/**
 * Copyright (c) 2011 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __MT9M114_H__
#define __MT9M114_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define MT9M114_IOCTL_SET_MODE		_IOW('o', 1, struct mt9m114_mode)
#define MT9M114_IOCTL_GET_STATUS		_IOR('o', 2, struct mt9m114_status)

struct mt9m114_mode {
	int xres;
	int yres;
};

struct mt9m114_status {
	int data;
	int status;
};

#ifdef __KERNEL__
struct mt9m114_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __MT9M114_H__ */

