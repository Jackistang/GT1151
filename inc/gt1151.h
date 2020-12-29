/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author          Notes
 * 2020-11-25     tang jia       the first version
 */
#ifndef APPLICATIONS_GT1151_H_
#define APPLICATIONS_GT1151_H_

#include "rtconfig.h"

#ifndef GT1151_BUS_NAME 
#define GT1151_BUS_NAME    "i2c1"
#endif

#ifndef GT1151_RST_PIN
#define GT1151_RST_PIN     51
#endif

#ifndef GT1151_IRQ_PIN
#define GT1151_IRQ_PIN     108
#endif

#ifndef GT1151_TOUCH_WIDTH
#define GT1151_TOUCH_WIDTH  800
#endif

#ifndef GT1151_TOUCH_HEIGHT
#define GT1151_TOUCH_HEIGHT 480
#endif

#ifndef GT1151_SUPPORT_POINTS
#define GT1151_SUPPORT_POINTS 10
#endif

#endif /* APPLICATIONS_GT1151_H_ */
