/*
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/* This file contains configuration settings for the demos. */

#ifndef IOT_CONFIG_H_
#define IOT_CONFIG_H_

/* Standard include. */
#include <stdbool.h>

#define AWS_IOT_MQTT_ENABLE_METRICS          	(0)

#define IOT_MQTT_SUBSCRIPTIONS		     	 	(4)
#define IOT_MQTT_MAX_IN_PROGRESS_OPERATIONS  	(3)

/* Platform thread stack size and priority. */
//#define IOT_THREAD_DEFAULT_STACK_SIZE    		6000
#define IOT_THREAD_DEFAULT_STACK_SIZE    		( 512 )
#define IOT_THREAD_DEFAULT_PRIORITY      		( configMAX_PRIORITIES-2 )


/* Include the common configuration file for FreeRTOS. */
#include "iot_config_common.h"

#endif /* ifndef IOT_CONFIG_H_ */
