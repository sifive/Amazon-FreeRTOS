/*
 * Copyright (C) 2018 Infineon Technologies AG. All rights reserved.
 *
 * Infineon Technologies AG (Infineon) is supplying this software for use with
 * Infineon's microcontrollers.
 * This file can be freely distributed within development tools that are
 * supporting such microcontrollers.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 */

#ifndef __ESP_CONFIG_H
#define __ESP_CONFIG_H

#include "FreeRTOS.h"
#include <stdio.h>
#include <stdlib.h>
/*
 * Rename this file to "esp_config.h" for your application
 */

/*
 * Open "include/esp/esp_config_default.h" and
 * copy & replace settings you want to change here
 */
#define ESP_CFG_MAX_CONNS                   3
#define ESP_CFG_SYS_PORT                    ESP_SYS_PORT_FREERTOS
#define ESP_CFG_MODE_ACCESS_POINT           0
#define ESP_CFG_NETCONN                     1
#define ESP_CFG_PING                        1
#define ESP_CFG_DNS                         1
#define ESP_CFG_SNTP                        1
#define TIMEZONE	                        -3
#define ESP_CFG_MAX_DETECTED_AP             10
#define ESP_CFG_NETCONN_RECEIVE_TIMEOUT     1
#define ESP_CFG_NETCONN_RECEIVE_QUEUE_LEN   16
#define ESP_CFG_RCV_BUFF_SIZE               0x600
//#define ESP_MEM_SIZE                        (12 * 1024)
#define ESP_CFG_IPD_MAX_BUFF_SIZE           512
#define ESP_CFG_CONN_MAX_DATA_LEN	    	512
#define ESP_MEMCPY(dst, src, len)           memcpy(dst, src, len)
#define ESP_CFG_INPUT_USE_PROCESS           1
#define ESP_CFG_MEM_CUSTOM                  1

#define ESP_CFG_ESP32                       1
#define ESP_CFG_ESP8266                     0
#define ESP_CFG_AT_ECHO                     1
#define ESP_CFG_DBG                         0
#define ESP_DBG_LVL_MIN ESP_DBG_LVL_ALL
#define ESP_CFG_DBG_TYPES_ON                ESP_DBG_TYPE_ALL
#define ESP_CFG_DBG_CONN                    ESP_DBG_ON
#define ESP_CFG_DBG_OUT(fmt, ...)           vLoggingPrintf(fmt, ## __VA_ARGS__);

/* After user configuration, call default config to merge config together */
#include "esp/esp_config_default.h"

/* First include debug */
#include "esp/esp_debug.h"

static inline void*   esp_mem_malloc(size_t size) {
    return pvPortMalloc(size);
}
static inline void*   esp_mem_realloc(void* ptr, size_t size){

	asm("ebreak");
	return (void *)NULL;
	//return realloc(ptr, size);
}
static inline void*   esp_mem_calloc(size_t num, size_t size){
	//return calloc(num, size);
	void * ptr = pvPortMalloc(size*num);
	memset(ptr,0,size*num);
	return ptr;
}
static inline void    esp_mem_free(void* ptr){
	vPortFree(ptr);
}

#endif /* __ESP_CONFIG_H */
