/**
 * \file            esp_ll_template.c
 * \brief           Low-level communication with ESP device template
 */

/*
 * Copyright (c) 2018 Tilen Majerle
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of ESP-AT library.
 *
 * Author:          Tilen MAJERLE <tilen@majerle.eu>
 */
#include "system/esp_ll.h"
#include "esp/esp.h"
#include "esp/esp_mem.h"
#include "esp/esp_input.h"

#include "metal/machine.h"
#include "metal/uart.h"
#include "metal/interrupt.h"

#include "system/circular_buffer.h"
#include "esp_sys_freertos_os.h"
#include "FreeRTOSConfig.h"

#define espatTHREAD_DELAY_MS         1
#define espatCIRCULAR_BUFFER_SIZE    1024
#define espatTHREAD_PRIO    		(configMAX_PRIORITIES)
#define espatTHREAD_STACK   		(360)

static uint8_t initialized = 0;

static uint8_t prvBuf[ espatCIRCULAR_BUFFER_SIZE ];
static cbuf_handle_t prvCircularBuffer = NULL;

struct metal_uart * uart;

#define UART_RXEMPTY            (1 << 31)
#define RXDATA_ADDR       		0x10023004
#define READ_DATA(addr)			(*(volatile uint32_t *)(addr))
#define UART_FIFO_SIZE          8

static uint32_t overFIFO = 0;

void __attribute((section(".data.prvUartInterruptHandler")))
prvUartInterruptHandler( int id, void * param )
{
    char c;
    uint32_t ch;

    ch = READ_DATA(RXDATA_ADDR);

	uint32_t count_len = 0;

	while (!(ch & UART_RXEMPTY))
	{
		count_len++;

		c = ch & 0xff;
		if (prvCircularBuffer)
		{
			circular_buf_put(prvCircularBuffer, c);
		}
		ch = READ_DATA(RXDATA_ADDR);
	}

	if (count_len > UART_FIFO_SIZE)
	{
		overFIFO = 1;
		configASSERT(overFIFO);
	}
}

static void configure_uart()
{
	int rc = 0;

    uart = metal_uart_get_device(1);
    metal_uart_init(uart, ESP_CFG_AT_PORT_BAUDRATE);

    struct metal_interrupt* uart_ic = metal_uart_interrupt_controller(uart);
    metal_interrupt_init(uart_ic);

    int uart_irq = metal_uart_get_interrupt_id(uart);
    rc = metal_interrupt_register_handler(uart_ic, uart_irq, prvUartInterruptHandler, uart);

	if (rc < 0)
	{
		printf("UART interrupt handler registration failed with code %d\n", rc);
	}

    if (metal_interrupt_enable(uart_ic, uart_irq) == -1) {
        printf("UART interrupt enable failed\n");
    }
}

/**
 * \brief           Send data to ESP device, function called from ESP stack when we have data to send
 * \param[in]       data: Pointer to data to send
 * \param[in]       len: Number of bytes to send
 * \return          Number of bytes sent
 */
static size_t send_data( const void * data, size_t len )
{
    /* Implement send function here */
    for( unsigned int i = 0; i < len; ++i )
    {
        metal_uart_putc( uart, ((unsigned char*) data)[ i ] );
        //printf("%c", ((unsigned char*)data)[i]);

    }

#ifdef configESP32_DATA_EXCHANGE_LOG
    _write(STDOUT_FILENO, data, len);
#endif

    return len; /* Return number of bytes actually sent to AT port */
}

void prvUartHandlingThread( void * param );

/**
 * \brief           Callback function called from initialization process
 *
 * \note            This function may be called multiple times if AT baudrate is changed from application.
 *                  It is important that every configuration except AT baudrate is configured only once!
 *
 * \note            This function may be called from different threads in ESP stack when using OS.
 *                  When \ref ESP_CFG_INPUT_USE_PROCESS is set to `1`, this function may be called from user UART thread.
 *
 * \param[in,out]   ll: Pointer to \ref esp_ll_t structure to fill data for communication functions
 * \return          \ref espOK on success, member of \ref espr_t enumeration otherwise
 */
espr_t esp_ll_init( esp_ll_t * ll )
{
    if( !initialized )
    {
        /* Set AT port send function to use when we have data to transmit */
        ll->send_fn = send_data; /* Set callback function to send data */

        /* Configure AT port to be able to send/receive data to/from ESP device */
        prvCircularBuffer = circular_buf_init( prvBuf, espatCIRCULAR_BUFFER_SIZE );
        configure_uart(); /* Initialize SPI for communication */
        esp_sys_thread_create(NULL, "esp-uart", prvUartHandlingThread, NULL, espatTHREAD_STACK, espatTHREAD_PRIO);
    }
    metal_uart_set_baud_rate( uart, ll->uart.baudrate );
    initialized = 1;
    return espOK;
}

/** \brief Thread that is responsible for reading the data from UART and sending it
 *   to ESP-AT-Lib handler
 */
void prvUartHandlingThread( void * param )
{
    /* We break the incapsulation here, tread carefully and debug thoroughly.
     * If we encounter strange bugs it could make sense to just copy everything
     * we have in the circular buffer to the linear one using the circular buffer API
     * and process it
     */

	uint8_t *pC = NULL;
	size_t len;

    for( ; ; )
    {
		if (overFIFO)
		{
//			configPRINT_STRING(("UART FIFO overflowed!\n"));
			overFIFO = 0;
		}

		portENTER_CRITICAL();
		pC = circular_buf_front(prvCircularBuffer, &len);
		portEXIT_CRITICAL();

		if (pC && len)
		{
			esp_input_process(pC, len);

#ifdef configESP32_DATA_EXCHANGE_LOG
			configPRINT_STRING( ( "\033[0;31m") );// red color
    		for (int i = 0; i < len; ++i)
    		{
    			metal_uart_putc(__METAL_DT_STDOUT_UART_HANDLE, pC[i]);
			}
    		configPRINT_STRING( ( "\033[0m") );// no color
#endif

			portENTER_CRITICAL();
			circular_buf_pop(prvCircularBuffer, len);
			portEXIT_CRITICAL();
		}
		else
		{
			vTaskDelay( espatTHREAD_DELAY_MS );
		}

    }
}

/**
 * \brief           Callback function to de-init low-level communication part
 * \param[in,out]   ll: Pointer to \ref esp_ll_t structure to fill data for communication functions
 * \return          \ref espOK on success, member of \ref espr_t enumeration otherwise
 */
espr_t esp_ll_deinit( esp_ll_t * ll )
{
    initialized = 0; /* Clear initialized flag */
    return espOK;
}
