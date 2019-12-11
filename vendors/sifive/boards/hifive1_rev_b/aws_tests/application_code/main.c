/*
 * Amazon FreeRTOS V1.4.7
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
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo includes */
#include "aws_test_runner.h"

/* AWS library includes. */
#include "iot_system_init.h"
#include "iot_logging_task.h"
#include "iot_wifi.h"
#include "aws_clientcredential.h"
#include "aws_application_version.h"
#include "aws_dev_mode_key_provisioning.h"

#include <string.h>

#include "metal/cpu.h"
#include "metal/machine.h"
#include <metal/machine/platform.h>
#include "metal/uart.h"
#include "metal/interrupt.h"
#include "metal/clock.h"

#define CPU_CLK					128000000

#define SPI_SCKDIV_ADDR			0x10014000
#define READ_DATA(addr)			(*(volatile uint32_t *)(addr))
#define WRITE_DATA(addr,val)	((*(volatile uint32_t *)(addr)) = (val))


/* Logging Task Defines. */
#define mainLOGGING_TASK_PRIORITY                       ( configMAX_PRIORITIES - 1 )
#define mainLOGGING_TASK_STACK_SIZE                     ( configMINIMAL_STACK_SIZE * 2 )
#define mainLOGGING_MESSAGE_QUEUE_LENGTH                ( 10 )


/* The task delay for allowing the lower priority logging task to print out Wi-Fi
 * failure status before blocking indefinitely. */
#define mainLOGGING_WIFI_STATUS_DELAY       pdMS_TO_TICKS( 1000 )

/* Unit test defines. */
#define mainTEST_RUNNER_TASK_STACK_SIZE     ( configMINIMAL_STACK_SIZE * 10 )

/* The name of the devices for xApplicationDNSQueryHook. */
#define mainDEVICE_NICK_NAME				"SiFive_tests" /* FIX ME.*/

#define AON_CTRL_ADDR		METAL_SIFIVE_AON0_10000000_BASE_ADDRESS
#define AON_PMUKEY			METAL_SIFIVE_AON0_PMUKEY
#define AON_UNBLOCK_VALUE	0x51F15E
#define AON_PMUSLEEP		METAL_SIFIVE_AON0_PMUSLEEP
#define AON_PMUSLEEPI0		METAL_SIFIVE_AON0_PWM_SLEEP_BASE
#define AON_PMUWAKEUPI0		METAL_SIFIVE_AON0_PMU_WAKEUP_BASE
#define AON_REG(offset) (*(volatile uint32_t *)(AON_CTRL_ADDR + offset))

/**
 * @brief Application task startup hook for applications using Wi-Fi. If you are not
 * using Wi-Fi, then start network dependent applications in the vApplicationIPNetorkEventHook
 * function. If you are not using Wi-Fi, this hook can be disabled by setting
 * configUSE_DAEMON_TASK_STARTUP_HOOK to 0.
 */
void vApplicationDaemonTaskStartupHook( void );

/**
 * @brief Connects to Wi-Fi.
 */
static int prvWifiConnect( void );

/**
 * @brief Initializes the board.
 */
static void prvMiscInitialization( void );
/*-----------------------------------------------------------*/

extern void __metal_plic0_handler (int id, void *priv);

/* External interrupt handler */
void handle_trap(){
	__metal_plic0_handler(0, &__metal_dt_interrupt_controller_c000000);
}

void reset_ESP()
{
    // Force the PMU SLEEP function to ONLY lower pmu_out_1 for a while
    // 0x20 is PMU_OUT_1, which is connected to WiFi EN
    // 0x10 is PMU_OUT_0, which is connected to CORE POWER EN

	// Force the PMU sleep function to turn off PMU1
	// This is about 20ms
	for(int i=0;i<4;i++) {
		AON_REG(AON_PMUKEY) = AON_UNBLOCK_VALUE;
		AON_REG(AON_PMUSLEEPI0+i*4) = 0x1a;
	}

	for(int i=4;i<8;i++) {
		AON_REG(AON_PMUKEY) = AON_UNBLOCK_VALUE;
		AON_REG(AON_PMUSLEEPI0+i*4) = 0x30;
	}

	// Force the PMU wake function to turn on PMU1 at all times
	for(int i=0;i<8;i++) {
		AON_REG(AON_PMUKEY) = AON_UNBLOCK_VALUE;
		AON_REG(AON_PMUWAKEUPI0+i*4) = 0x30;
	}

	// Execute SLEEP.  But this doesn't actually put the thing to sleep.
	// It does, however apparently execute WAKE as well as sleep.
	AON_REG(AON_PMUKEY) = AON_UNBLOCK_VALUE;
	AON_REG(AON_PMUSLEEP) = 0;
}

/**
 * @brief Application runtime entry point.
 */
int main( void )
{
    /* Perform any hardware initialization that does not require the RTOS to be
     * running.  */

    prvMiscInitialization();

    /* Create tasks that are not dependent on the Wi-Fi being initialized. */
    xLoggingTaskInitialize( mainLOGGING_TASK_STACK_SIZE,
    						mainLOGGING_TASK_PRIORITY,
                            mainLOGGING_MESSAGE_QUEUE_LENGTH );

    /* Start the scheduler.  Initialization that requires the OS to be running,
     * including the Wi-Fi initialization, is performed in the RTOS daemon task
     * startup hook. */
    vTaskStartScheduler();

    return 0;
}
/*-----------------------------------------------------------*/

extern void __metal_driver_sifive_fe310_g000_pll_init(struct __metal_driver_sifive_fe310_g000_pll *pll);

static void prvMiscInitialization( void )
{
	WRITE_DATA(SPI_SCKDIV_ADDR, 1);				//Divider = 1, i.r. SCK = CPU_CLK/4

    /* Null out the memory protection configuration */
    asm("CSRWI pmpcfg0, 0");
    asm("CSRWI pmpcfg1, 0");
    asm("CSRWI pmpcfg2, 0");
    asm("CSRWI pmpcfg3, 0");

    // disable all external interrupts
	*((volatile unsigned int *)(METAL_RISCV_PLIC0_C000000_BASE_ADDRESS + METAL_RISCV_PLIC0_ENABLE_BASE)) = 0;
	*((volatile unsigned int *)(METAL_RISCV_PLIC0_C000000_BASE_ADDRESS + METAL_RISCV_PLIC0_ENABLE_BASE+4)) = 0;

	reset_ESP();

    long init_rate = 0;

    __metal_driver_sifive_fe310_g000_pll_init(&__metal_dt_clock_4);
    init_rate = metal_clock_set_rate_hz(&__metal_dt_clock_4.clock, CPU_CLK);

    configPRINT_STRING("Test Message\n");

	printf("CLOCK: %d MHz\n", (int)(init_rate/1000000));

	struct metal_cpu *cpu;
	struct metal_interrupt *cpu_intr;

	cpu = metal_cpu_get(metal_cpu_get_current_hartid());
	if (cpu == NULL) {
		return;
	}

	cpu_intr = metal_cpu_interrupt_controller(cpu);
	if (cpu_intr == NULL) {
		return;
	}
	metal_interrupt_init(cpu_intr);

	if (metal_interrupt_enable(cpu_intr, 0) == -1) {
		return;
	}
}
/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook( void )
{
    /* FIX ME: Perform any hardware initialization, that require the RTOS to be
     * running, here. */

#if 1
    /* FIX ME: If your MCU is using Wi-Fi, delete surrounding compiler directives to
     * enable the unit tests and after MQTT, Bufferpool, and Secure Sockets libraries
     * have been imported into the project. If you are not using Wi-Fi, see the
     * vApplicationIPNetworkEventHook function. */

	if( SYSTEM_Init() == pdPASS )
	{
		/* Connect to the Wi-Fi before running the tests. */
		if (prvWifiConnect() > 0)
		{
			/* Provision the device with AWS certificate and private key. */
			vDevModeKeyProvisioning();

			/* Create the task to run tests. */
			xTaskCreate( TEST_RUNNER_RunTests_task,
						 "TestRunner",
						 mainTEST_RUNNER_TASK_STACK_SIZE,
						 NULL,
						 tskIDLE_PRIORITY,
						 NULL );
		}
		else
		{
	        configPRINTF( ( "WiFi module failed to initialize.\r\n" ) );
		}
	}

#endif
}

/*-----------------------------------------------------------*/

int prvWifiConnect( void )
{
	WIFINetworkParams_t xNetworkParams;
	WIFIReturnCode_t xWifiStatus;
	uint8_t ucTempIp[4] = { 0 };

	xWifiStatus = WIFI_On();

	if( xWifiStatus == eWiFiSuccess )
	{

		configPRINTF( ( "Wi-Fi module initialized. Connecting to AP...\r\n" ) );
	}
	else
	{
		configPRINTF( ( "Wi-Fi module failed to initialize.\r\n" ) );

		/* Delay to allow the lower priority logging task to print the above status.
		 * The while loop below will block the above printing. */
		vTaskDelay( mainLOGGING_WIFI_STATUS_DELAY );

		while( 1 )
		{
		}
		return 0;
	}

	/* Setup parameters. */
	xNetworkParams.pcSSID = clientcredentialWIFI_SSID;
	xNetworkParams.ucSSIDLength = sizeof( clientcredentialWIFI_SSID );
	xNetworkParams.pcPassword = clientcredentialWIFI_PASSWORD;
	xNetworkParams.ucPasswordLength = sizeof( clientcredentialWIFI_PASSWORD );
	xNetworkParams.xSecurity = clientcredentialWIFI_SECURITY;
	xNetworkParams.cChannel = 0;

	xWifiStatus = WIFI_ConnectAP( &( xNetworkParams ) );

	if( xWifiStatus == eWiFiSuccess )
	{
		configPRINTF( ( "Wi-Fi Connected to AP. Creating tasks which use network...\r\n" ) );

		xWifiStatus = WIFI_GetIP( ucTempIp );
		if ( eWiFiSuccess == xWifiStatus )
		{
			configPRINTF( ( "IP Address acquired %d.%d.%d.%d\r\n",
							ucTempIp[ 0 ], ucTempIp[ 1 ], ucTempIp[ 2 ], ucTempIp[ 3 ] ) );
		}

		return 1;
	}
	else
	{
		/* Connection failed, configure SoftAP. */
		configPRINTF( ( "Wi-Fi failed to connect to AP %s.\r\n", xNetworkParams.pcSSID ) );

		xNetworkParams.pcSSID = wificonfigACCESS_POINT_SSID_PREFIX;
		xNetworkParams.pcPassword = wificonfigACCESS_POINT_PASSKEY;
		xNetworkParams.xSecurity = wificonfigACCESS_POINT_SECURITY;
		xNetworkParams.cChannel = wificonfigACCESS_POINT_CHANNEL;

		configPRINTF( ( "Connect to SoftAP %s using password %s. \r\n",
						xNetworkParams.pcSSID, xNetworkParams.pcPassword ) );

		while( WIFI_ConfigureAP( &xNetworkParams ) != eWiFiSuccess )
		{
			configPRINTF( ( "Connect to SoftAP %s using password %s and configure Wi-Fi. \r\n",
							xNetworkParams.pcSSID, xNetworkParams.pcPassword ) );
		}

		configPRINTF( ( "Wi-Fi configuration successful. \r\n" ) );

		return 1;
	}

    return 0;
}

/*-----------------------------------------------------------*/

/**
 * @brief User defined Idle task function.
 *
 * @note Do not make any blocking operations in this function.
 */
void vApplicationIdleHook( void )
{
    /* FIX ME. If necessary, update to application idle periodic actions. */

    static TickType_t xLastPrint = 0;
    TickType_t xTimeNow;
    const TickType_t xPrintFrequency = pdMS_TO_TICKS( 5000 );

    xTimeNow = xTaskGetTickCount();

    if( ( xTimeNow - xLastPrint ) > xPrintFrequency )
    {
        configPRINT( ( ".") );
        xLastPrint = xTimeNow;
    }
}
/*-----------------------------------------------------------*/

/**
* @brief User defined application hook to process names returned by the DNS server.
*/
#if ( ipconfigUSE_LLMNR != 0 ) || ( ipconfigUSE_NBNS != 0 )
    BaseType_t xApplicationDNSQueryHook( const char * pcName )
    {
        /* FIX ME. If necessary, update to applicable DNS name lookup actions. */

        BaseType_t xReturn;

        /* Determine if a name lookup is for this node.  Two names are given
         * to this node: that returned by pcApplicationHostnameHook() and that set
         * by mainDEVICE_NICK_NAME. */
        if( strcmp( pcName, pcApplicationHostnameHook() ) == 0 )
        {
            xReturn = pdPASS;
        }
        else if( strcmp( pcName, mainDEVICE_NICK_NAME ) == 0 )
        {
            xReturn = pdPASS;
        }
        else
        {
            xReturn = pdFAIL;
        }

        return xReturn;
    }

#endif /* if ( ipconfigUSE_LLMNR != 0 ) || ( ipconfigUSE_NBNS != 0 ) */
/*-----------------------------------------------------------*/

/**
 * @brief User defined assertion call. This function is plugged into configASSERT.
 * See FreeRTOSConfig.h to define configASSERT to something different.
 */
void vAssertCalled(const char * pcFile,
    uint32_t ulLine)
{
    /* FIX ME. If necessary, update to applicable assertion routine actions. */

    const uint32_t ulLongSleep = 1000UL;
    volatile uint32_t ulBlockVariable = 0UL;
    volatile char * pcFileName = (volatile char *)pcFile;
    volatile uint32_t ulLineNumber = ulLine;

    (void)pcFileName;
    (void)ulLineNumber;

    configPRINTF( ("vAssertCalled %s, %ld\n", pcFile, (long)ulLine) );

    /* Setting ulBlockVariable to a non-zero value in the debugger will allow
    * this function to be exited. */
    taskDISABLE_INTERRUPTS();
    {
        while (ulBlockVariable == 0UL)
        {
            vTaskDelay( pdMS_TO_TICKS( ulLongSleep ) );
        }
    }
    taskENABLE_INTERRUPTS();
}
/*-----------------------------------------------------------*/

/**
 * @brief User defined application hook need by the FreeRTOS-Plus-TCP library.
 */
#if ( ipconfigUSE_LLMNR != 0 ) || ( ipconfigUSE_NBNS != 0 ) || ( ipconfigDHCP_REGISTER_HOSTNAME == 1 )
    const char * pcApplicationHostnameHook(void)
    {
        /* FIX ME: If necessary, update to applicable registration name. */

        /* This function will be called during the DHCP: the machine will be registered
         * with an IP address plus this name. */
        return clientcredentialIOT_THING_NAME;
    }


#endif
