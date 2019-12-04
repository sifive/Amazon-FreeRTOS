#ifndef H_WDG
#define H_WDG

#include "stdint.h"
#include "bsp/metal-platform.h"

#define WDT_AON_CTRL_ADDR			METAL_SIFIVE_AON0_10000000_BASE_ADDRESS
#define WDT_AON_REG(offset) 		(*(volatile uint32_t *)(WDT_AON_CTRL_ADDR + offset))
#define AON_WDOGKEY_VALUE			0x0051F15E
#define AON_WDOGFEED_VALUE			0x0D09F00D

void watchdog_enable(uint32_t milliseconds);
void watchdog_feed();
void watchdog_disable(void);

#endif // H_WDG