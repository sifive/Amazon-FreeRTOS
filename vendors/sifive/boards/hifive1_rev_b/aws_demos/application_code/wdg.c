/* */
#include "wdg.h"
#include "platform.h"
#include "metal/drivers/riscv_cpu.h"

#define RTC_FREQUENCY METAL_DEFAULT_RTC_FREQ

void watchdog_enable(uint32_t milliseconds) {
  uint8_t  scaleFactor = 0;
  uint32_t compareVal = (uint64_t)milliseconds*RTC_FREQUENCY/1000;

  /* Work out what scale factor to use, required to get cmp_val < 2^16 */
  while(compareVal > 65535)
  {
	scaleFactor++;
    compareVal /= 2;
  }

  WDT_AON_REG(METAL_SIFIVE_AON0_WDOGKEY) = AON_WDOGKEY_VALUE;
  WDT_AON_REG(METAL_SIFIVE_AON0_WDOGFEED) = AON_WDOGFEED_VALUE;

  WDT_AON_REG(METAL_SIFIVE_AON0_WDOGKEY)  = AON_WDOGKEY_VALUE;
  WDT_AON_REG(METAL_SIFIVE_AON0_WDOGCMP)  = compareVal;

  WDT_AON_REG(METAL_SIFIVE_AON0_WDOGKEY)  = AON_WDOGKEY_VALUE;
  WDT_AON_REG(METAL_SIFIVE_AON0_WDOGCFG)  = (1<<12)       /* wdogenalways - wdog runs always */
                        | (1<< 8)       /* wdogrsten    - enable reset     */
						| (scaleFactor << 0); /* wdogscale    - depend on timeout  */
}

void watchdog_feed() {
  WDT_AON_REG(METAL_SIFIVE_AON0_WDOGKEY)  = AON_WDOGKEY_VALUE;
  WDT_AON_REG(METAL_SIFIVE_AON0_WDOGFEED) = AON_WDOGFEED_VALUE;
}

void watchdog_disable(void) {

	watchdog_feed();

	WDT_AON_REG(METAL_SIFIVE_AON0_WDOGKEY)  = AON_WDOGKEY_VALUE;
	WDT_AON_REG(METAL_SIFIVE_AON0_WDOGCMP)  = 0;

	WDT_AON_REG(METAL_SIFIVE_AON0_WDOGKEY)  = AON_WDOGKEY_VALUE;
	WDT_AON_REG(METAL_SIFIVE_AON0_WDOGCFG)  = 0;
}

