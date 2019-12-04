
#include "wdg.h"

/// This Function completely resets the board
void resetBoard(void)
{
	watchdog_enable(1);
	while(1){};
}
