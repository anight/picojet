// esp_system.h - esp_restart(), which two JetExamples showcases call after their
// closing fade to begin the next loop.
//
// The watchdog is the RP2350's clean reset: it restarts both cores and every
// peripheral, which is what the showcases rely on to start again from nothing.
#pragma once

#include "hardware/watchdog.h"

static inline void esp_restart(void)
{
	watchdog_reboot(0, 0, 0);
	for (;;) { }
}
