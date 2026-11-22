#ifndef CORE1_MAIN_H
#define CORE1_MAIN_H

#include <stdint.h>
#include "hmi/hmi.h"

extern hmi_t hmi;
extern volatile uint32_t core1_heartbeat;

// Public API: Starts Core 1 and sets up the display task
void core1_main(void);

#endif