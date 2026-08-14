#ifndef CORE1_MAIN_H
#define CORE1_MAIN_H

#include "hmi/hmi.h"

extern hmi_t hmi;

// Public API: Starts Core 1 and sets up the display task
void core1_main(void);

#endif