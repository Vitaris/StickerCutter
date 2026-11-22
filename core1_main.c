#include <stdlib.h>
#include <string.h>
#include "hmi/hmi.h"
#include "core1_main.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>
#include "../machine/machine_controller.h"
#include "../rotary_encoder/rotary_encoder.h"
#include "../lcd/ant_lcd.h"

hmi_t hmi;
volatile uint32_t core1_heartbeat;

// LCD display configuration
#define DISPLAY_COLS 20
#define DISPLAY_ROWS 4
#define LCD_PIN_RS 10
#define LCD_PIN_RW 11
#define LCD_PIN_EN 12
#define LCD_PIN_D4 13
#define LCD_PIN_D5 14
#define LCD_PIN_D6 15
#define LCD_PIN_D7 16

// Compile-time constant configuration
static const lcd_config_t lcd_cfg = {
    .rs = LCD_PIN_RS,
    .rw = LCD_PIN_RW,
    .en = LCD_PIN_EN,
    .d4 = LCD_PIN_D4,
    .d5 = LCD_PIN_D5,
    .d6 = LCD_PIN_D6,
    .d7 = LCD_PIN_D7,
    .cols = DISPLAY_COLS,
    .rows = DISPLAY_ROWS
};


// Internal state hidden from main
static struct repeating_timer lcd_refresh_timer;
static volatile bool lcd_refresh_pending;

static bool lcd_refresh_timer_callback(struct repeating_timer *t) {
    (void)t;
    lcd_refresh_pending = true;
    return true;
}

// Private entry point for Core 1
void core1_main(void) {
    
    hmi_init(&hmi, &lcd_cfg);

    // The timer only schedules rendering; LCD delays run in thread mode.
    static alarm_pool_t *core1_pool;
    core1_pool = alarm_pool_create(2, 4); // hw alarm 2, owned by core 1
    alarm_pool_add_repeating_timer_ms(core1_pool, -20, lcd_refresh_timer_callback, NULL, &lcd_refresh_timer);

    lcd_refresh_pending = true;

    while (true) {
        if (lcd_refresh_pending) {
            lcd_refresh_pending = false;
            hmi_compute(&hmi);
            __atomic_add_fetch(&core1_heartbeat, 1, __ATOMIC_RELAXED);
        }
        tight_loop_contents();
    }
}
