#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "mark_detector.h"
#include "machine_controller.h"
#include "../servo_motor/button.h"
#include "../servo_motor/servo_motor.h"


typedef enum {
    AUTOMATIC_IDLE,      
    AUTOMATIC_MARK_GOTO,
    AUTOMATIC_MARK_GOTO_IN_PROGRESS,
    AUTOMATIC_MARK_POS_REACHED,
    AUTOMATIC_REACH_CONSTANT_SPEED,
    AUTOMATIC_WAITING_FOR_SAMPLES,     
    AUTOMATIC_SCANNING,     
    AUTOMATIC_FOUND,     
    AUTOMATIC_RETURN_TO_ZERO,     
    AUTOMATIC_FINISHED
} automatic_substate_t;

void activate_automatic_state();

void handle_automatic_state();

void handle_cutter_state();

