#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "mark_detector.h"
#include "machine_controller.h"
#include "../servo_motor/button.h"
#include "../servo_motor/servo_motor.h"

typedef struct {
    bool bottom_mark_detected;

    float sticker_height;
    float mark_distance;

    float first_mark_position;
    float second_mark_position;
    float third_mark_position;
} marks_monitor_t;

typedef enum {
    AUTOMATIC_IDLE,      
    AUTOMATIC_MARK_GOTO,
    AUTOMATIC_MARK_GOTO_IN_PROGRESS,
    AUTOMATIC_MARK_POS_REACHED,
    AUTOMATIC_WAIT_FOR_CONSTANT_SPEED,
    AUTOMATIC_WAITING_FOR_SAMPLES,     
    AUTOMATIC_SCANNING,     
    AUTOMATIC_FOUND,
    AUTOMATIC_SAVE_FIRST_MARK,
    AUTOMATIC_SAVE_SECOND_MARK,
    AUTOMATIC_SAVE_THIRD_MARK,
    AUTOMATIC_GOTO_CUT_POSITION,
    AUTOMATIC_WAIT_FOR_CUT_POSITION,
    AUTOMATIC_CUT_OPENING_SECTION,
    AUTOMATIC_CUT_RETURN_TO_MARK,
    AUTOMATIC_CUT_REST_SECTION,
    AUTOMATIC_CUT_RETURN_TO_MARK_2,
    AUTOMATIC_RETURN_TO_ZERO,     
    AUTOMATIC_FINISHED
} automatic_substate_t;

void activate_automatic_state();

void handle_automatic_state();

void handle_cutter_state();

