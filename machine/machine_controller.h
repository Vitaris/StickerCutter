#ifndef MACHINE_H
#define MACHINE_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "mark_detector.h"
#include "../servo_motor/button.h"
#include "../servo_motor/servo_motor.h"
#include "../lcd/ant_lcd.h"

// Physical constants
#define SENSOR_KNIFE_OFFSET_X 25.0f
#define SENSOR_KNIFE_OFFSET_Y 14.0f
#define FAR_AWAY_DISTANCE 1000.0f
#define POSITION_EDGE_RIGHT -45.0f
#define POSITION_EDGE_LEFT -1480.0f

// Speed constants
#define MANUAL_SPEED_SLOW 20.0f
#define MANUAL_SPEED_NORMAL 100.0f
#define MANUAL_SPEED_FAST 200.0f
#define AUTOMAT_SPEED_SCAN 15.0f
#define AUTOMAT_SPEED_SLOW 50.0f
#define AUTOMAT_SPEED_MID 100.0f
#define AUTOMAT_SPEED_NORMAL 200.0f
#define AUTOMAT_SPEED_FAST 250.0f
#define AUTOMAT_SPEED_CUT 180.0f

#define HALF_SECOND_DELAY 500

typedef enum {
	MANUAL,
	HOMING,
	PARAMS,
	AUTOMAT, 
	FAILURE
} machine_state_t;

extern machine_state_t machine_state;

typedef struct {
	servo_t* servo_cutter;
	servo_t* servo_feeder;

	button_t* F1;
	button_t* F2;
	button_t* Right;
	button_t* Left;
	button_t* In;
	button_t* Out;
	lcd_t* lcd;
} devices_t;

extern devices_t devices;

typedef struct {
	bool enable;
	bool homed;
	bool machine_error;
	char error_message[21];

	// Cutter
	bool params_ready;
	float paper_right_mark_position;

	// LCD Texts
	char state_text_1[21];
	char state_text_2[21];
	char condition_text[10];
	char position_cutter[8];
	char position_feeder[8];
	char F1_text[11];
	char F2_text[11];
} machine_t;

extern machine_t machine;

/**
 * @brief Initializes the machine controller
 */
void machine_init(void);

/**
 * @brief Main state machine computation function
 */
void machine_compute(void);

/**
 * @brief Activates the failure state of the machine
 * 
 * Transitions the machine into a failure state, which indicates
 * that an error or malfunction has occurred. This state typically
 * requires operator intervention to resolve.
 * 
 * @note Once activated, the machine will remain in failure state
 *       until explicitly reset by authorized personnel.
 */
void activate_failure_state(void);

/**
 * @brief Handles failure state
 */
void handle_failure_state(void);

/**
 * @brief Raises cutting knife
 */
void knife_up(void);

/**
 * @brief Lowers cutting knife
 */
void knife_down(void);

/**
 * @brief Sets error state with message
 * @param text Error message text
 */
void raise_error(char text[]);

/**
 * @brief Gets the current error message
 * @return Pointer to the error message text
 */
char* get_error_message(void);

/**
 * @brief Sets the text at index 10 in the LCD text array
 * 
 * @param LCD_text Array of strings storing the LCD text content
 * @param text The text to be set at index 10
 */
void set_text_10(char LCD_text[], char text[]);

/**
 * @brief Sets a text string in a 20-character LCD display position
 * 
 * @param LCD_text Array of character pointers representing LCD display text buffer
 * @param text The text string to be set (up to 20 characters)
 */
void set_text_20(char LCD_text[], char text[]);


#endif
