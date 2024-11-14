#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include <string.h>

#include "machine_controller.h"
#include "../servo_motor/servo_motor.h"
#include "../servo_motor/button.h"
#include "mark_detector.h"

bool end = false;
float pause_time;
bool waiting;

machine_t create_machine()
{
    // Create machine data structure
    machine_t machine = (machine_t)malloc(sizeof(struct machine));
    machine->state = MANUAL;
    machine->machine_condition = OK;

    // Init buttons
    machine->F1 = create_button(5);
    machine->F2 = create_button(2);
    machine->Right = create_button(1);
    machine->Left = create_button(3);
    machine->In = create_button(4);
    machine->Out = create_button(0);

    // Init servos
    machine->machine_error = false;
    machine->enable = true;
    machine->homed = false;
    set_text_20(machine->error_message, "OK");

    // Init PIO
    uint offset = pio_add_program(pio0, &quadrature_encoder_program);

    machine->servo_0 = servo_create("Cutter", offset, 0, ENC_0, PWM_0, -20.0, &machine->Right, &machine->Left, &machine->enable, &machine->machine_error, &machine->error_message);
    machine->servo_1 = servo_create("Feeder", offset, 1, ENC_1, PWM_1, 6.4, &machine->Out, &machine->In, &machine->enable, &machine->machine_error, &machine->error_message);

    // Knife
    gpio_init(KNIFE_OUTPUT_PIN);
    gpio_set_dir(KNIFE_OUTPUT_PIN, GPIO_OUT);

    // Cutter
    machine->cutter_state = CUTTER_IDLE;

    // Mark probe
    machine->detector = create_detector(0, &machine->servo_1->servo_position);
    return machine;
}

void machine_compute(machine_t machine)
{
    servo_compute(machine->servo_0);
    servo_compute(machine->servo_1);

    button_compute(machine->F1);
    button_compute(machine->F2);
    button_compute(machine->Right);
    button_compute(machine->Left);
    button_compute(machine->In);
    button_compute(machine->Out);

    detector_compute(machine->detector);

    // State machine
    switch(machine->state) {
        case MANUAL:

            // Motory on/off
            if (machine->enable == false) {
                set_text_20(machine->state_text, "Manual, Volne motory");
                set_text_10(machine->F1_text, "Mot->ON");

                if (machine->F1->state_raised == true) {
                    machine->enable = true;
                }
            }
            else {
                set_text_20(machine->state_text, "Manual");
                set_text_10(machine->F1_text, "Mot->OFF");

                servo_manual_handling(machine->servo_0);
                servo_manual_handling(machine->servo_1);

                if (machine->F1->state_raised == true) {
                    machine->enable = false;
                }
            }
            
            // Home / Start cutting
            if (machine->homed == true) {
                set_text_10(machine->F2_text, "Start");
                if (machine->F2->state_raised == true) {
                    stop_positioning(machine->servo_0);
                    stop_positioning(machine->servo_1);
                    knife_up();
                    machine->state = AUTOMAT;
                }
            }
            else { 
                set_text_10(machine->F2_text, "Noz->0");
                if (machine->F2->state_raised == true) {
                    machine->servo_0->set_zero = true;
                    machine->cutter_state = CUTTER_IDLE;
                    machine->homed = true;
                }
            }

            break;

        case AUTOMAT:
            set_text_20(machine->state_text, "Automat");
            set_text_10(machine->F1_text, "Stop");

            if (machine->F1->state_raised) {
                machine->state = MANUAL;
                machine->cutter_state = STOP_CUTTING;
            }

            if (machine->detector->detector_state == DETECTOR_UNCALIBRATED) {
                set_text_10(machine->F2_text, "Kalibrovat");
                if (machine->F2->state_raised) {
                    machine->detector->detector_state = DETECTOR_CALIBRATION;
                }
            } 
            else if (machine->detector->detector_state == DETECTOR_CALIBRATION) {
                set_text_10(machine->F2_text, "Kalibruje");
                if (machine->servo_1->positioning == IDLE) {
                    servo_goto_delayed(machine->servo_1, machine->servo_1->current_pos + 5.0, 2.5, 2000);
                }
                else if (machine->servo_1->positioning == POSITION_REACHED) {
                    machine->detector->detector_state = DETECTOR_READY;
                }
            } 
            else if (machine->detector->detector_state == DETECTOR_READY) {
                set_text_10(machine->F2_text, "");

                if (machine->cutter_state == CUTTER_IDLE) {
                    machine->cutter_state = CUTTER_REQUESTED;
                }

                if (machine->F1->state_raised) {
                    machine->state = MANUAL;
                    machine->cutter_state = STOP_CUTTING;
                }
            }
            break;

        case FAILURE:
            set_text_20(machine->state_text, "Porucha!");
            machine->enable = false;
            set_text_10(machine->F1_text, "Potvrdit");
            set_text_10(machine->F2_text, "");

            if (machine->F1->state_raised) {
                machine->state = MANUAL;
                set_text_20(machine->error_message, "OK");
                machine->machine_error = false;
            }
            break;
    }

    if (machine->machine_error) {
        machine->state = FAILURE;
    }
    // if (machine->state != AUTOMAT) {
    //     machine->cutter_state = CUTTER_IDLE;
    // }
    sticker_cut_compute(machine);
}

void sticker_cut_compute(machine_t machine) {
    switch(machine->cutter_state) {
        case CUTTER_IDLE:
            knife_up();
            break;

        case CUTTER_REQUESTED:
            if (machine->servo_0->current_pos == 0.0) {
                machine->cutter_state = AT_HOME;
            } else {
                machine->cutter_state = TO_HOME;
            }
            break;

        case TO_HOME:
            if (machine->servo_0->positioning == IDLE) {
                knife_up();
                servo_goto_delayed(machine->servo_0, 0.0, 10.0, 500);
            } 
            else if (machine->servo_0->positioning == POSITION_REACHED) {
                    machine->cutter_state = AT_HOME;
            }
            break;

        case AT_HOME:
            // Check if knife is above the mark
            if (get_next_stop(machine->detector, machine->servo_1->current_pos) != machine->servo_1->current_pos) {
                raise_error(machine, "Znacka nenajdena");
            }
            else {
                machine->cutter_state = TO_PRECUT;
            }
            break;

        case TO_PRECUT:
            if (machine->servo_0->positioning == IDLE) {
                knife_up();
                servo_goto_delayed(machine->servo_0, PRECUT_POSITION, 4.0, 500);
            } else {
                if (machine->servo_0->positioning == POSITION_REACHED) {
                    machine->cutter_state = BACK_HOME;
                }
            }
            break;

        case BACK_HOME:
            if (machine->servo_0->positioning == IDLE) {
                knife_down();
                servo_goto_delayed(machine->servo_0, 0.0, 10.0, 500);
            } else {
                if (machine->servo_0->positioning == POSITION_REACHED) {
                    machine->cutter_state = CUT_TO_END;
                }
            }
            break;

        case CUT_TO_END:
            if (machine->servo_0->positioning == IDLE) {
                servo_goto_delayed(machine->servo_0, CUT_LENGTH, 4.0, 500);
            } else {
                if (machine->servo_0->positioning == POSITION_REACHED) {
                    machine->cutter_state = FINAL_RETURN;
                }
            }
            break;

        case FINAL_RETURN:
            if (machine->servo_0->positioning == IDLE) {
                servo_goto_delayed(machine->servo_0, 0.0, 4.0, 500);
                knife_up();
            } else {
                if (machine->servo_0->positioning == POSITION_REACHED) {
                    machine->cutter_state = CUT_DONE;
                }
            }
            break;

        case CUT_DONE:
            machine->cutter_state = ROLL_OUT_PAPER;
            break; 
        
        case ROLL_OUT_PAPER:
            if (machine->servo_0->positioning == IDLE &&  machine->servo_1->positioning == IDLE) {
                servo_goto_delayed(machine->servo_1, machine->servo_1->current_pos + 5.0, 5.0, 500);
            }
            else if (machine->servo_1->positioning == POSITION_REACHED) {
                machine->cutter_state = CUTTER_IDLE;
            }
            break;

        case STOP_CUTTING:
            machine->servo_0->positioning = IDLE;
            machine->cutter_state = CUTTER_IDLE;
            break;
    }
}

void feeder_compute(machine_t machine) {
    
}

void set_text(char LCD_text[], char text[], uint8_t len) {
    bool fill_spaces = false;
    uint8_t i = 0;
	while (i < len)	{
        if (text[i] == '\0') {
            fill_spaces = true;
        }
        if (fill_spaces == false) {
            LCD_text[i] = text[i];
        } else {
            LCD_text[i] = ' ';
        }
		i++;
	}
    LCD_text[len] = '\0';
}

void set_text_10(char LCD_text[], char text[]) {
    set_text(LCD_text, text, 10);
}

void set_text_20(char LCD_text[], char text[]) {
    set_text(LCD_text, text, 20);
}

void set_pause() {
    waiting = true;
    pause_time = 0.0;
}

bool is_time(float cycle_time) {
    pause_time += cycle_time;
    if (pause_time >= 1.0) {
        waiting = false;
        return true;
    } else {
        return false;
    }
}

void perform_sticker_cut(machine_t machine) {
}

void knife_up() {
    gpio_put(KNIFE_OUTPUT_PIN, false);
}

void knife_down() {
    gpio_put(KNIFE_OUTPUT_PIN, true);
}

void raise_error(machine_t machine, char text[]) {
    set_text_20(machine->error_message, text);
    machine->machine_error = true;
}
