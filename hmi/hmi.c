#include <stdio.h>
#include <string.h>
#include "hmi.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "../core1_main.h"
#include "../lcd/ant_lcd.h"

#ifndef GIT_VERSION
#define GIT_VERSION "unknown"
#endif

#define BLINK_INTERVAL_US 500000ULL
#define WELCOME_INTERVAL_US 2000000ULL

static void place_string_to_left(char *line, const char *string, size_t line_length) {
    snprintf(line, line_length, "%-*s", (int)(line_length - 1), string);
}

static void place_string_centered(char *line, const char *string, size_t line_length) {
    size_t max_len = line_length - 1;
    size_t len = strlen(string);
    if (len >= max_len) {
        snprintf(line, line_length, "%s", string);
        return;
    }

    size_t left_pad = (max_len - len) / 2;
    size_t right_pad = max_len - len - left_pad;
    snprintf(line, line_length, "%*s%s%*s", (int)left_pad, "", string, (int)right_pad, "");
}

static void place_string_to_right(char *line, const char *string, size_t line_length) {
    snprintf(line, line_length, "%*s", (int)(line_length - 1), string);
}

static void place_float_centered(char *line, float value, int decimals, size_t line_length) {
    char value_text[LINE_LENGTH_FULL];
    snprintf(value_text, sizeof(value_text), "%.*f", decimals, value);
    place_string_centered(line, value_text, line_length);
}

static void place_float_with_unit_to_right(char *line, float value, int decimals, const char *unit, size_t line_length) {
    char value_text[LINE_LENGTH_FULL];
    snprintf(value_text, sizeof(value_text), "%.*f%s", decimals, value, unit);
    snprintf(line, line_length, "%*s", (int)(line_length - 1), value_text);
}

static void place_int_centered(char *line, int value, size_t line_length) {
    char value_text[LINE_LENGTH_FULL];
    snprintf(value_text, sizeof(value_text), "%d", value);
    place_string_centered(line, value_text, line_length);
}

static void place_int_to_right(char *line, int value, size_t line_length) {
    snprintf(line, line_length, "%*d", (int)(line_length - 1), value);
}

static void clear_line(lcd_line_t line) {
    memset(line, ' ', LINE_LENGTH_FULL - 1);
    line[LINE_LENGTH_FULL - 1] = '\0';
}

static void clear_screen(screen_buffer_t *screen) {
    for (size_t i = 0; i < LINE_COUNT; i++) {
        clear_line(screen->line[i]);
    }
}

static void merge_halfs_lines(lcd_line_t line, const char *half_first, const char *half_second) {
    snprintf(line, LINE_LENGTH_FULL, "%-10.10s%-10.10s", half_first, half_second);
}

static void render_standard(screen_buffer_t *screen, const machine_display_snapshot_t *state) {
    lcd_halfline_t left_top = {0};
    lcd_halfline_t right_top = {0};
    lcd_halfline_t left_bottom = {0};
    lcd_halfline_t right_bottom = {0};
    char number_0[LINE_LENGTH_HALF];
    char number_1[LINE_LENGTH_HALF];
    const char *status = "";

    switch (state->state) {
        case MANUAL:
            status = state->homed ? "Manual" : "Manual - NO Home";
            place_string_to_left(left_top, "Motory", LINE_LENGTH_HALF);
            place_string_to_left(left_bottom, state->enabled ? "vypnut" : "zapnut", LINE_LENGTH_HALF);
            if (state->manual_substate == MANUAL_READY && state->homed) {
                if (state->paper_positions_set) {
                    place_string_to_left(right_top, "", LINE_LENGTH_HALF);
                    place_string_to_right(right_bottom, "Automat", LINE_LENGTH_HALF);
                } else {
                    place_string_to_left(right_top, "Nastav", LINE_LENGTH_HALF);
                    place_string_to_right(right_bottom, "parametre", LINE_LENGTH_HALF);
                }
            } else {
                place_string_to_left(right_top, "", LINE_LENGTH_HALF);
                place_string_to_right(right_bottom, "", LINE_LENGTH_HALF);
            }
            break;
        case HOMING:
            status = "HOMING";
            place_string_to_left(left_top, "", LINE_LENGTH_HALF);
            place_string_to_left(left_bottom, "Stop", LINE_LENGTH_HALF);
            place_string_to_left(right_top, "", LINE_LENGTH_HALF);
            place_string_to_right(right_bottom,
                state->homing_substate == HOMING_SCANNING ? "Hlada sa->" : "", LINE_LENGTH_HALF);
            break;
        case PARAMS:
            place_string_to_left(left_top, "", LINE_LENGTH_HALF);
            place_string_to_left(left_bottom, "Exit", LINE_LENGTH_HALF);
            if (state->param_substate == PARAM_START) {
                if (state->feeder_zeroed) {
                    place_string_to_left(right_top, "", LINE_LENGTH_HALF);
                    place_string_to_right(right_bottom, "Pokracuj", LINE_LENGTH_HALF);
                } else {
                    place_string_to_left(right_top, "Vynuluj", LINE_LENGTH_HALF);
                    place_string_to_right(right_bottom, "feeder", LINE_LENGTH_HALF);
                }
            } else {
                place_string_to_left(right_top, "Zapnut", LINE_LENGTH_HALF);
                place_string_to_right(right_bottom, "Automat", LINE_LENGTH_HALF);
            }
            break;
        case AUTOMAT:
            status = "Automat";
            place_string_to_left(left_top, "", LINE_LENGTH_HALF);
            place_string_to_left(left_bottom, "Stop", LINE_LENGTH_HALF);
            if (state->automatic_substate == IDLE) {
                place_string_to_left(right_top, "", LINE_LENGTH_HALF);
                place_string_to_right(right_bottom, "Start", LINE_LENGTH_HALF);
            } else if (state->automatic_substate == COMPLETE) {
                place_string_to_left(right_top, "", LINE_LENGTH_HALF);
                place_string_to_right(right_bottom, "Hotovo!", LINE_LENGTH_HALF);
            } else {
                place_int_to_right(right_top, (int)state->rows_to_cut, LINE_LENGTH_HALF);
                place_int_to_right(right_bottom, (int)state->rows_cut_performed, LINE_LENGTH_HALF);
            }
            break;
        case FAILURE:
            break;
    }

    place_string_to_left(screen->line[0], status, LINE_LENGTH_FULL);
    place_float_with_unit_to_right(number_0, state->pos_cutter, 2, "mm", LINE_LENGTH_HALF);
    place_float_with_unit_to_right(number_1, state->pos_feeder - state->feeder_offset, 2, "mm", LINE_LENGTH_HALF);
    merge_halfs_lines(screen->line[1], number_0, number_1);
    merge_halfs_lines(screen->line[2], left_top, right_top);
    merge_halfs_lines(screen->line[3], left_bottom, right_bottom);
}

static void render_input(screen_buffer_t *screen, const machine_display_snapshot_t *state) {
    const char *question;
    const char *left_top = "";
    const char *left_bottom = "Exit";
    const char *right_top = "";
    const char *right_bottom;

    switch (state->param_substate) {
        case PARAM_STICKER_HEIGHT:
            question = "Nastav vysku nalepky:";
            right_bottom = "Uloz vysku";
            place_float_centered(screen->line[1], state->encoder_value, 2, LINE_LENGTH_FULL);
            break;
        case PARAM_ROWS_TO_CUT:
            question = "Nastav pocet rezov:";
            right_top = "Uloz";
            right_bottom = "pocet";
            place_int_centered(screen->line[1], state->encoder_value_int, LINE_LENGTH_FULL);
            break;
        case PARAM_PAPER_WIDTH:
            question = "Nastav sirku papiera:";
            right_top = "Uloz";
            right_bottom = "sirku";
            place_float_centered(screen->line[1], state->encoder_value, 2, LINE_LENGTH_FULL);
            break;
        default:
            question = "";
            right_bottom = "";
            clear_line(screen->line[1]);
            break;
    }

    place_string_to_left(screen->line[0], question, LINE_LENGTH_FULL);
    lcd_halfline_t left;
    lcd_halfline_t right;
    place_string_to_left(left, left_top, LINE_LENGTH_HALF);
    place_string_to_right(right, right_top, LINE_LENGTH_HALF);
    merge_halfs_lines(screen->line[2], left, right);
    place_string_to_left(left, left_bottom, LINE_LENGTH_HALF);
    place_string_to_right(right, right_bottom, LINE_LENGTH_HALF);
    merge_halfs_lines(screen->line[3], left, right);
}

static screen_state_t screen_for_state(const machine_display_snapshot_t *state, bool welcome) {
    if (state->state == FAILURE) {
        return FAILURE_SCREEN;
    }
    if (welcome) {
        return WELCOME_SCREEN;
    }
    if (state->state == PARAMS && state->param_substate != PARAM_START && state->param_substate != PARAM_FINSIHED) {
        return INPUT_SCREEN;
    }
    return STANDARD_SCREEN;
}

static void render_frame(hmi_t *hmi, const machine_display_snapshot_t *state) {
    clear_screen(&hmi->actual_screen_buffer);
    bool welcome = time_us_64() - hmi->start_us <= WELCOME_INTERVAL_US;
    screen_state_t next_screen = screen_for_state(state, welcome);

    if (next_screen != hmi->screen_state) {
        hmi->screen_state = next_screen;
        lcd_reinitialize(hmi->lcd);
        clear_screen(&hmi->displayed_screen_buffer);
    }

    switch (hmi->screen_state) {
        case WELCOME_SCREEN:
            hmi->actual_screen_buffer = hmi->welcome_screen_buffer;
            break;
        case STANDARD_SCREEN:
            render_standard(&hmi->actual_screen_buffer, state);
            break;
        case INPUT_SCREEN:
            render_input(&hmi->actual_screen_buffer, state);
            break;
        case FAILURE_SCREEN: {
            place_string_centered(hmi->actual_screen_buffer.line[0], "* PORUCHA! *", LINE_LENGTH_FULL);
            place_string_centered(hmi->actual_screen_buffer.line[2], state->error_message, LINE_LENGTH_FULL);
            place_string_to_left(hmi->actual_screen_buffer.line[3], "Potvrdit", LINE_LENGTH_FULL);
            bool blink_off = ((time_us_64() - hmi->start_us) / BLINK_INTERVAL_US) % 2;
            if (blink_off) {
                clear_line(hmi->actual_screen_buffer.line[0]);
            }
            break;
        }
    }
}

static void draw_changed_characters(hmi_t *hmi) {
    for (uint8_t row = 0; row < LINE_COUNT; row++) {
        uint8_t col = 0;
        while (col < LINE_LENGTH_FULL - 1) {
            while (col < LINE_LENGTH_FULL - 1 &&
                   hmi->actual_screen_buffer.line[row][col] == hmi->displayed_screen_buffer.line[row][col]) {
                col++;
            }
            if (col == LINE_LENGTH_FULL - 1) {
                break;
            }

            uint8_t start = col;
            char changed[LINE_LENGTH_FULL];
            size_t length = 0;
            while (col < LINE_LENGTH_FULL - 1 &&
                   hmi->actual_screen_buffer.line[row][col] != hmi->displayed_screen_buffer.line[row][col]) {
                changed[length++] = hmi->actual_screen_buffer.line[row][col];
                hmi->displayed_screen_buffer.line[row][col] = hmi->actual_screen_buffer.line[row][col];
                col++;
            }
            changed[length] = '\0';
            string2LCD(hmi->lcd, start, row, changed);
        }
    }
}

void hmi_init(hmi_t* hmi, const lcd_config_t* config) {
    hard_assert(hmi != NULL);
    memset(hmi, 0, sizeof(*hmi));
    clear_screen(&hmi->welcome_screen_buffer);
    place_string_centered(hmi->welcome_screen_buffer.line[1], "Sticker Cutter", LINE_LENGTH_FULL);
    place_string_to_right(hmi->welcome_screen_buffer.line[3], GIT_VERSION, LINE_LENGTH_FULL);
    hmi->lcd = lcd_create(config);
    hmi->start_us = time_us_64();
    hmi->screen_state = WELCOME_SCREEN;
    clear_screen(&hmi->displayed_screen_buffer);
}

void hmi_compute(hmi_t* hmi) {
    machine_display_snapshot_t state;
    machine_get_display_snapshot(&state);
    render_frame(hmi, &state);
    draw_changed_characters(hmi);
}
