#define _GNU_SOURCE
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "ant_lcd.h"
#include "pico/stdlib.h"

#define LCD_CLR 0x01        // Clear entire display and reset cursor position
#define LCD_DISP_ON 0x0C    // Turn display on without cursor
#define LCD_DISP_OFF 0x08   // Turn display off
#define LCD_BLINK_ON 0x0D   // Enable cursor blinking
#define LCD_BLINK_OFF 0x0C  // Disable cursor blinking
#define LCD_CURSOR_ON 0x0E  // Show cursor
#define LCD_CURSOR_OFF 0x0C // Hide cursor
#define LCD_2LINE 0x08      // Enable multi-line mode (>2 lines)
#define ENABLE_PULSE_DELAY_US 1
#define COMMAND_DELAY_US 50

struct lcd_controller {
    uint8_t data[4];     // D4-D7 data pins
    uint8_t RS;          // Register Select pin
    uint8_t RW;          // Read/Write pin
    uint8_t EN;          // Enable pin
    uint8_t COL;         // Number of columns
    uint8_t ROW;         // Number of rows
    uint8_t Xcurrent;    // Current X position (column)
    uint8_t Ycurrent;    // Current Y position (row)
};

void e_blink(lcd_t* lcd) {
	gpio_put(lcd->EN, 1);
	busy_wait_us(ENABLE_PULSE_DELAY_US);
	gpio_put(lcd->EN, 0);
	busy_wait_us(ENABLE_PULSE_DELAY_US);
}

void command4bit(lcd_t* lcd, uint8_t cmd) {
	gpio_put(lcd->data[0], (cmd & 0x01) >> 0);
	gpio_put(lcd->data[1], (cmd & 0x02) >> 1);
	gpio_put(lcd->data[2], (cmd & 0x04) >> 2);
	gpio_put(lcd->data[3], (cmd & 0x08) >> 3);
	e_blink(lcd);
}

void command(lcd_t* lcd, uint8_t cmd) {
	gpio_put(lcd->RS, 0);
	command4bit(lcd, cmd >> 4);
	command4bit(lcd, cmd & 0x0F);
	busy_wait_us(COMMAND_DELAY_US);
}

void gotoxy(lcd_t* lcd, uint8_t x, uint8_t y) {
	uint8_t row_addr[4] = {0x00, 0x40, 0x14, 0x54};
	if (y >= lcd->ROW)
	{
		y = 0;
	}
	lcd->Xcurrent = x;
	lcd->Ycurrent = y;

	command(lcd, 0x80 | (x + row_addr[y]));
}

void clrscr(lcd_t* lcd) {
	command(lcd, LCD_CLR);
	busy_wait_ms(2);
}

static void lcd_initialize(lcd_t* lcd, bool power_on) {
	uint8_t FuncCnTrL = 0;
	if (power_on) {
		busy_wait_ms(40);
	}

	// Changing Address if --> LCD Rows>2 Line
	if (lcd->ROW > 1)
	{
		FuncCnTrL |= LCD_2LINE;
	}

	// Recovery may follow a character write, which leaves RS in data mode.
	gpio_put(lcd->RS, 0);
	command4bit(lcd, 0x03);
	busy_wait_ms(5);
	command4bit(lcd, 0x03);
	busy_wait_us(150);
	command4bit(lcd, 0x03);
	busy_wait_us(150);
	command4bit(lcd, 0x02);
	busy_wait_us(150);

	// Turns Displays on - No Cursor - No Blinking - Position 0,0 - Default Font
	command(lcd, 0x20 | FuncCnTrL);
	command(lcd, 0x08 | 0x04);
	clrscr(lcd);
	command(lcd, 0x04 | 0x02);
	busy_wait_us(COMMAND_DELAY_US);
}

void lcd_reinitialize(lcd_t* lcd) {
	lcd_initialize(lcd, false);
}

lcd_t* lcd_create(const lcd_config_t *config) {
	// Create lcd data structure
	lcd_t* lcd = calloc(1, sizeof(struct lcd_controller));
	hard_assert(lcd != NULL);

	lcd->data[0] = config->d4;
	lcd->data[1] = config->d5;
	lcd->data[2] = config->d6;
	lcd->data[3] = config->d7;
	lcd->RS = config->rs;
	lcd->RW = config->rw;
	lcd->EN = config->en;

	gpio_init(lcd->data[0]);
    gpio_set_dir(lcd->data[0], GPIO_OUT);
	gpio_put(lcd->data[0], 0);
	gpio_init(lcd->data[1]);
    gpio_set_dir(lcd->data[1], GPIO_OUT);
	gpio_put(lcd->data[1], 0);
	gpio_init(lcd->data[2]);
    gpio_set_dir(lcd->data[2], GPIO_OUT);
	gpio_put(lcd->data[2], 0);
	gpio_init(lcd->data[3]);
    gpio_set_dir(lcd->data[3], GPIO_OUT);
	gpio_put(lcd->data[3], 0);
	gpio_init(lcd->RS);
    gpio_set_dir(lcd->RS, GPIO_OUT);
	gpio_put(lcd->RS, 0);
	gpio_init(lcd->RW);
    gpio_set_dir(lcd->RW, GPIO_OUT);
	gpio_put(lcd->RW, 0);
	gpio_init(lcd->EN);
    gpio_set_dir(lcd->EN, GPIO_OUT);
	gpio_put(lcd->EN, 0);
	
	lcd->COL = config->cols;
	lcd->ROW = config->rows;

	lcd->Xcurrent = 0;
	lcd->Ycurrent = 0;

	lcd_initialize(lcd, true);

	return lcd;
}

void write_data(lcd_t* lcd, uint8_t data) {
	gpio_put(lcd->RS, 1);
	command4bit(lcd, data >> 4);
	command4bit(lcd, data & 0x0F);
	busy_wait_us(COMMAND_DELAY_US);
}


void writeText(lcd_t* lcd, const char string[]) {
	uint8_t i = 0;
	while (string[i] != '\0')
	{
		write_data(lcd, string[i]);
		i++;
	}
}

void int2LCD(lcd_t* lcd, uint8_t x, uint8_t y, uint8_t max_length, int number) {
    gotoxy(lcd, x, y); 
    char* str;
    asprintf (&str, "%*i", max_length, number);
    writeText(lcd, str);
    free(str);
}

void float2LCD(lcd_t* lcd, uint8_t x, uint8_t y, uint8_t max_length, uint8_t decimals, float number) {
    gotoxy(lcd, x, y); 
    char* str;
    asprintf(&str, "%*.*f", max_length, decimals, number);
    writeText(lcd, str);
    free(str);
}

void string2LCD(lcd_t* lcd, uint8_t x, uint8_t y, const char string[]) {
    gotoxy(lcd, x, y); 
    writeText(lcd, string);
}
