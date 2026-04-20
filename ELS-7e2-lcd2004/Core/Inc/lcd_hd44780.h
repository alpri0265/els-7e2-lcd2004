#ifndef LCD_HD44780_H
#define LCD_HD44780_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct
{
	GPIO_TypeDef *rs_port;
	uint16_t rs_pin;
	GPIO_TypeDef *e_port;
	uint16_t e_pin;

	GPIO_TypeDef *d4_port;
	uint16_t d4_pin;
	GPIO_TypeDef *d5_port;
	uint16_t d5_pin;
	GPIO_TypeDef *d6_port;
	uint16_t d6_pin;
	GPIO_TypeDef *d7_port;
	uint16_t d7_pin;
} lcd_hd44780_t;

void lcd_hd44780_init(lcd_hd44780_t *lcd);
void lcd_hd44780_clear(lcd_hd44780_t *lcd);
void lcd_hd44780_home(lcd_hd44780_t *lcd);
void lcd_hd44780_set_cursor(lcd_hd44780_t *lcd, uint8_t col, uint8_t row);
void lcd_hd44780_write_char(lcd_hd44780_t *lcd, char c);
void lcd_hd44780_write_str(lcd_hd44780_t *lcd, const char *s);
void lcd_hd44780_write_u8(lcd_hd44780_t *lcd, uint8_t b);
void lcd_hd44780_create_char(lcd_hd44780_t *lcd, uint8_t location, const uint8_t charmap[8]);

#endif
