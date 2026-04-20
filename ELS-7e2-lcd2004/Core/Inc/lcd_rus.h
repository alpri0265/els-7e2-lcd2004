#ifndef LCD_RUS_H
#define LCD_RUS_H

#include <stdint.h>
#include "lcd_hd44780.h"

typedef struct
{
	lcd_hd44780_t *lcd;
	uint8_t utf_hi_char; /* like LiquidCrystalRus: UTF-8 high part */
} lcd_rus_t;

void lcd_rus_init(lcd_rus_t *r, lcd_hd44780_t *lcd);
void lcd_rus_set_cursor(lcd_rus_t *r, uint8_t col, uint8_t row);
void lcd_rus_clear(lcd_rus_t *r);

void lcd_rus_write_utf8(lcd_rus_t *r, const char *s);
void lcd_rus_write_char(lcd_rus_t *r, char c);
void lcd_rus_write_u8(lcd_rus_t *r, uint8_t b);

#endif

