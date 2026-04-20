#include "lcd_rus.h"

/*
 * Ported UTF-8 -> HD44780 (A02-like) recoding from LiquidCrystalRus.
 * It maps Cyrillic UTF-8 to LCD codepage used in the original Arduino project.
 */

static const uint8_t utf_recode[] =
{
	0x41,0xA0,0x42,0xA1,0xE0,0x45,0xA3,0xA4,0xA5,0xA6,0x4B,0xA7,0x4D,0x48,0x4F,
	0xA8,0x50,0x43,0x54,0xA9,0xAA,0x58,0xE1,0xAB,0xAC,0xE2,0xAD,0xAE,0x62,0xAF,0xB0,0xB1,
	0x61,0xB2,0xB3,0xB4,0xE3,0x65,0xB6,0xB7,0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0x6F,
	0xBE,0x70,0x63,0xBF,0x79,0xE4,0x78,0xE5,0xC0,0xC1,0xE6,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7
};

void lcd_rus_init(lcd_rus_t *r, lcd_hd44780_t *lcd)
{
	r->lcd = lcd;
	r->utf_hi_char = 0;
}

void lcd_rus_set_cursor(lcd_rus_t *r, uint8_t col, uint8_t row)
{
	lcd_hd44780_set_cursor(r->lcd, col, row);
}

void lcd_rus_clear(lcd_rus_t *r)
{
	lcd_hd44780_clear(r->lcd);
}

void lcd_rus_write_u8(lcd_rus_t *r, uint8_t b)
{
	lcd_hd44780_write_u8(r->lcd, b);
}

void lcd_rus_write_char(lcd_rus_t *r, char c)
{
	lcd_hd44780_write_char(r->lcd, c);
}

static void lcd_rus_write_byte(lcd_rus_t *r, uint8_t value)
{
	if (value < 0x80)
	{
		lcd_hd44780_write_u8(r->lcd, value);
		return;
	}

	/* UTF-8 handling: follow LiquidCrystalRus logic */
	if (value >= 0xC0)
	{
		r->utf_hi_char = (uint8_t)(value - 0xD0);
		return;
	}

	value &= 0x3F;
	if (r->utf_hi_char == 0 && value == 1)
	{
		/* Ё */
		lcd_hd44780_write_u8(r->lcd, 0xA2);
	}
	else if (r->utf_hi_char == 1 && value == 0x11)
	{
		/* ё */
		lcd_hd44780_write_u8(r->lcd, 0xB5);
	}
	else
	{
		uint16_t idx = (uint16_t)value + ((uint16_t)r->utf_hi_char << 6) - 0x10U;
		if (idx < (uint16_t)sizeof(utf_recode))
		{
			lcd_hd44780_write_u8(r->lcd, utf_recode[idx]);
		}
		else
		{
			lcd_hd44780_write_u8(r->lcd, '?');
		}
	}
}

void lcd_rus_write_utf8(lcd_rus_t *r, const char *s)
{
	while (s && *s)
	{
		lcd_rus_write_byte(r, (uint8_t)*s++);
	}
}

