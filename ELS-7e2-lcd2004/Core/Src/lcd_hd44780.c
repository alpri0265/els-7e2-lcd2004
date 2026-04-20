#include "lcd_hd44780.h"

static void lcd_delay_us(uint32_t us)
{
	/* DWT CYCCNT based delay (needs DWT enabled) */
	uint32_t start = DWT->CYCCNT;
	uint32_t cycles = (SystemCoreClock / 1000000U) * us;
	while ((DWT->CYCCNT - start) < cycles) {;}
}

static void lcd_enable_dwt_cycle_counter(void)
{
	/* Enable DWT CYCCNT for short delays (works on Cortex-M4) */
	if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
	{
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	}
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void lcd_set_data_nibble(lcd_hd44780_t *lcd, uint8_t nibble)
{
	HAL_GPIO_WritePin(lcd->d4_port, lcd->d4_pin, (nibble & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lcd->d5_port, lcd->d5_pin, (nibble & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lcd->d6_port, lcd->d6_pin, (nibble & 0x04U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lcd->d7_port, lcd->d7_pin, (nibble & 0x08U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void lcd_pulse_e(lcd_hd44780_t *lcd)
{
	HAL_GPIO_WritePin(lcd->e_port, lcd->e_pin, GPIO_PIN_SET);
	lcd_delay_us(1);
	HAL_GPIO_WritePin(lcd->e_port, lcd->e_pin, GPIO_PIN_RESET);
	lcd_delay_us(50);
}

static void lcd_write4(lcd_hd44780_t *lcd, uint8_t nibble)
{
	lcd_set_data_nibble(lcd, nibble & 0x0FU);
	lcd_pulse_e(lcd);
}

static void lcd_send(lcd_hd44780_t *lcd, uint8_t value, GPIO_PinState rs)
{
	HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, rs);
	lcd_write4(lcd, value >> 4);
	lcd_write4(lcd, value & 0x0F);
}

static void lcd_cmd(lcd_hd44780_t *lcd, uint8_t cmd)
{
	lcd_send(lcd, cmd, GPIO_PIN_RESET);
	if (cmd == 0x01U || cmd == 0x02U)
	{
		HAL_Delay(2);
	}
}

static void lcd_data(lcd_hd44780_t *lcd, uint8_t data)
{
	lcd_send(lcd, data, GPIO_PIN_SET);
}

void lcd_hd44780_init(lcd_hd44780_t *lcd)
{
	lcd_enable_dwt_cycle_counter();

	HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lcd->e_port, lcd->e_pin, GPIO_PIN_RESET);

	HAL_Delay(50);

	/* Init sequence for 4-bit mode */
	lcd_write4(lcd, 0x03);
	HAL_Delay(5);
	lcd_write4(lcd, 0x03);
	HAL_Delay(1);
	lcd_write4(lcd, 0x03);
	HAL_Delay(1);
	lcd_write4(lcd, 0x02);
	HAL_Delay(1);

	lcd_cmd(lcd, 0x28); /* 4-bit, 2-line, 5x8 */
	lcd_cmd(lcd, 0x0C); /* display on, cursor off */
	lcd_cmd(lcd, 0x06); /* entry mode */
	lcd_cmd(lcd, 0x01); /* clear */
}

void lcd_hd44780_clear(lcd_hd44780_t *lcd)
{
	lcd_cmd(lcd, 0x01);
}

void lcd_hd44780_home(lcd_hd44780_t *lcd)
{
	lcd_cmd(lcd, 0x02);
}

void lcd_hd44780_set_cursor(lcd_hd44780_t *lcd, uint8_t col, uint8_t row)
{
	static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
	if (row > 3) row = 3;
	lcd_cmd(lcd, (uint8_t)(0x80U | (row_offsets[row] + col)));
}

void lcd_hd44780_write_char(lcd_hd44780_t *lcd, char c)
{
	lcd_data(lcd, (uint8_t)c);
}

void lcd_hd44780_write_str(lcd_hd44780_t *lcd, const char *s)
{
	while (s && *s)
	{
		lcd_hd44780_write_char(lcd, *s++);
	}
}

void lcd_hd44780_write_u8(lcd_hd44780_t *lcd, uint8_t b)
{
	lcd_data(lcd, b);
}

void lcd_hd44780_create_char(lcd_hd44780_t *lcd, uint8_t location, const uint8_t charmap[8])
{
	location &= 0x07U;
	lcd_cmd(lcd, (uint8_t)(0x40U | (location << 3)));
	for (uint8_t i = 0; i < 8; i++)
	{
		lcd_data(lcd, charmap[i]);
	}
}

