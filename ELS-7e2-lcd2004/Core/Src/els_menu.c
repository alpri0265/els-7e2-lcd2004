#include "els_menu.h"

#include <stdio.h>
#include <string.h>

/* Custom chars from Arduino sketch (indexes 1..6) */
static const uint8_t chr_left[8]   = {0x00,0x04,0x08,0x1F,0x08,0x04,0x00,0x00};
static const uint8_t chr_right[8]  = {0x00,0x04,0x02,0x1F,0x02,0x04,0x00,0x00};
static const uint8_t chr_up[8]     = {0x00,0x04,0x0E,0x15,0x04,0x04,0x00,0x00};
static const uint8_t chr_down[8]   = {0x00,0x04,0x04,0x15,0x0E,0x04,0x00,0x00};
static const uint8_t chr_degree[8] = {0x0C,0x12,0x12,0x0C,0x00,0x00,0x00,0x00};
static const uint8_t chr_diam[8]   = {0x01,0x0E,0x13,0x15,0x19,0x0E,0x10,0x00};

static void lcd_write_padded(lcd_rus_t *r, const char *s)
{
	/* write exactly 20 cols: string then spaces */
	size_t len = s ? strlen(s) : 0;
	for (size_t i = 0; i < 20; i++)
	{
		if (s && i < len) lcd_rus_write_char(r, s[i]);
		else lcd_rus_write_char(r, ' ');
	}
}

static void lcd_put_custom(lcd_rus_t *r, uint8_t idx)
{
	lcd_rus_write_u8(r, idx);
}

static void els_beep_stub(void)
{
	/* TODO: wire to BEEPER pin later */
}

static void key_select_pressed(els_menu_t *m)
{
	switch (m->mode)
	{
		case ELS_MODE_FEED:
			m->select_menu = (m->select_menu == 0) ? 1 : 0;
			els_beep_stub();
			break;
		case ELS_MODE_AFEED:
		case ELS_MODE_CONE_L:
		case ELS_MODE_CONE_R:
			m->select_menu = (uint8_t)((m->select_menu + 1) % 3);
			els_beep_stub();
			break;
		case ELS_MODE_THREAD:
		case ELS_MODE_SPHERE:
			m->select_menu = (uint8_t)((m->select_menu + 1) % 3);
			els_beep_stub();
			break;
		case ELS_MODE_TACHO:
			m->select_menu = (m->select_menu == 0) ? 1 : 0;
			els_beep_stub();
			break;
		case ELS_MODE_RESERVE:
		default:
			break;
	}
}

static void key_up_pressed(els_menu_t *m)
{
	switch (m->mode)
	{
		case ELS_MODE_FEED:
		case ELS_MODE_CONE_L:
		case ELS_MODE_CONE_R:
			if (m->select_menu == 0)
			{
				if (m->ap < 500) { m->ap += 10; els_beep_stub(); }
				else if (m->ap < 900) { m->ap += 50; els_beep_stub(); }
			}
			else if (m->select_menu == 1)
			{
				m->x_pos = 0;
				els_beep_stub();
			}
			break;

		case ELS_MODE_AFEED:
			if (m->select_menu == 0)
			{
				if (m->ap < 10) { m->ap += 5; els_beep_stub(); }
				else if (m->ap < 500) { m->ap += 10; els_beep_stub(); }
				else if (m->ap < 900) { m->ap += 50; els_beep_stub(); }
			}
			else if (m->select_menu == 1)
			{
				if (m->total_tooth < 255) { m->total_tooth++; m->current_tooth = 1; els_beep_stub(); }
			}
			else if (m->select_menu == 2)
			{
				m->x_pos = 0;
				els_beep_stub();
			}
			break;

		case ELS_MODE_THREAD:
			if (m->select_menu == 0)
			{
				if (m->thread_step < 10) { m->thread_step++; els_beep_stub(); }
			}
			else if (m->select_menu == 2)
			{
				m->x_pos = 0;
				els_beep_stub();
			}
			break;

		case ELS_MODE_SPHERE:
			if (m->select_menu == 0)
			{
				if (m->sph_r_mm < 1250) { m->sph_r_mm += 25; els_beep_stub(); }
				else if (m->sph_r_mm < 2500) { m->sph_r_mm += 50; els_beep_stub(); }
				else if (m->sph_r_mm < 4750) { m->sph_r_mm += 250; els_beep_stub(); }
			}
			else if (m->select_menu == 1)
			{
				if (m->cutter_step < 8) { m->cutter_step++; els_beep_stub(); }
			}
			else if (m->select_menu == 2)
			{
				m->x_pos = 0;
				els_beep_stub();
			}
			break;

		default:
			break;
	}
}

static void key_down_pressed(els_menu_t *m)
{
	switch (m->mode)
	{
		case ELS_MODE_FEED:
		case ELS_MODE_CONE_L:
		case ELS_MODE_CONE_R:
			if (m->select_menu == 0)
			{
				if (m->ap > 500) { m->ap -= 50; els_beep_stub(); }
				else if (m->ap > 0) { m->ap -= 10; els_beep_stub(); }
			}
			else if (m->select_menu == 1)
			{
				m->x_pos = 0;
				els_beep_stub();
			}
			break;

		case ELS_MODE_AFEED:
			if (m->select_menu == 0)
			{
				if (m->ap > 500) { m->ap -= 50; els_beep_stub(); }
				else if (m->ap > 10) { m->ap -= 10; els_beep_stub(); }
				else if (m->ap > 0) { m->ap -= 5; els_beep_stub(); }
			}
			else if (m->select_menu == 1)
			{
				if (m->total_tooth > 1) { m->total_tooth--; m->current_tooth = 1; els_beep_stub(); }
			}
			else if (m->select_menu == 2)
			{
				m->x_pos = 0;
				els_beep_stub();
			}
			break;

		case ELS_MODE_THREAD:
			if (m->select_menu == 0)
			{
				if (m->thread_step > 0) { m->thread_step--; els_beep_stub(); }
			}
			else if (m->select_menu == 2)
			{
				m->x_pos = 0;
				els_beep_stub();
			}
			break;

		case ELS_MODE_SPHERE:
			if (m->select_menu == 0)
			{
				if (m->sph_r_mm > 2500) { m->sph_r_mm -= 250; els_beep_stub(); }
				else if (m->sph_r_mm > 1250) { m->sph_r_mm -= 50; els_beep_stub(); }
				else if (m->sph_r_mm > 50) { m->sph_r_mm -= 25; els_beep_stub(); }
				if (m->sph_r_mm < m->bar_r_mm) m->bar_r_mm = m->sph_r_mm;
			}
			else if (m->select_menu == 1)
			{
				if (m->cutter_step > 0) { m->cutter_step--; els_beep_stub(); }
			}
			else if (m->select_menu == 2)
			{
				m->x_pos = 0;
				els_beep_stub();
			}
			break;

		default:
			break;
	}
}

static void key_left_pressed(els_menu_t *m)
{
	switch (m->mode)
	{
		case ELS_MODE_FEED:
		case ELS_MODE_AFEED:
		case ELS_MODE_CONE_L:
		case ELS_MODE_CONE_R:
			if (m->select_menu == 0)
			{
				if (m->pass_total > 1) { m->pass_total--; els_beep_stub(); }
			}
			else if (m->select_menu == 1)
			{
				if (m->mode == ELS_MODE_AFEED)
				{
					if (m->current_tooth > 1) m->current_tooth--;
					else m->current_tooth = m->total_tooth;
					els_beep_stub();
				}
				else
				{
					m->z_pos = 0;
					els_beep_stub();
				}
			}
			else if (m->select_menu == 2)
			{
				m->z_pos = 0;
				els_beep_stub();
			}
			break;

		case ELS_MODE_SPHERE:
			if (m->select_menu == 0)
			{
				if (m->bar_r_mm > 0) { m->bar_r_mm -= 25; els_beep_stub(); }
			}
			else if (m->select_menu == 1)
			{
				if (m->cutting_step > 0) { m->cutting_step--; els_beep_stub(); }
			}
			else if (m->select_menu == 2)
			{
				m->z_pos = 0;
				els_beep_stub();
			}
			break;

		default:
			break;
	}
}

static void key_right_pressed(els_menu_t *m)
{
	switch (m->mode)
	{
		case ELS_MODE_FEED:
		case ELS_MODE_AFEED:
		case ELS_MODE_CONE_L:
		case ELS_MODE_CONE_R:
			if (m->select_menu == 0)
			{
				if (m->pass_total < 99) { m->pass_total++; els_beep_stub(); }
			}
			else if (m->select_menu == 1)
			{
				if (m->mode == ELS_MODE_AFEED)
				{
					if (m->current_tooth < m->total_tooth) m->current_tooth++;
					else m->current_tooth = 1;
					els_beep_stub();
				}
				else
				{
					m->z_pos = 0;
					els_beep_stub();
				}
			}
			else if (m->select_menu == 2)
			{
				m->z_pos = 0;
				els_beep_stub();
			}
			break;

		case ELS_MODE_SPHERE:
			if (m->select_menu == 0)
			{
				if (m->bar_r_mm < m->sph_r_mm) { m->bar_r_mm += 25; els_beep_stub(); }
			}
			else if (m->select_menu == 1)
			{
				if (m->cutting_step < 8) { m->cutting_step++; els_beep_stub(); }
			}
			else if (m->select_menu == 2)
			{
				m->z_pos = 0;
				els_beep_stub();
			}
			break;

		default:
			break;
	}
}

static void handle_key_event(els_menu_t *m, menu_key_t e)
{
	switch (e)
	{
		case MENU_KEY_SEL: key_select_pressed(m); break;
		case MENU_KEY_U:   key_up_pressed(m); break;
		case MENU_KEY_D:   key_down_pressed(m); break;
		case MENU_KEY_L:   key_left_pressed(m); break;
		case MENU_KEY_R:   key_right_pressed(m); break;
		default: break;
	}
	els_menu_render(m);
}

void els_menu_init(els_menu_t *m, lcd_hd44780_t *lcd, menu_keys_t *keys)
{
	memset(m, 0, sizeof(*m));
	m->lcd = lcd;
	m->keys = keys;
	lcd_rus_init(&m->rus, lcd);

	/* Arduino: DELAY_ENTER_KEYCYCLE=5, DELAY_INTO_KEYCYCLE=1 with ~1ms tick */
	m->repeat_enter_ms = 250;
	m->repeat_rate_ms = 80;

	m->mode = ELS_MODE_FEED;
	m->sub_feed = ELS_SUB_MAN;
	m->sub_thread = ELS_SUB_MAN;
	m->sub_afeed = ELS_SUB_MAN;
	m->sub_cone = ELS_SUB_MAN;
	m->sub_sphere = ELS_SUB_MAN;
	m->select_menu = 0;

	m->ap = 0;
	m->pass_total = 1;
	m->pass_nr = 1;
	m->pass_fin = 0;
	m->thr_pass_summ = 0;
	m->pass_total_sphr = 1;
	m->total_tooth = 1;
	m->current_tooth = 1;
	m->thread_step = 0;
	m->cone_step = 0;
	m->sph_r_mm = 1000;
	m->bar_r_mm = 0;
	m->cutter_step = 4;
	m->cutting_step = 2;
	m->enc_pos = 0;
	m->duration = 0;

	/* ADC initial state (like Arduino 16-sample average) */
	m->adc_feed = 0;
	m->sum_adc = 0;
	for (uint8_t i = 0; i < 16; i++) m->adc_array[i] = 0;
	m->adc_idx = 0;
	m->feed_mm = (uint16_t)MAX_FEED;
	m->afeedback_mm = (uint16_t)MAX_aFEED;

	/* load custom chars */
	lcd_hd44780_create_char(lcd, 1, chr_left);
	lcd_hd44780_create_char(lcd, 2, chr_right);
	lcd_hd44780_create_char(lcd, 3, chr_up);
	lcd_hd44780_create_char(lcd, 4, chr_down);
	lcd_hd44780_create_char(lcd, 5, chr_degree);
	lcd_hd44780_create_char(lcd, 6, chr_diam);

	els_menu_render(m);
}

void els_menu_set_adc_raw10(els_menu_t *m, uint16_t adc10)
{
	/* Port of Read_ADC_Feed() from ADC.ino */
	if (adc10 > 1023U) adc10 = 1023U;

	if ((adc10 > (uint16_t)(m->adc_feed + 4U)) || (adc10 + 4U < m->adc_feed))
	{
		m->adc_idx++;
		if (m->adc_idx > 15U) m->adc_idx = 0U;
		m->sum_adc -= m->adc_array[m->adc_idx];
		m->adc_array[m->adc_idx] = adc10;
		m->sum_adc += adc10;
		m->adc_feed = (uint16_t)(m->sum_adc / 16U);
	}

	if (m->mode == ELS_MODE_FEED || m->mode == ELS_MODE_CONE_L || m->mode == ELS_MODE_CONE_R || m->mode == ELS_MODE_SPHERE)
	{
		uint16_t feed_new = (uint16_t)(MAX_FEED - ((uint32_t)(MAX_FEED - MIN_FEED + 1) * m->adc_feed) / 1024U);
		if (feed_new != m->feed_mm)
		{
			m->feed_mm = feed_new;
			els_menu_render(m);
		}
	}
	else if (m->mode == ELS_MODE_AFEED)
	{
		uint16_t a_new = (uint16_t)((MAX_aFEED / 10) - ((uint32_t)((MAX_aFEED / 10) - (MIN_aFEED / 10) + 1) * m->adc_feed) / 1024U);
		a_new = (uint16_t)(a_new * 10U);
		if (a_new != m->afeedback_mm)
		{
			m->afeedback_mm = a_new;
			els_menu_render(m);
		}
	}
}

static uint8_t read_mode_byte(const els_menu_t *m)
{
	/* MODE_D0..D7 are on GPIOG0..7 in this project */
	uint32_t idr = m->mode_port->IDR;
	return (uint8_t)(idr & 0xFFU);
}

static uint8_t read_submode_bits(const els_menu_t *m)
{
	/* SUBMODE_0..2 are on GPIOD8..10, we want them in bits 5..7 like Arduino Submode_Read */
	uint32_t idr = m->submode_port->IDR;
	uint8_t b = 0;
	b |= ((idr & (1U << 8))  ? (1U << 5) : 0U);
	b |= ((idr & (1U << 9))  ? (1U << 6) : 0U);
	b |= ((idr & (1U << 10)) ? (1U << 7) : 0U);
	return b;
}

void els_menu_update(els_menu_t *m, uint32_t now_ms)
{
	menu_keys_update(m->keys, now_ms);

	/* edge events */
	menu_key_t e = menu_keys_pop_event(m->keys);
	if (e != MENU_KEY_NONE)
	{
		handle_key_event(m, e);
	}

	/* Mode switch (active-low patterns like Arduino) */
	if (m->mode_port)
	{
		uint8_t mode_new = read_mode_byte(m);
		if (mode_new == 0x7FU) m->mode = ELS_MODE_FEED;
		else if (mode_new == 0xBFU) m->mode = ELS_MODE_AFEED;
		else if (mode_new == 0xDFU) m->mode = ELS_MODE_CONE_L;
		else if (mode_new == 0xEFU) m->mode = ELS_MODE_CONE_R;
		else if (mode_new == 0xF7U) m->mode = ELS_MODE_THREAD;
		else if (mode_new == 0xFBU) m->mode = ELS_MODE_SPHERE;
		else if (mode_new == 0xFDU) m->mode = ELS_MODE_TACHO;
		else if (mode_new == 0xFEU) m->mode = ELS_MODE_RESERVE;
	}

	/* Submode switch */
	if (m->submode_port)
	{
		uint8_t sub = read_submode_bits(m);
		if (sub == 0xC0U)
		{
			m->sub_thread = ELS_SUB_INT;
			m->sub_feed = ELS_SUB_INT;
			m->sub_afeed = ELS_SUB_INT;
			m->sub_cone = ELS_SUB_INT;
			m->sub_sphere = ELS_SUB_INT;
		}
		else if (sub == 0xA0U)
		{
			m->sub_thread = ELS_SUB_MAN;
			m->sub_feed = ELS_SUB_MAN;
			m->sub_afeed = ELS_SUB_MAN;
			m->sub_cone = ELS_SUB_MAN;
			m->sub_sphere = ELS_SUB_MAN;
		}
		else if (sub == 0x60U)
		{
			m->sub_thread = ELS_SUB_EXT;
			m->sub_feed = ELS_SUB_EXT;
			m->sub_afeed = ELS_SUB_EXT;
			m->sub_cone = ELS_SUB_EXT;
			m->sub_sphere = ELS_SUB_EXT;
		}
	}

	/* autorepeat for held keys (U/D/L/R only) */
	uint8_t mask = menu_keys_get_pressed_mask(m->keys);
	uint8_t repeat_mask = mask & 0x0FU; /* bits: L,R,U,D */

	if (repeat_mask == 0)
	{
		m->last_pressed_mask = 0;
		return;
	}

	if (repeat_mask != m->last_pressed_mask)
	{
		m->last_pressed_mask = repeat_mask;
		m->pressed_since_ms = now_ms;
		m->last_repeat_ms = now_ms;
		return;
	}

	if ((now_ms - m->pressed_since_ms) < m->repeat_enter_ms) return;
	if ((now_ms - m->last_repeat_ms) < m->repeat_rate_ms) return;

	m->last_repeat_ms = now_ms;

	/* priority order like Arduino Buttons_Read decoding is effectively one key at a time */
	if (repeat_mask & (1U << 3)) handle_key_event(m, MENU_KEY_D);
	else if (repeat_mask & (1U << 2)) handle_key_event(m, MENU_KEY_U);
	else if (repeat_mask & (1U << 1)) handle_key_event(m, MENU_KEY_R);
	else if (repeat_mask & (1U << 0)) handle_key_event(m, MENU_KEY_L);
}

void els_menu_render(els_menu_t *m)
{
	lcd_rus_t *r = &m->rus;

	if (m->err_1)
	{
		lcd_rus_set_cursor(r, 0, 0); lcd_write_padded(r, "BHИMAHИE:");
		lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, "");
		lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, "    УПOPЫ HE 3AДAHЫ!");
		lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, "");
		return;
	}
	if (m->err_2)
	{
		lcd_rus_set_cursor(r, 0, 0); lcd_write_padded(r, "Установите суппорт");
		lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, "");
		lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, " в исходную позицию!");
		lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, "");
		return;
	}
	if (m->complete)
	{
		lcd_rus_set_cursor(r, 0, 0); lcd_write_padded(r, "");
		lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, "OПEPAЦИЯ 3ABEPШEHA!");
		lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, "");
		lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, "");
		return;
	}

	/* Bigger than 20 to avoid -Wformat-truncation warnings; LCD output is still padded to 20 */
	char row[64];
	memset(row, 0, sizeof(row));

	switch (m->mode)
	{
		case ELS_MODE_FEED:
		{
			if (m->select_menu == 0)
			{
				lcd_rus_set_cursor(r, 0, 0);
				lcd_write_padded(r, "СИНХРОННЫЙ ");
				lcd_rus_set_cursor(r, 11, 0);
				if (m->sub_feed == ELS_SUB_INT) lcd_rus_write_utf8(r, "  Внутр. ");
				else if (m->sub_feed == ELS_SUB_EXT) lcd_rus_write_utf8(r, "  Наружн.");
				else lcd_rus_write_utf8(r, "  Ручной ");

				snprintf(row, sizeof(row), "Подача,  мм/об: %1d.%02d", m->feed_mm / 100U, m->feed_mm % 100U);
				lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, row);

				if (m->sub_feed == ELS_SUB_MAN) snprintf(row, sizeof(row), "Проходов всего:   %2d", m->pass_total);
				else snprintf(row, sizeof(row), "Проходов осталось:%2d", (m->pass_total - m->pass_nr + 1));
				lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, row);

				snprintf(row, sizeof(row), "Съём на диаметр: %1d.%01d", m->ap / 100, (m->ap % 100) / 10);
				lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, row);
			}
			else
			{
				lcd_rus_set_cursor(r, 0, 0); lcd_write_padded(r, "");
				lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, "");
				snprintf(row, sizeof(row), "Ocь X:      %3ld.%02ldмм", (m->x_pos >= 0) ? (m->x_pos / 100) : (-m->x_pos / 100), (m->x_pos >= 0) ? (m->x_pos % 100) : (-m->x_pos % 100));
				lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, row);
				snprintf(row, sizeof(row), "Ocь Z:      %3ld.%02ldмм", (m->z_pos >= 0) ? (m->z_pos / 100) : (-m->z_pos / 100), (m->z_pos >= 0) ? (m->z_pos % 100) : (-m->z_pos % 100));
				lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, row);
			}
			break;
		}

		case ELS_MODE_AFEED:
		{
			if (m->select_menu == 0)
			{
				lcd_rus_set_cursor(r, 0, 0);
				lcd_write_padded(r, "АСИНХРОННЫЙ ");
				lcd_rus_set_cursor(r, 12, 0);
				if (m->sub_afeed == ELS_SUB_INT) lcd_rus_write_utf8(r, " Внутр. ");
				else if (m->sub_afeed == ELS_SUB_EXT) lcd_rus_write_utf8(r, " Наружн.");
				else lcd_rus_write_utf8(r, " Ручной ");

				snprintf(row, sizeof(row), "Подача, мм/мин:  %3d", (int)m->afeedback_mm);
				lcd_rus_set_cursor(r, 0, 1);
				lcd_write_padded(r, row);
				if (m->sub_afeed == ELS_SUB_MAN) snprintf(row, sizeof(row), "Проходов всего:   %2d", m->pass_total);
				else snprintf(row, sizeof(row), "Проходов осталось:%2d", (m->pass_total - m->pass_nr + 1));
				lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, row);
				snprintf(row, sizeof(row), "Съём на радиус:  %1d.%02d", m->ap / 100, m->ap % 100);
				lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, row);
			}
			else if (m->select_menu == 1)
			{
				/* Port of Print.ino angle screen (values will be correct once enc_pos is wired) */
				uint32_t enc_tick = (uint32_t)ENC_LINE_PER_REV * 2U;
				uint32_t spindle_angle = (uint32_t)((int64_t)m->enc_pos * 360000LL / (int64_t)enc_tick);
				uint32_t required_angle = (uint32_t)(360000U * (uint32_t)(m->current_tooth - 1U) / (uint32_t)m->total_tooth);

				lcd_rus_set_cursor(r, 0, 0);
				lcd_rus_write_utf8(r, "Текущий  угол:");
				snprintf(row, sizeof(row), "%3lu.%01lu", (unsigned long)(spindle_angle / 1000U), (unsigned long)((spindle_angle % 1000U) / 100U));
				lcd_rus_write_utf8(r, row);
				lcd_put_custom(r, 5);
				lcd_rus_set_cursor(r, 0, 0);
				lcd_write_padded(r, "Текущий  угол:        ");

				lcd_rus_set_cursor(r, 0, 1);
				lcd_rus_write_utf8(r, "Делим круг на:");
				snprintf(row, sizeof(row), "%3d ", m->total_tooth);
				lcd_rus_write_utf8(r, row);
				lcd_put_custom(r, 3); lcd_put_custom(r, 4);
				lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, "");

				lcd_rus_set_cursor(r, 0, 2);
				lcd_rus_write_utf8(r, "Выбор отметки:");
				snprintf(row, sizeof(row), "%3d ", m->current_tooth);
				lcd_rus_write_utf8(r, row);
				lcd_put_custom(r, 1); lcd_put_custom(r, 2);
				lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, "");

				lcd_rus_set_cursor(r, 0, 3);
				lcd_rus_write_utf8(r, "Угол  сектора:");
				snprintf(row, sizeof(row), "%3lu.%01lu", (unsigned long)(required_angle / 1000U), (unsigned long)((required_angle % 1000U) / 100U));
				lcd_rus_write_utf8(r, row);
				lcd_put_custom(r, 5);
				lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, "");
			}
			else
			{
				lcd_rus_set_cursor(r, 0, 0); lcd_write_padded(r, "");
				lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, "");
				snprintf(row, sizeof(row), "Ocь X:      %3ld.%02ldмм", (m->x_pos >= 0) ? (m->x_pos / 100) : (-m->x_pos / 100), (m->x_pos >= 0) ? (m->x_pos % 100) : (-m->x_pos % 100));
				lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, row);
				snprintf(row, sizeof(row), "Ocь Z:      %3ld.%02ldмм", (m->z_pos >= 0) ? (m->z_pos / 100) : (-m->z_pos / 100), (m->z_pos >= 0) ? (m->z_pos % 100) : (-m->z_pos % 100));
				lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, row);
			}
			break;
		}

		case ELS_MODE_THREAD:
		{
			if (m->select_menu == 0)
			{
				lcd_rus_set_cursor(r, 0, 0);
				lcd_write_padded(r, "PE3ЬБA ");
				lcd_rus_set_cursor(r, 7, 0);
				if (m->sub_thread == ELS_SUB_INT) lcd_rus_write_utf8(r, "   Bнyтpенняя");
				else if (m->sub_thread == ELS_SUB_EXT) lcd_rus_write_utf8(r, "     Hapyжняя");
				else lcd_rus_write_utf8(r, "       Pyчная");

				/* Match Print.ino formatting */
				lcd_rus_set_cursor(r, 0, 1);
				lcd_rus_write_utf8(r, "Шаг:          ");
				lcd_rus_write_utf8(r, Thread_Info[m->thread_step].Thread_Print);
				{
					size_t lenTP = strlen(Thread_Info[m->thread_step].Thread_Print);
					if (lenTP == 4)
					{
						lcd_rus_write_utf8(r, "мм");
					}
					else if (lenTP == 3)
					{
						lcd_rus_write_utf8(r, "tpi");
					}
				}
				/* pad rest of row */
				lcd_rus_set_cursor(r, 0, 1);
				memset(row, 0, sizeof(row));
				snprintf(row, sizeof(row), "Шаг:          %s", Thread_Info[m->thread_step].Thread_Print);
				lcd_write_padded(r, row);

				lcd_rus_set_cursor(r, 0, 2);
				if (m->sub_thread == ELS_SUB_MAN)
				{
					int total = (int)Thread_Info[m->thread_step].Pass + PASS_FINISH + m->pass_fin + m->thr_pass_summ;
					snprintf(row, sizeof(row), "Пpoxoдов всего:   %2d", total);
					lcd_write_padded(r, row);
				}
				else
				{
					int left = (int)(Thread_Info[m->thread_step].Pass - m->pass_nr + 1 + PASS_FINISH + m->pass_fin) + m->thr_pass_summ;
					snprintf(row, sizeof(row), "Пpoxoдов осталось:%2d", left);
					lcd_write_padded(r, row);
				}

				snprintf(row, sizeof(row), "Максимум,об/мин:%s", Thread_Info[m->thread_step].Limit_Print);
				lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, row);
			}
			else if (m->select_menu == 1)
			{
				lcd_rus_set_cursor(r, 0, 0); lcd_write_padded(r, "");
				lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, "Чистовых            ");
				snprintf(row, sizeof(row), "Проходов:         %2d", PASS_FINISH + m->pass_fin);
				lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, row);
				lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, "");
			}
			else
			{
				lcd_rus_set_cursor(r, 0, 0); lcd_write_padded(r, "");
				lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, "");
				snprintf(row, sizeof(row), "Ocь X:      %3ld.%02ldмм", (m->x_pos >= 0) ? (m->x_pos / 100) : (-m->x_pos / 100), (m->x_pos >= 0) ? (m->x_pos % 100) : (-m->x_pos % 100));
				lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, row);
				snprintf(row, sizeof(row), "Ocь Z:      %3ld.%02ldмм", (m->z_pos >= 0) ? (m->z_pos / 100) : (-m->z_pos / 100), (m->z_pos >= 0) ? (m->z_pos % 100) : (-m->z_pos % 100));
				lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, row);
			}
			break;
		}

		case ELS_MODE_CONE_L:
		case ELS_MODE_CONE_R:
		{
			const char *hdr = (m->mode == ELS_MODE_CONE_L) ? "KOHУC <" : "KOHУC >";
			if (m->select_menu == 0)
			{
				lcd_rus_set_cursor(r, 0, 0);
				lcd_rus_write_utf8(r, hdr);
				lcd_rus_write_utf8(r, " ");
				lcd_rus_write_utf8(r, Cone_Info[m->cone_step].Cone_Print);
				lcd_rus_set_cursor(r, 11, 0);
				if (m->sub_cone == ELS_SUB_INT) lcd_rus_write_utf8(r, "  Внутр. ");
				else if (m->sub_cone == ELS_SUB_EXT) lcd_rus_write_utf8(r, "  Наружн.");
				else lcd_rus_write_utf8(r, "  Ручной ");

				snprintf(row, sizeof(row), "Подача,  мм/об: %1d.%02d", m->feed_mm / 100U, m->feed_mm % 100U);
				lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, row);
				if (m->sub_cone == ELS_SUB_MAN) snprintf(row, sizeof(row), "Проходов всего:   %2d", m->pass_total);
				else snprintf(row, sizeof(row), "Проходов осталось:%2d", (m->pass_total - m->pass_nr + 1));
				lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, row);
				snprintf(row, sizeof(row), "Съём на диаметр: %1d.%01d", m->ap / 100, (m->ap % 100) / 10);
				lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, row);
			}
			else if (m->select_menu == 1)
			{
				lcd_rus_set_cursor(r, 0, 0);
				lcd_rus_write_utf8(r, hdr);
				lcd_rus_set_cursor(r, 11, 0);
				lcd_rus_write_utf8(r, "       ");
				lcd_put_custom(r, 1);
				lcd_put_custom(r, 2);
				lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, "");
				lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, "");
				lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, "");
			}
			else
			{
				lcd_rus_set_cursor(r, 0, 0); lcd_write_padded(r, "");
				lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, "");
				snprintf(row, sizeof(row), "Ocь X:      %3ld.%02ldмм", (m->x_pos >= 0) ? (m->x_pos / 100) : (-m->x_pos / 100), (m->x_pos >= 0) ? (m->x_pos % 100) : (-m->x_pos % 100));
				lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, row);
				snprintf(row, sizeof(row), "Ocь Z:      %3ld.%02ldмм", (m->z_pos >= 0) ? (m->z_pos / 100) : (-m->z_pos / 100), (m->z_pos >= 0) ? (m->z_pos % 100) : (-m->z_pos % 100));
				lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, row);
			}
			break;
		}

		case ELS_MODE_SPHERE:
		{
			if (m->select_menu == 0)
			{
				lcd_rus_set_cursor(r, 0, 0);
				lcd_rus_write_utf8(r, "ШAP ");
				lcd_put_custom(r, 6);
				snprintf(row, sizeof(row), "%2ld.%01ldмм", (m->sph_r_mm * 2) / 100, ((m->sph_r_mm * 2) / 10) % 10);
				lcd_write_padded(r, row);

				snprintf(row, sizeof(row), "Подача,  мм/об: %1d.%02d", m->feed_mm / 100U, m->feed_mm % 100U);
				lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, row);

				lcd_rus_set_cursor(r, 0, 2);
				lcd_rus_write_utf8(r, "Оставить ножку ");
				lcd_put_custom(r, 6);
				snprintf(row, sizeof(row), "%ld.%02ld", (m->bar_r_mm * 2) / 100, (m->bar_r_mm * 2) % 100);
				lcd_write_padded(r, row);

				lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, "Проходов осталось  0");
			}
			else if (m->select_menu == 1)
			{
				snprintf(row, sizeof(row), "Шиpина peзцa: %1d.%02dмм", Cutter_Width_array[m->cutter_step] / 100, Cutter_Width_array[m->cutter_step] % 100);
				lcd_rus_set_cursor(r, 0, 0); lcd_write_padded(r, row);
				snprintf(row, sizeof(row), "Шaг по ocи Z: %1d.%02dмм", Cutting_Width_array[m->cutting_step] / 100, Cutting_Width_array[m->cutting_step] % 100);
				lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, row);
				lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, "");
				lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, "");
			}
			else
			{
				lcd_rus_set_cursor(r, 0, 0); lcd_write_padded(r, "");
				lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, "");
				snprintf(row, sizeof(row), "Ocь X:      %3ld.%02ldмм", (m->x_pos >= 0) ? (m->x_pos / 100) : (-m->x_pos / 100), (m->x_pos >= 0) ? (m->x_pos % 100) : (-m->x_pos % 100));
				lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, row);
				snprintf(row, sizeof(row), "Ocь Z:      %3ld.%02ldмм", (m->z_pos >= 0) ? (m->z_pos / 100) : (-m->z_pos / 100), (m->z_pos >= 0) ? (m->z_pos % 100) : (-m->z_pos % 100));
				lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, row);
			}
			break;
		}

		case ELS_MODE_TACHO:
		{
			unsigned long freq = 0;
			unsigned long rpm = 0;
			if (m->duration != 0)
			{
				freq = (unsigned long)((double)ENC_LINE_PER_REV / (double)m->duration * (double)Th);
				rpm = freq * 60UL;
			}
			if (m->select_menu == 0)
			{
				lcd_rus_set_cursor(r, 0, 0); lcd_write_padded(r, "TAXOMETP            ");
				lcd_rus_set_cursor(r, 0, 1);
				lcd_rus_write_utf8(r, "         ");
				snprintf(row, sizeof(row), "%4lu", (unsigned long)(rpm / 10000UL));
				lcd_rus_write_utf8(r, row);
				lcd_rus_write_utf8(r, " Oб/мин");
				lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, "");
				lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, "");
				lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, "");
			}
			else
			{
				lcd_rus_set_cursor(r, 0, 0); lcd_write_padded(r, "Чacтoтa вpaщeния    ");
				lcd_rus_set_cursor(r, 0, 1);
				lcd_rus_write_utf8(r, "            ");
				snprintf(row, sizeof(row), "%3lu.%01lu", (unsigned long)(freq / 10000UL), (unsigned long)(freq % 10UL));
				lcd_rus_write_utf8(r, row);
				lcd_rus_write_utf8(r, " Hz");
				lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, "");
				lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, "");
				lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, "");
			}
			break;
		}

		case ELS_MODE_RESERVE:
		default:
			lcd_rus_set_cursor(r, 0, 0); lcd_write_padded(r, "");
			lcd_rus_set_cursor(r, 0, 1); lcd_write_padded(r, "");
			lcd_rus_set_cursor(r, 0, 2); lcd_write_padded(r, "");
			lcd_rus_set_cursor(r, 0, 3); lcd_write_padded(r, "              PE3EPB");
			break;
	}
}

