#include "els_limits.h"

#include <limits.h>
#include <string.h>

#include "main.h"
#include "els_menu.h"
#include "els_model.h"

#define LIM_DEBOUNCE_MS   30U
#define MECH_DEBOUNCE_MS  15U

#define SUB_HW_INT  0xC0U
#define SUB_HW_MAN  0xA0U
#define SUB_HW_EXT  0x60U

#define ELS_LIM_GAP  ((int32_t)((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * (int32_t)ELS_REPEAT * 2))

static struct
{
	long lim_z_left;
	long lim_z_right;
	long lim_x_front;
	long lim_x_rear;
	volatile long limit_pos;
	volatile long limit_pos_hc;
	bool flag_zl;
	bool flag_zr;
	bool flag_xf;
	bool flag_xr;
	bool lim_btn_latch;
	bool err1;
	bool err2;
	uint8_t lim_raw_last;
	uint32_t lim_change_ms;
	uint8_t lim_hw_stable;
	uint8_t mech_stable;
	uint8_t mech_raw_last;
	uint32_t mech_change_ms;
} L;

static long lim_sent_max(void) { return (long)LONG_MAX; }
static long lim_sent_min(void) { return (long)LONG_MIN; }

static uint8_t read_limit_pins_pressed(void)
{
	/* 1 = pressed (active LOW) */
	uint8_t p = 0;
	if (HAL_GPIO_ReadPin(LIM_REAR_GPIO_Port, LIM_REAR_Pin) == GPIO_PIN_RESET) p |= 1U << 0;
	if (HAL_GPIO_ReadPin(LIM_FRONT_GPIO_Port, LIM_FRONT_Pin) == GPIO_PIN_RESET) p |= 1U << 1;
	if (HAL_GPIO_ReadPin(LIM_RIGHT_GPIO_Port, LIM_RIGHT_Pin) == GPIO_PIN_RESET) p |= 1U << 2;
	if (HAL_GPIO_ReadPin(LIM_LEFT_GPIO_Port, LIM_LEFT_Pin) == GPIO_PIN_RESET) p |= 1U << 3;
	return p;
}

static void sync_limit_leds(void)
{
	HAL_GPIO_WritePin(LED_LIM_LEFT_GPIO_Port, LED_LIM_LEFT_Pin, L.flag_zl ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_LIM_RIGHT_GPIO_Port, LED_LIM_RIGHT_Pin, L.flag_zr ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_LIM_FRONT_GPIO_Port, LED_LIM_FRONT_Pin, L.flag_xf ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_LIM_REAR_GPIO_Port, LED_LIM_REAR_Pin, L.flag_xr ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void beep_ok(void)
{
	HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET);
	HAL_Delay(25);
	HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET);
}

static void beep_error(void)
{
	for (int i = 0; i < 3; i++)
	{
		HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET);
		HAL_Delay(35);
		HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET);
		HAL_Delay(35);
	}
}

static long quantize_z(long p)
{
	return (p + (long)(McSTEP_Z / 2)) & ~((long)McSTEP_Z - 1L);
}

static long quantize_x(long p)
{
	return (p + (long)(McSTEP_X / 2)) & ~((long)McSTEP_X - 1L);
}

static bool joy_z_active(const els_menu_t *m)
{
	if (!m->joy_port) return false;
	uint8_t n = 0x0FU;
	if (HAL_GPIO_ReadPin(m->joy_port, m->joy_pin_l) == GPIO_PIN_RESET) n &= (uint8_t)~(1U << 0);
	if (HAL_GPIO_ReadPin(m->joy_port, m->joy_pin_r) == GPIO_PIN_RESET) n &= (uint8_t)~(1U << 1);
	if (HAL_GPIO_ReadPin(m->joy_port, m->joy_pin_u) == GPIO_PIN_RESET) n &= (uint8_t)~(1U << 2);
	if (HAL_GPIO_ReadPin(m->joy_port, m->joy_pin_d) == GPIO_PIN_RESET) n &= (uint8_t)~(1U << 3);
	return (n == 0x0BU) || (n == 0x07U);
}

static bool joy_x_active(const els_menu_t *m)
{
	if (!m->joy_port) return false;
	uint8_t n = 0x0FU;
	if (HAL_GPIO_ReadPin(m->joy_port, m->joy_pin_l) == GPIO_PIN_RESET) n &= (uint8_t)~(1U << 0);
	if (HAL_GPIO_ReadPin(m->joy_port, m->joy_pin_r) == GPIO_PIN_RESET) n &= (uint8_t)~(1U << 1);
	if (HAL_GPIO_ReadPin(m->joy_port, m->joy_pin_u) == GPIO_PIN_RESET) n &= (uint8_t)~(1U << 2);
	if (HAL_GPIO_ReadPin(m->joy_port, m->joy_pin_d) == GPIO_PIN_RESET) n &= (uint8_t)~(1U << 3);
	return (n == 0x0EU) || (n == 0x0DU);
}

static uint8_t read_submode_hw(const els_menu_t *m)
{
	if (!m->submode_port) return 0xFFU;
	uint32_t idr = m->submode_port->IDR;
	uint8_t b = 0;
	b |= ((idr & (1U << 8)) ? (1U << 5) : 0U);
	b |= ((idr & (1U << 9)) ? (1U << 6) : 0U);
	b |= ((idr & (1U << 10)) ? (1U << 7) : 0U);
	return b;
}

static bool mode_uses_zx_limits(els_mode_t mode)
{
	return (mode == ELS_MODE_FEED || mode == ELS_MODE_AFEED || mode == ELS_MODE_THREAD
	        || mode == ELS_MODE_CONE_L || mode == ELS_MODE_CONE_R || mode == ELS_MODE_SPHERE);
}

static bool thread_feed_pair_ok(void)
{
	return (L.flag_zl && L.flag_zr) || (L.flag_xf && L.flag_xr);
}

static bool sphere_pair_ok(void)
{
	return (L.flag_zr && L.flag_xr) || (L.flag_zl && L.flag_xr);
}

static bool pos_ok_int_ext_common(const els_menu_t *m)
{
	long mz = m->z_pos;
	long mx = m->x_pos;
	return (mz == L.lim_z_right) || (mz == L.lim_z_left) || (mx == L.lim_x_rear) || (mx == L.lim_x_front);
}

static bool pos_ok_sphere_int_ext(const els_menu_t *m)
{
	long mz = m->z_pos;
	long mx = m->x_pos;
	return ((mz == L.lim_z_right) && (mx == L.lim_x_rear)) || ((mz == L.lim_z_left) && (mx == L.lim_x_rear));
}

static void switch_man_like_arduino(els_menu_t *m)
{
	L.err1 = false;
	L.err2 = false;
	m->submode_force_man = false;
}

static void switch_int_like_arduino(els_menu_t *m)
{
	switch (m->mode)
	{
		case ELS_MODE_THREAD:
		case ELS_MODE_FEED:
		case ELS_MODE_AFEED:
		case ELS_MODE_CONE_L:
		case ELS_MODE_CONE_R:
			if (thread_feed_pair_ok())
			{
				L.err1 = false;
				m->submode_force_man = false;
				if (pos_ok_int_ext_common(m)) L.err2 = false;
				else
				{
					L.err2 = true;
					m->submode_force_man = true;
					beep_error();
				}
			}
			else
			{
				L.err1 = true;
				m->submode_force_man = true;
				beep_error();
			}
			break;

		case ELS_MODE_SPHERE:
			if (sphere_pair_ok())
			{
				L.err1 = false;
				m->submode_force_man = false;
				if (pos_ok_sphere_int_ext(m)) L.err2 = false;
				else
				{
					L.err2 = true;
					m->submode_force_man = true;
					beep_error();
				}
			}
			else
			{
				L.err1 = true;
				m->submode_force_man = true;
				beep_error();
			}
			break;

		default:
			break;
	}
}

static void switch_ext_like_arduino(els_menu_t *m)
{
	switch_int_like_arduino(m);
}

static void limit_left_pressed(els_menu_t *m)
{
	if (!mode_uses_zx_limits(m->mode)) return;
	if (joy_z_active(m) || read_submode_hw(m) != SUB_HW_MAN) return;

	if (!L.flag_zl)
	{
		if (m->z_pos > (L.lim_z_right + (long)ELS_LIM_GAP))
		{
			L.flag_zl = true;
			L.lim_z_left = quantize_z(m->z_pos);
			beep_ok();
		}
		/*
		 * If opposite Z limit is already set but Motor_Z_Pos did not move (no step ISR yet),
		 * Arduino-style gap check can never pass from the same z_pos. Record a minimum
		 * corridor: left = right + GAP (same idea as jogging away before second teach).
		 */
		else if (L.flag_zr)
		{
			L.flag_zl = true;
			L.lim_z_left = quantize_z(L.lim_z_right + (long)ELS_LIM_GAP);
			beep_ok();
		}
	}
	else
	{
		L.flag_zl = false;
		L.lim_z_left = lim_sent_max();
		beep_ok();
	}
	sync_limit_leds();
	els_menu_render(m);
}

static void limit_right_pressed(els_menu_t *m)
{
	if (!mode_uses_zx_limits(m->mode)) return;
	if (joy_z_active(m) || read_submode_hw(m) != SUB_HW_MAN) return;

	if (!L.flag_zr)
	{
		if (m->z_pos < (L.lim_z_left - (long)ELS_LIM_GAP))
		{
			L.flag_zr = true;
			L.lim_z_right = quantize_z(m->z_pos);
			beep_ok();
		}
		else if (L.flag_zl)
		{
			L.flag_zr = true;
			L.lim_z_right = quantize_z(L.lim_z_left - (long)ELS_LIM_GAP);
			beep_ok();
		}
	}
	else
	{
		L.flag_zr = false;
		L.lim_z_right = lim_sent_min();
		beep_ok();
	}
	sync_limit_leds();
	els_menu_render(m);
}

static void limit_front_pressed(els_menu_t *m)
{
	if (!mode_uses_zx_limits(m->mode)) return;
	if (joy_x_active(m) || read_submode_hw(m) != SUB_HW_MAN) return;

	if (!L.flag_xf)
	{
		if (m->x_pos > (L.lim_x_rear + (long)ELS_LIM_GAP))
		{
			L.flag_xf = true;
			L.lim_x_front = quantize_x(m->x_pos);
			beep_ok();
		}
		else if (L.flag_xr)
		{
			L.flag_xf = true;
			L.lim_x_front = quantize_x(L.lim_x_rear + (long)ELS_LIM_GAP);
			beep_ok();
		}
	}
	else
	{
		L.flag_xf = false;
		L.lim_x_front = lim_sent_max();
		beep_ok();
	}
	sync_limit_leds();
	els_menu_render(m);
}

static void limit_rear_pressed(els_menu_t *m)
{
	if (!mode_uses_zx_limits(m->mode)) return;
	if (joy_x_active(m) || read_submode_hw(m) != SUB_HW_MAN) return;

	if (!L.flag_xr)
	{
		if (m->x_pos < (L.lim_x_front - (long)ELS_LIM_GAP))
		{
			L.flag_xr = true;
			L.lim_x_rear = quantize_x(m->x_pos);
			beep_ok();
		}
		else if (L.flag_xf)
		{
			L.flag_xr = true;
			L.lim_x_rear = quantize_x(L.lim_x_front - (long)ELS_LIM_GAP);
			beep_ok();
		}
	}
	else
	{
		L.flag_xr = false;
		L.lim_x_rear = lim_sent_min();
		beep_ok();
	}
	sync_limit_leds();
	els_menu_render(m);
}

void els_limits_init(void)
{
	memset(&L, 0, sizeof(L));
	L.lim_z_left = lim_sent_max();
	L.lim_z_right = lim_sent_min();
	L.lim_x_front = lim_sent_max();
	L.lim_x_rear = lim_sent_min();
	L.limit_pos = 0;
	L.limit_pos_hc = 0;
	L.mech_raw_last = 0xFFU;
	sync_limit_leds();
}

void els_limits_on_submode_edge(els_menu_t *m, uint8_t old_hw, uint8_t new_hw)
{
	(void)old_hw;
	if (new_hw == SUB_HW_MAN)
	{
		switch (m->mode)
		{
			case ELS_MODE_THREAD:
			case ELS_MODE_FEED:
			case ELS_MODE_AFEED:
			case ELS_MODE_CONE_L:
			case ELS_MODE_CONE_R:
			case ELS_MODE_SPHERE:
				switch_man_like_arduino(m);
				break;
			default:
				break;
		}
	}
	else if (new_hw == SUB_HW_INT) switch_int_like_arduino(m);
	else if (new_hw == SUB_HW_EXT) switch_ext_like_arduino(m);

	m->err_1 = L.err1;
	m->err_2 = L.err2;
	els_menu_render(m);
}

bool els_limits_mech_stop(void)
{
	uint8_t raw = 0;
	if (HAL_GPIO_ReadPin(LIM_MECH_1_GPIO_Port, LIM_MECH_1_Pin) == GPIO_PIN_RESET) raw |= 1U;
	if (HAL_GPIO_ReadPin(LIM_MECH_2_GPIO_Port, LIM_MECH_2_Pin) == GPIO_PIN_RESET) raw |= 2U;

	uint32_t now = HAL_GetTick();
	if (raw != L.mech_raw_last)
	{
		L.mech_raw_last = raw;
		L.mech_change_ms = now;
	}
	else if ((now - L.mech_change_ms) >= MECH_DEBOUNCE_MS)
	{
		L.mech_stable = raw;
	}
	return L.mech_stable != 0U;
}

long els_limits_limit_pos_read(void) { return L.limit_pos; }
void els_limits_limit_pos_write(long v) { L.limit_pos = v; }
long els_limits_limit_pos_hc_read(void) { return L.limit_pos_hc; }
void els_limits_limit_pos_hc_write(long v) { L.limit_pos_hc = v; }

bool els_limits_can_move_z(const els_menu_t *m, int dir_plus)
{
	if (els_limits_mech_stop()) return false;
	if (dir_plus > 0)
	{
		if (L.flag_zl && m->z_pos >= L.lim_z_left) return false;
	}
	else if (dir_plus < 0)
	{
		if (L.flag_zr && m->z_pos <= L.lim_z_right) return false;
	}
	return true;
}

bool els_limits_can_move_x(const els_menu_t *m, int dir_plus)
{
	if (els_limits_mech_stop()) return false;
	if (dir_plus > 0)
	{
		if (L.flag_xf && m->x_pos >= L.lim_x_front) return false;
	}
	else if (dir_plus < 0)
	{
		if (L.flag_xr && m->x_pos <= L.lim_x_rear) return false;
	}
	return true;
}

void els_limits_update(els_menu_t *m, uint32_t now_ms)
{
	(void)els_limits_mech_stop();

	uint8_t raw = read_limit_pins_pressed();
	if (raw != L.lim_raw_last)
	{
		L.lim_raw_last = raw;
		L.lim_change_ms = now_ms;
	}
	else if ((now_ms - L.lim_change_ms) >= LIM_DEBOUNCE_MS)
	{
		if (raw != L.lim_hw_stable)
		{
			uint8_t prev = L.lim_hw_stable;
			L.lim_hw_stable = raw;
			uint8_t rising = (uint8_t)((~prev) & raw);

			if (raw == 0U) L.lim_btn_latch = false;

			if (!L.lim_btn_latch && rising != 0U)
			{
				unsigned cnt = 0;
				uint8_t t = rising;
				while (t) { cnt += (unsigned)(t & 1U); t >>= 1U; }
				if (cnt == 1U)
				{
					L.lim_btn_latch = true;
					if (rising & (1U << 3)) limit_left_pressed(m);
					else if (rising & (1U << 2)) limit_right_pressed(m);
					else if (rising & (1U << 1)) limit_front_pressed(m);
					else if (rising & (1U << 0)) limit_rear_pressed(m);
				}
			}
		}
	}

	m->err_1 = L.err1;
	m->err_2 = L.err2;
}
