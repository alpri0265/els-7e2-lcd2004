#include "menu_keys.h"

static uint8_t keys_read_raw_mask(menu_keys_t *k)
{
	/* Active-low (pull-up): pressed => 1 in mask */
	uint8_t m = 0;
	m |= (HAL_GPIO_ReadPin(k->port_l, k->pin_l) == GPIO_PIN_RESET) ? (1U << 0) : 0U;
	m |= (HAL_GPIO_ReadPin(k->port_r, k->pin_r) == GPIO_PIN_RESET) ? (1U << 1) : 0U;
	m |= (HAL_GPIO_ReadPin(k->port_u, k->pin_u) == GPIO_PIN_RESET) ? (1U << 2) : 0U;
	m |= (HAL_GPIO_ReadPin(k->port_d, k->pin_d) == GPIO_PIN_RESET) ? (1U << 3) : 0U;
	m |= (HAL_GPIO_ReadPin(k->port_sel, k->pin_sel) == GPIO_PIN_RESET) ? (1U << 4) : 0U;
	return m;
}

static menu_key_t first_pressed_edge(uint8_t prev_stable, uint8_t new_stable)
{
	uint8_t rising = (uint8_t)((~prev_stable) & new_stable);
	if (rising & (1U << 0)) return MENU_KEY_L;
	if (rising & (1U << 1)) return MENU_KEY_R;
	if (rising & (1U << 2)) return MENU_KEY_U;
	if (rising & (1U << 3)) return MENU_KEY_D;
	if (rising & (1U << 4)) return MENU_KEY_SEL;
	return MENU_KEY_NONE;
}

void menu_keys_init(menu_keys_t *k, uint32_t debounce_ms)
{
	k->debounce_ms = debounce_ms;
	k->stable_mask = 0;
	k->last_raw_mask = keys_read_raw_mask(k);
	k->last_change_ms = HAL_GetTick();
	k->pending_event = MENU_KEY_NONE;
}

void menu_keys_update(menu_keys_t *k, uint32_t now_ms)
{
	uint8_t raw = keys_read_raw_mask(k);
	if (raw != k->last_raw_mask)
	{
		k->last_raw_mask = raw;
		k->last_change_ms = now_ms;
		return;
	}

	if ((now_ms - k->last_change_ms) < k->debounce_ms)
	{
		return;
	}

	if (raw != k->stable_mask)
	{
		uint8_t prev = k->stable_mask;
		k->stable_mask = raw;
		if (k->pending_event == MENU_KEY_NONE)
		{
			k->pending_event = first_pressed_edge(prev, raw);
		}
	}
}

menu_key_t menu_keys_pop_event(menu_keys_t *k)
{
	menu_key_t e = k->pending_event;
	k->pending_event = MENU_KEY_NONE;
	return e;
}

uint8_t menu_keys_get_pressed_mask(const menu_keys_t *k)
{
	return k->stable_mask;
}

