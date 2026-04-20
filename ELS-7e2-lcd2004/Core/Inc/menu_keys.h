#ifndef MENU_KEYS_H
#define MENU_KEYS_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef enum
{
	MENU_KEY_NONE = 0,
	MENU_KEY_L,
	MENU_KEY_R,
	MENU_KEY_U,
	MENU_KEY_D,
	MENU_KEY_SEL,
} menu_key_t;

typedef struct
{
	GPIO_TypeDef *port_l;
	uint16_t pin_l;
	GPIO_TypeDef *port_r;
	uint16_t pin_r;
	GPIO_TypeDef *port_u;
	uint16_t pin_u;
	GPIO_TypeDef *port_d;
	uint16_t pin_d;
	GPIO_TypeDef *port_sel;
	uint16_t pin_sel;

	uint32_t debounce_ms;

	uint8_t stable_mask;
	uint8_t last_raw_mask;
	uint32_t last_change_ms;

	menu_key_t pending_event;
} menu_keys_t;

void menu_keys_init(menu_keys_t *k, uint32_t debounce_ms);
void menu_keys_update(menu_keys_t *k, uint32_t now_ms);
menu_key_t menu_keys_pop_event(menu_keys_t *k);
uint8_t menu_keys_get_pressed_mask(const menu_keys_t *k);

#endif
