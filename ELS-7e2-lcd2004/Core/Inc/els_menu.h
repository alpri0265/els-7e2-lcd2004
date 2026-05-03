#ifndef ELS_MENU_H
#define ELS_MENU_H

#include <stdint.h>
#include <stdbool.h>

#include "lcd_hd44780.h"
#include "menu_keys.h"
#include "lcd_rus.h"
#include "els_model.h"

typedef enum
{
	ELS_MODE_FEED = 1,
	ELS_MODE_AFEED,
	ELS_MODE_THREAD,
	ELS_MODE_CONE_L,
	ELS_MODE_CONE_R,
	ELS_MODE_SPHERE,
	ELS_MODE_TACHO,
	ELS_MODE_RESERVE
} els_mode_t;

typedef enum { ELS_SUB_INT = 1, ELS_SUB_MAN, ELS_SUB_EXT } els_submode_t;

typedef struct els_menu_tag
{
	lcd_hd44780_t *lcd;
	lcd_rus_t rus;
	menu_keys_t *keys;

	/* timing (ms) to mimic Arduino KeyCycle behavior */
	uint32_t repeat_enter_ms;
	uint32_t repeat_rate_ms;

	/* key repeat state */
	uint8_t last_pressed_mask;
	uint32_t pressed_since_ms;
	uint32_t last_repeat_ms;

	/* menu state */
	els_mode_t mode;
	els_submode_t sub_thread;
	els_submode_t sub_feed;
	els_submode_t sub_afeed;
	els_submode_t sub_cone;
	els_submode_t sub_sphere; /* still uses INT/MAN/EXT in UI */

	uint8_t select_menu; /* 0..2 */

	/* variables adjusted from menu */
	int ap;               /* like Ap in sketch (0..900) */
	int pass_total;       /* 1..99 */
	int pass_nr;          /* current pass number (1..) */
	int pass_fin;         /* finish passes modifier */
	int thr_pass_summ;    /* extra passes */
	long pass_total_sphr; /* sphere passes */
	uint8_t total_tooth;  /* 1..255 */
	uint8_t current_tooth;/* 1..total_tooth */
	uint8_t thread_step;  /* index into Thread_Info (not fully ported) */
	uint8_t cone_step;    /* index into Cone_Info (not fully ported) */
	long sph_r_mm;        /* sphere radius in 0.01mm (as in sketch) */
	long bar_r_mm;        /* leg diameter/2 in 0.01mm */
	uint8_t cutter_step;
	uint8_t cutting_step;
	int enc_pos;          /* encoder position for angle display */
	uint32_t duration;    /* tachometer pulse width placeholder */

	/* ADC feed smoothing like Arduino */
	uint16_t adc_feed;      /* 0..1023 like Arduino */
	uint32_t sum_adc;
	uint16_t adc_array[16];
	uint8_t adc_idx;

	/* derived values shown on screen */
	uint16_t feed_mm;   /* сотки мм/об (MIN_FEED..MAX_FEED) */
	uint16_t afeedback_mm; /* mm/min (MIN_aFEED..MAX_aFEED) */

	/* Motor microstep counters (same as Arduino Motor_X_Pos / Motor_Z_Pos) */
	long x_pos;
	long z_pos;

	/* status flags */
	bool err_1;
	bool err_2;
	bool complete;

	/* raw switches (active-low like Arduino) */
	GPIO_TypeDef *mode_port;
	GPIO_TypeDef *submode_port;

	/* joystick (active-low, pull-up) */
	GPIO_TypeDef *joy_port;
	uint16_t joy_pin_l;
	uint16_t joy_pin_r;
	uint16_t joy_pin_u;
	uint16_t joy_pin_d;
	uint8_t joy_old_nibble; /* like Arduino Joy_Read low-nibble */
	uint32_t joy_last_change_ms;
	uint32_t joy_debounce_ms;

	/* raw submode lines (PD8..10 packed to bits 5..7); edge detect for limit INT/EXT */
	uint8_t submode_hw_prev;
	/* After failed INT/EXT (limits), treat submodes as MAN until HW=MAN (Arduino software revert) */
	bool submode_force_man;
} els_menu_t;

void els_menu_init(els_menu_t *m, lcd_hd44780_t *lcd, menu_keys_t *keys);
/* Call after mode_port / submode_port / joy pins are assigned (avoids false INT/EXT edge). */
void els_menu_pins_ready(els_menu_t *m);
void els_menu_update(els_menu_t *m, uint32_t now_ms);
void els_menu_render(els_menu_t *m);
void els_menu_set_adc_raw10(els_menu_t *m, uint16_t adc10);

#endif

