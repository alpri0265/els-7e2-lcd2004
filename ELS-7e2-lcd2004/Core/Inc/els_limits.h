#ifndef ELS_LIMITS_H
#define ELS_LIMITS_H

#include <stdint.h>
#include <stdbool.h>

struct els_menu_tag;

void els_limits_init(void);
void els_limits_update(struct els_menu_tag *m, uint32_t now_ms);
void els_limits_on_submode_edge(struct els_menu_tag *m, uint8_t old_hw, uint8_t new_hw);

bool els_limits_mech_stop(void);

/* Active threshold used by motion ISRs (Arduino volatile Limit_Pos) */
long els_limits_limit_pos_read(void);
void els_limits_limit_pos_write(long v);
long els_limits_limit_pos_hc_read(void);
void els_limits_limit_pos_hc_write(long v);

/* Optional guards for jog / future step generator (microsteps, +1 = increasing coordinate) */
bool els_limits_can_move_z(const struct els_menu_tag *m, int dir_plus);
bool els_limits_can_move_x(const struct els_menu_tag *m, int dir_plus);

#endif
