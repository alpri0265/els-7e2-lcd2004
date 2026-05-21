#pragma once

#include <Arduino.h>

/* Arduino-compatible async feed (aFEED INT/EXT): dynamic X/Z limits + pass counter.
 * Joystick: Menu.ino — EXT: L/R = Ext_Left/Right, U/D = Ext_Front/Ext_Rear; INT: L/R = Int_L/R. */

namespace els_afeed {

struct Hardware
{
  int32_t rebound_x;
  int32_t rebound_z;
  int32_t motor_x_steps_per_rev;
  int32_t screw_x_hundredths;
  int32_t mcstep_x;
  int32_t tol;
};

void setHardware(const Hardware& hw);

bool isBusy();

/* aFEED INT/EXT: joystick axes that start async cycles must not run manual Z/X jog (Menu.ino). */
bool joystickLeftRightReservedForAfeed(
    bool mode_is_afeed,
    uint8_t submode,
    uint16_t doc_x100,
    bool lz_on,
    bool rz_on,
    bool xf_on,
    bool xr_on);

/* aFEED EXT: Joy Up/Down starts Ext_Front/Ext_Rear — block manual X on those deflections. */
bool joystickExtUdReservedForAfeed(
    bool mode_is_afeed, uint8_t submode, bool lz_on, bool rz_on, bool xf_on, bool xr_on);

/* When true, soft X limits use dynamic infeed targets (min/max with taught envelope). */
bool adjustSoftX(int32_t taught_front, int32_t taught_rear, int32_t* cap_front, int32_t* cap_rear);

/* When true, soft Z limits use dynamic targets (aFeed_Ext_Front / aFeed_Ext_Rear). */
bool adjustSoftZ(int32_t taught_left, int32_t taught_right, int32_t* cap_left, int32_t* cap_right);

/* If returns true, caller must drive jogUpdate from *wantZ/*wantX and skip manual joystick + handwheel.
 * z_rapid_leg / x_rapid_leg: use interval_rapid for that axis (Arduino Rapid_Feed_*). */
bool driveJog(
    int32_t motor_z,
    int32_t motor_x,
    bool mode_is_afeed,
    uint8_t submode, /* SubMode: INT=0 MAN=1 EXT=2 */
    uint8_t joy,     /* 0 none, 1 L, 2 R, 3 U, 4 D */
    bool lz_on,
    bool rz_on,
    bool xf_on,
    bool xr_on,
    int32_t limit_z_left,
    int32_t limit_z_right,
    int32_t limit_x_front,
    int32_t limit_x_rear,
    uint16_t doc_x100,
    uint16_t feed_x100,
    uint8_t* pass_cur,
    uint8_t pass_total,
    uint32_t interval_feed_us,
    uint32_t interval_rapid_us,
    bool* wantZ,
    bool* zDir,
    bool* wantX,
    bool* xDir,
    bool* z_rapid_leg,
    bool* x_rapid_leg);

}  // namespace els_afeed
