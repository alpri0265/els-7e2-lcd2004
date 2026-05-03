#include "els_afeed.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace els_afeed {

namespace {

constexpr int32_t kReboundX = 1500;
constexpr int32_t kReboundZ = 1500; /* 7e2_Mod...ino REBOUND_Z */
constexpr int32_t kMotorXStepPerRev = 300;
constexpr int32_t kScrewXHundredths = 150;
constexpr int32_t kMcStepX = 4;
constexpr int32_t kTol = 56; /* allow teach / rounding vs exact Arduino == positions */

enum Var : uint8_t { VNone = 0, VExtL, VExtR, VIntL, VIntR, VExtUp, VExtDn };

enum Ph : uint8_t {
  PIdle = 0,
  PXf,
  PZl,
  PXr,
  PZrRap,
  PZf,
  PZr,
  PZrRapL,
  PZrRapR,
  EU_ZtoL,
  EU_XtoF,
  EU_ZtoR,
  EU_XrapR,
  ED_ZtoL,
  ED_XtoR,
  ED_ZtoR2,
  ED_XrapF
};

static Var s_var = VNone;
static Ph s_ph = PIdle;
static int32_t s_null_x = 0;
static int32_t s_null_z = 0;
static int32_t s_w_infeed = 0;
static int32_t s_pass_x_target = 0; /* dynamic X stop for current leg */
static int32_t s_pass_z_arm = 0;    /* dynamic Z "left" stop: aFeed_Ext_Front/Rear arm */
static int32_t s_z_inner = 0;       /* Limit_Pos_Right := Limit_Pos_Left - REBOUND_Z */
static int32_t s_dyn_cap_f = INT32_MAX;
static int32_t s_dyn_cap_r = INT32_MIN;
static int32_t s_dyn_z_l = INT32_MIN;
static int32_t s_dyn_z_r = INT32_MAX;
static bool s_soft_overlay = false;
static bool s_soft_z_overlay = false;
static uint8_t s_prev_joy_u = 0;

static int32_t infeed_u(uint16_t doc_x100)
{
  const float ap = (float)doc_x100;
  return (int32_t)lroundf((float)kMotorXStepPerRev * ap / (float)kScrewXHundredths * (float)kMcStepX);
}

static bool near_eq(int32_t a, int32_t b)
{
  const int32_t d = a - b;
  return d <= kTol && d >= -kTol;
}

/* Return-X target must stay inside taught X window; otherwise soft stop hits tr
 * while the state machine waits for an unreachable eff_r (no return motion). */
static int32_t clamp_x_target_to_limits(int32_t v, int32_t tr, int32_t tf)
{
  return std::min(tf, std::max(tr, v));
}

static int32_t clamp_span(int32_t v, int32_t a, int32_t b)
{
  const int32_t lo = std::min(a, b);
  const int32_t hi = std::max(a, b);
  return std::min(hi, std::max(lo, v));
}

/* Step Z toward target (handles either encoder sense). */
static void want_z_toward(int32_t mz, int32_t target, bool* wantZ, bool* zDir)
{
  if (near_eq(mz, target)) return;
  if (mz < target - kTol) {
    *wantZ = true;
    *zDir = true;
  } else if (mz > target + kTol) {
    *wantZ = true;
    *zDir = false;
  }
}

static void machine_reset()
{
  s_var = VNone;
  s_ph = PIdle;
  s_w_infeed = 0;
  s_pass_x_target = 0;
  s_pass_z_arm = 0;
  s_z_inner = 0;
  s_dyn_cap_f = INT32_MAX;
  s_dyn_cap_r = INT32_MIN;
  s_dyn_z_l = INT32_MIN;
  s_dyn_z_r = INT32_MAX;
  s_soft_overlay = false;
  s_soft_z_overlay = false;
  s_prev_joy_u = 0;
}

static Var pick_var(uint8_t sub, uint8_t joy)
{
  if (sub == 2) {
    if (joy == 1) return VExtL;
    if (joy == 2) return VExtR;
    if (joy == 3) return VExtUp;
    if (joy == 4) return VExtDn;
    return VNone;
  }
  if (sub == 0 && (joy == 1 || joy == 2)) return joy == 1 ? VIntL : VIntR;
  return VNone;
}

static void arm_null(int32_t mx, uint8_t* pass_cur)
{
  if (s_null_x == 0) s_null_x = mx;
  if (*pass_cur == 0) *pass_cur = 1;
}

/* ========== Ext Left: Z right / X rear start ========== */
static bool tick_ext_l(
    int32_t mz,
    int32_t mx,
    int32_t lz,
    int32_t rz,
    int32_t xf,
    int32_t xr,
    uint16_t doc_x100,
    uint8_t* pass_cur,
    uint8_t pass_total,
    bool* wantZ,
    bool* zDir,
    bool* wantX,
    bool* xDir,
    bool* z_rapid,
    bool* x_rapid)
{
  *z_rapid = false;
  *x_rapid = false;
  const int32_t tf = xf;
  const int32_t tr = xr;

  if (s_ph == PIdle) {
    s_dyn_cap_f = tf;
    s_dyn_cap_r = tr;
    if (!(near_eq(mz, rz) && near_eq(mx, tr))) return false;
    if (*pass_cur > pass_total) {
      *pass_cur = 1;
      machine_reset();
      return false;
    }
    arm_null(mx, pass_cur);
    s_w_infeed = infeed_u(doc_x100);
    if (s_w_infeed == 0) {
      s_pass_x_target = (*pass_cur == 1) ? (s_null_x + 1) : (tr + kReboundX);
    } else {
      s_pass_x_target =
          (*pass_cur == 1) ? (s_null_x + s_w_infeed) : (tr + kReboundX + s_w_infeed * (int32_t)(*pass_cur));
    }
    s_pass_x_target = std::min(s_pass_x_target, tf);
    s_dyn_cap_f = std::min(tf, s_pass_x_target);
    s_dyn_cap_r = tr;
    s_soft_overlay = true;
    s_ph = PXf;
  }

  if (s_ph == PXf) {
    s_dyn_cap_f = std::min(tf, s_pass_x_target);
    s_dyn_cap_r = tr;
    if (mx >= s_pass_x_target - kTol) {
      s_ph = PZl;
    } else {
      *wantX = true;
      *xDir = true;
      return true;
    }
  }

  if (s_ph == PZl) {
    s_dyn_cap_f = tf;
    s_dyn_cap_r = tr;
    if (mz <= lz + kTol) {
      const int32_t eff_r = clamp_x_target_to_limits(
          s_pass_x_target - kReboundX - s_w_infeed * (int32_t)(*pass_cur), tr, tf);
      s_dyn_cap_r = std::max(tr, eff_r);
      s_dyn_cap_f = tf;
      s_ph = PXr;
    } else {
      *wantZ = true;
      *zDir = false;
      return true;
    }
  }

  if (s_ph == PXr) {
    const int32_t eff_r = clamp_x_target_to_limits(
        s_pass_x_target - kReboundX - s_w_infeed * (int32_t)(*pass_cur), tr, tf);
    s_dyn_cap_r = std::max(tr, eff_r);
    s_dyn_cap_f = tf;
    if (mx <= eff_r + kTol) {
      if (*pass_cur < 255) (*pass_cur)++;
      s_ph = PZrRap;
    } else {
      *wantX = true;
      *xDir = false;
      return true;
    }
  }

  if (s_ph == PZrRap) {
    s_dyn_cap_f = tf;
    s_dyn_cap_r = tr;
    if (mz >= rz - kTol) {
      s_ph = PIdle;
      s_soft_overlay = false;
      return false;
    }
    *wantZ = true;
    *zDir = true;
    *z_rapid = true;
    return true;
  }

  return false;
}

/* ========== Ext Right: Z left / X rear start ========== */
static bool tick_ext_r(
    int32_t mz,
    int32_t mx,
    int32_t lz,
    int32_t rz,
    int32_t xf,
    int32_t xr,
    uint16_t doc_x100,
    uint8_t* pass_cur,
    uint8_t pass_total,
    bool* wantZ,
    bool* zDir,
    bool* wantX,
    bool* xDir,
    bool* z_rapid,
    bool* x_rapid)
{
  *z_rapid = false;
  *x_rapid = false;
  const int32_t tf = xf;
  const int32_t tr = xr;

  if (s_ph == PIdle) {
    s_dyn_cap_f = tf;
    s_dyn_cap_r = tr;
    if (!(near_eq(mz, lz) && near_eq(mx, tr))) return false;
    if (*pass_cur > pass_total) {
      *pass_cur = 1;
      machine_reset();
      return false;
    }
    arm_null(mx, pass_cur);
    s_w_infeed = infeed_u(doc_x100);
    if (s_w_infeed == 0) {
      s_pass_x_target = (*pass_cur == 1) ? (s_null_x + 1) : (tr + kReboundX);
    } else {
      s_pass_x_target =
          (*pass_cur == 1) ? (s_null_x + s_w_infeed) : (tr + kReboundX + s_w_infeed * (int32_t)(*pass_cur));
    }
    s_pass_x_target = std::min(s_pass_x_target, tf);
    s_dyn_cap_f = std::min(tf, s_pass_x_target);
    s_dyn_cap_r = tr;
    s_soft_overlay = true;
    s_ph = PXf;
  }

  if (s_ph == PXf) {
    s_dyn_cap_f = std::min(tf, s_pass_x_target);
    s_dyn_cap_r = tr;
    if (mx >= s_pass_x_target - kTol) {
      s_ph = PZf; /* Z toward right */
    } else {
      *wantX = true;
      *xDir = true;
      return true;
    }
  }

  if (s_ph == PZf) {
    s_dyn_cap_f = tf;
    s_dyn_cap_r = tr;
    if (mz >= rz - kTol) {
      const int32_t eff_r = clamp_x_target_to_limits(
          s_pass_x_target - kReboundX - s_w_infeed * (int32_t)(*pass_cur), tr, tf);
      s_dyn_cap_r = std::max(tr, eff_r);
      s_dyn_cap_f = tf;
      s_ph = PXr;
    } else {
      *wantZ = true;
      *zDir = true;
      return true;
    }
  }

  if (s_ph == PXr) {
    const int32_t eff_r = clamp_x_target_to_limits(
        s_pass_x_target - kReboundX - s_w_infeed * (int32_t)(*pass_cur), tr, tf);
    s_dyn_cap_r = std::max(tr, eff_r);
    s_dyn_cap_f = tf;
    if (mx <= eff_r + kTol) {
      if (*pass_cur < 255) (*pass_cur)++;
      s_ph = PZrRapL;
    } else {
      *wantX = true;
      *xDir = false;
      return true;
    }
  }

  if (s_ph == PZrRapL) {
    s_dyn_cap_f = tf;
    s_dyn_cap_r = tr;
    if (mz <= lz + kTol) {
      s_ph = PIdle;
      s_soft_overlay = false;
      return false;
    }
    *wantZ = true;
    *zDir = false;
    *z_rapid = true;
    return true;
  }

  return false;
}

/* ========== Int Left: Z right / X front (or null) start ========== */
static bool tick_int_l(
    int32_t mz,
    int32_t mx,
    int32_t lz,
    int32_t rz,
    int32_t xf,
    int32_t xr,
    uint16_t doc_x100,
    uint8_t* pass_cur,
    uint8_t pass_total,
    bool* wantZ,
    bool* zDir,
    bool* wantX,
    bool* xDir,
    bool* z_rapid,
    bool* x_rapid)
{
  *z_rapid = false;
  *x_rapid = false;
  const int32_t tf = xf;
  const int32_t tr = xr;

  if (s_ph == PIdle) {
    s_dyn_cap_f = tf;
    s_dyn_cap_r = tr;
    const bool at_front = near_eq(mx, xf);
    const bool at_null = (s_null_x != 0) && near_eq(mx, s_null_x);
    if (!(near_eq(mz, rz) && (at_front || at_null))) return false;
    if (*pass_cur > pass_total) {
      *pass_cur = 1;
      machine_reset();
      return false;
    }
    arm_null(mx, pass_cur);
    s_w_infeed = infeed_u(doc_x100);
    int32_t eff_r;
    if (s_w_infeed == 0) {
      eff_r = (*pass_cur == 1) ? (s_null_x - 1) : (tf - kReboundX);
    } else {
      eff_r = (*pass_cur == 1) ? (s_null_x - s_w_infeed)
                               : (tf - kReboundX - s_w_infeed * (int32_t)(*pass_cur));
    }
    eff_r = std::max(tr, eff_r);
    s_pass_x_target = eff_r;
    s_dyn_cap_r = eff_r;
    s_dyn_cap_f = tf;
    s_soft_overlay = true;
    s_ph = PXr; /* first leg: X toward rear */
  }

  if (s_ph == PXr) {
    s_dyn_cap_r = std::max(tr, s_pass_x_target);
    s_dyn_cap_f = tf;
    if (mx <= s_pass_x_target + kTol) {
      s_ph = PZl;
    } else {
      *wantX = true;
      *xDir = false;
      return true;
    }
  }

  if (s_ph == PZl) {
    s_dyn_cap_f = tf;
    s_dyn_cap_r = tr;
    if (mz <= lz + kTol) {
      s_pass_x_target = tr + kReboundX + s_w_infeed * (int32_t)(*pass_cur);
      s_pass_x_target = std::min(s_pass_x_target, tf);
      s_dyn_cap_f = std::min(tf, s_pass_x_target);
      s_dyn_cap_r = tr;
      s_ph = PXf;
    } else {
      *wantZ = true;
      *zDir = false;
      return true;
    }
  }

  if (s_ph == PXf) {
    s_dyn_cap_f = std::min(tf, s_pass_x_target);
    s_dyn_cap_r = tr;
    if (mx >= s_pass_x_target - kTol) {
      if (*pass_cur < 255) (*pass_cur)++;
      s_ph = PZrRap;
    } else {
      *wantX = true;
      *xDir = true;
      return true;
    }
  }

  if (s_ph == PZrRap) {
    s_dyn_cap_f = tf;
    s_dyn_cap_r = tr;
    if (mz >= rz - kTol) {
      s_ph = PIdle;
      s_soft_overlay = false;
      return false;
    }
    *wantZ = true;
    *zDir = true;
    *z_rapid = true;
    return true;
  }

  return false;
}

/* ========== Int Right: Z left / X front start ========== */
static bool tick_int_r(
    int32_t mz,
    int32_t mx,
    int32_t lz,
    int32_t rz,
    int32_t xf,
    int32_t xr,
    uint16_t doc_x100,
    uint8_t* pass_cur,
    uint8_t pass_total,
    bool* wantZ,
    bool* zDir,
    bool* wantX,
    bool* xDir,
    bool* z_rapid,
    bool* x_rapid)
{
  *z_rapid = false;
  *x_rapid = false;
  const int32_t tf = xf;
  const int32_t tr = xr;

  if (s_ph == PIdle) {
    s_dyn_cap_f = tf;
    s_dyn_cap_r = tr;
    const bool at_front = near_eq(mx, xf);
    const bool at_null = (s_null_x != 0) && near_eq(mx, s_null_x);
    if (!(near_eq(mz, lz) && (at_front || at_null))) return false;
    if (*pass_cur > pass_total) {
      *pass_cur = 1;
      machine_reset();
      return false;
    }
    arm_null(mx, pass_cur);
    s_w_infeed = infeed_u(doc_x100);
    int32_t eff_r;
    if (s_w_infeed == 0) {
      eff_r = (*pass_cur == 1) ? (s_null_x - 1) : (tf - kReboundX);
    } else {
      eff_r = (*pass_cur == 1) ? (s_null_x - s_w_infeed)
                               : (tf - kReboundX - s_w_infeed * (int32_t)(*pass_cur));
    }
    eff_r = std::max(tr, eff_r);
    s_pass_x_target = eff_r;
    s_dyn_cap_r = eff_r;
    s_dyn_cap_f = tf;
    s_soft_overlay = true;
    s_ph = PXr;
  }

  if (s_ph == PXr) {
    s_dyn_cap_r = std::max(tr, s_pass_x_target);
    s_dyn_cap_f = tf;
    if (mx <= s_pass_x_target + kTol) {
      s_ph = PZf;
    } else {
      *wantX = true;
      *xDir = false;
      return true;
    }
  }

  if (s_ph == PZf) {
    s_dyn_cap_f = tf;
    s_dyn_cap_r = tr;
    if (mz >= rz - kTol) {
      s_pass_x_target = tr + kReboundX + s_w_infeed * (int32_t)(*pass_cur);
      s_pass_x_target = std::min(s_pass_x_target, tf);
      s_dyn_cap_f = std::min(tf, s_pass_x_target);
      s_dyn_cap_r = tr;
      s_ph = PXf;
    } else {
      *wantZ = true;
      *zDir = true;
      return true;
    }
  }

  if (s_ph == PXf) {
    s_dyn_cap_f = std::min(tf, s_pass_x_target);
    s_dyn_cap_r = tr;
    if (mx >= s_pass_x_target - kTol) {
      if (*pass_cur < 255) (*pass_cur)++;
      s_ph = PZrRapL;
    } else {
      *wantX = true;
      *xDir = true;
      return true;
    }
  }

  if (s_ph == PZrRapL) {
    s_dyn_cap_f = tf;
    s_dyn_cap_r = tr;
    if (mz <= lz + kTol) {
      s_ph = PIdle;
      s_soft_overlay = false;
      return false;
    }
    *wantZ = true;
    *zDir = false;
    *z_rapid = true;
    return true;
  }

  return false;
}

/* ========== Ext "Front" (Menu Joy Up / aFeed_Ext_Front): X rear, Z right or Null_Z ========== */
static bool tick_ext_joy_up(
    int32_t mz,
    int32_t mx,
    int32_t lz,
    int32_t rz,
    int32_t xf,
    int32_t xr,
    uint16_t doc_x100,
    uint8_t* pass_cur,
    uint8_t pass_total,
    bool* wantZ,
    bool* zDir,
    bool* wantX,
    bool* xDir,
    bool* z_rapid,
    bool* x_rapid)
{
  *z_rapid = false;
  *x_rapid = false;
  const int32_t tf = xf;
  const int32_t tr = xr;

  if (s_ph == PIdle) {
    s_dyn_cap_f = tf;
    s_dyn_cap_r = tr;
    const bool at_rz = near_eq(mz, rz);
    const bool at_nz = (s_null_z != 0) && near_eq(mz, s_null_z);
    if (!(near_eq(mx, tr) && (at_rz || at_nz))) return false;
    if (*pass_cur > pass_total) {
      *pass_cur = 1;
      machine_reset();
      return false;
    }
    arm_null(mx, pass_cur);
    s_w_infeed = infeed_u(doc_x100);
    int32_t z_stop;
    if (s_w_infeed == 0) {
      z_stop = (*pass_cur == 1) ? ((s_null_z != 0) ? (s_null_z + 1) : (rz + 1)) : (rz + kReboundZ);
    } else {
      z_stop = (*pass_cur == 1) ? ((s_null_z != 0) ? (s_null_z + s_w_infeed) : (rz + s_w_infeed))
                                : (rz + kReboundZ + s_w_infeed);
    }
    s_pass_z_arm = clamp_span(z_stop, lz, rz);
    s_soft_z_overlay = true;
    s_dyn_z_l = s_pass_z_arm;
    s_dyn_z_r = std::max(lz, rz);
    s_ph = EU_ZtoL;
  }

  if (s_ph == EU_ZtoL) {
    s_dyn_z_l = s_pass_z_arm;
    s_dyn_z_r = std::max(lz, rz);
    s_soft_z_overlay = true;
    if (near_eq(mz, s_pass_z_arm)) {
      s_soft_z_overlay = false;
      s_dyn_z_l = INT32_MIN;
      s_dyn_z_r = INT32_MAX;
      s_ph = EU_XtoF;
    } else {
      want_z_toward(mz, s_pass_z_arm, wantZ, zDir);
      return true;
    }
  }

  if (s_ph == EU_XtoF) {
    s_soft_z_overlay = false;
    if (mx >= tf - kTol) {
      s_z_inner = clamp_span(s_pass_z_arm - kReboundZ, lz, rz);
      s_soft_z_overlay = true;
      s_dyn_z_l = std::min(lz, rz);
      s_dyn_z_r = s_z_inner;
      s_ph = EU_ZtoR;
    } else {
      *wantX = true;
      *xDir = true;
      return true;
    }
  }

  if (s_ph == EU_ZtoR) {
    s_dyn_z_l = std::min(lz, rz);
    s_dyn_z_r = s_z_inner;
    s_soft_z_overlay = true;
    if (near_eq(mz, s_z_inner)) {
      s_soft_z_overlay = false;
      s_dyn_z_l = INT32_MIN;
      s_dyn_z_r = INT32_MAX;
      s_ph = EU_XrapR;
    } else {
      want_z_toward(mz, s_z_inner, wantZ, zDir);
      return true;
    }
  }

  if (s_ph == EU_XrapR) {
    s_soft_z_overlay = false;
    if (mx <= tr + kTol) {
      if (*pass_cur < 255) (*pass_cur)++;
      s_ph = PIdle;
      return false;
    }
    *wantX = true;
    *xDir = false;
    *x_rapid = true;
    return true;
  }

  return false;
}

/* ========== Ext "Rear" (Menu Joy Down / aFeed_Ext_Rear): X front, Z right or Null_Z ========== */
static bool tick_ext_joy_dn(
    int32_t mz,
    int32_t mx,
    int32_t lz,
    int32_t rz,
    int32_t xf,
    int32_t xr,
    uint16_t doc_x100,
    uint8_t* pass_cur,
    uint8_t pass_total,
    bool* wantZ,
    bool* zDir,
    bool* wantX,
    bool* xDir,
    bool* z_rapid,
    bool* x_rapid)
{
  *z_rapid = false;
  *x_rapid = false;
  const int32_t tf = xf;
  const int32_t tr = xr;

  if (s_ph == PIdle) {
    s_dyn_cap_f = tf;
    s_dyn_cap_r = tr;
    const bool at_rz = near_eq(mz, rz);
    const bool at_nz = (s_null_z != 0) && near_eq(mz, s_null_z);
    if (!(near_eq(mx, tf) && (at_rz || at_nz))) return false;
    if (*pass_cur > pass_total) {
      *pass_cur = 1;
      machine_reset();
      return false;
    }
    arm_null(mx, pass_cur);
    s_w_infeed = infeed_u(doc_x100);
    int32_t z_stop;
    if (s_w_infeed == 0) {
      z_stop = (*pass_cur == 1) ? ((s_null_z != 0) ? (s_null_z + 1) : (rz + 1)) : (rz + kReboundZ);
    } else {
      z_stop = (*pass_cur == 1) ? ((s_null_z != 0) ? (s_null_z + s_w_infeed) : (rz + s_w_infeed))
                                : (rz + kReboundZ + s_w_infeed);
    }
    s_pass_z_arm = clamp_span(z_stop, lz, rz);
    s_soft_z_overlay = true;
    s_dyn_z_l = s_pass_z_arm;
    s_dyn_z_r = std::max(lz, rz);
    s_ph = ED_ZtoL;
  }

  if (s_ph == ED_ZtoL) {
    s_dyn_z_l = s_pass_z_arm;
    s_dyn_z_r = std::max(lz, rz);
    s_soft_z_overlay = true;
    if (near_eq(mz, s_pass_z_arm)) {
      s_soft_z_overlay = false;
      s_dyn_z_l = INT32_MIN;
      s_dyn_z_r = INT32_MAX;
      s_ph = ED_XtoR;
    } else {
      want_z_toward(mz, s_pass_z_arm, wantZ, zDir);
      return true;
    }
  }

  if (s_ph == ED_XtoR) {
    s_soft_z_overlay = false;
    if (mx <= tr + kTol) {
      s_z_inner = clamp_span(s_pass_z_arm - kReboundZ, lz, rz);
      s_soft_z_overlay = true;
      s_dyn_z_l = std::min(lz, rz);
      s_dyn_z_r = s_z_inner;
      s_ph = ED_ZtoR2;
    } else {
      *wantX = true;
      *xDir = false;
      return true;
    }
  }

  if (s_ph == ED_ZtoR2) {
    s_dyn_z_l = std::min(lz, rz);
    s_dyn_z_r = s_z_inner;
    s_soft_z_overlay = true;
    if (near_eq(mz, s_z_inner)) {
      s_soft_z_overlay = false;
      s_dyn_z_l = INT32_MIN;
      s_dyn_z_r = INT32_MAX;
      s_ph = ED_XrapF;
    } else {
      want_z_toward(mz, s_z_inner, wantZ, zDir);
      return true;
    }
  }

  if (s_ph == ED_XrapF) {
    s_soft_z_overlay = false;
    if (mx >= tf - kTol) {
      if (*pass_cur < 255) (*pass_cur)++;
      s_ph = PIdle;
      return false;
    }
    *wantX = true;
    *xDir = true;
    *x_rapid = true;
    return true;
  }

  return false;
}

}  // namespace

bool isBusy()
{
  return s_ph != PIdle || s_soft_overlay || s_soft_z_overlay;
}

bool joystickLeftRightReservedForAfeed(
    bool mode_is_afeed,
    uint8_t submode,
    uint16_t doc_x100,
    bool lz_on,
    bool rz_on,
    bool xf_on,
    bool xr_on)
{
  (void)doc_x100;
  return mode_is_afeed && submode != 1 && lz_on && rz_on && xf_on && xr_on;
}

bool joystickExtUdReservedForAfeed(
    bool mode_is_afeed, uint8_t submode, bool lz_on, bool rz_on, bool xf_on, bool xr_on)
{
  return mode_is_afeed && submode == 2 && lz_on && rz_on && xf_on && xr_on;
}

bool adjustSoftX(int32_t taught_front, int32_t taught_rear, int32_t* cap_front, int32_t* cap_rear)
{
  if (s_ph == PIdle && !s_soft_overlay) return false;
  *cap_front = std::min(taught_front, s_dyn_cap_f);
  *cap_rear = std::max(taught_rear, s_dyn_cap_r);
  return true;
}

bool adjustSoftZ(int32_t taught_left, int32_t taught_right, int32_t* cap_left, int32_t* cap_right)
{
  if (!s_soft_z_overlay) return false;
  *cap_left = std::max(taught_left, s_dyn_z_l);
  *cap_right = std::min(taught_right, s_dyn_z_r);
  return true;
}

bool driveJog(
    int32_t motor_z,
    int32_t motor_x,
    bool mode_is_afeed,
    uint8_t submode,
    uint8_t joy,
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
    bool* x_rapid_leg)
{
  (void)interval_feed_us;
  (void)interval_rapid_us;
  (void)feed_x100;
  *wantZ = *wantX = false;
  *zDir = *xDir = false;
  *z_rapid_leg = false;
  *x_rapid_leg = false;

  if (!mode_is_afeed || submode == 1) {
    machine_reset();
    return false;
  }

  if (!lz_on || !rz_on || !xf_on || !xr_on) {
    machine_reset();
    return false;
  }

  /* Joystick neutral at rest: snap Null_X / Null_Z like Joy_NoPressed (edge only) */
  if (joy == 0 && s_ph == PIdle) {
    if (s_prev_joy_u != 0) {
      s_null_x = motor_x;
      s_null_z = motor_z;
    }
    if (*pass_cur == 0 || *pass_cur > pass_total) *pass_cur = 1;
    s_var = VNone;
  }

  if (s_ph == PIdle && joy != 0) {
    const Var v = pick_var(submode, joy);
    if (v != VNone) s_var = v;
  }

  s_prev_joy_u = joy;

  if (s_var == VNone) return false;

  switch (s_var) {
  case VExtL:
    return tick_ext_l(motor_z, motor_x, limit_z_left, limit_z_right, limit_x_front, limit_x_rear, doc_x100,
                       pass_cur, pass_total, wantZ, zDir, wantX, xDir, z_rapid_leg, x_rapid_leg);
  case VExtR:
    return tick_ext_r(motor_z, motor_x, limit_z_left, limit_z_right, limit_x_front, limit_x_rear, doc_x100,
                       pass_cur, pass_total, wantZ, zDir, wantX, xDir, z_rapid_leg, x_rapid_leg);
  case VIntL:
    return tick_int_l(motor_z, motor_x, limit_z_left, limit_z_right, limit_x_front, limit_x_rear, doc_x100,
                       pass_cur, pass_total, wantZ, zDir, wantX, xDir, z_rapid_leg, x_rapid_leg);
  case VIntR:
    return tick_int_r(motor_z, motor_x, limit_z_left, limit_z_right, limit_x_front, limit_x_rear, doc_x100,
                       pass_cur, pass_total, wantZ, zDir, wantX, xDir, z_rapid_leg, x_rapid_leg);
  case VExtUp:
    return tick_ext_joy_up(motor_z, motor_x, limit_z_left, limit_z_right, limit_x_front, limit_x_rear, doc_x100,
                           pass_cur, pass_total, wantZ, zDir, wantX, xDir, z_rapid_leg, x_rapid_leg);
  case VExtDn:
    return tick_ext_joy_dn(motor_z, motor_x, limit_z_left, limit_z_right, limit_x_front, limit_x_rear, doc_x100,
                           pass_cur, pass_total, wantZ, zDir, wantX, xDir, z_rapid_leg, x_rapid_leg);
  default:
    machine_reset();
    return false;
  }
}

}  // namespace els_afeed
