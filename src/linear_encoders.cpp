#include "linear_encoders.h"

#if ENABLE_LINEAR_ENCODERS && defined(STM32F407xx)

/* CubeMX / pinout.md */
static constexpr uint8_t PIN_LIN_Z_A = PE0;
static constexpr uint8_t PIN_LIN_Z_B = PE1;
static constexpr uint8_t PIN_LIN_X_A = PE2;
static constexpr uint8_t PIN_LIN_X_B = PE3;
static constexpr uint8_t PIN_BTN_LIN_Z_ZERO = PC13;
static constexpr uint8_t PIN_BTN_LIN_X_ZERO = PD3;

/* 1 = інвертувати напрямок лічильника. */
#ifndef LINEAR_ENC_INVERT_Z
#define LINEAR_ENC_INVERT_Z 0
#endif
#ifndef LINEAR_ENC_INVERT_X
#define LINEAR_ENC_INVERT_X 0
#endif

#ifndef LINEAR_ZERO_DEBOUNCE_MS
#define LINEAR_ZERO_DEBOUNCE_MS 30u
#endif

static LinearEncZeroCallback s_cb_z;
static LinearEncZeroCallback s_cb_x;

static int32_t s_cnt_z;
static int32_t s_cnt_x;
static uint8_t s_prev_z;
static uint8_t s_prev_x;

/* (prev<<2)|curr → delta для квадратури A,B як 2-бітний цикл Грея. */
static const int8_t kQuad[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

static uint8_t readAB(uint8_t pin_a, uint8_t pin_b)
{
  const bool a = digitalRead(pin_a) == LOW;
  const bool b = digitalRead(pin_b) == LOW;
  return (uint8_t)((a ? 2u : 0u) | (b ? 1u : 0u));
}

template <int Invert>
static void decode_axis(uint8_t cur, uint8_t* prev, int32_t* cnt)
{
  const uint8_t idx = (uint8_t)((*prev << 2) | cur);
  int8_t d = kQuad[idx];
  if (Invert) d = (int8_t)-d;
  *cnt += (int32_t)d;
  *prev = cur;
}

/* Стан: 0 = відпущено, 1 = чекаємо debounce, 2 = вже спрацювали (тримають). */
static void poll_zero(uint8_t pin, uint8_t* state, uint32_t* t0, int32_t* axis_cnt, LinearEncZeroCallback cb)
{
  const bool low = (digitalRead(pin) == LOW);
  const uint32_t now = millis();
  if (low) {
    if (*state == 0) {
      *t0 = now;
      *state = 1;
    } else if (*state == 1 && (now - *t0) >= LINEAR_ZERO_DEBOUNCE_MS) {
      *axis_cnt = 0;
      if (cb) cb();
      *state = 2;
    }
  } else {
    *state = 0;
  }
}

void linearEncodersInit(LinearEncZeroCallback on_zero_z, LinearEncZeroCallback on_zero_x)
{
  s_cb_z = on_zero_z;
  s_cb_x = on_zero_x;
  pinMode(PIN_LIN_Z_A, INPUT_PULLUP);
  pinMode(PIN_LIN_Z_B, INPUT_PULLUP);
  pinMode(PIN_LIN_X_A, INPUT_PULLUP);
  pinMode(PIN_LIN_X_B, INPUT_PULLUP);
  pinMode(PIN_BTN_LIN_Z_ZERO, INPUT_PULLUP);
  pinMode(PIN_BTN_LIN_X_ZERO, INPUT_PULLUP);
  s_prev_z = readAB(PIN_LIN_Z_A, PIN_LIN_Z_B);
  s_prev_x = readAB(PIN_LIN_X_A, PIN_LIN_X_B);
}

void linearEncodersPoll()
{
  const uint8_t cz = readAB(PIN_LIN_Z_A, PIN_LIN_Z_B);
  if (cz != s_prev_z) {
#if LINEAR_ENC_INVERT_Z
    decode_axis<1>(cz, &s_prev_z, &s_cnt_z);
#else
    decode_axis<0>(cz, &s_prev_z, &s_cnt_z);
#endif
  }
  const uint8_t cx = readAB(PIN_LIN_X_A, PIN_LIN_X_B);
  if (cx != s_prev_x) {
#if LINEAR_ENC_INVERT_X
    decode_axis<1>(cx, &s_prev_x, &s_cnt_x);
#else
    decode_axis<0>(cx, &s_prev_x, &s_cnt_x);
#endif
  }

  static uint8_t st_z = 0, st_x = 0;
  static uint32_t t_z = 0, t_x = 0;
  poll_zero(PIN_BTN_LIN_Z_ZERO, &st_z, &t_z, &s_cnt_z, s_cb_z);
  poll_zero(PIN_BTN_LIN_X_ZERO, &st_x, &t_x, &s_cnt_x, s_cb_x);
}

int32_t linearEncodersCountsZ()
{
  return s_cnt_z;
}

int32_t linearEncodersCountsX()
{
  return s_cnt_x;
}

#endif
