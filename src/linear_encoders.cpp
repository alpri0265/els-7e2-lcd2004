#include "linear_encoders.h"

#if ENABLE_LINEAR_ENCODERS && defined(STM32F407xx)

/* CubeMX / pinout.md */
static constexpr uint8_t PIN_LIN_Z_A = PE0;
static constexpr uint8_t PIN_LIN_Z_B = PE1;
static constexpr uint8_t PIN_LIN_X_A = PE2;
static constexpr uint8_t PIN_LIN_X_B = PE3;
static constexpr uint8_t PIN_BTN_LIN_Z_ZERO = PC13;
static constexpr uint8_t PIN_BTN_LIN_X_ZERO = PD3;

/* 1 = квадратура через EXTI (CHANGE) на A і B; 0 = polling у loop(). */
#ifndef LINEAR_ENCODERS_USE_INTERRUPTS
#define LINEAR_ENCODERS_USE_INTERRUPTS 1
#endif

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

static volatile int32_t s_cnt_z;
static volatile int32_t s_cnt_x;
static volatile uint8_t s_prev_z;
static volatile uint8_t s_prev_x;

/* (prev<<2)|curr → delta для квадратури A,B як 2-бітний цикл Грея. */
static const int8_t kQuad[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

static uint8_t readAB(uint8_t pin_a, uint8_t pin_b)
{
  const bool a = digitalRead(pin_a) == LOW;
  const bool b = digitalRead(pin_b) == LOW;
  return (uint8_t)((a ? 2u : 0u) | (b ? 1u : 0u));
}

static inline uint8_t readAB_Z_fast()
{
  const uint32_t idr = GPIOE->IDR;
  const bool a = (idr & LL_GPIO_PIN_0) == 0;
  const bool b = (idr & LL_GPIO_PIN_1) == 0;
  return (uint8_t)((a ? 2u : 0u) | (b ? 1u : 0u));
}

static inline uint8_t readAB_X_fast()
{
  const uint32_t idr = GPIOE->IDR;
  const bool a = (idr & LL_GPIO_PIN_2) == 0;
  const bool b = (idr & LL_GPIO_PIN_3) == 0;
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

template <int Invert>
static inline void isr_decode_axis(uint8_t cur, volatile uint8_t* prev, volatile int32_t* cnt)
{
  const uint8_t p = (uint8_t)(*prev);
  const uint8_t idx = (uint8_t)((p << 2) | cur);
  int8_t d = kQuad[idx];
  if (Invert) d = (int8_t)-d;
  *cnt += (int32_t)d;
  *prev = cur;
}

#if LINEAR_ENCODERS_USE_INTERRUPTS
static void linearIsrZ()
{
  const uint8_t cur = readAB_Z_fast();
#if LINEAR_ENC_INVERT_Z
  isr_decode_axis<1>(cur, &s_prev_z, &s_cnt_z);
#else
  isr_decode_axis<0>(cur, &s_prev_z, &s_cnt_z);
#endif
}

static void linearIsrX()
{
  const uint8_t cur = readAB_X_fast();
#if LINEAR_ENC_INVERT_X
  isr_decode_axis<1>(cur, &s_prev_x, &s_cnt_x);
#else
  isr_decode_axis<0>(cur, &s_prev_x, &s_cnt_x);
#endif
}
#endif

static inline void linearCriticalBegin()
{
  noInterrupts();
}

static inline void linearCriticalEnd()
{
  interrupts();
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
      linearCriticalBegin();
      *axis_cnt = 0;
      linearCriticalEnd();
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

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

  pinMode(PIN_LIN_Z_A, INPUT);
  pinMode(PIN_LIN_Z_B, INPUT);
  pinMode(PIN_LIN_X_A, INPUT);
  pinMode(PIN_LIN_X_B, INPUT);
  pinMode(PIN_BTN_LIN_Z_ZERO, INPUT_PULLUP);
  pinMode(PIN_BTN_LIN_X_ZERO, INPUT_PULLUP);

  linearCriticalBegin();
  s_cnt_z = 0;
  s_cnt_x = 0;
  s_prev_z = readAB_Z_fast();
  s_prev_x = readAB_X_fast();
  linearCriticalEnd();

#if LINEAR_ENCODERS_USE_INTERRUPTS
  attachInterrupt(digitalPinToInterrupt(PIN_LIN_Z_A), linearIsrZ, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_LIN_Z_B), linearIsrZ, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_LIN_X_A), linearIsrX, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_LIN_X_B), linearIsrX, CHANGE);
#endif
}

void linearEncodersPoll()
{
#if !LINEAR_ENCODERS_USE_INTERRUPTS
  const uint8_t cz = readAB(PIN_LIN_Z_A, PIN_LIN_Z_B);
  if (cz != s_prev_z) {
#if LINEAR_ENC_INVERT_Z
    decode_axis<1>(cz, (uint8_t*)&s_prev_z, (int32_t*)&s_cnt_z);
#else
    decode_axis<0>(cz, (uint8_t*)&s_prev_z, (int32_t*)&s_cnt_z);
#endif
  }
  const uint8_t cx = readAB(PIN_LIN_X_A, PIN_LIN_X_B);
  if (cx != s_prev_x) {
#if LINEAR_ENC_INVERT_X
    decode_axis<1>(cx, (uint8_t*)&s_prev_x, (int32_t*)&s_cnt_x);
#else
    decode_axis<0>(cx, (uint8_t*)&s_prev_x, (int32_t*)&s_cnt_x);
#endif
  }
#endif

  static uint8_t st_z = 0, st_x = 0;
  static uint32_t t_z = 0, t_x = 0;
  poll_zero(PIN_BTN_LIN_Z_ZERO, &st_z, &t_z, (int32_t*)&s_cnt_z, s_cb_z);
  poll_zero(PIN_BTN_LIN_X_ZERO, &st_x, &t_x, (int32_t*)&s_cnt_x, s_cb_x);
}

int32_t linearEncodersCountsZ()
{
  linearCriticalBegin();
  const int32_t v = s_cnt_z;
  linearCriticalEnd();
  return v;
}

int32_t linearEncodersCountsX()
{
  linearCriticalBegin();
  const int32_t v = s_cnt_x;
  linearCriticalEnd();
  return v;
}

#endif
