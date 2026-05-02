#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#if defined(STM32F407xx)
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"
#endif

// ---------------------------------------------------------------------------
// CNC Menu (minimal) adapted to LCD2004 via I2C (PCF8574)
// STM32F407ZE + PlatformIO/Arduino
//
// LCD: 20x4, address 0x27
// I2C pins: SDA=PB7, SCL=PB6
//
// Buttons (existing project wiring): INPUT_PULLUP, active LOW
// ---------------------------------------------------------------------------

static constexpr uint8_t I2C_ADDR = 0x27;
static constexpr uint8_t I2C_COLS = 20;
static constexpr uint8_t I2C_ROWS = 4;

LiquidCrystal_I2C lcd(I2C_ADDR, I2C_COLS, I2C_ROWS);

// I2C pins
static constexpr uint8_t I2C_SCL_PIN = PB6;
static constexpr uint8_t I2C_SDA_PIN = PB7;

// Buttons (from earlier project)
static constexpr uint8_t BTN_LEFT  = PB8;
static constexpr uint8_t BTN_RIGHT = PB9;
static constexpr uint8_t BTN_UP    = PB10;
static constexpr uint8_t BTN_DOWN  = PB11;
static constexpr uint8_t BTN_SEL   = PB12;

// Joystick (CubeMX wiring): active LOW with pull-ups
static constexpr uint8_t JOY_L_PIN = PF12;
static constexpr uint8_t JOY_R_PIN = PF13;
static constexpr uint8_t JOY_U_PIN = PF14;
static constexpr uint8_t JOY_D_PIN = PF15;

// Rapid button (CubeMX wiring): active LOW with pull-up
static constexpr uint8_t BTN_RAPID_PIN = PD7;

// Optional: STOP / motion inhibit (CubeMX label `BTN_STOP` = PB13), active LOW.
// Leave disabled unless your panel actually wires this pin as a normally-open switch to GND.
// If enabled while the pin is accidentally held LOW, motion will never start.
#define ENABLE_HW_MOTION_STOP 0
#if ENABLE_HW_MOTION_STOP
static constexpr uint8_t BTN_STOP_PIN = PB13;
#endif

// Feed potentiometer (CubeMX wiring): ADC on PF6
static constexpr uint8_t ADC_FEED_PIN = PF6;

static constexpr uint8_t BEEPER_PIN = PD0;

// Hand wheel: TIM4 quadrature on PD12 (CH1) + PD13 (CH2). After handEncoderHwInit() do not pinMode these.
static constexpr uint8_t ENC_HC_A_PIN = PD12;
static constexpr uint8_t ENC_HC_B_PIN = PD13;
static constexpr uint8_t AXIS_Z_PIN   = PD11;
static constexpr uint8_t AXIS_X_PIN   = PD15;
static constexpr uint8_t SCALE_X100_PIN = PD4;
static constexpr uint8_t SCALE_X1_PIN   = PD5;
static constexpr uint8_t SCALE_X10_PIN  = PD6;

// If X moves opposite to Z for the same encoder rotation, set to 1.
static constexpr bool HAND_ENCODER_INVERT_X = false;

// Stepper driver outputs (CubeMX wiring)
static constexpr uint8_t Z_STEP_PIN = PC0;
static constexpr uint8_t Z_DIR_PIN  = PC1;
static constexpr uint8_t Z_EN_PIN   = PC2;
static constexpr uint8_t X_STEP_PIN = PC3;
static constexpr uint8_t X_DIR_PIN  = PC4;
static constexpr uint8_t X_EN_PIN   = PC5;

// Driver polarity (for DM556D via 74HCT245 -> DM556D PUL-/DIR-/ENA- with PUL+/DIR+/ENA+ tied to +5V)
// In that wiring, sinking current (LOW) typically activates the opto input => ACTIVE_LOW=true.
static constexpr bool STEP_ACTIVE_LOW = true;
static constexpr bool DIR_ACTIVE_LOW  = true;
static constexpr bool EN_ACTIVE_LOW   = true;

// DM556 "ENA" polarity/wiring is the #1 footgun: if firmware drives ENA incorrectly,
// the driver can remain disabled forever -> joystick UI moves but motors don't.
//
// Default: do NOT toggle EN from MCU during jog. Idle level keeps the ENA opto *inactive* (GPIO HIGH
// with EN_ACTIVE_LOW), matching boards that default-enable when ENA is not sunk — same as leaving ENA unwired.
//
// Set to 1 only if you are sure your ENA wiring matches EN_ACTIVE_LOW semantics.
#define STEPPER_DRIVE_ENABLE_PIN 0

// Debounce
static constexpr uint32_t DEBOUNCE_MS = 25;
static constexpr uint32_t LONGPRESS_MS = 650;
static constexpr uint32_t KEY_REPEAT_DELAY_MS = 450;
static constexpr uint32_t KEY_REPEAT_RATE_MS = 120;
static constexpr uint16_t BEEP_MS = 25;

// Joystick debounce for motion (prevents "stuck direction" from a single noisy edge)
static constexpr uint32_t JOY_DEBOUNCE_MS = 5;
static constexpr uint8_t JOY_STABLE_SAMPLES = 2;

enum Key : uint8_t
{
  KEY_SEL = 0,
  KEY_UP,
  KEY_DOWN,
  KEY_LEFT,
  KEY_RIGHT
};

enum Mode : uint8_t
{
  MODE_FEED = 0,
  MODE_AFEED,
  MODE_THREAD,
  MODE_CONE,
  MODE_SPHERE,
  MODE_TACHO,
  MODE_RESERVE,
  MODE_COUNT
};

enum SubMode : uint8_t
{
  SUB_INT = 0,
  SUB_MAN = 1,
  SUB_EXT = 2
};

// State shown on screen
static Mode current_mode = MODE_FEED;
static SubMode submode_per_mode[MODE_COUNT] = {
  SUB_INT,  // FEED
  SUB_MAN,  // AFEED
  SUB_MAN,  // THREAD
  SUB_MAN,  // CONE
  SUB_MAN,  // SPHERE
  SUB_MAN,  // TACHO
  SUB_MAN   // RESERVE
};

static uint16_t feed_x100 = 25;   // 0.25 mm/rev (stored as *100)
static uint8_t pass_total = 10;   // TOTAL passes
static uint8_t pass_cur = 1;      // Current pass number (1..pass_total)
static uint16_t doc_x100 = 50;    // 0.50 mm (stored as *100)
static bool rapid_enabled = false; // "RAPID active now" (momentary like in main)

// Joystick "manual move" flags (ported from main semantics)
static bool joy_z_active = false;
static bool joy_x_active = false;

// ADC smoothing for potentiometer
static uint16_t adc_ring[16] = {0};
static uint8_t adc_ring_idx = 0;
static uint32_t adc_sum = 0;
static bool pot_locked = false;

// Selected menu row (0..3). Marker is shown on the right edge.
static uint8_t selected_row = 0;
static bool in_submenu = false;

static char last_rows[4][21] = {{0}};
static bool cache_valid = false;

// Raw switch/key state (kept for internal decode/robustness)
static uint8_t last_pd8_10 = 0x07; // bits: 0=PD8,1=PD9,2=PD10 (1=HIGH)
static uint8_t last_keys_reading = 0;

static bool beeper_on = false;
static uint32_t beeper_until_ms = 0;

// Hand wheel: live values for LCD (TIM4 counter + axis/scale switches)
enum class HandAxisSel : uint8_t { None, Z, X };
static HandAxisSel g_hand_axis_disp = HandAxisSel::None;
static int16_t g_hand_cnt_disp = 0;
static uint16_t g_hand_scale_mult_disp = 1;

static const char* modeName(Mode m)
{
  switch (m) {
    case MODE_FEED:    return "FEED";
    case MODE_AFEED:   return "aFEED";
    case MODE_THREAD:  return "THREAD";
    case MODE_CONE:    return "CONE";
    case MODE_SPHERE:  return "SPHERE";
    case MODE_TACHO:   return "TACHO";
    case MODE_RESERVE: return "RESERVE";
    default:           return "UNK";
  }
}

static const char* subModeName(SubMode s)
{
  switch (s) {
    case SUB_INT: return "INT";
    case SUB_MAN: return "MAN";
    case SUB_EXT: return "EXT";
    default:      return "UNK";
  }
}

// Mode/Submode hardware switches (CubeMX: MODE_D0..D7 = PG0..7, SUBMODE_0..2 = PD8..10)
static constexpr uint8_t MODE_PINS[8] = {PG0, PG1, PG2, PG3, PG4, PG5, PG6, PG7};
static constexpr uint8_t SUBMODE_PINS[3] = {PD8, PD9, PD10};

static uint8_t readModeByte()
{
  // Build a byte matching CubeMX IDR bits: bit i = pin level (1=HIGH, 0=LOW)
  uint8_t b = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (digitalRead(MODE_PINS[i]) == HIGH) b |= (1u << i);
  }
  return b;
}

static uint8_t readSubmodeBits()
{
  // Match els_menu.c behavior: map PD8..10 into bits 5..7
  uint8_t b = 0;
  if (digitalRead(SUBMODE_PINS[0]) == HIGH) b |= (1u << 5);
  if (digitalRead(SUBMODE_PINS[1]) == HIGH) b |= (1u << 6);
  if (digitalRead(SUBMODE_PINS[2]) == HIGH) b |= (1u << 7);
  return b;
}

static bool decodeModeFromByte(uint8_t modeByte, Mode& out)
{
  // Active-low patterns copied from CubeMX/HAL project (els_menu.c)
  switch (modeByte) {
    case 0x7F: out = MODE_FEED;    return true; // bit7 low
    case 0xBF: out = MODE_AFEED;   return true; // bit6 low
    case 0xF7: out = MODE_THREAD;  return true; // bit3 low
    case 0xDF: out = MODE_CONE;    return true; // cone L/R collapsed into one mode here
    case 0xEF: out = MODE_CONE;    return true; // cone R
    case 0xFB: out = MODE_SPHERE;  return true; // bit2 low
    case 0xFD: out = MODE_TACHO;   return true; // bit1 low
    case 0xFE: out = MODE_RESERVE; return true; // bit0 low
    default:   return false;
  }
}

static SubMode decodeSubmodeFromRawPd(uint8_t pd8_10_highmask)
{
  // Robust decode for the 3-position SUBMODE switch.
  // With INPUT_PULLUP: non-selected lines read HIGH, selected line is pulled LOW.
  //
  // We decode by "which line is LOW". If multiple go LOW (wiring fault),
  // priority order below is deterministic.
  const bool pd8_high  = (pd8_10_highmask & 0x01U) != 0;
  const bool pd9_high  = (pd8_10_highmask & 0x02U) != 0;
  const bool pd10_high = (pd8_10_highmask & 0x04U) != 0;

  // Current observed states from hardware test:
  // - D:011 (PD8 LOW) is one position of the switch.
  // If your wiring is different, swap these mappings.
  if (!pd8_high) return SUB_MAN;
  if (!pd9_high) return SUB_EXT;
  if (!pd10_high) return SUB_INT;
  return SUB_INT; // open position (D:111)
}

static void applySelectionMarker(uint8_t row, char s[21])
{
  // Cursor must be on the left side (col 0).
  // Shift content right by 1 and put marker at [0].
  // Keep total width at 20 chars.
  memmove(&s[1], &s[0], 19);
  s[0] = (row == selected_row) ? '>' : ' ';
  s[20] = '\0';
}

static void makeRow0(char out[21])
{
  // "MODE: FEED SYNC"
  // Show RAPID + manual-axis flags in the last columns (keeps layout stable)
  // Layout: "MODE:xxxxxxx yyy <axis><R>"
  const char axis = joy_z_active ? 'Z' : (joy_x_active ? 'X' : ' ');
  snprintf(out, 21, "MODE:%-7s %-3s %c%c",
           modeName(current_mode),
           subModeName(submode_per_mode[current_mode]),
           axis,
           rapid_enabled ? 'R' : ' ');
}

static void makeRow1(char out[21])
{
  switch (current_mode) {
    case MODE_FEED:
      snprintf(out, 21, "FEED: %u.%02u mm/rev", feed_x100 / 100, feed_x100 % 100);
      break;
    case MODE_AFEED:
      snprintf(out, 21, "aFEED:%u.%02u mm/rev", feed_x100 / 100, feed_x100 % 100);
      break;
    case MODE_THREAD:
      snprintf(out, 21, "STEP: %u.%02u mm", feed_x100 / 100, feed_x100 % 100);
      break;
    case MODE_CONE:
      snprintf(out, 21, "CONE: %u.%02u mm", feed_x100 / 100, feed_x100 % 100);
      break;
    case MODE_SPHERE:
      snprintf(out, 21, "R:    %u.%02u mm", feed_x100 / 100, feed_x100 % 100);
      break;
    case MODE_TACHO:
      snprintf(out, 21, "RPM:  %u", (unsigned)(feed_x100 * 10));
      break;
    case MODE_RESERVE:
      snprintf(out, 21, "RES:  %u", (unsigned)(feed_x100));
      break;
    default:
      snprintf(out, 21, " ");
      break;
  }
}

static void makeRow2(char out[21])
{
  // Show current/total passes
  snprintf(out, 21, "PASS: %u/%u", pass_cur, pass_total);
}

static void makeRow3(char out[21])
{
  char axc = '.';
  if (g_hand_axis_disp == HandAxisSel::Z) axc = 'Z';
  else if (g_hand_axis_disp == HandAxisSel::X) axc = 'X';
  snprintf(out, 21, "DOC%u.%02u %c%3u %5d",
           (unsigned)(doc_x100 / 100),
           (unsigned)(doc_x100 % 100),
           axc,
           (unsigned)g_hand_scale_mult_disp,
           (int)g_hand_cnt_disp);
}

static void makeSubmenuRow0(char out[21])
{
  snprintf(out, 21, "SET:%-12s", modeName(current_mode));
}

static void makeSubmenuRow1(char out[21])
{
  snprintf(out, 21, "SUB:%-4s", subModeName(submode_per_mode[current_mode]));
}

static void makeSubmenuRow2(char out[21])
{
  snprintf(out, 21, "PASS: %u/%u", pass_cur, pass_total);
}

static void makeSubmenuRow3(char out[21])
{
  snprintf(out, 21, "DOC:  %u.%02u mm", (unsigned)(doc_x100 / 100), (unsigned)(doc_x100 % 100));
}

static void padTo20(char s[21])
{
  const size_t n = strnlen(s, 20);
  for (size_t i = n; i < 20; i++) s[i] = ' ';
  s[20] = '\0';
}

static void writeRowDiff(uint8_t row, const char current[21])
{
  if (!cache_valid) {
    lcd.setCursor(0, row);
    lcd.print(current);
    strncpy(last_rows[row], current, 21);
    return;
  }

  // Update only changed segments
  const char* prev = last_rows[row];
  uint8_t col = 0;
  while (col < 20) {
    // Skip equal chars
    while (col < 20 && prev[col] == current[col]) col++;
    if (col >= 20) break;

    // Find run of changed chars
    const uint8_t start = col;
    while (col < 20 && prev[col] != current[col]) col++;
    const uint8_t end = col; // [start, end)

    lcd.setCursor(start, row);
    for (uint8_t i = start; i < end; i++) lcd.print(current[i]);
  }

  strncpy(last_rows[row], current, 21);
}

static void updateDisplay()
{
  char rows[4][21];

  if (!in_submenu) {
    makeRow0(rows[0]); padTo20(rows[0]); applySelectionMarker(0, rows[0]);
    makeRow1(rows[1]); padTo20(rows[1]); applySelectionMarker(1, rows[1]);
    makeRow2(rows[2]); padTo20(rows[2]); applySelectionMarker(2, rows[2]);
    makeRow3(rows[3]); padTo20(rows[3]); applySelectionMarker(3, rows[3]);
  } else {
    makeSubmenuRow0(rows[0]); padTo20(rows[0]); applySelectionMarker(0, rows[0]);
    makeSubmenuRow1(rows[1]); padTo20(rows[1]); applySelectionMarker(1, rows[1]);
    makeSubmenuRow2(rows[2]); padTo20(rows[2]); applySelectionMarker(2, rows[2]);
    makeSubmenuRow3(rows[3]); padTo20(rows[3]); applySelectionMarker(3, rows[3]);
  }

  // If whole rows are unchanged, do nothing
  if (cache_valid &&
      strncmp(rows[0], last_rows[0], 20) == 0 &&
      strncmp(rows[1], last_rows[1], 20) == 0 &&
      strncmp(rows[2], last_rows[2], 20) == 0 &&
      strncmp(rows[3], last_rows[3], 20) == 0) {
    return;
  }

  for (uint8_t r = 0; r < 4; r++) writeRowDiff(r, rows[r]);
  cache_valid = true;
}

static uint8_t readKeysBitmask()
{
  uint8_t m = 0;
  if (digitalRead(BTN_SEL) == LOW)   m |= (1u << KEY_SEL);
  if (digitalRead(BTN_UP) == LOW)    m |= (1u << KEY_UP);
  if (digitalRead(BTN_DOWN) == LOW)  m |= (1u << KEY_DOWN);
  if (digitalRead(BTN_LEFT) == LOW)  m |= (1u << KEY_LEFT);
  if (digitalRead(BTN_RIGHT) == LOW) m |= (1u << KEY_RIGHT);

  // Joystick is intentionally NOT mapped to menu keys here.
  // Otherwise moving the joystick would move the on-screen menu cursor (KEY_UP/DOWN/LEFT/RIGHT).
  // Axis jogging uses PF12..PF15 via debouncedJoystickDir() / updateStepperJog().
  return m;
}

// ---------------------------------------------------------------------------
// Joystick logic (ported from main):
// - Active LOW with pull-ups.
// - Directions map to axis activity:
//   LEFT/RIGHT => Z axis active
//   UP/DOWN    => X axis active
// - Neutral clears both axis flags.
// - While joystick axis active, mode/submode switches are ignored.
// ---------------------------------------------------------------------------
enum class JoyDir : uint8_t { None, Left, Right, Up, Down };

static JoyDir joy_dir = JoyDir::None;

static JoyDir readJoystickDir()
{
  // Equivalent to main's "one line low" scheme.
  // If multiple are pressed, deterministic priority is used.
  const bool l = (digitalRead(JOY_L_PIN) == LOW);
  const bool r = (digitalRead(JOY_R_PIN) == LOW);
  const bool u = (digitalRead(JOY_U_PIN) == LOW);
  const bool d = (digitalRead(JOY_D_PIN) == LOW);

  if (l) return JoyDir::Left;
  if (r) return JoyDir::Right;
  if (u) return JoyDir::Up;
  if (d) return JoyDir::Down;
  return JoyDir::None;
}

static JoyDir debouncedJoystickDir()
{
  static JoyDir stable = JoyDir::None;
  static JoyDir candidate = JoyDir::None;
  static uint8_t stableCount = 0;
  static uint32_t lastChangeMs = 0;

  const uint32_t now = millis();
  const JoyDir raw = readJoystickDir();

  if (raw != candidate) {
    candidate = raw;
    stableCount = 1;
    lastChangeMs = now;
    return stable;
  }

  // same as candidate
  if ((now - lastChangeMs) < JOY_DEBOUNCE_MS) return stable;
  if (stableCount < 255) stableCount++;

  if (stableCount >= JOY_STABLE_SAMPLES && stable != candidate) {
    stable = candidate;
  }
  return stable;
}

static void joyNoPressed()
{
  if (!joy_z_active && !joy_x_active) return;
  joy_z_active = false;
  joy_x_active = false;
  cache_valid = false;
  updateDisplay();
}

static void joyLeftPressed()
{
  joy_x_active = false;
  joy_z_active = true;
  cache_valid = false;
  updateDisplay();
}

static void joyRightPressed()
{
  joy_x_active = false;
  joy_z_active = true;
  cache_valid = false;
  updateDisplay();
}

static void joyUpPressed()
{
  joy_z_active = false;
  joy_x_active = true;
  cache_valid = false;
  updateDisplay();
}

static void joyDownPressed()
{
  joy_z_active = false;
  joy_x_active = true;
  cache_valid = false;
  updateDisplay();
}

static void updateJoystickMainStyle()
{
  static JoyDir last = JoyDir::None;
  const JoyDir cur = debouncedJoystickDir();
  if (cur == last) return;
  last = cur;

  switch (cur) {
    case JoyDir::Left:  joyLeftPressed();  break;
    case JoyDir::Right: joyRightPressed(); break;
    case JoyDir::Up:    joyUpPressed();    break;
    case JoyDir::Down:  joyDownPressed();  break;
    case JoyDir::None:  joyNoPressed();    break;
  }
}

// ---------------------------------------------------------------------------
// Stepper pulse generator (jog)
// - Non-blocking, driven from loop() via micros().
// - Generates STEP pulses while joystick is held.
// - SPEED is derived from feed_x100 and RAPID modifier.
// ---------------------------------------------------------------------------
struct StepperJog
{
  uint8_t stepPin;
  uint8_t dirPin;
  uint8_t enPin;

  bool enabled = false;
  bool dir = false;

  bool stepIsActive = false;
  uint32_t stepOffAtUs = 0;
  uint32_t nextStepAtUs = 0;
  bool dirSetupPending = false;
  uint32_t dirSetupUntilUs = 0;
};

static StepperJog jogZ {Z_STEP_PIN, Z_DIR_PIN, Z_EN_PIN};
static StepperJog jogX {X_STEP_PIN, X_DIR_PIN, X_EN_PIN};

static inline void writePolarityPin(uint8_t pin, bool logicalHigh, bool activeLow)
{
  digitalWrite(pin, (logicalHigh ^ activeLow) ? HIGH : LOW);
}

static void stepperInitPins()
{
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Z_EN_PIN, OUTPUT);
  pinMode(X_EN_PIN, OUTPUT);

  // Default: STEP/DIR inactive.
  // EN pins stay driven as outputs so the 74HCT245 input isn't floating.
  //
  // Many DM556 boards default to "motor enabled" when the ENA opto is *off* (no sink on ENA-).
  // Holding ENA- active (LOW through the buffer) can latch fault / stay disabled on some units —
  // same symptom as "disconnect ENA from MCU and it runs".
  //
  // - STEPPER_DRIVE_ENABLE_PIN=0: hold EN *inactive* (release opto) so the driver behaves like
  //   unconnected ENA logic — not the same as "logical enable=true" on the GPIO.
  // - STEPPER_DRIVE_ENABLE_PIN=1: same idle level at boot; jogSetEnabled() toggles EN during motion.
  writePolarityPin(Z_EN_PIN, false, EN_ACTIVE_LOW);
  writePolarityPin(X_EN_PIN, false, EN_ACTIVE_LOW);
  writePolarityPin(Z_STEP_PIN, false, STEP_ACTIVE_LOW);
  writePolarityPin(X_STEP_PIN, false, STEP_ACTIVE_LOW);
}

static uint32_t feedToStepsPerSecond(uint16_t fx100, bool rapid)
{
  // Simple mapping for manual jog:
  // fx100 (5..999) -> base speed (80..2500 steps/s)
  const uint32_t minSps = 80;
  const uint32_t maxSps = 2500;
  const uint32_t sps = minSps + (uint32_t)(maxSps - minSps) * (uint32_t)fx100 / 999U;
  return rapid ? (sps * 3U) : sps;
}

static void jogSetEnabled(StepperJog& j, bool en)
{
  if (j.enabled == en) return;
  const bool was = j.enabled;
  j.enabled = en;
#if STEPPER_DRIVE_ENABLE_PIN
  writePolarityPin(j.enPin, en, EN_ACTIVE_LOW);
#endif
  if (!en) {
    j.stepIsActive = false;
    writePolarityPin(j.stepPin, false, STEP_ACTIVE_LOW);
  } else if (!was) {
    // Fresh start: don't inherit stale scheduling from a previous jog session.
    j.stepIsActive = false;
    j.nextStepAtUs = micros();
    j.dirSetupPending = false;
    j.dirSetupUntilUs = 0;
  }
}

static void jogSetDir(StepperJog& j, bool dirLogical)
{
  if (j.dir == dirLogical) return;
  j.dir = dirLogical;
  writePolarityPin(j.dirPin, dirLogical, DIR_ACTIVE_LOW);
}

static void jogUpdate(StepperJog& j, bool wantMove, bool dirLogical, uint32_t stepIntervalUs)
{
  const uint32_t now = micros();

  if (!wantMove || stepIntervalUs == 0) {
    jogSetEnabled(j, false);
    return;
  }

  jogSetEnabled(j, true);
  const bool dirChanged = (j.dir != dirLogical);
  jogSetDir(j, dirLogical);
  if (dirChanged) {
    // DM556 family typically wants >=5us DIR setup before a PUL active edge.
    static constexpr uint32_t DIR_SETUP_US = 6;
    j.dirSetupPending = true;
    j.dirSetupUntilUs = now + DIR_SETUP_US;
    // Also restart step scheduling after a DIR change.
    j.stepIsActive = false;
    j.nextStepAtUs = now;
  }

  // STEP pulse width (active) for opto inputs: keep >= 5us.
  static constexpr uint32_t PULSE_US = 6;

  // Turn STEP off when pulse time elapses
  if (j.stepIsActive && (int32_t)(now - j.stepOffAtUs) >= 0) {
    j.stepIsActive = false;
    writePolarityPin(j.stepPin, false, STEP_ACTIVE_LOW);
  }

  // Schedule new step
  if (!j.stepIsActive && (int32_t)(now - j.nextStepAtUs) >= 0) {
    // Important: do not `return` early from this function — we must always finish
    // the STEP "off" phase above; otherwise the line can get stuck active and
    // no further pulses will be generated.
    if (!j.dirSetupPending || (int32_t)(now - j.dirSetupUntilUs) >= 0) {
      if (j.dirSetupPending) j.dirSetupPending = false;
      j.stepIsActive = true;
      writePolarityPin(j.stepPin, true, STEP_ACTIVE_LOW);
      j.stepOffAtUs = now + PULSE_US;
      j.nextStepAtUs = now + stepIntervalUs;
    }
  }
}

static bool motionInhibited()
{
#if !ENABLE_HW_MOTION_STOP
  return false;
#else
  static uint8_t last = HIGH;
  static uint32_t lastChangeMs = 0;

  const uint32_t now = millis();
  const uint8_t cur = digitalRead(BTN_STOP_PIN); // active LOW

  if (cur != last) {
    last = cur;
    lastChangeMs = now;
  }
  if ((now - lastChangeMs) < DEBOUNCE_MS) {
    return digitalRead(BTN_STOP_PIN) == LOW;
  }

  return cur == LOW;
#endif
}

// ---------------------------------------------------------------------------
// Hand encoder (TIM4 quadrature on PD12/PD13) + axis / scale switches
// Joystick has priority: hand wheel is ignored while the joystick is off-center.
// ---------------------------------------------------------------------------
#if defined(STM32F407xx)
static uint16_t s_hand_enc_last = 0;

static void handEncoderHwInit()
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

  // LL needs GPIO pin bit masks, not Arduino pin numbers.
  LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_12 | LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_12, LL_GPIO_AF_2);
  LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_13, LL_GPIO_AF_2);
  LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_12 | LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_12 | LL_GPIO_PIN_13, LL_GPIO_PULL_UP);

  LL_TIM_InitTypeDef tim = {};
  tim.Prescaler = 0;
  tim.CounterMode = LL_TIM_COUNTERMODE_UP;
  tim.Autoreload = 0xFFFF;
  tim.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &tim);

  LL_TIM_ENCODER_InitTypeDef enc = {};
  enc.EncoderMode = LL_TIM_ENCODERMODE_X4_TI12;
  enc.IC1Polarity = LL_TIM_IC_POLARITY_RISING;
  enc.IC1ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;
  enc.IC1Prescaler = LL_TIM_ICPSC_DIV1;
  enc.IC1Filter = 0x6;
  enc.IC2Polarity = LL_TIM_IC_POLARITY_RISING;
  enc.IC2ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;
  enc.IC2Prescaler = LL_TIM_ICPSC_DIV1;
  enc.IC2Filter = 0x6;
  LL_TIM_ENCODER_Init(TIM4, &enc);

  LL_TIM_SetCounter(TIM4, 0);
  LL_TIM_EnableCounter(TIM4);
  s_hand_enc_last = 0;
}

static int16_t handEncoderReadDelta()
{
  const uint16_t c = LL_TIM_GetCounter(TIM4);
  const int16_t d = (int16_t)(c - s_hand_enc_last);
  s_hand_enc_last = c;
  return d;
}
#else
static void handEncoderHwInit() {}
static int16_t handEncoderReadDelta() { return 0; }
#endif

static HandAxisSel readHandAxisDebounced()
{
  static HandAxisSel stable = HandAxisSel::None;
  static HandAxisSel candidate = HandAxisSel::None;
  static uint32_t tStable = 0;

  HandAxisSel raw = HandAxisSel::None;
  const bool z = (digitalRead(AXIS_Z_PIN) == LOW);
  const bool x = (digitalRead(AXIS_X_PIN) == LOW);
  if (z && !x) raw = HandAxisSel::Z;
  else if (x && !z) raw = HandAxisSel::X;

  const uint32_t m = millis();
  if (raw != candidate) {
    candidate = raw;
    tStable = m;
  } else if ((m - tStable) >= 20 && candidate != stable) {
    stable = candidate;
  }
  return stable;
}

// 3-position switch: exactly one of PD4/PD5/PD6 is LOW (×100 / ×1 / ×10).
// If none or more than one LOW (transition/bad wiring), default to ×1.
static uint16_t readHandScaleMultiplier()
{
  const bool s100 = (digitalRead(SCALE_X100_PIN) == LOW);
  const bool s1 = (digitalRead(SCALE_X1_PIN) == LOW);
  const bool s10 = (digitalRead(SCALE_X10_PIN) == LOW);
  const int n = (s100 ? 1 : 0) + (s1 ? 1 : 0) + (s10 ? 1 : 0);
  if (n != 1) return 1;
  if (s100) return 100;
  if (s10) return 10;
  return 1;
}

static void handPulseOne(StepperJog& j, bool dirPlus)
{
  jogSetDir(j, dirPlus);
  delayMicroseconds(6);
  writePolarityPin(j.stepPin, true, STEP_ACTIVE_LOW);
  delayMicroseconds(6);
  writePolarityPin(j.stepPin, false, STEP_ACTIVE_LOW);
}

static void updateHandWheelJog()
{
  static int32_t hand_q_steps = 0;
  static HandAxisSel hand_q_axis = HandAxisSel::None;
  static uint32_t hand_next_us = 0;

  if (motionInhibited()) {
    hand_q_steps = 0;
    hand_q_axis = HandAxisSel::None;
    hand_next_us = 0;
    return;
  }

  if (joy_dir != JoyDir::None) {
    hand_q_steps = 0;
    hand_q_axis = HandAxisSel::None;
    hand_next_us = 0;
    return;
  }

  const HandAxisSel ax = readHandAxisDebounced();
  if (ax == HandAxisSel::None) {
    hand_q_steps = 0;
    hand_q_axis = HandAxisSel::None;
    hand_next_us = 0;
    return;
  }

  int16_t d = handEncoderReadDelta();
  if (HAND_ENCODER_INVERT_X && ax == HandAxisSel::X) d = (int16_t)-d;
  if (d != 0) {
    const int mult = (int)readHandScaleMultiplier();
    const int32_t add = (int32_t)d * mult;
    if (hand_q_axis != HandAxisSel::None && hand_q_axis != ax) {
      hand_q_steps = 0;
    }
    hand_q_axis = ax;
    hand_q_steps += add;
  }

  if (hand_q_steps == 0) {
    hand_next_us = 0;
    return;
  }

  // Min. interval from *real* time after each pulse. Slower at ×10 / ×100 so the motor
  // keeps up (burst "catch-up" with the old scheduler could fire dozens of steps ~20µs
  // apart → stall / lost steps).
  // Hand-wheel step period (one step per loop when due). ×100 still below burst rates
  // that caused stalls; tune here if you need more speed vs reliability.
  auto handPeriodUs = []() -> uint32_t {
    switch (readHandScaleMultiplier()) {
      case 100: return 700;
      case 10:  return 600;
      default:  return 500;
    }
  };

  const uint32_t now = micros();
  if (hand_next_us == 0) hand_next_us = now;
  if ((int32_t)(now - hand_next_us) < 0) return;

  const bool forward = hand_q_steps > 0;
  StepperJog& j = (hand_q_axis == HandAxisSel::Z) ? jogZ : jogX;
  handPulseOne(j, forward);
  hand_q_steps += forward ? -1 : 1;
  hand_next_us = micros() + handPeriodUs();
}

static void handEncoderUiSnapshot()
{
#if defined(STM32F407xx)
  g_hand_cnt_disp = (int16_t)LL_TIM_GetCounter(TIM4);
#else
  g_hand_cnt_disp = 0;
#endif
  g_hand_axis_disp = readHandAxisDebounced();
  g_hand_scale_mult_disp = readHandScaleMultiplier();

  static int16_t s_last_cnt = 0x7fff;
  static HandAxisSel s_last_ax = HandAxisSel::None;
  static uint16_t s_last_sc = 0;
  if (g_hand_cnt_disp != s_last_cnt || g_hand_axis_disp != s_last_ax || g_hand_scale_mult_disp != s_last_sc) {
    s_last_cnt = g_hand_cnt_disp;
    s_last_ax = g_hand_axis_disp;
    s_last_sc = g_hand_scale_mult_disp;
    cache_valid = false;
  }
}

static void updateStepperJog()
{
  const uint32_t sps = feedToStepsPerSecond(feed_x100, rapid_enabled);
  const uint32_t intervalUs = (sps == 0) ? 0 : (1000000UL / sps);

  // Keep motion direction in sync with debounced joystick reading.
  // NOTE: updateJoystickMainStyle() only runs on *changes*; stepper must use the latest stable direction.
  joy_dir = debouncedJoystickDir();

  // Decide which axis/direction is requested by joystick
  bool wantZ = false, wantX = false;
  bool zDir = false, xDir = false;

  if (motionInhibited()) {
    wantZ = false;
    wantX = false;
  }

  switch (joy_dir) {
    case JoyDir::Left:  wantZ = true; zDir = false; break;
    case JoyDir::Right: wantZ = true; zDir = true;  break;
    case JoyDir::Up:    wantX = true; xDir = true;  break;
    case JoyDir::Down:  wantX = true; xDir = false; break;
    case JoyDir::None:  default: break;
  }

  jogUpdate(jogZ, wantZ, zDir, intervalUs);
  jogUpdate(jogX, wantX, xDir, intervalUs);

  updateHandWheelJog();
}

static void beeperInit()
{
  pinMode(BEEPER_PIN, OUTPUT);
  digitalWrite(BEEPER_PIN, LOW);
  beeper_on = false;
  beeper_until_ms = 0;
}

static void beeperTrigger(uint16_t duration_ms)
{
  const uint32_t now = millis();
  beeper_until_ms = now + duration_ms;
  if (!beeper_on) {
    beeper_on = true;
    digitalWrite(BEEPER_PIN, HIGH);
  }
}

static void updateBeeper()
{
  if (!beeper_on) return;
  const uint32_t now = millis();
  if ((int32_t)(now - beeper_until_ms) >= 0) {
    beeper_on = false;
    digitalWrite(BEEPER_PIN, LOW);
  }
}

static void applyKeyPress(uint8_t key, bool isRepeat = false)
{
  switch (key) {
    case KEY_SEL:
      // Short press SEL: toggle submode on row0 (main screen) or row1 (submenu)
      if (!in_submenu) {
        if (selected_row == 0) {
          switch (submode_per_mode[current_mode]) {
            case SUB_INT: submode_per_mode[current_mode] = SUB_MAN; break;
            case SUB_MAN: submode_per_mode[current_mode] = SUB_EXT; break;
            case SUB_EXT: submode_per_mode[current_mode] = SUB_INT; break;
            default:      submode_per_mode[current_mode] = SUB_INT; break;
          }
        }
      } else {
        if (selected_row == 1) {
          switch (submode_per_mode[current_mode]) {
            case SUB_INT: submode_per_mode[current_mode] = SUB_MAN; break;
            case SUB_MAN: submode_per_mode[current_mode] = SUB_EXT; break;
            case SUB_EXT: submode_per_mode[current_mode] = SUB_INT; break;
            default:      submode_per_mode[current_mode] = SUB_INT; break;
          }
        }
      }
      break;
    case KEY_UP:
      // Move selection up
      selected_row = (selected_row == 0) ? 3 : (uint8_t)(selected_row - 1);
      break;
    case KEY_DOWN:
      // Move selection down
      selected_row = (selected_row == 3) ? 0 : (uint8_t)(selected_row + 1);
      break;
    case KEY_LEFT:
      if (!in_submenu) {
        // On main screen: adjust selected values (mode comes from hardware switch)
        if (selected_row == 1) {
          if (feed_x100 > 5) feed_x100 -= 5;
          pot_locked = true;
        } else if (selected_row == 2) {
          if (pass_total > 1) pass_total -= 1;
          if (pass_cur > pass_total) pass_cur = pass_total;
        } else if (selected_row == 3) {
          if (doc_x100 > 5) doc_x100 -= 5;
        }
      } else {
        // Submenu: adjust selected value only
        if (selected_row == 2 && pass_total > 1) { pass_total -= 1; if (pass_cur > pass_total) pass_cur = pass_total; }
        if (selected_row == 3 && doc_x100 > 5) doc_x100 -= 5;
      }
      break;
    case KEY_RIGHT:
      if (!in_submenu) {
        if (selected_row == 1) {
          if (feed_x100 < 999) feed_x100 += 5;
          pot_locked = true;
        } else if (selected_row == 2) {
          if (pass_total < 99) pass_total += 1;
        } else if (selected_row == 3) {
          if (doc_x100 < 999) doc_x100 += 5;
        }
      } else {
        if (selected_row == 2 && pass_total < 99) pass_total += 1;
        if (selected_row == 3 && doc_x100 < 999) doc_x100 += 5;
      }
      break;
    default:
      break;
  }
  if (!isRepeat) beeperTrigger(BEEP_MS);
  updateDisplay();
}

static void updateButtonsDebounced()
{
  // Debounce all keys as a single bitmask, but make SEL long-press use raw-held time.
  static uint8_t rawLast = 0;
  static uint8_t stable = 0;
  static uint32_t lastRawChangeMs = 0;

  static uint32_t selRawDownMs = 0;
  static bool selLongFired = false;

  const uint32_t now = millis();
  const uint8_t raw = readKeysBitmask();
  last_keys_reading = raw;

  if (raw != rawLast) {
    rawLast = raw;
    lastRawChangeMs = now;
  }

  // SEL long-press based on continuous RAW hold (avoids debounce corner cases)
  const bool selRawHeld = (raw & (1u << KEY_SEL)) != 0;
  if (selRawHeld) {
    if (selRawDownMs == 0) {
      selRawDownMs = now;
      selLongFired = false;
    } else if (!selLongFired && (now - selRawDownMs) >= LONGPRESS_MS) {
      in_submenu = !in_submenu;
      selected_row = in_submenu ? 1 : 0;
      cache_valid = false;
      updateDisplay();
      selLongFired = true;
    }
  } else {
    // On SEL release: if no long-press, treat as short press
    if (selRawDownMs != 0 && !selLongFired) applyKeyPress(KEY_SEL);
    selRawDownMs = 0;
    selLongFired = false;
  }

  static int8_t repeat_key = -1;
  static uint32_t repeat_next_ms = 0;

  // Debounce for other keys (and for stable-state debug)
  const uint8_t prevStable = stable;
  if ((now - lastRawChangeMs) >= DEBOUNCE_MS) stable = raw;
  const uint8_t pressed = stable & ~prevStable;

  auto selectRepeatCandidate = [](uint8_t mask) -> int8_t {
    if (mask & (1u << KEY_UP)) return KEY_UP;
    if (mask & (1u << KEY_DOWN)) return KEY_DOWN;
    if (mask & (1u << KEY_LEFT)) return KEY_LEFT;
    if (mask & (1u << KEY_RIGHT)) return KEY_RIGHT;
    return -1;
  };

  if (pressed & (1u << KEY_UP))    applyKeyPress(KEY_UP, false);
  if (pressed & (1u << KEY_DOWN))  applyKeyPress(KEY_DOWN, false);
  if (pressed & (1u << KEY_LEFT))  applyKeyPress(KEY_LEFT, false);
  if (pressed & (1u << KEY_RIGHT)) applyKeyPress(KEY_RIGHT, false);

  const int8_t cand = selectRepeatCandidate(stable);
  if (cand != -1) {
    if (repeat_key != cand) {
      repeat_key = cand;
      repeat_next_ms = now + KEY_REPEAT_DELAY_MS;
    } else if ((int32_t)(now - repeat_next_ms) >= 0) {
      applyKeyPress((uint8_t)repeat_key, true);
      repeat_next_ms = now + KEY_REPEAT_RATE_MS;
    }
  } else {
    repeat_key = -1;
  }
}

static void updateRapidButton()
{
  // Ported from main semantics: RAPID is a modifier while button is held.
  static uint8_t last = HIGH;
  static uint32_t lastChangeMs = 0;
  const uint32_t now = millis();
  const uint8_t cur = digitalRead(BTN_RAPID_PIN); // active LOW

  if (cur != last) {
    last = cur;
    lastChangeMs = now;
  }
  if ((now - lastChangeMs) < DEBOUNCE_MS) return;

  const bool newRapid = (cur == LOW);
  if (newRapid != rapid_enabled) {
    rapid_enabled = newRapid;
    cache_valid = false;
    updateDisplay();
  }
}

static uint16_t mapAdcToFeedX100(uint16_t adc12)
{
  // Map 0..4095 to 0.05..9.99 (x100 => 5..999)
  const uint16_t minv = 5;
  const uint16_t maxv = 999;
  return (uint16_t)(minv + (uint32_t)(maxv - minv) * adc12 / 4095U);
}

static void updateFeedFromPot()
{
  // Read ADC, smooth, then update feed_x100 when change is meaningful
  const uint16_t raw = analogRead(ADC_FEED_PIN); // STM32 core typically returns 0..4095

  // init ring on first call
  static bool adc_inited = false;
  if (!adc_inited) {
    adc_sum = 0;
    for (uint8_t i = 0; i < 16; i++) { adc_ring[i] = raw; adc_sum += raw; }
    adc_ring_idx = 0;
    adc_inited = true;
  }

  adc_sum -= adc_ring[adc_ring_idx];
  adc_ring[adc_ring_idx] = raw;
  adc_sum += raw;
  adc_ring_idx = (uint8_t)((adc_ring_idx + 1) & 0x0F);

  const uint16_t avg = (uint16_t)(adc_sum / 16U);
  const uint16_t new_feed = mapAdcToFeedX100(avg);

  if (pot_locked) {
    const uint16_t a = (new_feed > feed_x100) ? (new_feed - feed_x100) : (feed_x100 - new_feed);
    if (a < 10) return;
    pot_locked = false;
  }

  if (new_feed != feed_x100) {
    feed_x100 = new_feed;
    updateDisplay();
  }
}

static void syncModeSubmodeNow()
{
  const uint8_t m = readModeByte();

  last_pd8_10 = 0;
  last_pd8_10 |= (digitalRead(SUBMODE_PINS[0]) == HIGH) ? 1U : 0U;
  last_pd8_10 |= (digitalRead(SUBMODE_PINS[1]) == HIGH) ? 2U : 0U;
  last_pd8_10 |= (digitalRead(SUBMODE_PINS[2]) == HIGH) ? 4U : 0U;

  Mode newMode;
  const bool okMode = decodeModeFromByte(m, newMode);
  if (okMode) current_mode = newMode;

  const SubMode newSub = decodeSubmodeFromRawPd(last_pd8_10);
  for (uint8_t i = 0; i < MODE_COUNT; i++) submode_per_mode[i] = newSub;

  cache_valid = false;
  updateDisplay();
}

static void updateModeSubmodeFromSwitches()
{
  // main behavior: ignore mode/submode switching while joystick is driving an axis
  if (joy_z_active || joy_x_active) return;

  // Debounce mode + submode switches as one combined state
  static uint8_t lastModeByte = 0xFF;
  static uint8_t lastSubBits = 0xFF;
  static uint8_t stableModeByte = 0xFF;
  static uint8_t stableSubBits = 0xFF;
  static uint32_t lastChangeMs = 0;

  const uint32_t now = millis();
  const uint8_t m = readModeByte();
  const uint8_t s = readSubmodeBits();

  // Store raw switch snapshot (used by SUBMODE decode)
  last_pd8_10 = 0;
  last_pd8_10 |= (digitalRead(SUBMODE_PINS[0]) == HIGH) ? 1U : 0U;
  last_pd8_10 |= (digitalRead(SUBMODE_PINS[1]) == HIGH) ? 2U : 0U;
  last_pd8_10 |= (digitalRead(SUBMODE_PINS[2]) == HIGH) ? 4U : 0U;

  if (m != lastModeByte || s != lastSubBits) {
    lastModeByte = m;
    lastSubBits = s;
    lastChangeMs = now;
  }

  if ((now - lastChangeMs) < 50) return; // switch debounce
  if (m == stableModeByte && s == stableSubBits) return;

  stableModeByte = m;
  stableSubBits = s;

  Mode newMode;
  const bool okMode = decodeModeFromByte(stableModeByte, newMode);

  if (okMode) current_mode = newMode;
  // Decode SUBMODE using raw PD9/PD10 low detection (more tolerant than bit patterns)
  const SubMode newSub = decodeSubmodeFromRawPd(last_pd8_10);
  for (uint8_t i = 0; i < MODE_COUNT; i++) submode_per_mode[i] = newSub;

  // repaint
  cache_valid = false;
  beeperTrigger(BEEP_MS);
  updateDisplay();
}

static void i2cBusRecover(uint8_t sclPin, uint8_t sdaPin)
{
  // If SDA is stuck low (e.g. backpack mid-byte), Wire.begin() may hang.
  // Standard recovery: pulse SCL up to 9 times, then issue a STOP.
  pinMode(sclPin, OUTPUT_OPEN_DRAIN);
  pinMode(sdaPin, OUTPUT_OPEN_DRAIN);
  digitalWrite(sclPin, HIGH);
  digitalWrite(sdaPin, HIGH);
  delayMicroseconds(10);

  pinMode(sdaPin, INPUT_PULLUP);
  if (digitalRead(sdaPin) == LOW)
  {
    for (int i = 0; i < 9; i++)
    {
      digitalWrite(sclPin, LOW);
      delayMicroseconds(10);
      digitalWrite(sclPin, HIGH);
      delayMicroseconds(10);
    }
  }

  // STOP condition (best-effort)
  pinMode(sdaPin, OUTPUT_OPEN_DRAIN);
  digitalWrite(sdaPin, LOW);
  delayMicroseconds(10);
  digitalWrite(sclPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sdaPin, HIGH);
  delayMicroseconds(10);

  pinMode(sclPin, INPUT_PULLUP);
  pinMode(sdaPin, INPUT_PULLUP);
}

void setup()
{
  // After flashing, the backpack can power up in a bad I2C state.
  delay(300);

  // Explicit I2C pin selection before Wire.begin()
  i2cBusRecover(I2C_SCL_PIN, I2C_SDA_PIN);
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  // Some PCF8574 backpacks need a retry right after boot
  lcd.init();
  delay(10);
  lcd.init();
  lcd.backlight();

  beeperInit();
  stepperInitPins();

  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SEL, INPUT_PULLUP);

  // Joystick + RAPID button
  pinMode(JOY_L_PIN, INPUT_PULLUP);
  pinMode(JOY_R_PIN, INPUT_PULLUP);
  pinMode(JOY_U_PIN, INPUT_PULLUP);
  pinMode(JOY_D_PIN, INPUT_PULLUP);
  pinMode(BTN_RAPID_PIN, INPUT_PULLUP);
#if ENABLE_HW_MOTION_STOP
  pinMode(BTN_STOP_PIN, INPUT_PULLUP);
#endif

  // ADC feed potentiometer
  pinMode(ADC_FEED_PIN, INPUT_ANALOG);

  // Mode/submode switches (use pullups to avoid floating)
  for (uint8_t i = 0; i < 8; i++) pinMode(MODE_PINS[i], INPUT_PULLUP);
  for (uint8_t i = 0; i < 3; i++) pinMode(SUBMODE_PINS[i], INPUT_PULLUP);

  pinMode(AXIS_Z_PIN, INPUT_PULLUP);
  pinMode(AXIS_X_PIN, INPUT_PULLUP);
  pinMode(SCALE_X100_PIN, INPUT_PULLUP);
  pinMode(SCALE_X1_PIN, INPUT_PULLUP);
  pinMode(SCALE_X10_PIN, INPUT_PULLUP);

  handEncoderHwInit();

  delay(5);

  syncModeSubmodeNow();
  updateDisplay(); // initial paint
}

void loop()
{
  handEncoderUiSnapshot();

  updateJoystickMainStyle();
  updateModeSubmodeFromSwitches();
  updateRapidButton();
  updateFeedFromPot();
  updateStepperJog();
  updateButtonsDebounced();
  updateBeeper();
}
