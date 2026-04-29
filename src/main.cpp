#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

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

// Feed potentiometer (CubeMX wiring): ADC on PF6
static constexpr uint8_t ADC_FEED_PIN = PF6;

// Debounce
static constexpr uint32_t DEBOUNCE_MS = 25;
static constexpr uint32_t LONGPRESS_MS = 650;

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

// Selected menu row (0..3). Marker is shown on the right edge.
static uint8_t selected_row = 0;
static bool in_submenu = false;

static char last_rows[4][21] = {{0}};
static bool cache_valid = false;

// Raw switch/key state (kept for internal decode/robustness)
static uint8_t last_pd8_10 = 0x07; // bits: 0=PD8,1=PD9,2=PD10 (1=HIGH)
static uint8_t last_keys_reading = 0;

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
  // "DOC:  0.50 mm"
  snprintf(out, 21, "DOC:  %u.%02u mm",
           (unsigned)(doc_x100 / 100),
           (unsigned)(doc_x100 % 100));
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

  // Joystick maps to same directions (active LOW)
  // NOTE: we still read joystick here, but "main-style joystick logic"
  // is handled separately in updateJoystickMainStyle().
  if (digitalRead(JOY_U_PIN) == LOW) m |= (1u << KEY_UP);
  if (digitalRead(JOY_D_PIN) == LOW) m |= (1u << KEY_DOWN);
  if (digitalRead(JOY_L_PIN) == LOW) m |= (1u << KEY_LEFT);
  if (digitalRead(JOY_R_PIN) == LOW) m |= (1u << KEY_RIGHT);
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
  const JoyDir cur = readJoystickDir();
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

static void applyKeyPress(uint8_t key)
{
  switch (key) {
    case KEY_SEL:
      // Short press SEL: toggle submode on row0 (main screen) or row1 (submenu)
      if (!in_submenu) {
        if (selected_row == 0) {
          submode_per_mode[current_mode] = (submode_per_mode[current_mode] == SUB_MAN) ? SUB_INT : SUB_MAN;
        }
      } else {
        if (selected_row == 1) {
          submode_per_mode[current_mode] = (submode_per_mode[current_mode] == SUB_MAN) ? SUB_INT : SUB_MAN;
        }
      }
      break;
    case KEY_UP:
      // Move selection up
      if (selected_row > 0) selected_row--;
      break;
    case KEY_DOWN:
      // Move selection down
      if (selected_row < 3) selected_row++;
      break;
    case KEY_LEFT:
      if (!in_submenu) {
        // On main screen: adjust selected values (mode comes from hardware switch)
        if (selected_row == 1) {
          if (feed_x100 > 5) feed_x100 -= 5;
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

  // Debounce for other keys (and for stable-state debug)
  const uint8_t prevStable = stable;
  if ((now - lastRawChangeMs) >= DEBOUNCE_MS) stable = raw;
  const uint8_t pressed = stable & ~prevStable;
  if (pressed & (1u << KEY_UP))    applyKeyPress(KEY_UP);
  if (pressed & (1u << KEY_DOWN))  applyKeyPress(KEY_DOWN);
  if (pressed & (1u << KEY_LEFT))  applyKeyPress(KEY_LEFT);
  if (pressed & (1u << KEY_RIGHT)) applyKeyPress(KEY_RIGHT);
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

  if (new_feed != feed_x100) {
    feed_x100 = new_feed;
    updateDisplay();
  }
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

  // ADC feed potentiometer
  pinMode(ADC_FEED_PIN, INPUT_ANALOG);

  // Mode/submode switches (use pullups to avoid floating)
  for (uint8_t i = 0; i < 8; i++) pinMode(MODE_PINS[i], INPUT_PULLUP);
  for (uint8_t i = 0; i < 3; i++) pinMode(SUBMODE_PINS[i], INPUT_PULLUP);
  delay(5);

  updateDisplay(); // initial paint
}

void loop()
{
  updateJoystickMainStyle();
  updateModeSubmodeFromSwitches();
  updateRapidButton();
  updateFeedFromPot();
  updateButtonsDebounced();
}
