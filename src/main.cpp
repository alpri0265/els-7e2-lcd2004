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

// Buttons (from earlier project)
static constexpr uint8_t BTN_LEFT  = PB8;
static constexpr uint8_t BTN_RIGHT = PB9;
static constexpr uint8_t BTN_UP    = PB10;
static constexpr uint8_t BTN_DOWN  = PB11;
static constexpr uint8_t BTN_SEL   = PB12;

// Debounce
static constexpr uint32_t DEBOUNCE_MS = 25;

enum Key : uint8_t
{
  KEY_SEL = 0,
  KEY_UP,
  KEY_DOWN,
  KEY_LEFT,
  KEY_RIGHT
};

// State shown on screen
static bool mode_sync = true;     // SYNC / MAN
static uint16_t feed_x100 = 25;   // 0.25 mm/rev (stored as *100)
static uint8_t pass_total = 10;   // TOTAL passes
static uint16_t doc_x100 = 50;    // 0.50 mm (stored as *100)

// Selected menu row (0..3). Marker is shown on the right edge.
static uint8_t selected_row = 0;

static char last_rows[4][21] = {{0}};
static bool cache_valid = false;

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
  // "MODE: SYNC" or "MODE: MAN"
  snprintf(out, 21, "MODE: %-4s", mode_sync ? "SYNC" : "MAN");
}

static void makeRow1(char out[21])
{
  // "FEED: 0.25 mm/rev"
  snprintf(out, 21, "FEED: %u.%02u mm/rev", feed_x100 / 100, feed_x100 % 100);
}

static void makeRow2(char out[21])
{
  // "PASS: 10 TOTAL"
  snprintf(out, 21, "PASS: %u TOTAL", pass_total);
}

static void makeRow3(char out[21])
{
  // "DOC:  0.50 mm"
  snprintf(out, 21, "DOC:  %u.%02u mm", doc_x100 / 100, doc_x100 % 100);
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

  makeRow0(rows[0]); padTo20(rows[0]); applySelectionMarker(0, rows[0]);
  makeRow1(rows[1]); padTo20(rows[1]); applySelectionMarker(1, rows[1]);
  makeRow2(rows[2]); padTo20(rows[2]); applySelectionMarker(2, rows[2]);
  makeRow3(rows[3]); padTo20(rows[3]); applySelectionMarker(3, rows[3]);

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
  return m;
}

static void applyKeyPress(uint8_t key)
{
  switch (key) {
    case KEY_SEL:
      // Toggle mode only when MODE row is selected
      if (selected_row == 0) mode_sync = !mode_sync;
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
      // Decrease value of selected item
      if (selected_row == 1) { // FEED
        if (feed_x100 > 5) feed_x100 -= 5; // -0.05
      } else if (selected_row == 2) { // PASS
        if (pass_total > 1) pass_total -= 1;
      } else if (selected_row == 3) { // DOC
        if (doc_x100 > 5) doc_x100 -= 5;   // -0.05
      }
      break;
    case KEY_RIGHT:
      // Increase value of selected item
      if (selected_row == 1) { // FEED
        if (feed_x100 < 999) feed_x100 += 5; // +0.05
      } else if (selected_row == 2) { // PASS
        if (pass_total < 99) pass_total += 1;
      } else if (selected_row == 3) { // DOC
        if (doc_x100 < 999) doc_x100 += 5;   // +0.05
      }
      break;
    default:
      break;
  }
  updateDisplay();
}

static void updateButtonsDebounced()
{
  static uint8_t lastReading = 0;
  static uint8_t stableState = 0;
  static uint32_t lastChangeMs = 0;

  const uint32_t now = millis();
  const uint8_t reading = readKeysBitmask();

  if (reading != lastReading) {
    lastChangeMs = now;
    lastReading = reading;
  }

  if ((now - lastChangeMs) < DEBOUNCE_MS) return;
  if (reading == stableState) return;

  const uint8_t pressed = reading & ~stableState;
  stableState = reading;

  // Fire events on new presses only
  if (pressed & (1u << KEY_SEL))   applyKeyPress(KEY_SEL);
  if (pressed & (1u << KEY_UP))    applyKeyPress(KEY_UP);
  if (pressed & (1u << KEY_DOWN))  applyKeyPress(KEY_DOWN);
  if (pressed & (1u << KEY_LEFT))  applyKeyPress(KEY_LEFT);
  if (pressed & (1u << KEY_RIGHT)) applyKeyPress(KEY_RIGHT);
}

void setup()
{
  // Explicit I2C pin selection before Wire.begin()
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Wire.setClock(400000);

  lcd.init();
  lcd.backlight();

  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SEL, INPUT_PULLUP);

  updateDisplay(); // initial paint
}

void loop()
{
  updateButtonsDebounced();
}
