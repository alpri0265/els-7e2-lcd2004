#include <Arduino.h>
#include <LiquidCrystal.h>
#include "menu.h"

// Піни LCD на Port E (відповідно до main.h з оригінального проекту)
#define LCD_RS PE7
#define LCD_EN PE8
#define LCD_D4 PE9
#define LCD_D5 PE10
#define LCD_D6 PE11
#define LCD_D7 PE12

// Піни кнопок меню на Port B (згідно з main.h)
#define BTN_LEFT  PB8
#define BTN_RIGHT PB9
#define BTN_UP    PB10
#define BTN_DOWN  PB11
#define BTN_SEL   PB12

// Ініціалізація дисплея
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Ініціалізація менеджера меню
MenuManager menu(lcd);

static void forceLcdPulseEnable() {
  digitalWrite(LCD_EN, LOW);
  delayMicroseconds(250);
  digitalWrite(LCD_EN, HIGH);
  delayMicroseconds(2500);
  digitalWrite(LCD_EN, LOW);
  delayMicroseconds(25000);
}

static void forceLcdWrite4(uint8_t nibble) {
  digitalWrite(LCD_D4, (nibble >> 0) & 0x01);
  digitalWrite(LCD_D5, (nibble >> 1) & 0x01);
  digitalWrite(LCD_D6, (nibble >> 2) & 0x01);
  digitalWrite(LCD_D7, (nibble >> 3) & 0x01);
  delayMicroseconds(2500);
  forceLcdPulseEnable();
}

void forceLcdReset() {
  // Manual HD44780 4-bit init sequence (robust for fast MCUs / 3.3V logic).
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_EN, OUTPUT);
  pinMode(LCD_D4, OUTPUT);
  pinMode(LCD_D5, OUTPUT);
  pinMode(LCD_D6, OUTPUT);
  pinMode(LCD_D7, OUTPUT);

  digitalWrite(LCD_RS, LOW);
  digitalWrite(LCD_EN, LOW);
  digitalWrite(LCD_D4, LOW);
  digitalWrite(LCD_D5, LOW);
  digitalWrite(LCD_D6, LOW);
  digitalWrite(LCD_D7, LOW);

  delay(60); // >40ms after power-up (and give LCD time after MCU reset)

  // We start in 8-bit mode: send 0x3 (high nibble) three times
  forceLcdWrite4(0x03);
  delay(6);  // >4.1ms
  forceLcdWrite4(0x03);
  delay(6);  // >4.1ms
  forceLcdWrite4(0x03);
  delay(2);  // >100us

  // Switch to 4-bit mode: send 0x2 (high nibble)
  forceLcdWrite4(0x02);
  delay(2);
}

// Функція зчитування фізичного стану кнопок
// Повертає байт, де кожен біт відповідає за свою кнопку (1 - натиснуто, 0 - відпущено)
uint8_t readButtons() {
    uint8_t state = 0;
    // Оскільки кнопки підтягнуті до VCC (INPUT_PULLUP), натискання замикає пін на GND (LOW)
    if (digitalRead(BTN_SEL) == LOW)   state |= (1 << 0);
    if (digitalRead(BTN_UP) == LOW)    state |= (1 << 1);
    if (digitalRead(BTN_DOWN) == LOW)  state |= (1 << 2);
    if (digitalRead(BTN_LEFT) == LOW)  state |= (1 << 3);
    if (digitalRead(BTN_RIGHT) == LOW) state |= (1 << 4);
    return state;
}

// Опит кнопок з антидребезгом (debounce) та автоповтором (як було на Mega)
void updateButtons() {
    static uint8_t lastReading = 0;
    static uint8_t validatedState = 0;
    static uint32_t lastDebounceTime = 0;
    static uint32_t lastRepeatTime = 0;
    
    uint8_t reading = readButtons();
    uint32_t now = millis();
    
    // Якщо стан пінів змінився через брязкіт або нове натискання, скидаємо таймер
    if (reading != lastReading) {
        lastDebounceTime = now;
    }
    
    // Якщо стан стабільний протягом 30 мс
    if ((now - lastDebounceTime) > 30) {
        if (reading != validatedState) {
            // Визначаємо, які кнопки були щойно натиснуті
            uint8_t pressed = reading & ~validatedState;
            validatedState = reading;
            
            // Відправляємо події в MenuManager
            if (pressed & (1 << 0)) menu.handleKeyEvent(0); // Select
            if (pressed & (1 << 1)) menu.handleKeyEvent(1); // Up
            if (pressed & (1 << 2)) menu.handleKeyEvent(2); // Down
            if (pressed & (1 << 3)) menu.handleKeyEvent(3); // Left
            if (pressed & (1 << 4)) menu.handleKeyEvent(4); // Right
            
            // Встановлюємо затримку перед початком автоповтору (400 мс)
            lastRepeatTime = now + 400; 
        } 
        else if (validatedState != 0) {
            // Якщо кнопка утримується довго, генеруємо події автоповтору (крім кнопки Select)
            if (now > lastRepeatTime) {
                lastRepeatTime = now + 80; // Швидкість автоповтору (кожні 80 мс)
                
                if (validatedState & (1 << 1)) menu.handleKeyEvent(1);
                if (validatedState & (1 << 2)) menu.handleKeyEvent(2);
                if (validatedState & (1 << 3)) menu.handleKeyEvent(3);
                if (validatedState & (1 << 4)) menu.handleKeyEvent(4);
            }
        }
    }
    lastReading = reading;
}

void setup() {
  Serial.begin(115200);
  Serial.println("STM32F407ZE ELS Start...");

  // Налаштування пінів кнопок як входи з підтягувальним резистором (INPUT_PULLUP)
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SEL, INPUT_PULLUP);

  forceLcdReset();

  // Ініціалізація LCD та меню
  menu.init();
  lcd.noCursor();
  lcd.noBlink();
  
  // Виводимо початковий екран
  menu.render();

  Serial.println("Menu initialized. Hardware buttons active.");
}

void loop() {
  // Виклик оновлення внутрішньої логіки меню
  menu.update();

  // Опит апаратних кнопок
  updateButtons();

  // Тестове перемикання екранів меню через Serial (для зручності відлагодження)
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') menu.handleKeyEvent(0); // Select
    if (c == 'u') menu.handleKeyEvent(1); // Up
    if (c == 'd') menu.handleKeyEvent(2); // Down
    if (c == 'l') menu.handleKeyEvent(3); // Left
    if (c == 'r') menu.handleKeyEvent(4); // Right
  }

  // Невелика затримка для стабільності
  delay(1);
}
