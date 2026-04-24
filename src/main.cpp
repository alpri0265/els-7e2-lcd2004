#include <Arduino.h>
#include "LiquidCrystalRus.h"
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
LiquidCrystalRus lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Ініціалізація менеджера меню
MenuManager menu(lcd);

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

  // Ініціалізація LCD та меню
  menu.init();
  
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
