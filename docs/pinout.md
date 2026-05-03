# Pinout (CubeMX) — ELS-7e2-lcd2004

Джерело: `ELS-7e2-lcd2004/ELS-7e2-lcd2004.ioc` + `ELS-7e2-lcd2004/Core/Inc/main.h` (STM32F407ZET6, LQFP144).

Примітки:
- Входи з `PullUp`: **норма = HIGH**, активний стан = **LOW (замикання на GND)**.
- LCD у цьому проєкті — **I2C1 на PB6/PB7**. Паралельні `LCD_*` (PE7..PE12) прибрані з `.ioc`, щоб не плутались.

---

## Stepper (оси X/Z) — STEP/DIR/EN

| Сигнал | Пін MCU | Напрям | Підтяжка | Примітка |
|---|---|---:|---:|---|
| `Z_STEP` | PC0 | OUT | — | STEP осі Z |
| `Z_DIR`  | PC1 | OUT | — | DIR осі Z |
| `Z_EN`   | PC2 | OUT | — | ENABLE осі Z |
| `X_STEP` | PC3 | OUT | — | STEP осі X |
| `X_DIR`  | PC4 | OUT | — | DIR осі X |
| `X_EN`   | PC5 | OUT | — | ENABLE осі X |

---

## LCD (I2C)

| Сигнал | Пін MCU | Напрям | Підтяжка | Примітка |
|---|---|---:|---:|---|
| `I2C1_SCL` | PB6 | AF | PullUp | SCL для LCD2004 (PCF8574) |
| `I2C1_SDA` | PB7 | AF | PullUp | SDA для LCD2004 (PCF8574) |

---

## ADC (потенціометр подачі)

| Сигнал | Пін MCU | Напрям | Підтяжка | Примітка |
|---|---|---:|---:|---|
| `ADC_FEED` (ADC3_IN4) | PF6 | IN | — | Потенціометр FEED |

---

## Кнопки меню / керування

| Сигнал | Пін MCU | Напрям | Підтяжка | Примітка |
|---|---|---:|---:|---|
| `BTN_MENU_L`   | PB8  | IN | PullUp | LEFT |
| `BTN_MENU_R`   | PB9  | IN | PullUp | RIGHT |
| `BTN_MENU_U`   | PB10 | IN | PullUp | UP |
| `BTN_MENU_D`   | PB11 | IN | PullUp | DOWN |
| `BTN_MENU_SEL` | PB12 | IN | PullUp | SELECT |
| `BTN_STOP`     | PB13 | IN | PullUp | STOP |
| `BTN_RAPID`    | PD7  | IN | PullUp | RAPID |

---

## Джойстик (4 напрями)

| Сигнал | Пін MCU | Напрям | Підтяжка | Примітка |
|---|---|---:|---:|---|
| `JOY_L` | PF12 | IN | PullUp | Left |
| `JOY_R` | PF13 | IN | PullUp | Right |
| `JOY_U` | PF14 | IN | PullUp | Up |
| `JOY_D` | PF15 | IN | PullUp | Down |

---

## Перемикач MODE (8 ліній)

| Сигнал | Пін MCU | Напрям | Підтяжка | Примітка |
|---|---|---:|---:|---|
| `MODE_D0` | PG0 | IN | PullUp | MODE bit0 |
| `MODE_D1` | PG1 | IN | PullUp | MODE bit1 |
| `MODE_D2` | PG2 | IN | PullUp | MODE bit2 |
| `MODE_D3` | PG3 | IN | PullUp | MODE bit3 |
| `MODE_D4` | PG4 | IN | PullUp | MODE bit4 |
| `MODE_D5` | PG5 | IN | PullUp | MODE bit5 |
| `MODE_D6` | PG6 | IN | PullUp | MODE bit6 |
| `MODE_D7` | PG7 | IN | PullUp | MODE bit7 |

---

## Перемикач SUBMODE (3 лінії)

| Сигнал | Пін MCU | Напрям | Підтяжка | Примітка |
|---|---|---:|---:|---|
| `SUBMODE_0` | PD8  | IN | PullUp | SUBMODE line 0 |
| `SUBMODE_1` | PD9  | IN | PullUp | SUBMODE line 1 |
| `SUBMODE_2` | PD10 | IN | PullUp | SUBMODE line 2 |

---

## Вибір осі / SCALE (РГІ)

| Сигнал | Пін MCU | Напрям | Підтяжка | Примітка |
|---|---|---:|---:|---|
| `AXIS_Z`   | PD11 | IN | PullUp | Активна вісь Z |
| `AXIS_X`   | PD15 | IN | PullUp | Активна вісь X |
| `SCALE_X100` | PD4  | IN | PullUp | Масштаб ×100 (3-поз. перемикач: рівно одна лінія LOW) |
| `SCALE_X1`   | PD5  | IN | PullUp | Масштаб ×1 |
| `SCALE_X10`  | PD6  | IN | PullUp | Масштаб ×10 |

---

## Енкодери

### Енкодер шпинделя (TIM3 Encoder)
| Сигнал | Пін MCU | Функція |
|---|---|---|
| `ENC_SP_A` | PA6 | TIM3_CH1 |
| `ENC_SP_B` | PA7 | TIM3_CH2 |

### HandCoder (TIM4 Encoder)
| Сигнал | Пін MCU | Функція |
|---|---|---|
| `ENC_HC_A` | PD12 | TIM4_CH1 |
| `ENC_HC_B` | PD13 | TIM4_CH2 |

---

## Лінійки (inputs) + кнопки “нуль”

| Сигнал | Пін MCU | Напрям | Підтяжка | Примітка |
|---|---|---:|---:|---|
| `LIN_Z_A` | PE0  | IN | PullUp | Лінійка Z A |
| `LIN_Z_B` | PE1  | IN | PullUp | Лінійка Z B |
| `LIN_X_A` | PE2  | IN | PullUp | Лінійка X A |
| `LIN_X_B` | PE3  | IN | PullUp | Лінійка X B |
| `BTN_LIN_Z_ZERO` | PC13 | IN | PullUp | Нуль Z |
| `BTN_LIN_X_ZERO` | PD3  | IN | PullUp | Нуль X |

---

## Кінцевики/ліміти + індикатори

### Ліміти (inputs)
| Сигнал | Пін MCU | Напрям | Підтяжка | Примітка |
|---|---|---:|---:|---|
| `LIM_REAR`  | PA8  | IN | PullUp | Rear |
| `LIM_FRONT` | PA9  | IN | PullUp | Front |
| `LIM_RIGHT` | PA10 | IN | PullUp | Right |
| `LIM_LEFT`  | PA11 | IN | PullUp | Left |
| `LIM_MECH_1`| PD1  | IN | PullUp | Mech 1 |
| `LIM_MECH_2`| PD14 | IN | PullUp | Mech 2 |

### LED лімітів (outputs)
| Сигнал | Пін MCU | Напрям | Примітка |
|---|---|---:|---|
| `LED_LIM_REAR`  | PG12 | OUT | LED rear |
| `LED_LIM_FRONT` | PG13 | OUT | LED front |
| `LED_LIM_RIGHT` | PG14 | OUT | LED right |
| `LED_LIM_LEFT`  | PG15 | OUT | LED left |

Прошивка (`ENABLE_SOFTWARE_LIMITS`, логіка як у Arduino ELS): **PA8–PA11** — не «кінцевик у лінію», а **кнопки навчання межі** (активний LOW): коротке натискання **записує** поточну позицію в кроках для відповідної осі/напрямку, повторне — **знімає** межу. Відповідний **LED** показує, що програмний ліміт **увімкнений**; рух джойстиком і ручним колесом обмежується цими координатами. Перед записом відповідна вісь на джойстику має бути в нейтралі. **LIM_MECH_1/2** (LOW) — апаратна тривога, блокує весь рух. Дебаунс кнопок навчання ~30 ms, MECH ~15 ms. Напрямки осей — у `softLimitAllowsZ` / `softLimitAllowsX` у `src/main.cpp`.

---

## Виходи (бузер/тахо/дільник)

| Сигнал | Пін MCU | Напрям | Примітка |
|---|---|---:|---|
| `BEEPER`      | PD0  | OUT | Бузер |
| `TACHO_OUT`   | PE13 | OUT | Tacho out |
| `OUT_DIVIDER` | PE14 | OUT | Divider out |

---

## Тактування (HSE)

| Сигнал | Пін MCU | Примітка |
|---|---|---|
| `OSC_IN`  | PH0 | HSE external oscillator input |
| `OSC_OUT` | PH1 | HSE external oscillator output |

