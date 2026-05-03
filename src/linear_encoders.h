#pragma once

#include <Arduino.h>

/* Лінійні енкодери A/B (квадратура) за docs/pinout.md: Z PE0/PE1, X PE2/PE3; нуль PC13 / PD3. */

#ifndef ENABLE_LINEAR_ENCODERS
#define ENABLE_LINEAR_ENCODERS 1
#endif

#if ENABLE_LINEAR_ENCODERS && defined(STM32F407xx)

typedef void (*LinearEncZeroCallback)(void);

void linearEncodersInit(LinearEncZeroCallback on_zero_z, LinearEncZeroCallback on_zero_x);

/* Опитування квадратури (викликати з loop, після кроків або до них — однаково для лічильника). */
void linearEncodersPoll();

int32_t linearEncodersCountsZ();
int32_t linearEncodersCountsX();

#else

typedef void (*LinearEncZeroCallback)(void);
static inline void linearEncodersInit(LinearEncZeroCallback, LinearEncZeroCallback) {}
static inline void linearEncodersPoll() {}
static inline int32_t linearEncodersCountsZ() { return 0; }
static inline int32_t linearEncodersCountsX() { return 0; }

#endif
