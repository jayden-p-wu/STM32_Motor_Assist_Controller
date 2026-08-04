/* fardriver.h
 *   PA6 - PWM (TIM3_CH1) -> Fardriver throttle signal, 0.5-3.3V
 *   PC8 - GPIO out -> Low-side driver gate -> Fardriver BL pin 20 (regen brake)
 *   PC9 - GPIO out -> Fardriver KEY pin 11 (ignition/enable)
 *
 * Fardriver accelerator range must be set to 0.5V-3.3V via PC software
 * to match the STM32 PWM maximum of VDDA = 3.3V.
 */

#ifndef FARDRIVER_H
#define FARDRIVER_H

#include "main.h"
#include <stdint.h>

/* --- Pin definitions (must match CubeMX labels) --- */
#define REGEN_BRAKE_Pin       GPIO_PIN_8
#define REGEN_BRAKE_GPIO_Port GPIOC

#define FARDRIVER_KEY_Pin       GPIO_PIN_9
#define FARDRIVER_KEY_GPIO_Port GPIOC

/* --- Throttle voltage mapping ---
 * Fardriver configured (via PC software) for 0.5V (idle) to 3.3V (full).
 */
#define FARDRIVER_THROTTLE_MIN_V  0.5f
#define FARDRIVER_THROTTLE_MAX_V  3.3f
#define PWM_PERIOD_COUNTS         3199U

/* --- BL (low-voltage brake, pin 20) polarity ---
 * Per the Fardriver manual, the BL input's polarity is a parameter set
 * INSIDE the Fardriver's own PC tuning software ("Low-voltage brake"
 * setting), not something firmware can detect:
 *   Mode A: Hanging(float) = Drive,        GND = Park/Brake
 *   Mode B: Hanging(float) = Park/Brake,   GND = Drive
 * Confirmed via wire-disconnect test that this controller is configured
 * for Mode A (grounding BL engages the brake) - firmware assumes Mode A. */

/* Public API */
void Fardriver_Init(void);
float Fardriver_SetThrottlePct(float percent);   /* 0.0 = idle, 100.0 = full */
float Fardriver_SetThrottleVoltage(float voltage);
void Fardriver_SetKey(uint8_t on);           /* 1 = key on, 0 = key off */
void Fardriver_SetRegenBrake(uint8_t engage); /* 1 = engage regen, 0 = release */

#endif /* FARDRIVER_H */
