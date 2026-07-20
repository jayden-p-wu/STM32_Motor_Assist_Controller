/* fardriver_sif.h
 * Decoder for the Fardriver Y29/Y34 "one-line" SIF feedback protocol.
 *
 * The Fardriver transmits motor status on its TXD pin (pin 12, brown+blue wire)
 * using the SIF protocol — NOT standard UART.  Bits are encoded by the ratio of
 * the high-pulse width to the low-pulse width within each bit period:
 *
 *   Bit 0: high = 2/3 of period, low = 1/3  (64 Tosc high, 32 Tosc low)
 *   Bit 1: high = 1/3 of period, low = 2/3  (32 Tosc high, 64 Tosc low)
 *   Sync:  line low for > 992 Tosc, then high for > 32 Tosc
 *
 * Where 32 µs < Tosc < 320 µs.
 *
 * Frame = 12 bytes: DATA0(0x08) DATA1(0x61) DATA2..DATA10 DATA11(XOR checksum)
 *
 * Hardware: connect Fardriver TXD to PA0 (TIM2_CH1 input capture, both edges).
 *           TIM2 must run at 1 MHz (prescaler = 71 @ 72 MHz sysclk).
 */

#ifndef FARDRIVER_SIF_H
#define FARDRIVER_SIF_H

#include <stdint.h>

/* Decoded status from the most recent valid frame */
typedef struct {
    /* DATA3 faults / modes */
    uint8_t push_cart_6km  : 1;
    uint8_t hall_fault     : 1;
    uint8_t handlebar_fault: 1;
    uint8_t controller_fault: 1;
    uint8_t undervoltage   : 1;
    uint8_t cruise_active  : 1;
    uint8_t boost_active   : 1;
    uint8_t phase_loss     : 1;

    /* DATA4 status */
    uint8_t fourth_gear    : 1;   /* D7 - fourth gear indicator */
    uint8_t motor_running  : 1;   /* PWM output active */
    uint8_t brake_active   : 1;
    uint8_t controller_protection: 1;  /* D4 - controller in protection state */
    uint8_t coasting_charge: 1;
    uint8_t antiskid       : 1;
    uint8_t speed_gear;           /* 0=no 3-speed, 1=low, 2=mid, 3=high */

    /* DATA5 status */
    uint8_t overcurrent    : 1;
    uint8_t stall_protect  : 1;
    uint8_t reverse_active : 1;
    uint8_t elec_brake     : 1;
    uint8_t speed_limit    : 1;

    /* DATA6 — running current, signed (A) */
    int8_t  current_a;

    /* DATA7:8 — hall-change count in 500 ms window (use for speed calc) */
    uint16_t speed_hall_count;

    /* DATA9 — battery capacity 0..100 % */
    uint8_t  battery_pct;

    /* DATA10 — current ratio 0..100 % */
    uint8_t  current_pct;

    /* Set to 1 whenever a fresh valid frame is available */
    uint8_t  fresh;
} FardriverSIF_t;

extern FardriverSIF_t g_fardriver_status;

/* Call from TIM2 global interrupt handler in stm32f3xx_it.c:
 *   void TIM2_IRQHandler(void) { FardriverSIF_TIM2_IRQHandler(); }
 */
void FardriverSIF_TIM2_IRQHandler(void);

/* Call once in main before starting TIM2 input capture */
void FardriverSIF_Init(void);

/* Compute vehicle speed in km/h from hall count.
 *   pole_pairs: motor pole pairs (e.g. 23)
 *   wheel_dia_in: wheel diameter in inches (e.g. 16)
 */
float FardriverSIF_SpeedKMH(uint8_t pole_pairs, float wheel_dia_in);

#endif /* FARDRIVER_SIF_H */
