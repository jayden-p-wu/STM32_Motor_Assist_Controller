/* fardriver_sif.c
 * Y29/Y34 SIF one-line protocol decoder using TIM2 input capture on PA0.
 *
 * Timer setup (configure in CubeMX before calling FardriverSIF_Init):
 *   TIM2, prescaler = 71, period = 0xFFFFFFFF
 *   CH1 on PA0: input capture, IC1Selection = Direct TI1, IC1Filter = 0
 *   Enable TIM2 global interrupt.
 *
 * How it works:
 *   The ISR is called on every edge of the SIF signal.  We measure the
 *   high-pulse width and low-pulse width of each bit, then classify the bit
 *   by which was longer.  A very long low pulse (> ~31 ms) is the sync.
 */

#include "fardriver_sif.h"
#include "main.h"
#include <string.h>

/* --- Protocol constants (in µs at Tosc = 32 µs minimum) --- */
#define SIF_SYNC_MIN_US    31744U   /* 992 * 32 µs */
#define SIF_BIT_MIN_US       960U   /* 30 * 32 µs  — below this = noise */

#define SIF_FRAME_BYTES      12U
#define SIF_DEVICE_CODE     0x08U
#define SIF_SEQ_CODE        0x61U

/* Published state */
FardriverSIF_t g_fardriver_status;

/* --- Internal state machine --- */
typedef enum {
    SIF_WAIT_SYNC_LOW = 0,  /* waiting for the long low sync pulse */
    SIF_WAIT_SYNC_HIGH,     /* sync low seen, waiting for the trailing high */
    SIF_RECV_BIT_HIGH,      /* measuring high portion of a data bit */
    SIF_RECV_BIT_LOW        /* measuring low portion of a data bit */
} SifState_t;

static SifState_t  s_state      = SIF_WAIT_SYNC_LOW;
static uint32_t    s_edge_ts    = 0;      /* timestamp of last edge (µs) */
static uint32_t    s_high_us    = 0;      /* width of current bit's high pulse */
static uint8_t     s_byte_buf   = 0;
static uint8_t     s_bit_idx    = 0;      /* bit within current byte (0..7, MSB first) */
static uint8_t     s_byte_idx   = 0;      /* byte within frame */
static uint8_t     s_frame[SIF_FRAME_BYTES];

extern TIM_HandleTypeDef htim2;

/* ------------------------------------------------------------------ */
static void sif_reset(void)
{
    s_state    = SIF_WAIT_SYNC_LOW;
    s_bit_idx  = 0;
    s_byte_idx = 0;
    s_byte_buf = 0;
}

/* Push one decoded bit (MSB first) into the frame buffer */
static void sif_push_bit(uint8_t bit)
{
    s_byte_buf = (uint8_t)((s_byte_buf << 1) | (bit & 0x01U));
    s_bit_idx++;

    if (s_bit_idx == 8U) {
        if (s_byte_idx < SIF_FRAME_BYTES) {
            s_frame[s_byte_idx++] = s_byte_buf;
        }
        s_bit_idx  = 0;
        s_byte_buf = 0;

        if (s_byte_idx == SIF_FRAME_BYTES) {
            /* Validate fixed header bytes */
            if (s_frame[0] != SIF_DEVICE_CODE || s_frame[1] != SIF_SEQ_CODE) {
                sif_reset();
                return;
            }

            /* Verify XOR checksum (DATA0..DATA10 XOR'd = DATA11) */
            uint8_t chk = 0;
            for (uint8_t i = 0; i < SIF_FRAME_BYTES - 1U; i++) {
                chk ^= s_frame[i];
            }
            if (chk != s_frame[SIF_FRAME_BYTES - 1U]) {
                sif_reset();
                return;
            }

            /* ---- Decode payload into g_fardriver_status ---- */
            uint8_t d3 = s_frame[3];
            uint8_t d4 = s_frame[4];
            uint8_t d5 = s_frame[5];

            g_fardriver_status.push_cart_6km    = (d3 >> 7) & 1U;
            g_fardriver_status.hall_fault        = (d3 >> 6) & 1U;
            g_fardriver_status.handlebar_fault   = (d3 >> 5) & 1U;
            g_fardriver_status.controller_fault  = (d3 >> 4) & 1U;
            g_fardriver_status.undervoltage      = (d3 >> 3) & 1U;
            g_fardriver_status.cruise_active     = (d3 >> 2) & 1U;
            g_fardriver_status.boost_active      = (d3 >> 1) & 1U;
            g_fardriver_status.phase_loss        = (d3 >> 0) & 1U;

            g_fardriver_status.motor_running     = (d4 >> 6) & 1U;
            g_fardriver_status.brake_active       = (d4 >> 5) & 1U;
            g_fardriver_status.coasting_charge   = (d4 >> 3) & 1U;
            g_fardriver_status.antiskid          = (d4 >> 2) & 1U;
            g_fardriver_status.fourth_gear        = (d4 >> 7) & 1U;
            g_fardriver_status.controller_protection = (d4 >> 4) & 1U;
            /* Speed gear: D1=1 D0=1 → high, D1=1 D0=0 → mid, D1=0 D0=1 → low */
            uint8_t g = d4 & 0x03U;
            g_fardriver_status.speed_gear = (g == 3U) ? 3U : (g == 2U) ? 2U : (g == 1U) ? 1U : 0U;

            g_fardriver_status.overcurrent       = (d5 >> 4) & 1U;
            g_fardriver_status.stall_protect     = (d5 >> 3) & 1U;
            g_fardriver_status.reverse_active    = (d5 >> 2) & 1U;
            g_fardriver_status.elec_brake        = (d5 >> 1) & 1U;
            g_fardriver_status.speed_limit       = (d5 >> 0) & 1U;

            /* NOTE: DATA6 sign convention is unverified.  The manual says
             * "high bit of negative current is 1, high bit of positive is 0",
             * which matches both two's-complement (assumed here) AND
             * sign-magnitude (bit7=sign, bits6:0=0..127 magnitude) - a
             * common alternative in this protocol family.  If negative
             * (regen/coast) currents look implausibly large, switch to:
             *   int8_t mag = (int8_t)(s_frame[6] & 0x7FU);
             *   g_fardriver_status.current_a = (s_frame[6] & 0x80U) ? -mag : mag;
             */
            g_fardriver_status.current_a         = (int8_t)s_frame[6];
            g_fardriver_status.speed_hall_count  = ((uint16_t)s_frame[7] << 8) | s_frame[8];
            g_fardriver_status.battery_pct       = s_frame[9];
            g_fardriver_status.current_pct       = s_frame[10];
            g_fardriver_status.fresh             = 1U;

            sif_reset();
        }
    }
}

/* ------------------------------------------------------------------ */
void FardriverSIF_Init(void)
{
    memset(&g_fardriver_status, 0, sizeof(g_fardriver_status));
    sif_reset();

    /* Start TIM2 in input-capture mode on CH1 */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}

/* Called from TIM2_IRQHandler — handle input capture event */
void FardriverSIF_TIM2_IRQHandler(void)
{
    uint32_t sr = TIM2->SR;

    /* Input capture on CH1 */
    if (sr & TIM_SR_CC1IF) {
        uint32_t ts  = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
        uint32_t pin = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);  /* current level after edge */
        uint32_t dt  = ts - s_edge_ts;                       /* µs elapsed since last edge */
        s_edge_ts    = ts;

        /* Toggle the capture polarity so we catch the opposite edge next time */
        static uint32_t s_polarity = TIM_INPUTCHANNELPOLARITY_RISING;
        if (s_polarity == TIM_INPUTCHANNELPOLARITY_RISING) {
            s_polarity = TIM_INPUTCHANNELPOLARITY_FALLING;
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        } else {
            s_polarity = TIM_INPUTCHANNELPOLARITY_RISING;
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        }

        /* pin == 1 means we just saw a rising edge (end of a low pulse) */
        /* pin == 0 means we just saw a falling edge (end of a high pulse) */

        switch (s_state) {
        case SIF_WAIT_SYNC_LOW:
            if (pin == 1U) {
                /* We measured a low pulse: check if it's the sync */
                if (dt >= SIF_SYNC_MIN_US) {
                    s_state = SIF_WAIT_SYNC_HIGH;
                }
            }
            break;

        case SIF_WAIT_SYNC_HIGH:
            if (pin == 0U) {
                /* We measured the sync high pulse — now start receiving data bits */
                s_state    = SIF_RECV_BIT_HIGH;
                s_bit_idx  = 0;
                s_byte_idx = 0;
                s_byte_buf = 0;
            } else {
                /* Another low pulse before the sync high — restart */
                if (dt >= SIF_SYNC_MIN_US) {
                    /* Still in sync region */
                } else {
                    s_state = SIF_WAIT_SYNC_LOW;
                }
            }
            break;

        case SIF_RECV_BIT_HIGH:
            if (pin == 0U) {
                /* End of the high portion: save its width */
                if (dt < SIF_BIT_MIN_US) {
                    /* Too short — noise, abort frame */
                    sif_reset();
                } else {
                    s_high_us = dt;
                    s_state   = SIF_RECV_BIT_LOW;
                }
            } else {
                /* Unexpected: a rising edge when we expected falling */
                sif_reset();
            }
            break;

        case SIF_RECV_BIT_LOW:
            if (pin == 1U) {
                /* End of the low portion — now we have a full bit */
                uint32_t low_us = dt;
                if (low_us < SIF_BIT_MIN_US) {
                    sif_reset();
                    break;
                }
                /* Bit 0: high > low (2:1 ratio); Bit 1: high < low (1:2 ratio) */
                uint8_t bit = (s_high_us > low_us) ? 0U : 1U;
                sif_push_bit(bit);

                /* Check if we fell back into sync (a new frame starting) */
                if (s_state != SIF_WAIT_SYNC_LOW) {
                    s_state = SIF_RECV_BIT_HIGH;
                }
            } else {
                /* Unexpected falling edge in low portion */
                sif_reset();
            }
            break;

        default:
            sif_reset();
            break;
        }

        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
    }
}

/* Speed calculation per Y29/Y34 protocol spec:
 *   V = (N * D * pi) / (6 * P)   [km/h]
 *   where N = speed_hall_count * 7200 (halls/hour)
 */
float FardriverSIF_SpeedKMH(uint8_t pole_pairs, float wheel_dia_in)
{
    if (pole_pairs == 0U) return 0.0f;
    float wheel_dia_m = wheel_dia_in * 0.0254f;
    float N = (float)g_fardriver_status.speed_hall_count * 7200.0f;
    return (N * wheel_dia_m * 3.14159265f) / (6.0f * (float)pole_pairs);
}
