/* fardriver_sif.c
 * Y29/Y34 SIF one-line protocol decoder using TIM1 input capture on PA8.
 *
 * Timer setup (configure in CubeMX before calling FardriverSIF_Init):
 *   TIM1, prescaler = 71, period = 0xFFFF
 *   CH1 on PA8: input capture, IC1Selection = Direct TI1, IC1Filter = 0
 *   Use GPIO pull-down so the single-wire bus idles low when the display is
 *   disconnected or not transmitting.
 *   Enable TIM1 capture/compare interrupt.
 *
 * How it works:
 *   The ISR is called on every edge of the SIF signal.  We measure the
 *   high-pulse width and low-pulse width of each bit, then classify the bit
 *   by which was longer.  A very long low pulse (> ~31 ms) is the sync.
 */

#include "fardriver_sif.h"
#include "main.h"
#include <string.h>

/* --- Protocol constants, derived from the documented Tosc range (32-320 us),
 * not from any one observed trace -- so decode stays correct even if this
 * bus (or another unit) runs at a different Tosc than we've measured.
 *   min data-bit half-phase = 32 * Tosc_min = 32 * 32   =  1024 us
 *   max data-bit half-phase = 64 * Tosc_max = 64 * 320  = 20480 us
 *   min sync low pulse      = 992 * Tosc_min = 992 * 32 = 31744 us
 * SIF_BIT_MAX_US and SIF_SYNC_MIN_US straddle the gap between the largest
 * possible bit phase and the smallest possible sync, so the two can never
 * be confused regardless of the bus's actual Tosc. */
#define SIF_SYNC_MIN_US    26000U   /* long low pulse for sync detection */
#define SIF_BIT_MIN_US       512U   /* smallest valid single-phase duration for a data bit */
#define SIF_BIT_MAX_US     22000U   /* largest valid single-phase duration for a data bit */
#define SIF_NOISE_MAX_US     700U   /* ignore short noise/glitch edges (EMI); documented min
                                      * real bit phase is 1024us, so this has large margin */
/* If no edges for this long, abandon current frame parsing (µs) */
#define SIF_FRAME_TIMEOUT_US 200000U

#define SIF_FRAME_BYTES      12U
#define SIF_DEVICE_CODE     0x08U
#define SIF_SEQ_CODE        0x61U

/* Published state */
FardriverSIF_t g_fardriver_status;
volatile uint8_t g_fardriver_sif_debug_event = SIF_DEBUG_NONE;
volatile uint32_t g_fardriver_sif_last_dt = 0;
volatile uint8_t  g_fardriver_sif_last_pin = 0;
/* Snapshot of last invalid frame for debug */
volatile uint8_t g_fardriver_sif_last_frame[SIF_FRAME_BYTES];
volatile uint8_t g_fardriver_sif_last_frame_len = 0;

static uint16_t    s_edge_ts    = 0;      /* timestamp of last edge (µs, 16-bit timer wrap) */
static uint8_t     s_sync_seen  = 0;      /* long low observed, waiting out the meaningless high */
static uint8_t     s_have_low   = 0;      /* s_pending_low holds this bit's low-phase duration */
static uint32_t    s_pending_low = 0;     /* low-phase duration of the bit currently being timed */
static uint8_t     s_byte_buf   = 0;
static uint8_t     s_bit_idx    = 0;      /* bit within current byte (0..7, MSB first) */
static uint8_t     s_byte_idx   = 0;      /* byte within frame */
static uint8_t     s_frame[SIF_FRAME_BYTES];

extern TIM_HandleTypeDef htim1;

/* ------------------------------------------------------------------ */
static void sif_reset(void)
{
    s_sync_seen   = 0;
    s_have_low    = 0;
    s_pending_low = 0;
    s_bit_idx    = 0;
    s_byte_idx   = 0;
    s_byte_buf   = 0;
}

/* Snapshot whatever bytes were decoded so far, for debug, then reset. */
static void sif_fail(uint8_t event)
{
    uint8_t copy_len = (s_byte_idx < SIF_FRAME_BYTES) ? s_byte_idx : SIF_FRAME_BYTES;
    for (uint8_t i = 0; i < copy_len; i++) g_fardriver_sif_last_frame[i] = s_frame[i];
    g_fardriver_sif_last_frame_len = copy_len;
    g_fardriver_sif_debug_event = event;
    sif_reset();
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
                sif_fail(SIF_DEBUG_INVALID_FRAME);
                return;
            }

            /* Verify XOR checksum (DATA0..DATA10 XOR'd = DATA11) */
            uint8_t chk = 0;
            for (uint8_t i = 0; i < SIF_FRAME_BYTES - 1U; i++) {
                chk ^= s_frame[i];
            }
            if (chk != s_frame[SIF_FRAME_BYTES - 1U]) {
                sif_fail(SIF_DEBUG_INVALID_FRAME);
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

    /* Start TIM1 in input-capture mode on CH1 */
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
}

/* Called from TIM1_CC_IRQHandler — handle input capture event
 *
 * Each bit is one LOW phase followed by one HIGH phase (see header comment).
 * A rising edge ends a LOW phase (dt = low duration); a falling edge ends a
 * HIGH phase (dt = high duration). We pair the low duration measured at a
 * rising edge with the high duration measured at the very next falling edge
 * -- that pair belongs to the same bit. The frame sync is simply a LOW phase
 * far longer than any data bit's low phase (> SIF_SYNC_MIN_US), followed by
 * one "meaningless" high phase that is discarded before DATA0 begins.
 */
void FardriverSIF_TIM1_IRQHandler(void)
{
    uint32_t sr = TIM1->SR;

    /* Input capture on CH1 */
    if (sr & TIM_SR_CC1IF) {
        uint16_t ts   = (uint16_t)HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
        uint32_t pin  = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);  /* current level after edge */
        uint16_t dt16 = (uint16_t)(ts - s_edge_ts);
        uint32_t dt   = (uint32_t)dt16;                       /* µs elapsed since last edge, wrap-safe */

        /* Reject very short edges as noise: keep the old edge timestamp so the
         * glitch is absorbed into the surrounding phase instead of splitting it. */
        if (dt < SIF_NOISE_MAX_US) {
            g_fardriver_sif_debug_event = SIF_DEBUG_SHORT_PULSE;
            __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
            return;
        }
        s_edge_ts = ts;

        g_fardriver_sif_last_dt = dt;
        g_fardriver_sif_last_pin = (uint8_t)pin;
        g_fardriver_sif_debug_event = SIF_DEBUG_CAPTURE;

        /* If there is a very long gap, abandon any in-progress frame parse. */
        if (dt > SIF_FRAME_TIMEOUT_US) {
            sif_reset();
            __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
            return;
        }

        if (pin == 1U) {
            /* Rising edge: dt is the LOW phase duration that just ended. */
            if (dt >= SIF_SYNC_MIN_US) {
                /* Long low = start of frame. The following high is meaningless;
                 * discard it (handled below via s_sync_seen) and start fresh. */
                memset(s_frame, 0, sizeof(s_frame));
                s_byte_idx  = 0;
                s_bit_idx   = 0;
                s_byte_buf  = 0;
                s_have_low  = 0;
                s_sync_seen = 1U;
                g_fardriver_sif_debug_event = SIF_DEBUG_SYNC_LOW;
                __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
                return;
            }

            if (!s_sync_seen) {
                /* Not synced yet; ignore data bits until we see the long low. */
                __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
                return;
            }

            if (dt < SIF_BIT_MIN_US || dt > SIF_BIT_MAX_US) {
                sif_fail(SIF_DEBUG_INVALID_FRAME);
                __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
                return;
            }

            /* This low phase belongs to the next bit (skip one after sync: the
             * meaningless post-sync high is discarded below, not here). */
            s_pending_low = dt;
            s_have_low    = 1U;
        } else {
            /* Falling edge: dt is the HIGH phase duration that just ended. */
            if (!s_sync_seen) {
                __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
                return;
            }

            if (!s_have_low) {
                /* This is the meaningless high right after the sync's long low. */
                g_fardriver_sif_debug_event = SIF_DEBUG_SYNC_HIGH;
                __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
                return;
            }

            if (dt < SIF_BIT_MIN_US || dt > SIF_BIT_MAX_US) {
                sif_fail(SIF_DEBUG_INVALID_FRAME);
                __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
                return;
            }

            /* Decide the bit from the low:high ratio of this single bit period
             * (bit0 = low longer, bit1 = high longer; nominal ratio 2:1). */
            uint32_t low = s_pending_low;
            uint8_t bit;
            if (dt * 3U >= low * 4U) {
                bit = 1U;
            } else if (low * 3U >= dt * 4U) {
                bit = 0U;
            } else {
                sif_fail(SIF_DEBUG_INVALID_FRAME);
                __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
                return;
            }

            s_have_low = 0U;
            sif_push_bit(bit);
        }

        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
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
