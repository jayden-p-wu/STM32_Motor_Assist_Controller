/* vehicle_can.c
 * F-150 PT CAN decoder — F150_PT_CAN_External_Data.dbc
 *
 * CAN1: PB8 = CAN_RX, PB9 = CAN_TX  (Waveshare RS485 CAN Shield)
 * Baud: 500 kbps
 *
 * All signals use Motorola (big-endian) byte order as specified in the DBC.
 * Bit extraction comments reference the DBC notation: startbit|length@0+/-
 */

#include "vehicle_can.h"
#include <string.h>
#include <stdio.h>

VehicleCANData_t g_vehicle;

extern CAN_HandleTypeDef hcan;

/* ---- Message IDs (11-bit standard) ---- */
#define CAN_ID_PT1   0x7A1U   /* Powertrain_Data_1 */
#define CAN_ID_PT2   0x7A2U   /* Powertrain_Data_2 */
#define CAN_ID_PT3   0x7A3U   /* Powertrain_Data_3 */

/* ---- CAN filter: accept all, filter by ID in software ---- */
static void can_filter_init(void)
{
    CAN_FilterTypeDef f = {0};
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    HAL_CAN_ConfigFilter(&hcan, &f);
}

/* ---- Sign-extension helpers ---- */
static inline int16_t sext10(uint16_t v)
{
    return (v & 0x200U) ? (int16_t)(v | 0xFC00U) : (int16_t)v;
}
static inline int16_t sext9(uint16_t v)
{
    return (v & 0x100U) ? (int16_t)(v | 0xFE00U) : (int16_t)v;
}

/* ================================================================== */
/* 0x7A1  Powertrain_Data_1                                            */
/*                                                                     */
/*  Battery_SOC      :  7|8  @0+  (0.5,0)      byte 0                 */
/*  Battery_Power    : 23|10 @0-  (0.5,0)      bytes 2-1 (top 2 bits) */
/*  Battery_Temp     : 15|8  @0+  (0.5,-40)    byte 1                 */
/*  HV_Bus_Voltage   : 55|9  @0+  (1,0)        bytes 6+top1 of byte5  */
/*  Motor_Elec_Power : 39|10 @0-  (0.5,0)      bytes 4+top2 of byte3  */
/*  Operating_Mode   : 29|4  @0+  (1,0)        byte 3 bits [5:2]      */
/*  PRND             : 58|3  @0+  (1,0)        byte 7 bits [2:0]      */
/* ================================================================== */
static void parse_pt1(const uint8_t *d, uint8_t dlc)
{
    if (dlc < 8U) return;
    PowertrainData1_t *p = &g_vehicle.pt1;

    /* Battery_SOC : 7|8 @0+ — byte 0 entirely */
    p->battery_soc_pct = (float)d[0] * 0.5f;

    /* Battery_Temp : 15|8 @0+ — byte 1 entirely */
    p->battery_temp_c = (float)d[1] * 0.5f - 40.0f;

    /* Battery_Power : 23|10 @0- — byte 2 (MSB 8 bits) + byte 1 bits [7:6] */
    {
        uint16_t raw = ((uint16_t)d[2] << 2) | ((d[1] & 0xC0U) >> 6);
        p->battery_power_kw = (float)sext10(raw) * 0.5f;
    }

    /* Motor_Elec_Power : 39|10 @0- — byte 4 (MSB 8 bits) + byte 3 bits [7:6] */
    {
        uint16_t raw = ((uint16_t)d[4] << 2) | ((d[3] & 0xC0U) >> 6);
        p->motor_elec_power_kw = (float)sext10(raw) * 0.5f;
    }

    /* HV_Bus_Voltage : 55|9 @0+ — byte 6 (MSB 8 bits) + byte 5 bit 7 */
    {
        uint16_t raw = ((uint16_t)d[6] << 1) | ((d[5] & 0x80U) >> 7);
        p->hv_bus_voltage_v = (float)raw;
    }

    /* Operating_Mode : 29|4 @0+ — byte 3 bits [5:2] */
    p->operating_mode = (d[3] >> 2) & 0x0FU;

    /* PRND : 58|3 @0+ — byte 7 bits [2:0] */
    p->prnd = (PRND_t)(d[7] & 0x07U);

    /* Update convenience alias */
    g_vehicle.gear = p->prnd;
    g_vehicle.ts_0x7A1 = HAL_GetTick();
}

/* ================================================================== */
/* 0x7A2  Powertrain_Data_2                                            */
/*                                                                     */
/*  Engine_Speed   :  7|10 @0+  (10,0)       bytes 0+top2 of byte 1   */
/*  Motor_Speed    : 23|10 @0-  (25,0)       bytes 2+top2 of byte 1   */
/*  Motor_Torque   : 39|9  @0-  (2.5,0)      bytes 4+top1 of byte 3   */
/*  C1_Clutch      : 11|2  @0+  (1,0)        byte 1 bits [3:2]        */
/*  C2_Clutch      :  9|2  @0+  (1,0)        byte 1 bits [1:0]        */
/*  C3_Clutch      : 29|2  @0+  (1,0)        byte 3 bits [5:4]        */
/*  C4_Clutch      : 27|2  @0+  (1,0)        byte 3 bits [3:2]        */
/*  AccelPedal     : 46|7  @0+  (1,0)        byte 5 bits [6:0]        */
/*  Engine_Torque  : 55|8  @0+  (2.5,-50)    byte 6                   */
/*  Vehicle_Speed  : 63|7  @0+  (1,-25)      byte 7 bits [7:1]        */
/* ================================================================== */
static void parse_pt2(const uint8_t *d, uint8_t dlc)
{
    if (dlc < 8U) return;
    PowertrainData2_t *p = &g_vehicle.pt2;

    /* Engine_Speed : 7|10 @0+ — byte 0 (MSB 8) + byte 1 bits [7:6] */
    {
        uint16_t raw = ((uint16_t)d[0] << 2) | ((d[1] & 0xC0U) >> 6);
        p->engine_speed_rpm = (float)raw * 10.0f;
    }

    /* Motor_Speed : 23|10 @0- — byte 2 (MSB 8) + byte 1 bits [7:6]
     * Note: byte 1 bits [7:6] are the LOW 2 bits of both Engine_Speed and
     * Motor_Speed since they share the same bit positions in the DBC.
     * Motor_Speed MSB = bit 23 (byte 2), next 2 bits = bits 15,14 (byte 1 b7,b6) */
    {
        uint16_t raw = ((uint16_t)d[2] << 2) | ((d[1] & 0xC0U) >> 6);
        p->motor_speed_rpm = (float)sext10(raw) * 25.0f;
    }

    /* Motor_Torque : 39|9 @0- — byte 4 (MSB 8) + byte 3 bit 7 */
    {
        uint16_t raw = ((uint16_t)d[4] << 1) | ((d[3] & 0x80U) >> 7);
        p->motor_torque_nm = (float)sext9(raw) * 2.5f;
    }

    /* Clutch statuses — byte 1 and byte 3 */
    p->c1_clutch = (ClutchStatus_t)((d[1] >> 2) & 0x03U);  /* bits [3:2] */
    p->c2_clutch = (ClutchStatus_t)(d[1] & 0x03U);          /* bits [1:0] */
    p->c3_clutch = (ClutchStatus_t)((d[3] >> 4) & 0x03U);  /* bits [5:4] */
    p->c4_clutch = (ClutchStatus_t)((d[3] >> 2) & 0x03U);  /* bits [3:2] */

    /* AccelPedal : 46|7 @0+ — byte 5 bits [6:0] */
    p->accel_pedal_pct = (float)(d[5] & 0x7FU);

    /* Engine_Torque : 55|8 @0+ — byte 6 */
    p->engine_torque_nm = (float)d[6] * 2.5f - 50.0f;

    /* Vehicle_Speed : 63|7 @0+ — byte 7 bits [7:1] */
    p->vehicle_speed_mph = (float)(d[7] >> 1) - 25.0f;

    /* Update convenience aliases */
    g_vehicle.accel_pedal_pct = p->accel_pedal_pct;
    g_vehicle.vehicle_speed_mph = p->vehicle_speed_mph;
    g_vehicle.ts_0x7A2 = HAL_GetTick();
}

/* ================================================================== */
/* 0x7A3  Powertrain_Data_3                                            */
/*                                                                     */
/*  TCase_Supply_Pressure :  7|8 @0+ (0.25,0)  byte 0                 */
/*  C1_Pressure           : 15|8 @0+ (0.25,0)  byte 1                 */
/*  C2_Pressure           : 23|8 @0+ (0.25,0)  byte 2                 */
/*  C4_Pressure           : 31|8 @0+ (0.25,0)  byte 3                 */
/* ================================================================== */
static void parse_pt3(const uint8_t *d, uint8_t dlc)
{
    if (dlc < 4U) return;
    PowertrainData3_t *p = &g_vehicle.pt3;
    p->tcase_pressure_bar = (float)d[0] * 0.25f;
    p->c1_pressure_bar    = (float)d[1] * 0.25f;
    p->c2_pressure_bar    = (float)d[2] * 0.25f;
    p->c4_pressure_bar    = (float)d[3] * 0.25f;
    g_vehicle.ts_0x7A3 = HAL_GetTick();
}

/* ---- Public API ---- */
void VehicleCAN_Init(void)
{
    memset(&g_vehicle, 0, sizeof(g_vehicle));
    g_vehicle.pt1.prnd = PRND_INDETERMINATE;
    can_filter_init();
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan);
}

void VehicleCAN_RxIRQHandler(void)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t             data[8];

    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0U) {
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &hdr, data) != HAL_OK) {
            break;
        }
        switch (hdr.StdId) {
        case CAN_ID_PT1: parse_pt1(data, (uint8_t)hdr.DLC); break;
        case CAN_ID_PT2: parse_pt2(data, (uint8_t)hdr.DLC); break;
        case CAN_ID_PT3: parse_pt3(data, (uint8_t)hdr.DLC); break;
        default: break;
        }
        /* printf("[%lu] 0x%03lX [%u]  %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            HAL_GetTick(), hdr.StdId, (unsigned)hdr.DLC,
            data[0], data[1], data[2], data[3],
            data[4], data[5], data[6], data[7]); */
    }
}

uint8_t VehicleCAN_IsAlive(uint32_t timeout_ms)
{
    uint32_t now = HAL_GetTick();
    return ((now - g_vehicle.ts_0x7A1) < timeout_ms ||
            (now - g_vehicle.ts_0x7A2) < timeout_ms ||
            (now - g_vehicle.ts_0x7A3) < timeout_ms) ? 1U : 0U;
}
