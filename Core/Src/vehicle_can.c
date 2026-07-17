/* vehicle_can.c
 * Ford F-150 PT CAN decoder - P415_11_v11_HSCGEA_Release.dbc
 *
 * CAN RX: PB8 (Waveshare RS485 CAN Shield), 500 kbps
 *
 * Message 0x201 (513) _EngVehicleSpThrottle (8 bytes, 8 ms, PCM):
 *   EngAout_N_Actl     :  7|13@0+ (2,0)    RPM
 *   Veh_V_ActlEng      : 23|16@0+ (0.01,0) kph
 *   ApedPos_Pc_ActlArb : 39|10@0+ (0.1,0)  %
 *
 * Message 0x230 (560) gear lever (SCCM/TCCM):
 *   GearLvrPos_D_Actl  : 23|4@0+  (1,0)   enum
 *
 * All signals Motorola (big-endian) byte order.
 */

#include "vehicle_can.h"
#include <string.h>
#include <stdio.h>

VehicleCANData_t g_vehicle;

extern CAN_HandleTypeDef hcan;

#define CAN_ID_ENG_VEH_THROTTLE  0x201U
#define CAN_ID_ENG_BRAKE_DATA    0x165U
#define CAN_ID_GEAR              0x230U

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

/* 0x201 _EngVehicleSpThrottle */
static void parse_eng_veh_throttle(const uint8_t *d, uint8_t dlc)
{
    if (dlc < 6U) return;

    /* EngAout_N_Actl : 7|13@0+ (2,0)
     * byte 0 [7:0] + byte 1 [7:3] */
    {
        uint16_t raw = ((uint16_t)d[0] << 5) | (d[1] >> 3);
        g_vehicle.engine_speed_rpm = (float)raw * 2.0f;
    }

    /* Veh_V_ActlEng : 23|16@0+ (0.01,0)
     * byte 2 + byte 3 */
    {
        uint16_t raw = ((uint16_t)d[2] << 8) | d[3];
        g_vehicle.vehicle_speed_kph = (float)raw * 0.01f;
    }

    /* ApedPos_Pc_ActlArb : 39|10@0+ (0.1,0)
     * byte 4 [7:0] + byte 5 [7:6] */
    {
        uint16_t raw = ((uint16_t)d[4] << 2) | (d[5] >> 6);
        g_vehicle.accel_pedal_pct = (float)raw * 0.1f;
    }

    g_vehicle.ts_0x201 = HAL_GetTick();
}

/* 0x230 gear lever */
static void parse_gear(const uint8_t *d, uint8_t dlc)
{
    if (dlc < 3U) return;

    /* GearLvrPos_D_Actl : 23|4@0+ (1,0) - byte 2 bits [7:4] */
    g_vehicle.gear = (PRND_t)((d[2] >> 4) & 0x0FU);
    g_vehicle.ts_0x230 = HAL_GetTick();
}

/* 0x165 _EngBrakeData */
static void parse_eng_brake_data(const uint8_t *d, uint8_t dlc)
{
    if (dlc < 4U) return;

    /* BpedDrvAppl_D_Actl : 31|2@0+ (1,0) - byte 3 bits [7:6]
     * 2 = Driver_Braking, 1 = Driver_Not_Braking */
    uint8_t val = (d[3] >> 6) & 0x03U;
    g_vehicle.brake_applied = (val == 2U) ? 1U : 0U;
    g_vehicle.ts_0x165 = HAL_GetTick();
}

void VehicleCAN_Init(void)
{
    memset(&g_vehicle, 0, sizeof(g_vehicle));
    g_vehicle.gear = PRND_UNKNOWN;
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
        case CAN_ID_ENG_VEH_THROTTLE: parse_eng_veh_throttle(data, (uint8_t)hdr.DLC); break;
        case CAN_ID_ENG_BRAKE_DATA:   parse_eng_brake_data(data, (uint8_t)hdr.DLC);   break;
        case CAN_ID_GEAR:             parse_gear(data, (uint8_t)hdr.DLC);              break;
        default: break;
        }
        printf("[%lu] 0x%03lX [%u]  %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            HAL_GetTick(), hdr.StdId, (unsigned)hdr.DLC,
            data[0], data[1], data[2], data[3],
            data[4], data[5], data[6], data[7]);
    }
}

uint8_t VehicleCAN_IsAlive(uint32_t timeout_ms)
{
    uint32_t now = HAL_GetTick();
    return ((now - g_vehicle.ts_0x201) < timeout_ms ||
            (now - g_vehicle.ts_0x230) < timeout_ms) ? 1U : 0U;
}
