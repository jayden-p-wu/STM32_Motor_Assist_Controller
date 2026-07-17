#ifndef VEHICLE_CAN_H
#define VEHICLE_CAN_H

#include "main.h"
#include <stdint.h>

/*
 * Ford F-150 PT CAN — P415_11_v11_HSCGEA_Release.dbc
 *
 * Message 0x201 (513) _EngVehicleSpThrottle — 8 bytes, 8 ms, from PCM
 *   EngAout_N_Actl      :  7|13@0+ (2,0)    RPM
 *   Veh_V_ActlEng       : 23|16@0+ (0.01,0) kph
 *   ApedPos_Pc_ActlArb  : 39|10@0+ (0.1,0)  %
 *
 * Message 0x230 (560) — gear lever, from SCCM/TCCM
 *   GearLvrPos_D_Actl   : 23|4@0+  (1,0)    enum
 */

/* GearLvrPos_D_Actl values */
typedef enum {
    PRND_PARK    = 0,
    PRND_REVERSE = 1,
    PRND_NEUTRAL = 2,
    PRND_DRIVE   = 3,
    PRND_SPORT   = 4,
    PRND_LOW     = 5,
    PRND_UNKNOWN = 15
} PRND_t;

typedef struct {
    float    accel_pedal_pct;   /* ApedPos_Pc_ActlArb  0.0–102.3 % */
    float    vehicle_speed_kph; /* Veh_V_ActlEng        0.0–655 kph */
    float    engine_speed_rpm;  /* EngAout_N_Actl       0–16382 RPM */
    PRND_t   gear;              /* GearLvrPos_D_Actl */
    uint8_t  brake_applied;     /* BpedDrvAppl_D_Actl: 1 = driver braking */

    uint32_t ts_0x201;
    uint32_t ts_0x165;
    uint32_t ts_0x230;
} VehicleCANData_t;

extern VehicleCANData_t g_vehicle;

/* Drive = D, Sport, or Low — all forward drive ranges */
#define VEHICLE_IN_DRIVE()   (g_vehicle.gear == PRND_DRIVE  || \
                              g_vehicle.gear == PRND_SPORT  || \
                              g_vehicle.gear == PRND_LOW)
#define VEHICLE_IN_REVERSE() (g_vehicle.gear == PRND_REVERSE)
#define VEHICLE_IN_PARK()    (g_vehicle.gear == PRND_PARK || g_vehicle.gear == PRND_NEUTRAL)
#define VEHICLE_MOVING()     (g_vehicle.vehicle_speed_kph > 3.2f)  /* ~2 mph */

void    VehicleCAN_Init(void);
void    VehicleCAN_RxIRQHandler(void);
uint8_t VehicleCAN_IsAlive(uint32_t timeout_ms);

#endif /* VEHICLE_CAN_H */
