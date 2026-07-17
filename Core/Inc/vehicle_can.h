#ifndef VEHICLE_CAN_H
#define VEHICLE_CAN_H

#include "main.h"
#include <stdint.h>


typedef enum {
    PRND_INDETERMINATE = 0,
    PRND_NEUTRAL       = 1,
    PRND_PARK          = 2,
    PRND_REVERSE       = 3,
    PRND_DRIVE         = 4,
    PRND_LOW           = 5
} PRND_t;

typedef enum {
    CLUTCH_INDETERMINATE = 0,
    CLUTCH_OPEN          = 1,
    CLUTCH_CLOSED        = 2
} ClutchStatus_t;


/* 0x7A1  Powertrain_Data_1  (8 bytes, 100 ms, from HCU) */
typedef struct {
    PRND_t  prnd;               /* gear lever position */
    uint8_t operating_mode;     /* 0=Indeterminate .. 8=StationaryGen */
    float   battery_soc_pct;   /* 0.0 – 110.0 % */
    float   battery_power_kw;  /* signed, -250 – +250 kW */
    float   battery_temp_c;    /* -40 – +87.5 °C */
    float   hv_bus_voltage_v;  /* 0 – 505 V */
    float   motor_elec_power_kw; /* signed, -250 – +250 kWe */
} PowertrainData1_t;

/* 0x7A2  Powertrain_Data_2  (8 bytes, 100 ms, from HCU) */
typedef struct {
    float        accel_pedal_pct;   /* 0.0 – 110.0 % */
    float        vehicle_speed_mph; /* -25 – +101 MPH */
    float        engine_speed_rpm;  /* 0 – 9000 RPM */
    float        engine_torque_nm;  /* -50 – +587.5 N-m */
    float        motor_speed_rpm;   /* signed, -12700 – +12700 RPM */
    float        motor_torque_nm;   /* signed, -610 – +610 N-m */
    ClutchStatus_t c1_clutch;
    ClutchStatus_t c2_clutch;
    ClutchStatus_t c3_clutch;
    ClutchStatus_t c4_clutch;
} PowertrainData2_t;

/* 0x7A3  Powertrain_Data_3  (8 bytes, 100 ms, from HCU) */
typedef struct {
    float tcase_pressure_bar;  /* 0 – 63.75 bar */
    float c1_pressure_bar;
    float c2_pressure_bar;
    float c4_pressure_bar;
} PowertrainData3_t;

typedef struct {
    PowertrainData1_t pt1;
    PowertrainData2_t pt2;
    PowertrainData3_t pt3;

    /* Convenience aliases (most-used signals) */
    float   accel_pedal_pct;    /* mirrors pt2.accel_pedal_pct */
    PRND_t  gear;               /* mirrors pt1.prnd */
    float   vehicle_speed_mph;  /* mirrors pt2.vehicle_speed_mph */

    /* RX timestamps (HAL_GetTick ms) — one per message ID */
    uint32_t ts_0x7A1;
    uint32_t ts_0x7A2;
    uint32_t ts_0x7A3;
} VehicleCANData_t;

extern VehicleCANData_t g_vehicle;


#define VEHICLE_IN_DRIVE() (g_vehicle.gear == PRND_DRIVE)
#define VEHICLE_IN_REVERSE() (g_vehicle.gear == PRND_REVERSE)
#define VEHICLE_IN_PARK() (g_vehicle.gear == PRND_PARK || g_vehicle.gear == PRND_NEUTRAL)
#define VEHICLE_MOVING() (g_vehicle.vehicle_speed_mph > 2.0f)


/* Call once after CAN1 is initialised by CubeMX */
void VehicleCAN_Init(void);

/* Call from USB_LP_CAN_RX0_IRQHandler in stm32f3xx_it.c */
void VehicleCAN_RxIRQHandler(void);

/* Returns 1 if at least one message arrived within timeout_ms */
uint8_t VehicleCAN_IsAlive(uint32_t timeout_ms);

#endif /* VEHICLE_CAN_H */
