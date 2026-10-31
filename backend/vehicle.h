/* vehicle.h - vehicle presets and longitudinal motion. */
#ifndef VEHICLE_H
#define VEHICLE_H

#include "engine.h"
#include "transmission.h"

#define GRAVITY        9.81
#define AIR_DENSITY    1.2
#define VEHICLE_COUNT  3

typedef struct {
    char   name[24];
    char   description[64];
    double mass_kg;
    double drag_area_m2;        /* Cd * frontal area */
    double rolling_coeff;
    double wheel_radius_m;
    double tank_l;
    double max_brake_n;
    EngineSpec       engine;
    TransmissionSpec transmission;
} VehicleSpec;

typedef struct {
    VehicleSpec spec;
    double speed_mps;     /* signed: negative = reversing */
    double position_m;    /* signed position along the road */
    double fuel_l;
    double accel_mps2;
    double slope_pct;     /* + uphill, - downhill */
} Vehicle;

int                vehicle_preset_count(void);
const VehicleSpec *vehicle_preset(int index);               /* NULL if out of range */
int                vehicle_init(Vehicle *v, int preset);    /* 0 ok, -1 error */
void               vehicle_integrate(Vehicle *v, double drive_force_n,
                                     double brake_force_n, double dt);

#endif
