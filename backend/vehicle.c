#include "vehicle.h"
#include <math.h>
#include <string.h>

#define STANDSTILL_MPS 0.02

static const VehicleSpec PRESETS[VEHICLE_COUNT] = {
    {   /* small petrol hatchback */
        "City Hatch", "Light and economical. Gentle torque, short revs.",
        1050.0, 0.62, 0.012, 0.29, 40.0, 7500.0,
        { 130.0, 850.0, 6200.0, 350.0, 0.18, 9.0, 0.005, 80.0, 18000.0,
          { 0.50, 0.65, 0.82, 0.95, 1.00, 0.97, 0.88, 0.70, 0.50 } },
        { { 3.45, 1.95, 1.30, 0.97, 0.78 }, 3.30, 4.10, 0.92, 220.0, 0.15, 0.85 }
    },
    {   /* sport coupe */
        "Sport Coupe", "Strong top-end power and close gear ratios.",
        1280.0, 0.66, 0.012, 0.31, 55.0, 9500.0,
        { 240.0, 900.0, 7200.0, 400.0, 0.22, 11.0, 0.006, 95.0, 20000.0,
          { 0.55, 0.70, 0.84, 0.93, 0.98, 1.00, 0.97, 0.88, 0.70 } },
        { { 3.60, 2.10, 1.50, 1.15, 0.92 }, 3.40, 3.70, 0.92, 380.0, 0.15, 0.85 }
    },
    {   /* diesel-like cargo van */
        "Cargo Van", "Heavy, torquey and low-revving. Punishes lugging.",
        1850.0, 0.95, 0.014, 0.33, 70.0, 11000.0,
        { 280.0, 800.0, 4800.0, 400.0, 0.35, 14.0, 0.008, 110.0, 26000.0,
          { 0.55, 0.80, 0.97, 1.00, 0.96, 0.82, 0.55, 0.25, 0.10 } },
        { { 4.20, 2.30, 1.40, 1.00, 0.80 }, 3.90, 4.00, 0.90, 420.0, 0.15, 0.85 }
    }
};

int vehicle_preset_count(void) { return VEHICLE_COUNT; }

const VehicleSpec *vehicle_preset(int index)
{
    return (index >= 0 && index < VEHICLE_COUNT) ? &PRESETS[index] : NULL;
}

int vehicle_init(Vehicle *v, int preset)
{
    const VehicleSpec *p = vehicle_preset(preset);
    if (v == NULL || p == NULL) return -1;
    memset(v, 0, sizeof *v);
    v->spec = *p;
    v->fuel_l = p->tank_l;
    return 0;
}

/* One integration step of F = m*a with drive, brake, rolling, grade and air drag. */
void vehicle_integrate(Vehicle *v, double drive_force_n, double brake_force_n, double dt)
{
    double m     = v->spec.mass_kg;
    double slope = atan(v->slope_pct / 100.0);
    double f_ext = drive_force_n - m * GRAVITY * sin(slope);
    double f_roll = v->spec.rolling_coeff * m * GRAVITY * cos(slope);
    double vel = v->speed_mps, a;

    if (fabs(vel) < STANDSTILL_MPS) {
        /* At rest: brakes and rolling resistance can hold the car (static friction). */
        double hold = brake_force_n + f_roll;
        if (fabs(f_ext) <= hold) {
            a = 0.0;
            vel = 0.0;
        } else {
            a = (f_ext - copysign(hold, f_ext)) / m;
            vel += a * dt;
        }
    } else {
        double sgn  = vel > 0.0 ? 1.0 : -1.0;
        double drag = 0.5 * AIR_DENSITY * v->spec.drag_area_m2 * vel * fabs(vel);
        double nv;
        a  = (f_ext - sgn * (brake_force_n + f_roll) - drag) / m;
        nv = vel + a * dt;
        if (nv * vel < 0.0) nv = 0.0;     /* brakes/resistance cannot reverse the car */
        vel = nv;
    }
    v->speed_mps  = vel;
    v->accel_mps2 = a;
    v->position_m += vel * dt;
}
