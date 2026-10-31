#include "engine.h"
#include <math.h>
#include <string.h>

#define RADS_TO_RPM         9.549296585513721
#define RPM_TO_RADS         0.104719755119660
#define ENGINE_EFFICIENCY   0.30        /* fuel energy -> crank work */
#define FUEL_ENERGY_J_PER_L 32.0e6      /* petrol */

static double clampd(double x, double lo, double hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

void engine_init(Engine *e, const EngineSpec *spec)
{
    if (e == NULL || spec == NULL) return;
    memset(e, 0, sizeof *e);
    e->spec = *spec;
    e->temp_c = 65.0;
}

EngineStatus engine_start(Engine *e, double fuel_l)
{
    if (e == NULL)                          return ENGINE_ERR_NULL;
    if (e->running)                         return ENGINE_ERR_ALREADY_RUNNING;
    if (fuel_l <= 0.0)                      return ENGINE_ERR_NO_FUEL;
    if (e->temp_c >= ENGINE_TEMP_RESTART)   return ENGINE_ERR_OVERHEATED;
    e->running    = true;
    e->rpm        = e->spec.idle_rpm;
    e->omega_rads = e->rpm * RPM_TO_RADS;
    return ENGINE_OK;
}

void engine_stop(Engine *e)
{
    if (e == NULL) return;
    e->running = false;
    e->fuel_rate_lps = 0.0;
    e->heat_w = 0.0;
}

double engine_curve_torque(const EngineSpec *spec, double rpm)
{
    double pos, frac;
    int i;
    if (rpm <= 0.0) return spec->peak_torque_nm * spec->torque_curve[0];
    pos = rpm / 1000.0;
    i = (int)pos;
    if (i >= TORQUE_POINTS - 1)
        return spec->peak_torque_nm * spec->torque_curve[TORQUE_POINTS - 1];
    frac = pos - i;   /* linear interpolation between table entries */
    return spec->peak_torque_nm *
           (spec->torque_curve[i] * (1.0 - frac) + spec->torque_curve[i + 1] * frac);
}

double engine_produce_torque(Engine *e, double throttle, double power_mult, bool fuel_cut)
{
    const EngineSpec *s = &e->spec;
    double th = e->running ? clampd(throttle, 0.0, 1.0) : 0.0;
    double fric = s->friction_base_nm + s->friction_per_rpm * e->rpm * (1.0 - 0.6 * th);
    double gross = 0.0, idle = 0.0, fuel_power = 0.0;

    if (e->omega_rads <= 0.0) fric = 0.0;

    if (e->running) {
        bool cut = fuel_cut || e->rpm >= s->redline_rpm;   /* DFCO or rev limiter */
        if (!cut) {
            gross = th * engine_curve_torque(s, e->rpm) * power_mult;
            if (e->rpm < s->idle_rpm)
                idle = clampd(fric + 0.30 * (s->idle_rpm - e->rpm), 0.0, s->idle_gov_max_nm);
            fuel_power = (gross + idle) * e->omega_rads / ENGINE_EFFICIENCY;
        }
        /* coolant heat; extra friction heat when held near the redline */
        double hot = (e->rpm / s->redline_rpm - 0.85) / 0.15;
        e->heat_w = 0.36 * fuel_power * (1.0 + 1.5 * clampd(hot, 0.0, 1.0));
    } else {
        e->heat_w = 0.0;
    }

    e->gross_torque_nm = gross + idle;
    e->net_torque_nm   = gross + idle - fric;
    e->fuel_rate_lps   = fuel_power / FUEL_ENERGY_J_PER_L;
    return e->net_torque_nm;
}

void engine_integrate(Engine *e, double clutch_torque_nm, double dt)
{
    e->omega_rads += (e->net_torque_nm - clutch_torque_nm) / e->spec.inertia_kgm2 * dt;
    if (e->omega_rads < 0.0) e->omega_rads = 0.0;
    e->rpm = e->omega_rads * RADS_TO_RPM;
}

bool engine_should_stall(const Engine *e)
{
    return e->running && e->rpm < e->spec.stall_rpm;
}

void engine_update_thermal(Engine *e, double speed_mps, double dt)
{
    /* Simplified lumped model: heat in - (radiator + fan) heat out. */
    double ua = 20.0;                                  /* W/K, thermostat closed */
    if (e->temp_c > 88.0) ua += 40.0 + 6.0 * fabs(speed_mps);   /* radiator + ram air */
    if (e->temp_c > 96.0) ua += 150.0;                 /* cooling fan */
    e->temp_c += (e->heat_w - ua * (e->temp_c - AMBIENT_TEMP_C))
                 / e->spec.heat_capacity_j_per_k * dt;
}
