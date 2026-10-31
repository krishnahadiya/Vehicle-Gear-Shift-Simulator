/* engine.h - engine model: torque curve, friction, fuel use, heat. */
#ifndef ENGINE_H
#define ENGINE_H

#include <stdbool.h>

#define TORQUE_POINTS        9      /* curve sampled every 1000 rpm: 0..8000 */
#define AMBIENT_TEMP_C       25.0
#define ENGINE_TEMP_WARN     105.0
#define ENGINE_TEMP_SHUTDOWN 120.0
#define ENGINE_TEMP_RESTART  100.0

typedef struct {
    double peak_torque_nm;
    double idle_rpm;
    double redline_rpm;
    double stall_rpm;
    double inertia_kgm2;
    double friction_base_nm;
    double friction_per_rpm;
    double idle_gov_max_nm;                  /* max torque of the idle governor */
    double heat_capacity_j_per_k;
    double torque_curve[TORQUE_POINTS];      /* fraction of peak torque */
} EngineSpec;

typedef struct {
    EngineSpec spec;
    double omega_rads;       /* crankshaft angular speed */
    double rpm;
    double temp_c;
    bool   running;
    double gross_torque_nm;  /* throttle + idle governor torque */
    double net_torque_nm;    /* gross - friction */
    double fuel_rate_lps;
    double heat_w;           /* heat sent to the coolant */
} Engine;

typedef enum {
    ENGINE_OK = 0,
    ENGINE_ERR_NULL,
    ENGINE_ERR_NO_FUEL,
    ENGINE_ERR_OVERHEATED,
    ENGINE_ERR_ALREADY_RUNNING
} EngineStatus;

void         engine_init(Engine *e, const EngineSpec *spec);
EngineStatus engine_start(Engine *e, double fuel_l);
void         engine_stop(Engine *e);
double       engine_curve_torque(const EngineSpec *spec, double rpm);
/* Computes torque, fuel flow and heat for the current state. Returns net torque (Nm). */
double       engine_produce_torque(Engine *e, double throttle, double power_mult, bool fuel_cut);
void         engine_integrate(Engine *e, double clutch_torque_nm, double dt);
bool         engine_should_stall(const Engine *e);
void         engine_update_thermal(Engine *e, double speed_mps, double dt);

#endif
