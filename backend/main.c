#include <math.h>
/*
 * main.c
 *  - Emscripten build : exports the api_* functions that script.js calls.
 *  - Native build     : runs an automated self-test of the simulation engine.
 */
#include "simulation.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>

static Simulation *g_sim = NULL;
static char g_report[32768];

EMSCRIPTEN_KEEPALIVE int api_init(int vehicle, int mode)
{
    Simulation *next = sim_create(vehicle, (DriveMode)mode);
    if (next == NULL) return SIM_ERR_RANGE;
    sim_destroy(g_sim);
    g_sim = next;
    return SIM_OK;
}
EMSCRIPTEN_KEEPALIVE int    api_reset(void)                       { return g_sim ? sim_reset(g_sim) : SIM_ERR_NULL; }
EMSCRIPTEN_KEEPALIVE void   api_update(double dt)                 { sim_update(g_sim, dt); }
EMSCRIPTEN_KEEPALIVE void   api_set_inputs(double t, double b, double c) { sim_set_inputs(g_sim, t, b, c); }
EMSCRIPTEN_KEEPALIVE int    api_set_gear(int gear)                { return g_sim ? (int)sim_request_gear(g_sim, gear) : SHIFT_ERR_INVALID_GEAR; }
EMSCRIPTEN_KEEPALIVE int    api_ignition(int on)                  { return sim_ignition(g_sim, on != 0); }
EMSCRIPTEN_KEEPALIVE int    api_set_slope(double pct)             { return sim_set_slope(g_sim, pct); }
EMSCRIPTEN_KEEPALIVE void   api_set_paused(int p)                 { sim_set_paused(g_sim, p != 0); }
EMSCRIPTEN_KEEPALIVE double api_get(int field)                    { return sim_get_field(g_sim, (SimField)field); }
EMSCRIPTEN_KEEPALIVE double api_history_total(void)               { return g_sim ? (double)g_sim->history_total : 0.0; }

/* field: 0 time, 1 type, 2 gear, 3 speed km/h, 4 rpm; returns -1 if the entry is gone */
EMSCRIPTEN_KEEPALIVE double api_history_get(int index, int field)
{
    const HistoryEntry *h = sim_history_get(g_sim, (unsigned long)index);
    if (h == NULL) return -1.0;
    switch (field) {
    case 0: return h->time_s;
    case 1: return (double)h->type;
    case 2: return (double)h->gear;
    case 3: return h->speed_kmh;
    case 4: return h->rpm;
    }
    return -1.0;
}
EMSCRIPTEN_KEEPALIVE const char *api_history_message(int index)
{
    const HistoryEntry *h = sim_history_get(g_sim, (unsigned long)index);
    return h ? h->message : "";
}
EMSCRIPTEN_KEEPALIVE const char *api_report(void)
{
    sim_report(g_sim, g_report, sizeof g_report);
    return g_report;
}
EMSCRIPTEN_KEEPALIVE int         api_vehicle_count(void)          { return vehicle_preset_count(); }
EMSCRIPTEN_KEEPALIVE const char *api_vehicle_name(int i)          { const VehicleSpec *v = vehicle_preset(i); return v ? v->name : ""; }
EMSCRIPTEN_KEEPALIVE const char *api_vehicle_desc(int i)          { const VehicleSpec *v = vehicle_preset(i); return v ? v->description : ""; }
/* stat: 0 mass kg, 1 peak torque Nm, 2 redline rpm, 3 tank L */
EMSCRIPTEN_KEEPALIVE double      api_vehicle_stat(int i, int k)
{
    const VehicleSpec *v = vehicle_preset(i);
    if (v == NULL) return 0.0;
    return k == 0 ? v->mass_kg : k == 1 ? v->engine.peak_torque_nm : k == 2 ? v->engine.redline_rpm : v->tank_l;
}
EMSCRIPTEN_KEEPALIVE int         api_mode_count(void)             { return MODE_COUNT; }
EMSCRIPTEN_KEEPALIVE const char *api_mode_name(int i)             { return sim_mode_name(i); }
EMSCRIPTEN_KEEPALIVE const char *api_status_text(int code)        { return sim_status_text(code); }
EMSCRIPTEN_KEEPALIVE const char *api_shift_text(int code)         { return transmission_shift_text((ShiftResult)code); }

#else  /* ------------------------- native self-test ------------------------- */

static int g_checks = 0, g_failed = 0;

#define CHECK(cond, label) do { g_checks++; \
    if (cond) printf("  [PASS] %s\n", label); \
    else { g_failed++; printf("  [FAIL] %s\n", label); } } while (0)

static void drive(Simulation *s, double seconds, double th, double br, double cl)
{
    int i, n = (int)(seconds / 0.02);
    for (i = 0; i < n; i++) { sim_set_inputs(s, th, br, cl); sim_update(s, 0.02); }
}

/* Clutch released linearly (in seconds) while throttle is held. */
static void release_clutch(Simulation *s, double seconds, double th)
{
    int i, n = (int)(seconds / 0.02);
    for (i = 0; i < n; i++) {
        sim_set_inputs(s, th, 0.0, 1.0 - (double)i / n);
        sim_update(s, 0.02);
    }
    drive(s, 0.1, th, 0.0, 0.0);
}

static Simulation *fresh(int vehicle, DriveMode mode)
{
    Simulation *s = sim_create(vehicle, mode);
    if (s == NULL) { fprintf(stderr, "out of memory\n"); exit(2); }
    return s;
}

static void launch(Simulation *s)
{
    drive(s, 0.5, 0.0, 0.0, 1.0);
    sim_request_gear(s, 1);
    release_clutch(s, 1.2, 0.35);
}

static void upshift(Simulation *s, int gear, double th)
{
    drive(s, 0.3, 0.0, 0.0, 1.0);      /* lift + clutch in */
    sim_request_gear(s, gear);
    release_clutch(s, 0.8, th);
}

int main(void)
{
    Simulation *s;
    double best = 0.0;
    int v, m;

    printf("Vehicle Gear Shift Simulator - C engine self-test\n\n");

    printf("Idle and starting\n");
    s = fresh(0, MODE_ECO);
    CHECK(sim_ignition(s, true) == SIM_OK, "engine starts in neutral");
    drive(s, 2.0, 0.0, 0.0, 0.0);
    CHECK(s->engine.running && s->engine.rpm > 700.0 && s->engine.rpm < 1000.0, "engine idles near 850 rpm");
    sim_ignition(s, false);
    sim_request_gear(s, 1);
    CHECK(sim_ignition(s, true) == SIM_ERR_CLUTCH_TO_START, "start in gear without clutch is refused");
    sim_destroy(s);

    printf("Launch, upshift, fuel\n");
    s = fresh(0, MODE_ECO);
    sim_ignition(s, true);
    launch(s);
    CHECK(s->engine.running, "smooth launch does not stall");
    drive(s, 3.0, 0.5, 0.0, 0.0);
    CHECK(sim_get_field(s, F_SPEED_KMH) > 10.0, "car accelerates in first gear");
    upshift(s, 2, 0.5);
    CHECK(s->trans.gear == GEAR_2 && s->engine.running, "clean shift 1 to 2");
    drive(s, 4.0, 0.6, 0.0, 0.0);
    printf("      speed %.1f km/h, rpm %.0f, gear %d\n",
           sim_get_field(s, F_SPEED_KMH), s->engine.rpm, (int)s->trans.gear);
    CHECK(s->stats.fuel_used_l > 0.0, "fuel is consumed");
    CHECK(s->vehicle.fuel_l < s->vehicle.spec.tank_l, "tank level drops");

    printf("Invalid shifts\n");
    CHECK(sim_request_gear(s, 3) == SHIFT_ERR_CLUTCH && s->trans.gear == GEAR_2, "grind without clutch is rejected");
    drive(s, 0.4, 0.0, 0.0, 1.0);
    CHECK(sim_request_gear(s, -1) == SHIFT_ERR_REVERSE_WHILE_MOVING, "reverse while moving is rejected");
    CHECK(sim_request_gear(s, 9) == SHIFT_ERR_INVALID_GEAR, "gear 9 does not exist");
    CHECK(transmission_condition(&s->trans) < 100.0, "invalid shifts wear the gearbox");
    printf("      score %.0f (shift %.0f, smooth %.0f, fuel %.0f)\n", s->stats.score_total,
           s->stats.score_shift, s->stats.score_smooth, s->stats.score_fuel);

    printf("Braking\n");
    drive(s, 0.3, 0.0, 0.0, 1.0);
    sim_request_gear(s, 0);
    drive(s, 8.0, 0.0, 1.0, 0.0);
    CHECK(fabs(sim_get_field(s, F_SPEED_KMH)) < 0.5, "full braking stops the car");

    printf("Report and files\n");
    CHECK(sim_save_report(s, "session_report.txt") == SIM_OK, "session report written to file");
    CHECK(sim_save_best_score("best_score.dat", s->stats.score_total) == SIM_OK, "best score saved");
    CHECK(sim_load_best_score("best_score.dat", &best) == SIM_OK && best > 0.0, "best score reloaded");
    CHECK(sim_save_report(s, "/nonexistent-dir/x.txt") == SIM_ERR_IO, "bad path reports an I/O error");
    sim_destroy(s);

    printf("Stalling\n");
    s = fresh(0, MODE_ECO);
    sim_ignition(s, true);
    drive(s, 0.5, 0.0, 0.0, 1.0);
    sim_request_gear(s, 3);
    release_clutch(s, 0.3, 0.0);
    drive(s, 1.0, 0.0, 0.0, 0.0);
    CHECK(!s->engine.running && s->stats.stalls == 1, "dumping the clutch in 3rd stalls the engine");
    sim_destroy(s);

    printf("Overheating\n");
    s = fresh(1, MODE_ECO);
    sim_ignition(s, true);
    drive(s, 40.0, 1.0, 0.0, 0.0);
    CHECK(s->warn_mask & WARN_REDLINE || s->engine.temp_c > 100.0, "revving in neutral heats the engine");
    CHECK(s->engine.temp_c >= ENGINE_TEMP_WARN || !s->engine.running, "overheat warning or shutdown triggers");
    sim_destroy(s);

    printf("Hills\n");
    s = fresh(2, MODE_ECO);
    sim_set_slope(s, -8.0);
    drive(s, 10.0, 0.0, 0.0, 0.0);
    CHECK(sim_get_field(s, F_SPEED_KMH) > 10.0, "car rolls downhill in neutral");
    drive(s, 10.0, 0.0, 1.0, 0.0);
    sim_set_slope(s, 8.0);
    CHECK(sim_set_slope(s, 40.0) == SIM_ERR_RANGE, "slope outside range is rejected");
    sim_destroy(s);

    printf("Normal driving stays within limits (all vehicles, 5 gears)\n");
    for (v = 0; v < VEHICLE_COUNT; v++) {
        s = fresh(v, MODE_LEARNER);
        sim_ignition(s, true);
        launch(s);
        drive(s, 2.5, 0.6, 0.0, 0.0);
        upshift(s, 2, 0.6); drive(s, 3.0, 0.6, 0.0, 0.0);
        upshift(s, 3, 0.6); drive(s, 4.0, 0.6, 0.0, 0.0);
        upshift(s, 4, 0.6); drive(s, 5.0, 0.5, 0.0, 0.0);
        upshift(s, 5, 0.5); drive(s, 12.0, 0.4, 0.0, 0.0);
        printf("      %-12s speed %5.1f km/h  rpm %4.0f  temp %5.1f C  %.1f L/100km  score %.0f\n",
               s->vehicle.spec.name, sim_get_field(s, F_SPEED_KMH), s->engine.rpm,
               s->engine.temp_c, sim_get_field(s, F_AVG_L100), s->stats.score_total);
        CHECK(s->engine.running && s->engine.temp_c < ENGINE_TEMP_WARN, s->vehicle.spec.name);
        sim_destroy(s);
    }
    for (m = 0; m < MODE_COUNT; m++) printf("  mode %d = %s\n", m, sim_mode_name(m));

    printf("\n%d checks, %d failed\n", g_checks, g_failed);
    return g_failed ? 1 : 0;
}
#endif
