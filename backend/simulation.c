#include "simulation.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI                  3.14159265358979323846
#define SUBSTEP_S           0.004
#define MAX_FRAME_S         0.1
#define CLUTCH_SLIP_SMOOTH  15.0      /* rad/s over which clutch torque saturates */
#define RADS_TO_RPM         9.549296585513721
#define MS_TO_KMH           3.6
#define HISTORY_CAPACITY    256
#define CLUTCH_WEAR_PER_J   4.0e-5

typedef struct {
    const char *name;
    double power_mult;        /* engine power scaling                */
    double clutch_release;    /* pedal return speed (1/s)            */
    double stall_scale;       /* <1 = anti-stall assist              */
    double idle_scale;        /* >1 = stronger idle governor         */
    double fuel_ref_l100;     /* target consumption for scoring      */
    double w_shift, w_smooth, w_fuel;
} ModeParams;

static const ModeParams MODES[MODE_COUNT] = {
    { "Learner", 1.00, 1.10, 0.70, 1.50, 7.5, 0.40, 0.35, 0.25 },
    { "Eco",     0.90, 1.80, 1.00, 1.00, 6.0, 0.30, 0.25, 0.45 },
    { "Sport",   1.15, 2.40, 1.00, 1.00, 9.0, 0.45, 0.30, 0.25 },
    { "Hill",    1.00, 1.80, 1.00, 1.00, 8.0, 0.40, 0.30, 0.30 }
};

static const struct { unsigned bit; EventType type; const char *text; } WARNINGS[] = {
    { WARN_OVERHEAT,     EV_WARNING, "Engine overheating" },
    { WARN_REDLINE,      EV_WARNING, "Redline: shift up or lift off" },
    { WARN_STALL,        EV_ERROR,   "Engine stalled" },
    { WARN_LOW_FUEL,     EV_WARNING, "Fuel level low" },
    { WARN_LUGGING,      EV_WARNING, "Lugging: engine too slow for this gear" },
    { WARN_GRIND,        EV_ERROR,   "Gear grind" },
    { WARN_CLUTCH,       EV_WARNING, "Riding the clutch" },
    { WARN_TRANSMISSION, EV_WARNING, "Transmission condition low" },
    { WARN_NO_FUEL,      EV_ERROR,   "Out of fuel" }
};

static double clampd(double x, double lo, double hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static double approach(double cur, double target, double up, double down, double dt)
{
    double step = (target > cur ? up : down) * dt;
    if (fabs(target - cur) <= step) return target;
    return cur + (target > cur ? step : -step);
}

static void history_add(Simulation *s, EventType type, const char *fmt, ...)
{
    HistoryEntry *h;
    va_list ap;
    if (s->history == NULL) return;
    h = &s->history[s->history_total % s->history_cap];
    h->time_s    = s->time_s;
    h->type      = type;
    h->gear      = (int)s->trans.gear;
    h->speed_kmh = fabs(s->vehicle.speed_mps) * MS_TO_KMH;
    h->rpm       = s->engine.rpm;
    va_start(ap, fmt);
    vsnprintf(h->message, sizeof h->message, fmt, ap);
    va_end(ap);
    s->history_total++;
}

/* ---------- lifecycle ---------- */

Simulation *sim_create(int vehicle_index, DriveMode mode)
{
    Simulation *s = calloc(1, sizeof *s);
    if (s == NULL) return NULL;
    s->history_cap = HISTORY_CAPACITY;
    s->history = calloc(s->history_cap, sizeof *s->history);
    if (s->history == NULL || sim_configure(s, vehicle_index, mode) != SIM_OK) {
        sim_destroy(s);
        return NULL;
    }
    return s;
}

void sim_destroy(Simulation *s)
{
    if (s == NULL) return;
    free(s->history);
    free(s);
}

SimStatus sim_configure(Simulation *s, int vehicle_index, DriveMode mode)
{
    HistoryEntry *hist;
    size_t cap;
    if (s == NULL) return SIM_ERR_NULL;
    if ((int)mode < 0 || mode >= MODE_COUNT) return SIM_ERR_RANGE;
    hist = s->history;
    cap  = s->history_cap;
    memset(s, 0, sizeof *s);
    s->history = hist;
    s->history_cap = cap;

    if (vehicle_init(&s->vehicle, vehicle_index) != 0) return SIM_ERR_RANGE;
    s->vehicle_index = vehicle_index;
    s->mode = mode;
    engine_init(&s->engine, &s->vehicle.spec.engine);
    s->engine.spec.stall_rpm       *= MODES[mode].stall_scale;
    s->engine.spec.idle_gov_max_nm *= MODES[mode].idle_scale;
    transmission_init(&s->trans, &s->vehicle.spec.transmission);
    s->power_mult = MODES[mode].power_mult;
    s->stats.score_shift = s->stats.score_smooth = s->stats.score_fuel = s->stats.score_total = 100.0;
    history_add(s, EV_INFO, "Session started: %s, %s mode",
                s->vehicle.spec.name, MODES[mode].name);
    return SIM_OK;
}

SimStatus sim_reset(Simulation *s)
{
    if (s == NULL) return SIM_ERR_NULL;
    return sim_configure(s, s->vehicle_index, s->mode);
}

/* ---------- inputs ---------- */

void sim_set_inputs(Simulation *s, double throttle, double brake, double clutch)
{
    if (s == NULL) return;
    s->target.throttle = clampd(throttle, 0.0, 1.0);
    s->target.brake    = clampd(brake, 0.0, 1.0);
    s->target.clutch   = clampd(clutch, 0.0, 1.0);
}

SimStatus sim_set_slope(Simulation *s, double slope_pct)
{
    if (s == NULL) return SIM_ERR_NULL;
    if (slope_pct < -15.0 || slope_pct > 15.0) return SIM_ERR_RANGE;
    if (s->mode != MODE_HILL) s->vehicle.slope_pct = slope_pct;
    return SIM_OK;
}

void sim_set_paused(Simulation *s, bool paused)
{
    if (s != NULL) s->paused = paused;
}

SimStatus sim_ignition(Simulation *s, bool on)
{
    EngineStatus st;
    if (s == NULL) return SIM_ERR_NULL;
    if (!on) {
        if (s->engine.running) {
            engine_stop(&s->engine);
            history_add(s, EV_INFO, "Engine switched off");
        }
        return SIM_OK;
    }
    if (s->trans.gear != GEAR_NEUTRAL && s->trans.clutch_pos < CLUTCH_SHIFT_THRESHOLD) {
        history_add(s, EV_ERROR, "Start refused: press clutch or select neutral");
        return SIM_ERR_CLUTCH_TO_START;
    }
    st = engine_start(&s->engine, s->vehicle.fuel_l);
    switch (st) {
    case ENGINE_OK:
        history_add(s, EV_INFO, "Engine started");
        return SIM_OK;
    case ENGINE_ERR_NO_FUEL:         return SIM_ERR_NO_FUEL;
    case ENGINE_ERR_OVERHEATED:
        history_add(s, EV_ERROR, "Start refused: engine too hot");
        return SIM_ERR_OVERHEATED;
    case ENGINE_ERR_ALREADY_RUNNING: return SIM_ERR_ALREADY_RUNNING;
    default:                         return SIM_ERR_NULL;
    }
}

static double shift_points(const Simulation *s, double rpm_after)
{
    double red = s->engine.spec.redline_rpm;
    if (rpm_after < 1200.0 || rpm_after > red * 0.90) return 0.0;   /* lugging / too high */
    if (rpm_after >= 1500.0 && rpm_after <= red * 0.72) return 1.0; /* in the sweet spot  */
    return 0.5;
}

ShiftResult sim_request_gear(Simulation *s, int gear)
{
    double rpm_after = 0.0;
    Gear from;
    ShiftResult r;
    bool counts;
    if (s == NULL) return SHIFT_ERR_INVALID_GEAR;
    from = s->trans.gear;
    if (gear == (int)from) return SHIFT_OK;

    r = transmission_validate_shift(&s->trans, gear, s->vehicle.speed_mps, s->engine.running,
                                    s->vehicle.spec.wheel_radius_m,
                                    s->engine.spec.redline_rpm, &rpm_after);
    if (!transmission_shift_accepted(r)) {
        if (r == SHIFT_ERR_CLUTCH) {
            s->stats.grinds++;
            s->grind_flash = 2.0;
            transmission_add_wear(&s->trans, 4.0, 0.0);
        } else {
            s->stats.invalid_shifts++;
            transmission_add_wear(&s->trans, 6.0, 0.0);
        }
        s->stats.shift_attempts += 1.0;
        history_add(s, EV_ERROR, "%s", transmission_shift_text(r));
        return r;
    }

    s->trans.gear = (Gear)gear;
    counts = gear >= GEAR_1 && fabs(s->vehicle.speed_mps) > 1.5 && s->engine.running;
    if (r == SHIFT_WARN_OVERREV) {
        s->stats.overrevs++;
        s->overrev_flash = 2.0;
        transmission_add_wear(&s->trans, 3.0, 0.0);
        s->stats.shift_attempts += 1.0;            /* counted, zero points */
        history_add(s, EV_ERROR, "Over-rev shift %s to %s (%.0f rpm)",
                    transmission_gear_label(from), transmission_gear_label((Gear)gear), rpm_after);
    } else {
        if (counts) {
            s->stats.shift_attempts += 1.0;
            s->stats.shift_points   += shift_points(s, rpm_after);
        }
        history_add(s, EV_SHIFT, "Shift %s to %s",
                    transmission_gear_label(from), transmission_gear_label((Gear)gear));
    }
    return r;
}

/* ---------- physics step ---------- */

static double hill_profile(double pos_m)
{
    return 5.5 * sin(pos_m / 140.0) + 3.0 * sin(pos_m / 45.0 + 1.0);   /* percent */
}

static void substep(Simulation *s, double h)
{
    Vehicle *v = &s->vehicle;
    Engine *e = &s->engine;
    Transmission *t = &s->trans;
    double ratio = transmission_ratio(t, t->gear);
    bool in_gear = ratio != 0.0;
    double r = v->spec.wheel_radius_m;
    double wheel_omega = in_gear ? v->speed_mps / r * ratio : 0.0;
    double f = transmission_clutch_engagement(t);
    double slip = e->omega_rads - wheel_omega;
    double tc = 0.0, drive_force = 0.0, net;
    bool locked = in_gear && f > 0.95 && fabs(slip) < 15.0;
    bool fuel_cut = (locked && s->pedal.throttle < 0.02 && e->rpm > 1300.0) || v->fuel_l <= 0.0;

    if (in_gear) {   /* friction clutch: torque grows with engagement and slip */
        tc = t->spec.clutch_max_torque_nm * f * tanh(slip / CLUTCH_SLIP_SMOOTH);
        drive_force = tc * ratio * t->spec.efficiency / r;
        if (f > 0.05 && f < 0.95 && fabs(slip) > 20.0)
            transmission_add_wear(t, 0.0, fabs(tc * slip) * h * CLUTCH_WEAR_PER_J);
    }

    net = engine_produce_torque(e, s->pedal.throttle, s->power_mult, fuel_cut);
    (void)net;
    engine_integrate(e, tc, h);
    vehicle_integrate(v, drive_force, s->pedal.brake * v->spec.max_brake_n, h);
    engine_update_thermal(e, v->speed_mps, h);

    v->fuel_l -= e->fuel_rate_lps * h;
    s->stats.fuel_used_l += e->fuel_rate_lps * h;
    if (v->fuel_l < 0.0) v->fuel_l = 0.0;
    s->slip_rpm = fabs(slip) * RADS_TO_RPM;
}

static void update_score(Simulation *s)
{
    const ModeParams *m = &MODES[s->mode];
    SessionStats *st = &s->stats;
    double dist_km = st->distance_m / 1000.0, w, avg, penalty;

    st->score_shift = st->shift_attempts > 0.0 ? 100.0 * st->shift_points / st->shift_attempts : 100.0;

    st->score_smooth = st->moving_time_s > 2.0
        ? clampd(100.0 - 18.0 * st->jerk_integral / st->moving_time_s, 0.0, 100.0) : 100.0;

    avg = dist_km > 0.01 ? st->fuel_used_l / dist_km * 100.0 : m->fuel_ref_l100;
    w = clampd(dist_km / 0.5, 0.0, 1.0);            /* trust fuel data after 0.5 km */
    st->score_fuel = (1.0 - w) * 100.0 + w * clampd(100.0 - (avg - m->fuel_ref_l100) * 12.0, 0.0, 100.0);

    penalty = clampd(st->stalls * 3.0 + st->overheat_time_s * 0.5 + st->redline_time_s * 0.3
                     + st->lug_time_s * 0.3, 0.0, 35.0);
    st->score_total = clampd(m->w_shift * st->score_shift + m->w_smooth * st->score_smooth
                             + m->w_fuel * st->score_fuel - penalty, 0.0, 100.0);
}

static void post_frame(Simulation *s, double dt)
{
    Vehicle *v = &s->vehicle;
    Engine *e = &s->engine;
    Transmission *t = &s->trans;
    SessionStats *st = &s->stats;
    double kmh = fabs(v->speed_mps) * MS_TO_KMH;
    double ratio = transmission_ratio(t, t->gear), f = transmission_clutch_engagement(t);
    double prev_a = s->accel_filtered, jerk;
    bool in_gear = ratio != 0.0, lugging, riding;
    unsigned mask = 0, fresh;
    size_t i;

    st->elapsed_s += dt;
    st->distance_m += fabs(v->speed_mps) * dt;
    if (kmh > st->max_speed_kmh) st->max_speed_kmh = kmh;
    if (e->rpm > st->max_rpm) st->max_rpm = e->rpm;
    s->grind_flash  = fmax(0.0, s->grind_flash  - dt);
    s->stall_flash  = fmax(0.0, s->stall_flash  - dt);
    s->overrev_flash = fmax(0.0, s->overrev_flash - dt);

    if (engine_should_stall(e)) {
        engine_stop(e);
        st->stalls++;
        s->stall_flash = 2.0;
    }
    if (e->running && e->temp_c >= ENGINE_TEMP_SHUTDOWN) {
        engine_stop(e);
        history_add(s, EV_ERROR, "Engine shut down: overheated");
    }
    if (e->running && v->fuel_l <= 0.0) engine_stop(e);

    s->accel_filtered += (v->accel_mps2 - s->accel_filtered) * clampd(dt * 6.0, 0.0, 1.0);
    jerk = dt > 0.0 ? (s->accel_filtered - prev_a) / dt : 0.0;
    if (kmh > 3.6) { st->jerk_integral += fabs(jerk) * dt; st->moving_time_s += dt; }

    lugging = e->running && in_gear && t->gear >= GEAR_2 && e->rpm < 1300.0 &&
              s->pedal.throttle > 0.5 && kmh > 3.6;
    riding  = e->running && in_gear && f > 0.05 && f < 0.95 && s->slip_rpm > 300.0 && kmh > 10.0;
    s->ride_timer = riding ? s->ride_timer + dt : fmax(0.0, s->ride_timer - dt);

    if (lugging) { st->lug_time_s += dt; transmission_add_wear(t, 0.2 * dt, 0.0); }
    if (e->running && e->rpm >= e->spec.redline_rpm * 0.97) st->redline_time_s += dt;
    if (e->rpm > e->spec.redline_rpm * 1.02) transmission_add_wear(t, 1.5 * dt, 0.0);
    if (e->temp_c >= ENGINE_TEMP_WARN) st->overheat_time_s += dt;

    if (e->temp_c >= ENGINE_TEMP_WARN)                         mask |= WARN_OVERHEAT;
    if (e->running && e->rpm >= e->spec.redline_rpm * 0.97)    mask |= WARN_REDLINE;
    if (s->stall_flash > 0.0)                                  mask |= WARN_STALL;
    if (v->fuel_l < 0.12 * v->spec.tank_l)                     mask |= WARN_LOW_FUEL;
    if (v->fuel_l <= 0.0)                                      mask |= WARN_NO_FUEL;
    if (lugging)                                               mask |= WARN_LUGGING;
    if (s->grind_flash > 0.0 || s->overrev_flash > 0.0)        mask |= WARN_GRIND;
    if (s->ride_timer > 1.0)                                   mask |= WARN_CLUTCH;
    if (transmission_condition(t) < 30.0)                      mask |= WARN_TRANSMISSION;

    fresh = mask & ~s->prev_warn_mask;
    for (i = 0; i < sizeof WARNINGS / sizeof WARNINGS[0]; i++) {
        if ((fresh & WARNINGS[i].bit) && WARNINGS[i].bit != WARN_GRIND)  /* grind logged on shift */
            history_add(s, WARNINGS[i].type, "%s", WARNINGS[i].text);
    }
    s->prev_warn_mask = mask;
    s->warn_mask = mask;

    s->hint = HINT_NONE;
    if (s->mode == MODE_LEARNER && e->running && in_gear && f > 0.9) {
        if (t->gear >= GEAR_1 && t->gear < GEAR_5 && e->rpm > e->spec.redline_rpm * 0.70 &&
            s->pedal.throttle > 0.3)
            s->hint = HINT_UPSHIFT;
        else if (t->gear >= GEAR_2 && e->rpm < 1300.0 && kmh > 3.6)
            s->hint = HINT_DOWNSHIFT;
    }

    s->inst_l100 = kmh > 3.0 ? e->fuel_rate_lps / (kmh / MS_TO_KMH) * 1.0e5 : 0.0;
    update_score(s);
}

void sim_update(Simulation *s, double dt)
{
    int n, i;
    double h;
    if (s == NULL || s->paused || dt <= 0.0) return;
    dt = clampd(dt, 0.0, MAX_FRAME_S);
    s->time_s += dt;

    s->pedal.throttle = approach(s->pedal.throttle, s->target.throttle, 3.0, 6.0, dt);
    s->pedal.brake    = approach(s->pedal.brake,    s->target.brake,    5.0, 6.0, dt);
    s->pedal.clutch   = approach(s->pedal.clutch,   s->target.clutch,   6.0,
                                 MODES[s->mode].clutch_release, dt);
    s->trans.clutch_pos = s->pedal.clutch;

    if (s->mode == MODE_HILL) s->vehicle.slope_pct = hill_profile(s->vehicle.position_m);

    n = (int)ceil(dt / SUBSTEP_S);
    h = dt / n;
    for (i = 0; i < n; i++) substep(s, h);
    post_frame(s, dt);
}

/* ---------- queries ---------- */

double sim_get_field(const Simulation *s, SimField f)
{
    const SessionStats *st;
    double dist_km;
    if (s == NULL) return 0.0;
    st = &s->stats;
    dist_km = st->distance_m / 1000.0;
    switch (f) {
    case F_SPEED_KMH:    return s->vehicle.speed_mps * MS_TO_KMH;
    case F_RPM:          return s->engine.rpm;
    case F_GEAR:         return (double)s->trans.gear;
    case F_THROTTLE:     return s->pedal.throttle;
    case F_BRAKE:        return s->pedal.brake;
    case F_CLUTCH:       return s->pedal.clutch;
    case F_FUEL_L:       return s->vehicle.fuel_l;
    case F_FUEL_PCT:     return 100.0 * s->vehicle.fuel_l / s->vehicle.spec.tank_l;
    case F_TEMP_C:       return s->engine.temp_c;
    case F_RUNNING:      return s->engine.running ? 1.0 : 0.0;
    case F_WARN_MASK:    return (double)s->warn_mask;
    case F_SLOPE:        return s->vehicle.slope_pct;
    case F_FUEL_LPH:     return s->engine.fuel_rate_lps * 3600.0;
    case F_INST_L100:    return s->inst_l100;
    case F_AVG_L100:     return dist_km > 0.01 ? st->fuel_used_l / dist_km * 100.0 : 0.0;
    case F_DIST_KM:      return dist_km;
    case F_TIME_S:       return st->elapsed_s;
    case F_SCORE:        return st->score_total;
    case F_SCORE_SHIFT:  return st->score_shift;
    case F_SCORE_SMOOTH: return st->score_smooth;
    case F_SCORE_FUEL:   return st->score_fuel;
    case F_TRANS_COND:   return transmission_condition(&s->trans);
    case F_CLUTCH_COND:  return transmission_clutch_condition(&s->trans);
    case F_ACCEL:        return s->vehicle.accel_mps2;
    case F_HINT:         return (double)s->hint;
    case F_REDLINE:      return s->engine.spec.redline_rpm;
    case F_IDLE:         return s->engine.spec.idle_rpm;
    case F_MAX_SPEED:    return st->max_speed_kmh;
    case F_STALLS:       return st->stalls;
    case F_SHIFTS_GOOD:  return st->shift_points;
    case F_SHIFTS_TOTAL: return st->shift_attempts;
    case F_GRINDS:       return st->grinds;
    case F_PAUSED:       return s->paused ? 1.0 : 0.0;
    case F_HIST_TOTAL:   return (double)s->history_total;
    case F_FUEL_USED_L:  return st->fuel_used_l;
    case F_SLIP_RPM:     return s->slip_rpm;
    default:             return 0.0;
    }
}

const HistoryEntry *sim_history_get(const Simulation *s, unsigned long index)
{
    if (s == NULL || index >= s->history_total) return NULL;
    if (s->history_total - index > s->history_cap) return NULL;   /* overwritten */
    return &s->history[index % s->history_cap];
}

const char *sim_mode_name(int mode)
{
    return (mode >= 0 && mode < MODE_COUNT) ? MODES[mode].name : "?";
}

const char *sim_status_text(int status)
{
    switch (status) {
    case SIM_OK:                    return "OK";
    case SIM_ERR_NULL:              return "Simulation not initialised";
    case SIM_ERR_RANGE:             return "Value out of range";
    case SIM_ERR_IO:                return "File could not be written";
    case SIM_ERR_CLUTCH_TO_START:   return "Press the clutch or select neutral to start";
    case SIM_ERR_NO_FUEL:           return "No fuel left";
    case SIM_ERR_OVERHEATED:        return "Engine is too hot to restart: let it cool";
    case SIM_ERR_ALREADY_RUNNING:   return "Engine already running";
    case SIM_ERR_NOMEM:             return "Out of memory";
    }
    return "Unknown error";
}

/* ---------- reports and files ---------- */

static const char *event_name(EventType t)
{
    static const char *n[] = { "info", "shift", "warning", "error" };
    return (t >= EV_INFO && t <= EV_ERROR) ? n[t] : "?";
}

size_t sim_report(const Simulation *s, char *buf, size_t cap)
{
    size_t len = 0;
    unsigned long i, first;
    const SessionStats *st;
    if (s == NULL || buf == NULL || cap < 2) return 0;
    st = &s->stats;
    buf[0] = '\0';

#define APPEND(...) do { int n_ = snprintf(buf + len, cap - len, __VA_ARGS__); \
    if (n_ < 0 || (size_t)n_ >= cap - len) { return len; } len += (size_t)n_; } while (0)

    APPEND("VEHICLE GEAR SHIFT SIMULATOR - SESSION REPORT\n");
    APPEND("Vehicle: %s | Mode: %s\n", s->vehicle.spec.name, MODES[s->mode].name);
    APPEND("Duration: %.1f s | Distance: %.2f km | Max speed: %.0f km/h | Max rpm: %.0f\n",
           st->elapsed_s, st->distance_m / 1000.0, st->max_speed_kmh, st->max_rpm);
    APPEND("Fuel used: %.2f L | Average: %.1f L/100 km\n", st->fuel_used_l,
           st->distance_m > 10.0 ? st->fuel_used_l / (st->distance_m / 1000.0) * 100.0 : 0.0);
    APPEND("Shifts: %.1f good-points of %.0f | Grinds: %d | Invalid: %d | Over-revs: %d | Stalls: %d\n",
           st->shift_points, st->shift_attempts, st->grinds, st->invalid_shifts, st->overrevs, st->stalls);
    APPEND("Score: %.0f (shifting %.0f, smoothness %.0f, fuel %.0f)\n",
           st->score_total, st->score_shift, st->score_smooth, st->score_fuel);
    APPEND("Transmission condition: %.0f%%\n\n", transmission_condition(&s->trans));
    APPEND("time_s,type,gear,speed_kmh,rpm,message\n");

    first = s->history_total > s->history_cap ? s->history_total - s->history_cap : 0;
    for (i = first; i < s->history_total; i++) {
        const HistoryEntry *h = sim_history_get(s, i);
        if (h == NULL) continue;
        APPEND("%.1f,%s,%s,%.0f,%.0f,\"%s\"\n", h->time_s, event_name(h->type),
               transmission_gear_label((Gear)h->gear), h->speed_kmh, h->rpm, h->message);
    }
#undef APPEND
    return len;
}

SimStatus sim_save_report(const Simulation *s, const char *path)
{
    char *buf;
    size_t len;
    FILE *fp;
    if (s == NULL || path == NULL) return SIM_ERR_NULL;
    buf = malloc(32768);
    if (buf == NULL) return SIM_ERR_NOMEM;
    len = sim_report(s, buf, 32768);
    fp = fopen(path, "w");
    if (fp == NULL) { free(buf); return SIM_ERR_IO; }
    if (fwrite(buf, 1, len, fp) != len) { fclose(fp); free(buf); return SIM_ERR_IO; }
    fclose(fp);
    free(buf);
    return SIM_OK;
}

SimStatus sim_load_best_score(const char *path, double *best)
{
    FILE *fp;
    if (path == NULL || best == NULL) return SIM_ERR_NULL;
    *best = 0.0;
    fp = fopen(path, "r");
    if (fp == NULL) return SIM_ERR_IO;
    if (fscanf(fp, "%lf", best) != 1) { fclose(fp); *best = 0.0; return SIM_ERR_IO; }
    fclose(fp);
    return SIM_OK;
}

SimStatus sim_save_best_score(const char *path, double score)
{
    double old = 0.0;
    FILE *fp;
    if (path == NULL) return SIM_ERR_NULL;
    if (sim_load_best_score(path, &old) == SIM_OK && old >= score) return SIM_OK;
    fp = fopen(path, "w");
    if (fp == NULL) return SIM_ERR_IO;
    fprintf(fp, "%.1f\n", score);
    fclose(fp);
    return SIM_OK;
}
