/* simulation.h - ties vehicle, engine and transmission into one driving session. */
#ifndef SIMULATION_H
#define SIMULATION_H

#include <stdbool.h>
#include <stddef.h>
#include "vehicle.h"

typedef enum { MODE_LEARNER = 0, MODE_ECO, MODE_SPORT, MODE_HILL, MODE_COUNT } DriveMode;
typedef enum { EV_INFO = 0, EV_SHIFT, EV_WARNING, EV_ERROR } EventType;
typedef enum { HINT_NONE = 0, HINT_UPSHIFT, HINT_DOWNSHIFT } ShiftHint;

typedef enum {
    SIM_OK = 0,
    SIM_ERR_NULL = -1,
    SIM_ERR_RANGE = -2,
    SIM_ERR_IO = -3,
    SIM_ERR_CLUTCH_TO_START = -4,
    SIM_ERR_NO_FUEL = -5,
    SIM_ERR_OVERHEATED = -6,
    SIM_ERR_ALREADY_RUNNING = -7,
    SIM_ERR_NOMEM = -8
} SimStatus;

/* Warning bit flags (WARN_* values are mirrored in script.js). */
enum {
    WARN_OVERHEAT = 1, WARN_REDLINE = 2, WARN_STALL = 4, WARN_LOW_FUEL = 8,
    WARN_LUGGING = 16, WARN_GRIND = 32, WARN_CLUTCH = 64, WARN_TRANSMISSION = 128,
    WARN_NO_FUEL = 256
};

/* Order matters: script.js uses the same numbering. */
typedef enum {
    F_SPEED_KMH = 0, F_RPM, F_GEAR, F_THROTTLE, F_BRAKE, F_CLUTCH,
    F_FUEL_L, F_FUEL_PCT, F_TEMP_C, F_RUNNING, F_WARN_MASK, F_SLOPE,
    F_FUEL_LPH, F_INST_L100, F_AVG_L100, F_DIST_KM, F_TIME_S,
    F_SCORE, F_SCORE_SHIFT, F_SCORE_SMOOTH, F_SCORE_FUEL,
    F_TRANS_COND, F_CLUTCH_COND, F_ACCEL, F_HINT, F_REDLINE, F_IDLE,
    F_MAX_SPEED, F_STALLS, F_SHIFTS_GOOD, F_SHIFTS_TOTAL, F_GRINDS,
    F_PAUSED, F_HIST_TOTAL, F_FUEL_USED_L, F_SLIP_RPM,
    F_FIELD_COUNT
} SimField;

typedef struct {
    double    time_s;
    EventType type;
    int       gear;
    double    speed_kmh;
    double    rpm;
    char      message[64];
} HistoryEntry;

typedef struct {
    double elapsed_s, distance_m, fuel_used_l, max_speed_kmh, max_rpm;
    double shift_attempts, shift_points;
    int    grinds, invalid_shifts, stalls, overrevs;
    double lug_time_s, redline_time_s, overheat_time_s, slip_time_s;
    double jerk_integral, moving_time_s;
    double score_shift, score_smooth, score_fuel, score_total;
} SessionStats;

typedef struct { double throttle, brake, clutch; } PedalInput;

typedef struct {
    int        vehicle_index;
    DriveMode  mode;
    Vehicle    vehicle;
    Engine     engine;
    Transmission trans;
    PedalInput target;          /* what the player asks for           */
    PedalInput pedal;           /* actual pedal travel (rate limited) */
    double     power_mult;
    double     time_s;
    bool       paused;
    unsigned   warn_mask, prev_warn_mask;
    double     grind_flash, stall_flash, overrev_flash, ride_timer;
    double     accel_filtered, slip_rpm, inst_l100;
    ShiftHint  hint;
    SessionStats stats;
    HistoryEntry *history;      /* heap-allocated ring buffer */
    size_t     history_cap;
    unsigned long history_total;
} Simulation;

Simulation *sim_create(int vehicle_index, DriveMode mode);
void        sim_destroy(Simulation *s);
SimStatus   sim_configure(Simulation *s, int vehicle_index, DriveMode mode);
SimStatus   sim_reset(Simulation *s);
void        sim_set_inputs(Simulation *s, double throttle, double brake, double clutch);
SimStatus   sim_set_slope(Simulation *s, double slope_pct);
ShiftResult sim_request_gear(Simulation *s, int gear);
SimStatus   sim_ignition(Simulation *s, bool on);
void        sim_set_paused(Simulation *s, bool paused);
void        sim_update(Simulation *s, double dt);

double      sim_get_field(const Simulation *s, SimField f);
const HistoryEntry *sim_history_get(const Simulation *s, unsigned long index);
size_t      sim_report(const Simulation *s, char *buf, size_t cap);
SimStatus   sim_save_report(const Simulation *s, const char *path);
SimStatus   sim_load_best_score(const char *path, double *best);
SimStatus   sim_save_best_score(const char *path, double score);
const char *sim_mode_name(int mode);
const char *sim_status_text(int status);

#endif
