/* transmission.h - gearbox ratios, clutch, shift validation, wear. */
#ifndef TRANSMISSION_H
#define TRANSMISSION_H

#include <stdbool.h>

#define FORWARD_GEARS            5
#define CLUTCH_SHIFT_THRESHOLD   0.80   /* pedal position needed to change gear */
#define REVERSE_SPEED_LIMIT_MPS  0.8

typedef enum {
    GEAR_REVERSE = -1,
    GEAR_NEUTRAL = 0,
    GEAR_1, GEAR_2, GEAR_3, GEAR_4, GEAR_5
} Gear;

typedef enum {
    SHIFT_OK = 0,
    SHIFT_ERR_INVALID_GEAR,
    SHIFT_ERR_CLUTCH,
    SHIFT_ERR_REVERSE_WHILE_MOVING,
    SHIFT_ERR_FORWARD_WHILE_REVERSING,
    SHIFT_WARN_OVERREV          /* allowed, but damages the gearbox */
} ShiftResult;

typedef struct {
    double forward[FORWARD_GEARS];
    double reverse;
    double final_drive;
    double efficiency;
    double clutch_max_torque_nm;
    double bite_full;      /* pedal position below which the clutch is fully engaged */
    double bite_release;   /* pedal position above which the clutch is fully open    */
} TransmissionSpec;

typedef struct {
    TransmissionSpec spec;
    Gear   gear;
    double clutch_pos;     /* 0 = pedal up (engaged), 1 = pedal down (open) */
    double gear_wear;      /* percent */
    double clutch_wear;    /* percent */
} Transmission;

void        transmission_init(Transmission *t, const TransmissionSpec *spec);
double      transmission_ratio(const Transmission *t, Gear g);   /* signed overall ratio */
double      transmission_clutch_engagement(const Transmission *t); /* 0..1 */
ShiftResult transmission_validate_shift(const Transmission *t, int target, double speed_mps,
                                        bool engine_running, double wheel_radius_m,
                                        double redline_rpm, double *rpm_after);
bool        transmission_shift_accepted(ShiftResult r);
void        transmission_add_wear(Transmission *t, double gear_amount, double clutch_amount);
double      transmission_condition(const Transmission *t);        /* 0..100 */
double      transmission_clutch_condition(const Transmission *t); /* 0..100 */
const char *transmission_gear_label(Gear g);
const char *transmission_shift_text(ShiftResult r);

#endif
