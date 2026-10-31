#include "transmission.h"
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846

void transmission_init(Transmission *t, const TransmissionSpec *spec)
{
    if (t == NULL || spec == NULL) return;
    memset(t, 0, sizeof *t);
    t->spec = *spec;
    t->gear = GEAR_NEUTRAL;
}

double transmission_ratio(const Transmission *t, Gear g)
{
    if (g == GEAR_REVERSE)           return -t->spec.reverse * t->spec.final_drive;
    if (g >= GEAR_1 && g <= GEAR_5)  return t->spec.forward[g - 1] * t->spec.final_drive;
    return 0.0;                      /* neutral: nothing is connected */
}

double transmission_clutch_engagement(const Transmission *t)
{
    double span = t->spec.bite_release - t->spec.bite_full;
    double x = (t->spec.bite_release - t->clutch_pos) / span;
    if (x < 0.0) x = 0.0;
    if (x > 1.0) x = 1.0;
    return x * x * (3.0 - 2.0 * x);  /* smoothstep: progressive bite */
}

ShiftResult transmission_validate_shift(const Transmission *t, int target, double speed_mps,
                                        bool engine_running, double wheel_radius_m,
                                        double redline_rpm, double *rpm_after)
{
    double ratio, rpm;
    if (rpm_after != NULL) *rpm_after = 0.0;
    if (t == NULL || target < GEAR_REVERSE || target > GEAR_5) return SHIFT_ERR_INVALID_GEAR;
    if (target == GEAR_NEUTRAL) return SHIFT_OK;      /* always possible */

    if (engine_running && t->clutch_pos < CLUTCH_SHIFT_THRESHOLD) return SHIFT_ERR_CLUTCH;
    if (target == GEAR_REVERSE && speed_mps >  REVERSE_SPEED_LIMIT_MPS)
        return SHIFT_ERR_REVERSE_WHILE_MOVING;
    if (target > GEAR_NEUTRAL && speed_mps < -REVERSE_SPEED_LIMIT_MPS)
        return SHIFT_ERR_FORWARD_WHILE_REVERSING;

    ratio = fabs(transmission_ratio(t, (Gear)target));
    rpm = fabs(speed_mps) / wheel_radius_m * ratio * 60.0 / (2.0 * PI);
    if (rpm_after != NULL) *rpm_after = rpm;
    return rpm > redline_rpm ? SHIFT_WARN_OVERREV : SHIFT_OK;
}

bool transmission_shift_accepted(ShiftResult r)
{
    return r == SHIFT_OK || r == SHIFT_WARN_OVERREV;
}

void transmission_add_wear(Transmission *t, double gear_amount, double clutch_amount)
{
    t->gear_wear   += gear_amount;
    t->clutch_wear += clutch_amount;
    if (t->gear_wear   > 100.0) t->gear_wear   = 100.0;
    if (t->clutch_wear > 100.0) t->clutch_wear = 100.0;
}

double transmission_condition(const Transmission *t)
{
    double c = 100.0 - t->gear_wear - t->clutch_wear;
    return c < 0.0 ? 0.0 : c;
}

double transmission_clutch_condition(const Transmission *t)
{
    return 100.0 - t->clutch_wear;
}

const char *transmission_gear_label(Gear g)
{
    static const char *labels[] = { "R", "N", "1", "2", "3", "4", "5" };
    if (g < GEAR_REVERSE || g > GEAR_5) return "?";
    return labels[g + 1];
}

const char *transmission_shift_text(ShiftResult r)
{
    switch (r) {
    case SHIFT_OK:                         return "Gear engaged";
    case SHIFT_ERR_INVALID_GEAR:           return "That gear does not exist";
    case SHIFT_ERR_CLUTCH:                 return "Gear grind: press the clutch fully before shifting";
    case SHIFT_ERR_REVERSE_WHILE_MOVING:   return "Reverse refused: stop the car first";
    case SHIFT_ERR_FORWARD_WHILE_REVERSING:return "Forward gear refused: stop reversing first";
    case SHIFT_WARN_OVERREV:               return "Over-rev downshift: gearbox stressed";
    }
    return "Unknown shift result";
}
