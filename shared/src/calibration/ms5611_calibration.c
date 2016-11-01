#include "ms5611_calibration.h"
#include <math.h>

/* Constants from the US Standard Atmosphere 1976 */
const float Rs = 8.31432f;
const float g0 = 9.80665f;
const float M = 0.0289644f;
const float Lb[7] = { -0.0065f, 0.0f, 0.001f, 0.0028f, 0.0f, -0.0028f, -0.002f };
const float Pb[7] = { 101325.0f, 22632.10f, 5474.89f, 868.02f, 110.91f, 66.94f, 3.96f };
const float Tb[7] = { 288.15f, 216.65f, 216.65f, 228.65f, 270.65f, 270.65f, 214.65f };
const float Hb[7] = { 0.0f, 11000.0f, 20000.0f, 32000.0f, 47000.0f, 51000.0f, 71000.0f };

/*
* Convert a pressure and an atmospheric level b into an altitude.
* Reverses the standard equation for non-zero lapse regions,
* P = Pb (Tb / (Tb + Lb(h - hb)))^(M g0 / R* Lb)
*/
static float state_estimation_p2a_nonzero_lapse(float pressure, int b)
{
    float lb = Lb[b];
    float hb = Hb[b];
    float pb = Pb[b];
    float tb = Tb[b];

    return hb + tb / lb * (powf(pressure / pb, (-Rs*lb) / (g0*M)) - 1.0f);
}

/* Convert a pressure and an atmospheric level b into an altitude.
* Reverses the standard equation for zero-lapse regions,
* P = Pb exp( -g0 M (h-hb) / R* Tb)
*/
static float state_estimation_p2a_zero_lapse(float pressure, int b)
{
    float hb = Hb[b];
    float pb = Pb[b];
    float tb = Tb[b];

    return hb + (Rs * tb) / (g0 * M) * (logf(pressure) - logf(pb));
}

float ms5611_get_altitude(float pressure)
{
    int b;
    /* For each level of the US Standard Atmosphere 1976, check if the pressure
    * is inside that level, and use the appropriate conversion based on lapse
    * rate at that level.
    */
    if (pressure > Pb[0]) {
        return state_estimation_p2a_nonzero_lapse(pressure, 0);
    }
    for (b = 0; b < 6; b++) {
        if (pressure <= Pb[b] && pressure > Pb[b + 1]) {
            if (Lb[b] == 0.0f) {
                return state_estimation_p2a_zero_lapse(pressure, b);
            }
            else {
                return state_estimation_p2a_nonzero_lapse(pressure, b);
            }
        }
    }

    /* If no levels matched, something is wrong, returning -9999f will cause
    * this pressure value to be ignored.
    */
    return -9999.0f;
}