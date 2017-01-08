#ifndef WMM_UTIL_H
#define WMM_UTIL_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const double wmm_compile_time;
void wmm_util_init(double model_time);

typedef struct MagneticFieldParams {
    float declination;
    float inclination;
    float field_strength;
    float horizontal_field_strength;
    // North, East, Vertical
    float field_vector[3];
} MagneticFieldParams;


// Elevation is in kilometres relative to the WGS84 ellipsoid
void wmm_util_get_magnetic_field(float latitude, float longitude, float elevation, MagneticFieldParams* out);

bool wmm_util_get_year(int year, int month, int day, double* out);

#ifdef __cplusplus
}
#endif

#endif
