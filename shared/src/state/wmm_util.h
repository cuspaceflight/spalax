#ifndef WMM_UTIL_H
#define WMM_UTIL_H
#include <stdint.h>

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
    float field_vector[3];
} MagneticFieldParams;

void wmm_util_get_magnetic_field(float latitude, float longitude, float elevation, MagneticFieldParams* out);

#ifdef __cplusplus
}
#endif

#endif
