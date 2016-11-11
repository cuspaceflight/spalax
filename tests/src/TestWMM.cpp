#include <gmock/gmock.h>
#include "component_state.h"
#include "state/wmm_util.h"
#include "math_debug_util.h"

TEST(TestWMM, TestDeclination) {
    component_state_start();
    wmm_util_init(2016.8790832455215);

    MagneticFieldParams params;
    wmm_util_get_magnetic_field(52,0,0, &params);

    // We only care about absolute error for these tests
    expect_fuzzy_eq(params.declination, -0.6189, 0, 0.1);
    expect_fuzzy_eq(params.inclination, 66.8696, 0, 0.1);
    expect_fuzzy_eq(params.field_strength, 48860.0, 0, 50);
    expect_fuzzy_eq(params.horizontal_field_strength, 19193.4, 0, 50);
    expect_fuzzy_eq(params.field_vector[0], 19191.8, 0, 50);
    expect_fuzzy_eq(params.field_vector[1], -250.2, 0, 50);
    expect_fuzzy_eq(params.field_vector[2], 44932.3, 0, 50);
}