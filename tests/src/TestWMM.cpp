#include <gmock/gmock.h>
#include "component_state.h"
#include "state/wmm_util.h"
#include "math_debug_util.h"
#include "../../external/wmm/src/EGM9615.h"

static void update_handler(avionics_component_t component, avionics_component_state_t state, int line) {
    ASSERT_NE(state, state_error);
}

TEST(TestWMM, TestDeclination) {
    component_state_start(update_handler, false);

    // 11/11/2016
    wmm_util_init(2016.860655737705);

    MagneticFieldParams params;
    wmm_util_get_magnetic_field(52,0,0, &params);

    // We only care about absolute error for these tests
    expect_fuzzy_eq(params.declination, -0.6189, 0, 0.01);
    expect_fuzzy_eq(params.inclination, 66.8696, 0, 0.01);
    expect_fuzzy_eq(params.field_strength, 48860.0, 0, 50);
    expect_fuzzy_eq(params.horizontal_field_strength, 19193.4, 0, 50);
    expect_fuzzy_eq(params.field_vector[0], 19191.8, 0, 50);
    expect_fuzzy_eq(params.field_vector[1], -250.2, 0, 50);
    expect_fuzzy_eq(params.field_vector[2], 44932.3, 0, 50);
}

TEST(TestWMM, TestAltitude) {
    component_state_start(update_handler, false);

    // 11/11/2016
    wmm_util_init(2016.860655737705);

    MagneticFieldParams params;
    wmm_util_get_magnetic_field(52,0,10, &params);

    // We only care about absolute error for these tests
    expect_fuzzy_eq(params.declination, -0.6363, 0, 0.01);
    expect_fuzzy_eq(params.inclination, 66.8532, 0, 0.01);
    expect_fuzzy_eq(params.field_strength, 48658.7, 0, 50);
    expect_fuzzy_eq(params.horizontal_field_strength, 19127.2, 0, 50);
    expect_fuzzy_eq(params.field_vector[0], 19126.0, 0, 50);
    expect_fuzzy_eq(params.field_vector[1], -212.4, 0, 50);
    expect_fuzzy_eq(params.field_vector[2], 44741.7, 0, 50);
}

TEST(TestWMM, TestDate) {
    component_state_start(update_handler, false);

    // 1/1/2016
    wmm_util_init(2016);

    MagneticFieldParams params;
    wmm_util_get_magnetic_field(52,0,10, &params);

    // We only care about absolute error for these tests
    expect_fuzzy_eq(params.declination, -0.7642, 0, 0.01);
    expect_fuzzy_eq(params.inclination, 66.8571, 0, 0.01);
    expect_fuzzy_eq(params.field_strength, 48639.1, 0, 50);
    expect_fuzzy_eq(params.horizontal_field_strength, 19116.4, 0, 50);
    expect_fuzzy_eq(params.field_vector[0], 19114.7, 0, 50);
    expect_fuzzy_eq(params.field_vector[1], -255.0, 0, 50);
    expect_fuzzy_eq(params.field_vector[2], 44725.0, 0, 50);
}

TEST(TestWMM, TestDateConversion) {
    double out;
    EXPECT_TRUE(wmm_util_get_year(2016,11,11, &out));
    expect_fuzzy_eq((float) out, (float) 2016.860655737705, 0, (float) (1 / 365.0));
}