#include <gmock/gmock.h>
#include "Eigen/Core"
#include "state/kalman.h"
#include "math_debug_util.h"
#include <random>
#include "state/math_util.h"

#define NUM_TESTS 1000

using namespace Eigen;


TEST(TestStateMathUtils, TestMRPToQuat) {
    for (int i = 0; i < NUM_TESTS; i++) {
        Matrix<fp, 3, 1> mrp = Matrix<fp, 3, 1>(get_rand(), get_rand() , get_rand());
        auto quat = mrpToQuat(mrp);
        Matrix<fp, 3, 1> mrp_2 = quatToMrp(quat);

        for (int i = 0; i < 3; i++)
            expect_fuzzy_eq(mrp[i], mrp_2[i]);
    }
}

TEST(TestStateMathUtils, TestQuatToMrp) {
    for (int i = 0; i < NUM_TESTS; i++) {
        Quaternion<fp> quat1(AngleAxis<fp>(get_rand(), Matrix<fp, 3, 1>::UnitZ()) * AngleAxis<fp>(get_rand(), Matrix<fp, 3, 1>::UnitY()) * AngleAxis<fp>(get_rand(), Matrix<fp, 3, 1>::UnitX()));
        auto mrp = quatToMrp(quat1);
        Quaternion<fp> quat2 = mrpToQuat(mrp);

        expect_fuzzy_eq(quat1.x(), quat2.x(), 1e-9f, 1e-6f);
        expect_fuzzy_eq(quat1.y(), quat2.y(), 1e-9f, 1e-6f);
        expect_fuzzy_eq(quat1.z(), quat2.z(), 1e-9f, 1e-6f);
        expect_fuzzy_eq(quat1.w(), quat2.w(), 1e-9f, 1e-6f);
    }
}

TEST(TestStateMathUtils, TestGeodeticToECEF) {

    Matrix<float, 3, 1> out = geodetic_to_ecef(52, 1, 100);
    Matrix<float, 3, 1> expected_out(3934423, 68676, 5002882);

    for (int i = 0; i < 3; i++)
        expect_fuzzy_eq(out[i], expected_out[i]);

    out = geodetic_to_ecef(-21, 3, -100);
    expected_out = Matrix<float, 3, 1>(5948808, 311764, -2271359);

    for (int i = 0; i < 3; i++)
        expect_fuzzy_eq(out[i], expected_out[i]);
}