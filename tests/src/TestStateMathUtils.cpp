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