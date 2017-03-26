#include <gmock/gmock.h>
#include "math_debug_util.h"
#include <random>
#include "KalmanTestUtils.h"

#define NUM_TESTS 5000

using namespace Eigen;

void accelMagnoTest(const Quaternion<fp>& quat) {
    kalman_test_setup(quat.inverse());

    state_estimate_t estimate;

    Matrix<fp, 3, 1> unrotated_magno = Matrix<fp, 3, 1>(magno_reference[0], magno_reference[1], magno_reference[2]);
    Matrix<fp, 3, 1> unrotated_accel = Matrix<fp, 3, 1>(accel_reference[0] , accel_reference[1], accel_reference[2]);

    Matrix<fp, 3, 1> rotated_magno = quat * unrotated_magno;
    Matrix<fp, 3, 1> rotated_accel = quat * unrotated_accel;


    std::default_random_engine generator(3452456);

    // TODO: get this from kalman
    std::normal_distribution<fp> magno_distribution (0.0, kalman_magno_cov);
    std::normal_distribution<fp> accel_distribution (0.0, kalman_accelerometer_cov);

    for (int i = 0; i < NUM_TESTS; i++) {
        kalman_predict(1 / 1000.0f);

        fp magno[3] = {rotated_magno.x() + magno_distribution(generator), rotated_magno.y()  + magno_distribution(generator), rotated_magno.z() + magno_distribution(generator)};
        fp accel[3] = {rotated_accel.x() + accel_distribution(generator), rotated_accel.y()  + accel_distribution(generator), rotated_accel.z() + accel_distribution(generator)};

        kalman_new_magno(magno);
        kalman_new_accel(accel);

        kalman_get_state(&estimate);
        testEstimateStable(estimate);
    }

    kalman_get_state(&estimate);
    testEstimateStable(estimate);

    fp P[KALMAN_NUM_STATES];
    kalman_get_covariance(P);

    // We expect the attitude error covariance to be close to zero
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 0], 1e-3f);
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 1], 1e-3f);
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 2], 1e-3f);

    Quaternion<fp> out(estimate.orientation_q[3], estimate.orientation_q[0], estimate.orientation_q[1],
                    estimate.orientation_q[2]);

    // The output should rotate the observations onto the references
    Matrix<fp, 3, 1> magno_test = out * (quat * unrotated_magno);
    Matrix<fp, 3, 1> accel_test = out * (quat * unrotated_accel);

    // Clampf is necessary due to occasional rounding errors leading to results slight out of the range (-1, 1)
    fp angle = acos(clampf(magno_test.normalized().transpose() * unrotated_magno.normalized(), -1, 1)) * 180.0f / (fp)M_PI;
    fp angle2 = acos(clampf(accel_test.normalized().transpose() * unrotated_accel.normalized(), -1, 1)) * 180.0f / (fp)M_PI;

    // We expect accuracy to a degree - any less is unreasonable with 32-bit fping point
    EXPECT_LT(angle, 1.f);
    EXPECT_LT(angle2, 1.f);
}

TEST(TestKalmanStatic, TestAccelMagno) {
    Quaternion<fp> quat(AngleAxis<fp>(1.2f, Matrix<fp, 3, 1>(1, 0, 0)));
    accelMagnoTest(quat);
}



TEST(TestKalmanStatic, TestAccelMagno2) {
    Quaternion<fp> quat(AngleAxis<fp>(1.2f, Matrix<fp, 3, 1>(0, 1, 0)));
    accelMagnoTest(quat);
}

TEST(TestKalmanStatic, TestAccelMagno3) {
    Quaternion<fp> quat(-0.66327167417264843f, -0.34436319883409405f, 0.039508389760580714f, -0.66326748802235758f);
    accelMagnoTest(quat);
}

TEST(TestKalmanStatic, TestAccelMagnoRandom) {
    for (int i = 0; i < 1; i++) {
        Quaternion<fp> quat(AngleAxis<fp>(get_rand(), Matrix<fp, 3, 1>::UnitZ()) * AngleAxis<fp>(get_rand(), Matrix<fp, 3, 1>::UnitY()) * AngleAxis<fp>(get_rand(), Matrix<fp, 3, 1>::UnitX()));
        accelMagnoTest(quat);
    }
}