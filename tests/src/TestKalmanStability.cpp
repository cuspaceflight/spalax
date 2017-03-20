#include <gmock/gmock.h>
#include "Eigen/Core"
#include "state/kalman.h"
#include "math_debug_util.h"
#include "KalmanTestUtils.h"

#define NUM_TESTS 100

TEST(TestKalmanStability, TestPredict) {
    kalman_test_setup();

    for (int i = 0; i < NUM_TESTS; i++) {
        kalman_predict(1 / 1000.0f);

        state_estimate_t estimate;
        kalman_get_state(&estimate);
        testEstimateStable(estimate);
    }
}

TEST(TestKalmanStability, TestAccel) {
    kalman_test_setup();

    state_estimate_t estimate;

    for (int i = 0; i < NUM_TESTS; i++) {
        kalman_predict(1 / 1000.0f);


        fp accel[3] = {0, 0, 1};
        kalman_new_accel(accel);

        kalman_get_state(&estimate);
        testEstimateStable(estimate);
    }

    fp P[KALMAN_NUM_STATES];
    kalman_get_covariance(P);

    // We expect the covariance of the first two terms (attitude error in x and y) to be close to zero
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 0], 1e-3f);
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 1], 1e-3f);

    expect_quat_eq(Eigen::Quaternionf(1, 0, 0, 0), estimate.orientation_q);
}

TEST(TestKalmanStability, TestMagno) {
    kalman_test_setup();

    state_estimate_t estimate;

    for (int i = 0; i < NUM_TESTS; i++) {
        kalman_predict(1 / 1000.0f);


        fp magno[3] = {magno_reference[0], magno_reference[1], magno_reference[2]};
        kalman_new_magno(magno);

        kalman_get_state(&estimate);
        testEstimateStable(estimate);
    }

    fp P[KALMAN_NUM_STATES];
    kalman_get_covariance(P);

    // We expect the covariance of the second two terms (attitude error in x and y) to be close to zero
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 1], 1e-3f);
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 2], 1e-3f);

    expect_quat_eq(Eigen::Quaternionf(1, 0, 0, 0), estimate.orientation_q);
}

TEST(TestKalmanStability, TestMagnoAccel) {
    kalman_test_setup();

    state_estimate_t estimate;

    for (int i = 0; i < NUM_TESTS; i++) {
        kalman_predict(1 / 1000.0f);


        fp magno[3] = {magno_reference[0], magno_reference[1], magno_reference[2]};
        fp accel[3] = {accel_reference[0], accel_reference[1], accel_reference[2]};
        kalman_new_magno(magno);
        kalman_new_accel(accel);

        kalman_get_state(&estimate);
        testEstimateStable(estimate);
    }

    fp P[KALMAN_NUM_STATES];
    kalman_get_covariance(P);

    // We expect the attitude error covariance to be close to zero
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 0], 1e-3f);
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 1], 1e-3f);
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 2], 1e-3f);

    expect_quat_eq(Eigen::Quaternionf(1, 0, 0, 0), estimate.orientation_q);
}

TEST(TestKalmanStability, TestMagnoAccelGyro) {
    kalman_test_setup();

    state_estimate_t estimate;
    for (int i = 0; i < NUM_TESTS; i++) {
        kalman_predict(1 / 1000.0f);


        fp magno[3] = {magno_reference[0], magno_reference[1], magno_reference[2]};
        fp accel[3] = {accel_reference[0], accel_reference[1], accel_reference[2]};
        fp gyro[3] = {0, 0, 0};
        kalman_new_magno(magno);
        kalman_new_accel(accel);
        kalman_new_gyro(gyro);

        kalman_get_state(&estimate);
        testEstimateStable(estimate);
    }

    fp P[KALMAN_NUM_STATES];
    kalman_get_covariance(P);

    // We expect the attitude error covariance to be close to zero
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 0], kalman_magno_cov);
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 1], kalman_magno_cov);
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 2], kalman_magno_cov);

    // We expect angular velocity covariance to be close to zero
    EXPECT_LT(P[KALMAN_ANGULAR_VEL_IDX + 0], 0.01);
    EXPECT_LT(P[KALMAN_ANGULAR_VEL_IDX + 1], 0.01);
    EXPECT_LT(P[KALMAN_ANGULAR_VEL_IDX + 2], 0.01);

    // We expect gyro bias covariance to be close to zero
    EXPECT_LT(P[KALMAN_GYRO_BIAS_IDX + 0], 0.01);
    EXPECT_LT(P[KALMAN_GYRO_BIAS_IDX + 1], 0.01);
    EXPECT_LT(P[KALMAN_GYRO_BIAS_IDX + 2], 0.01);

    expect_quat_eq(Eigen::Quaternionf(1, 0, 0, 0), estimate.orientation_q);
}

