#include <gmock/gmock.h>
#include "Eigen/Core"
#include "state/kalman.h"
#include "math_debug_util.h"

#define NUM_TESTS 100

inline void setup() {
    fp accel_reference[3] = {0, 0, 1};
    fp magno_reference[3] = {1, 0, 0};
    fp quat[4] = {0, 0, 0, 1};

    kalman_init(accel_reference, magno_reference, quat);
}

inline void testEstimateStable(const state_estimate_t &estimate) {
    for (int i = 0; i < 4; i++)
        EXPECT_FALSE(isnan(estimate.orientation_q[i]));

    EXPECT_FALSE(isnan(estimate.altitude));
    EXPECT_FALSE(isnan(estimate.latitude));
    EXPECT_FALSE(isnan(estimate.longitude));

    for (int i = 0; i < 4; i++) {
        EXPECT_FALSE(isnan(estimate.angular_velocity[i]));
    }
}

TEST(TestKalmanStability, TestPredict) {
    setup();

    for (int i = 0; i < NUM_TESTS; i++) {
        kalman_predict(1 / 1000.0f);

        state_estimate_t estimate;
        kalman_get_state(&estimate);
        testEstimateStable(estimate);
    }
}

TEST(TestKalmanStability, TestAccel) {
    setup();

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
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 0], 1e-5f);
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 1], 1e-5f);

    expect_quat_eq(Eigen::Quaternion<fp>(1, 0, 0, 0), estimate.orientation_q);
}

TEST(TestKalmanStability, TestMagno) {
    setup();

    state_estimate_t estimate;

    for (int i = 0; i < NUM_TESTS; i++) {
        kalman_predict(1 / 1000.0f);


        fp magno[3] = {1, 0, 0};
        kalman_new_magno(magno);

        kalman_get_state(&estimate);
        testEstimateStable(estimate);
    }

    fp P[KALMAN_NUM_STATES];
    kalman_get_covariance(P);

    // We expect the covariance of the second two terms (attitude error in x and y) to be close to zero
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 1], 1e-5f);
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 2], 1e-5f);

    expect_quat_eq(Eigen::Quaternion<fp>(1, 0, 0, 0), estimate.orientation_q);
}

TEST(TestKalmanStability, TestMagnoAccel) {
    setup();

    state_estimate_t estimate;

    for (int i = 0; i < NUM_TESTS; i++) {
        kalman_predict(1 / 1000.0f);


        fp magno[3] = {1, 0, 0};
        fp accel[3] = {0, 0, 1};
        kalman_new_magno(magno);
        kalman_new_accel(accel);

        kalman_get_state(&estimate);
        testEstimateStable(estimate);
    }

    fp P[KALMAN_NUM_STATES];
    kalman_get_covariance(P);

    // We expect the attitude error covariance to be close to zero
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 0], 1e-5f);
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 1], 1e-5f);
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 2], 1e-5f);

    expect_quat_eq(Eigen::Quaternion<fp>(1, 0, 0, 0), estimate.orientation_q);
}

TEST(TestKalmanStability, TestMagnoAccelGyro) {
    setup();

    state_estimate_t estimate;
    for (int i = 0; i < NUM_TESTS; i++) {
        kalman_predict(1 / 1000.0f);


        fp magno[3] = {1, 0, 0};
        fp accel[3] = {0, 0, 1};
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
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 0], 1e-5f);
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 1], 1e-5f);
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 2], 1e-5f);

    // We expect angular velocity covariance to be close to zero
    EXPECT_LT(P[KALMAN_ANGULAR_VEL_IDX + 0], 1e-5f);
    EXPECT_LT(P[KALMAN_ANGULAR_VEL_IDX + 1], 1e-5f);
    EXPECT_LT(P[KALMAN_ANGULAR_VEL_IDX + 2], 1e-5f);

    // We expect gyro bias covariance to be close to zero
    EXPECT_LT(P[KALMAN_GYRO_BIAS_IDX + 0], 1e-5f);
    EXPECT_LT(P[KALMAN_GYRO_BIAS_IDX + 1], 1e-5f);
    EXPECT_LT(P[KALMAN_GYRO_BIAS_IDX + 2], 1e-5f);

    expect_quat_eq(Eigen::Quaternion<fp>(1, 0, 0, 0), estimate.orientation_q);
}

