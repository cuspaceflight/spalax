#include <gmock/gmock.h>
#include "Eigen/Core"
#include "state/kalman.h"
#include "math_debug_util.h"


inline void setup() {
    float accel_reference[3] = {0, 0, 1};
    float magno_reference[3] = {1, 0, 0};
    kalman_init(accel_reference, magno_reference);
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

TEST(TestKalman, TestPredict) {
    setup();

    for (int i = 0; i < 1000; i++) {
        kalman_predict(1 / 1000.0f);

        state_estimate_t estimate;
        kalman_get_state(&estimate);
        testEstimateStable(estimate);
    }
}

TEST(TestKalman, TestAccel) {
    setup();

    state_estimate_t estimate;
    float P_initial[12];
    kalman_get_covariance(P_initial);

    for (int i = 0; i < 29; i++) {
        kalman_predict(1 / 1000.0f);


        float accel[3] = {0, 0, 1};
        kalman_new_accel(accel);

        kalman_get_state(&estimate);
        testEstimateStable(estimate);
    }

    float P[12];
    kalman_get_covariance(P);

    // We expect the covariance of the first two terms (attitude error in x and y) to be close to zero
    EXPECT_LT(P[kalman_attitude_err_idx + 0], 1e-5f);
    EXPECT_LT(P[kalman_attitude_err_idx + 1], 1e-5f);


    // We expect the covariance of the third term to be close to its initial value as we haven't given it
    // any information about this axis
    expect_fuzzy_eq(P[kalman_attitude_err_idx + 2], P_initial[kalman_attitude_err_idx + 2]);

    for (int i = 3; i < 12; i++)
        expect_fuzzy_eq(P[i], P_initial[i]);

    expect_quat_eq(Eigen::Quaternionf(1, 0, 0, 0), estimate.orientation_q);
}

TEST(TestKalman, TestMagno) {
    setup();

    state_estimate_t estimate;
    float P_initial[12];
    kalman_get_covariance(P_initial);

    for (int i = 0; i < 29; i++) {
        kalman_predict(1 / 1000.0f);


        float magno[3] = {1, 0, 0};
        kalman_new_magno(magno);

        kalman_get_state(&estimate);
        testEstimateStable(estimate);
    }

    float P[12];
    kalman_get_covariance(P);

    // We expect the covariance of the second two terms (attitude error in x and y) to be close to zero
    EXPECT_LT(P[kalman_attitude_err_idx + 1], 1e-5f);
    EXPECT_LT(P[kalman_attitude_err_idx + 2], 1e-5f);


    // We expect the covariance of the first term to be close to its initial value as we haven't given it
    // any information about this axis
    expect_fuzzy_eq(P[kalman_attitude_err_idx + 0], P_initial[kalman_attitude_err_idx + 0]);

    for (int i = 3; i < 12; i++)
        expect_fuzzy_eq(P[i], P_initial[i]);

    expect_quat_eq(Eigen::Quaternionf(1, 0, 0, 0), estimate.orientation_q);
}

TEST(TestKalmanStability, TestMagnoAccel) {
    setup();

    state_estimate_t estimate;
    float P_initial[12];
    kalman_get_covariance(P_initial);

    for (int i = 0; i < 29; i++) {
        kalman_predict(1 / 1000.0f);


        float magno[3] = {1, 0, 0};
        float accel[3] = {0, 0, 1};
        kalman_new_magno(magno);
        kalman_new_accel(accel);

        kalman_get_state(&estimate);
        testEstimateStable(estimate);
    }

    float P[12];
    kalman_get_covariance(P);

    // We expect the attitude error covariance to be close to zero
    EXPECT_LT(P[kalman_attitude_err_idx + 0], 1e-5f);
    EXPECT_LT(P[kalman_attitude_err_idx + 1], 1e-5f);
    EXPECT_LT(P[kalman_attitude_err_idx + 2], 1e-5f);

    for (int i = 3; i < 12; i++)
        expect_fuzzy_eq(P[i], P_initial[i]);

    expect_quat_eq(Eigen::Quaternionf(1, 0, 0, 0), estimate.orientation_q);
}

TEST(TestKalmanStability, TestMagnoAccelGyro) {
    setup();

    state_estimate_t estimate;
    float P_initial[12];
    kalman_get_covariance(P_initial);

    for (int i = 0; i < 29; i++) {
        kalman_predict(1 / 1000.0f);


        float magno[3] = {1, 0, 0};
        float accel[3] = {0, 0, 1};
        float gyro[3] = {0, 0, 0};
        kalman_new_magno(magno);
        kalman_new_accel(accel);
        kalman_new_gyro(gyro);

        kalman_get_state(&estimate);
        testEstimateStable(estimate);
    }

    float P[12];
    kalman_get_covariance(P);

    // We expect the attitude error covariance to be close to zero
    EXPECT_LT(P[kalman_attitude_err_idx + 0], 1e-5f);
    EXPECT_LT(P[kalman_attitude_err_idx + 1], 1e-5f);
    EXPECT_LT(P[kalman_attitude_err_idx + 2], 1e-5f);

    // We expect angular velocity covariance to be close to zero
    EXPECT_LT(P[kalman_angular_vel_idx + 0], 1e-5f);
    EXPECT_LT(P[kalman_angular_vel_idx + 1], 1e-5f);
    EXPECT_LT(P[kalman_angular_vel_idx + 2], 1e-5f);

    // We expect gyro bias covariance to be close to zero
    EXPECT_LT(P[kalman_gyro_bias_idx + 0], 1e-5f);
    EXPECT_LT(P[kalman_gyro_bias_idx + 1], 1e-5f);
    EXPECT_LT(P[kalman_gyro_bias_idx + 2], 1e-5f);

    expect_fuzzy_eq(P[kalman_angular_acc_idx + 0], P_initial[kalman_angular_acc_idx + 0]);
    expect_fuzzy_eq(P[kalman_angular_acc_idx + 1], P_initial[kalman_angular_acc_idx + 1]);
    expect_fuzzy_eq(P[kalman_angular_acc_idx + 2], P_initial[kalman_angular_acc_idx + 2]);

    expect_quat_eq(Eigen::Quaternionf(1, 0, 0, 0), estimate.orientation_q);
}

