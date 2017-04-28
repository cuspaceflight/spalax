#include "KalmanTestUtils.h"
#include <gmock/gmock.h>

fp accel_reference[3] = {0, 0, 1};
fp magno_reference[3] = {0.39134267f, -0.00455851434f, -0.920233727f};


void kalman_test_setup(Eigen::Quaternion<fp> quat, Eigen::Matrix<fp, 3, 1> angular_velocity, Eigen::Matrix<fp, 3, 1> position,
                       Eigen::Matrix<fp, 3, 1> velocity, Eigen::Matrix<fp, 3, 1> acceleration) {
    fp quat_arr[4] = {quat.x(), quat.y(), quat.z(), quat.w()};
    fp ang_vel[3] = {angular_velocity.x(), angular_velocity.y(), angular_velocity.z()};
    fp pos[3] = {position.x(), position.y(), position.z()};
    fp vel[3] = {velocity.x(), velocity.y(), velocity.z()};
    fp accel[3] = {acceleration.x(), acceleration.y(), acceleration.z()};
    fp initial_gyro_bias[3] = {0,0,0};
    fp initial_accel_bias[3] = {0,0,0};
    fp initial_magno_bias[3] = {0,0,0};

    kalman_init(accel_reference, magno_reference, quat_arr, ang_vel, pos, vel, accel, initial_gyro_bias, initial_accel_bias, initial_magno_bias);
}

void testEstimateStable(const state_estimate_t &estimate) {
    for (int i = 0; i < 4; i++)
        EXPECT_FALSE(isnan(estimate.orientation_q[i]));

    for (int i = 0; i < 3; i++) {
        EXPECT_FALSE(isnan(estimate.angular_velocity[i]));
        EXPECT_FALSE(isnan(estimate.position[i]));
        EXPECT_FALSE(isnan(estimate.velocity[i]));
        EXPECT_FALSE(isnan(estimate.acceleration[i]));
    }
}
