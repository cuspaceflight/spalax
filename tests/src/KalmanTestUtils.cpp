#include "KalmanTestUtils.h"
#include <gmock/gmock.h>

fp accel_reference[3] = {0, 0, 1};
fp magno_reference[3] = {0.39134267f, -0.00455851434f, -0.920233727f};


void kalman_test_setup(Eigen::Quaternion<fp> quat, Eigen::Vector3f angular_velocity, Eigen::Vector3f acceleration) {
    fp quat_arr[4] = {quat.x(), quat.y(), quat.z(), quat.w()};
    fp ang_vel[3] = {angular_velocity.x(), angular_velocity.y(), angular_velocity.z()};
    fp accel[3] = {acceleration.x(), acceleration.y(), acceleration.z()};

    kalman_init(accel_reference, magno_reference, quat_arr, ang_vel, accel);
}

void testEstimateStable(const state_estimate_t &estimate) {
    for (int i = 0; i < 4; i++)
        EXPECT_FALSE(isnan(estimate.orientation_q[i]));

    EXPECT_FALSE(isnan(estimate.north));
    EXPECT_FALSE(isnan(estimate.east));
    EXPECT_FALSE(isnan(estimate.down));

    for (int i = 0; i < 3; i++) {
        EXPECT_FALSE(isnan(estimate.angular_velocity[i]));
    }
}
