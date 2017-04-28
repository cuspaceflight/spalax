#pragma once
#include "math_debug_util.h"
#include "state/kalman.h"

extern fp accel_reference[3];
extern fp magno_reference[3];

void kalman_test_setup(Eigen::Quaternion<fp> quat = Eigen::Quaternion<fp>(1, 0, 0, 0),
                       Eigen::Matrix<fp, 3, 1> angular_velocity = Eigen::Matrix<fp, 3, 1>(0, 0, 0),
                       Eigen::Matrix<fp, 3, 1> position = Eigen::Matrix<fp, 3, 1>(0, 0, 0),
                       Eigen::Matrix<fp, 3, 1> velocity = Eigen::Matrix<fp, 3, 1>(0, 0, 0),
                       Eigen::Matrix<fp, 3, 1> acceleration = Eigen::Matrix<fp, 3, 1>(0, 0, 0));

void testEstimateStable(const state_estimate_t &estimate);

inline fp clampf(fp v, fp min, fp max) {
    if (v < min) v = min;
    if (v > max) v = max;
    return v;
}