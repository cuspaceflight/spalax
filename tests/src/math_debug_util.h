#pragma once
#include "Eigen/Core"
#include <Eigen/Geometry>

float get_rand(float range = 20.0f);

bool fuzzy_eq(float A, float B, float maxRelativeError = 0.00005f, float maxAbsoluteError = 1e-5f);

void expect_fuzzy_eq(float A, float B, float maxRelativeError = 0.00005f, float maxAbsoluteError = 1e-5f);

bool quat_eq(const Eigen::Quaternionf& q, float q_arr[4], float maxRelativeError = 0.00005f);

void expect_quat_eq(const Eigen::Quaternionf& q, float q_arr[4], float maxRelativeError = 0.00005f);