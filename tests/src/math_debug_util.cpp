#include "math_debug_util.h"
#include <gmock/gmock.h>

float get_rand(float range) {
    float divisor = RAND_MAX / 2 / range;
    return (float) (rand() % RAND_MAX) / divisor - range;
}

bool fuzzy_eq(float A, float B, float maxRelativeError, float maxAbsoluteError) {
    if (fabs(A - B) < maxAbsoluteError)
        return true;
    float relativeError;
    if (fabs(B) > fabs(A))
        relativeError = fabs((A - B) / B);
    else
        relativeError = fabs((A - B) / A);
    return relativeError <= maxRelativeError;
}

void expect_fuzzy_eq(float A, float B, float maxRelativeError, float maxAbsoluteError) {
    EXPECT_TRUE(fuzzy_eq(A, B, maxRelativeError, maxAbsoluteError));
}

bool quat_eq(const Eigen::Quaternionf &q, float q_arr[4], float maxRelativeError) {
    if (fuzzy_eq(q.x(), q_arr[0], maxRelativeError) && fuzzy_eq(q.y(), q_arr[1], maxRelativeError) &&
        fuzzy_eq(q.z(), q_arr[2], maxRelativeError) && fuzzy_eq(q.w(), q_arr[3], maxRelativeError))
        return true;
    return fuzzy_eq(-q.x(), q_arr[0], maxRelativeError) && fuzzy_eq(-q.y(), q_arr[1], maxRelativeError) &&
           fuzzy_eq(-q.z(), q_arr[2], maxRelativeError) && fuzzy_eq(-q.w(), q_arr[3], maxRelativeError);
}

void expect_quat_eq(const Eigen::Quaternionf &q, float q_arr[4], float maxRelativeError) {
    EXPECT_TRUE(quat_eq(q, q_arr, maxRelativeError));
}