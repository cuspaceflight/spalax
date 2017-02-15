#include <gmock/gmock.h>
#include "Eigen/Core"
#include "state/kalman.h"
#include "math_debug_util.h"
#include <random>

#define NUM_TESTS 100

using namespace Eigen;

static float accel_reference[3] = {0, 0, 1};
static float magno_reference[3] = {1, 0, 0};

inline void setup() {

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

inline float clampf(float v, float min, float max) {
    if (v < min) v = min;
    if (v > max) v = max;
    return v;
}

void accelMagnoTest(const Quaternionf& quat) {
    state_estimate_t estimate;

    Vector3f unrotated_magno = Vector3f(1, 0, 0);
    Vector3f unrotated_accel = Vector3f(0, 0, 1);

    Vector3f rotated_magno = quat * unrotated_magno;
    Vector3f rotated_accel = quat * unrotated_accel;


    std::default_random_engine generator(3452456);

    // TODO: get this from kalman
    std::normal_distribution<float> distribution (0.0, 0.001f);

    for (int i = 0; i < NUM_TESTS; i++) {
        kalman_predict(1 / 1000.0f);

        float magno[3] = {rotated_magno.x() + distribution(generator), rotated_magno.y() + distribution(generator), rotated_magno.z() + distribution(generator)};
        float accel[3] = {rotated_accel.x() + distribution(generator), rotated_accel.y() + distribution(generator), rotated_accel.z() + distribution(generator)};

        kalman_new_magno(magno);
        kalman_new_accel(accel);

        kalman_get_state(&estimate);
        testEstimateStable(estimate);
    }

    kalman_get_state(&estimate);
    testEstimateStable(estimate);

    float P[KALMAN_NUM_STATES];
    kalman_get_covariance(P);

    // We expect the attitude error covariance to be close to zero
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 0], 1e-3f);
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 1], 1e-3f);
    EXPECT_LT(P[KALMAN_ATTITUDE_ERR_IDX + 2], 1e-3f);

    Quaternionf out(estimate.orientation_q[3], estimate.orientation_q[0], estimate.orientation_q[1],
                    estimate.orientation_q[2]);

    Vector3f magno_test = out * unrotated_magno;
    Vector3f accel_test = out * unrotated_accel;

    // Clampf is necessary due to occasional rounding errors leading to results slight out of the range (-1, 1)
    float angle = acos(clampf(magno_test.normalized().transpose() * rotated_magno.normalized(), -1, 1)) * 180.0f / (float)M_PI;
    float angle2 = acos(clampf(accel_test.normalized().transpose() * rotated_accel.normalized(), -1, 1)) * 180.0f / (float)M_PI;

    // We expect accuracy to a degree - any less is unreasonable with 32-bit floating point
    EXPECT_LT(angle, 1);
    EXPECT_LT(angle2, 1);
}

TEST(TestKalmanStatic, TestAccelMagno) {
    setup();

    Quaternionf quat(AngleAxisf(1.2f, Vector3f(1, 0, 0)));

    accelMagnoTest(quat);
}



TEST(TestKalmanStatic, TestAccelMagno2) {
    setup();

    Quaternionf quat(AngleAxisf(1.2f, Vector3f(1, 7, 0).normalized()));

    accelMagnoTest(quat);
}

//TEST(TestKalmanStatic, TestAccelMagno3) {
//    setup();
//
//    Quaternionf quat(0.132637814f, -0.319218159f, 0.0934810638f, 0.933685303f);
//
//    accelMagnoTest(quat);
//}

//TEST(TestKalmanStatic, TestAccelMagnoRandom) {
//    for (int i = 0; i < 100; i++) {
//        setup();
//
//        Quaternionf rotation(AngleAxisf(get_rand(), Vector3f::UnitZ()) * AngleAxisf(get_rand(), Vector3f::UnitY()) * AngleAxisf(get_rand(), Vector3f::UnitX()));
//
//        accelMagnoTest(rotation);
//    }
//}