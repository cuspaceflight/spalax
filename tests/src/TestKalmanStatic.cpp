#include <gmock/gmock.h>
#include "Eigen/Core"
#include "state/kalman.h"
#include "math_debug_util.h"
#include <random>

#define NUM_TESTS 100

using namespace Eigen;

static fp accel_reference[3] = {0, 0, 1};
static fp magno_reference[3] = {1, 0, 0};

inline void setup(Quaternion<fp>& quat) {
    fp quat_arr[4] = {quat.x(), quat.y(), quat.z(), quat.w()};

    kalman_init(accel_reference, magno_reference, quat_arr);
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

inline fp clampf(fp v, fp min, fp max) {
    if (v < min) v = min;
    if (v > max) v = max;
    return v;
}

void accelMagnoTest(const Quaternion<fp>& quat) {
    state_estimate_t estimate;

    Matrix<fp, 3, 1> unrotated_magno = Matrix<fp, 3, 1>(1, 0, 0);
    Matrix<fp, 3, 1> unrotated_accel = Matrix<fp, 3, 1>(0, 0, 1);

    Matrix<fp, 3, 1> rotated_magno = quat * unrotated_magno;
    Matrix<fp, 3, 1> rotated_accel = quat * unrotated_accel;


    std::default_random_engine generator(3452456);

    // TODO: get this from kalman
    std::normal_distribution<fp> distribution (0.0, 0.001f);

    for (int i = 0; i < NUM_TESTS; i++) {
        //kalman_predict(1 / 1000.0f);

        fp magno[3] = {rotated_magno.x() + distribution(generator), rotated_magno.y()  + distribution(generator), rotated_magno.z() + distribution(generator)};
        fp accel[3] = {rotated_accel.x() + distribution(generator), rotated_accel.y()  + distribution(generator), rotated_accel.z() + distribution(generator)};

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

    Matrix<fp, 3, 1> magno_test = out * unrotated_magno;
    Matrix<fp, 3, 1> accel_test = out * unrotated_accel;

    // Clampf is necessary due to occasional rounding errors leading to results slight out of the range (-1, 1)
    fp angle = acos(clampf(magno_test.normalized().transpose() * rotated_magno.normalized(), -1, 1)) * 180.0f / (fp)M_PI;
    fp angle2 = acos(clampf(accel_test.normalized().transpose() * rotated_accel.normalized(), -1, 1)) * 180.0f / (fp)M_PI;

    // We expect accuracy to a degree - any less is unreasonable with 32-bit fping point
    EXPECT_LT(angle, 0.1f);
    EXPECT_LT(angle2, 0.1f);
}

TEST(TestKalmanStatic, TestAccelMagno) {
    Quaternion<fp> quat(AngleAxis<fp>(1.2f, Matrix<fp, 3, 1>(1, 0, 0)));
    setup(quat);

    accelMagnoTest(quat);
}



TEST(TestKalmanStatic, TestAccelMagno2) {
    Quaternion<fp> quat(AngleAxis<fp>(1.2f, Matrix<fp, 3, 1>(1, 0, 0)));
    setup(quat);

    accelMagnoTest(quat);
}

TEST(TestKalmanStatic, TestAccelMagno3) {
    Quaternion<fp> quat(-0.0133420276f, -0.981948674f, 0.0829190016f, 0.169478953f);
    setup(quat);

    accelMagnoTest(quat);
}

TEST(TestKalmanStatic, TestAccelMagnoRandom) {
    for (int i = 0; i < 100; i++) {
        Quaternion<fp> quat(AngleAxis<fp>(get_rand(), Matrix<fp, 3, 1>::UnitZ()) * AngleAxis<fp>(get_rand(), Matrix<fp, 3, 1>::UnitY()) * AngleAxis<fp>(get_rand(), Matrix<fp, 3, 1>::UnitX()));

        setup(quat);

        accelMagnoTest(quat);
    }
}