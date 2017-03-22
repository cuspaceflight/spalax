#include <gmock/gmock.h>
#include "Eigen/Core"
#include "state/kalman.h"
#include "math_debug_util.h"
#include <random>
#include <matplotlibcpp.h>
#include "KalmanTestUtils.h"

#define NUM_TESTS 1000

using namespace Eigen;
namespace plt = matplotlibcpp;

static void gyro_test(const Matrix<fp, 3, 1> &angle_increment, const char *filename = nullptr) {
    state_estimate_t estimate;

    Matrix<fp, 3, 1> unrotated_magno = Matrix<fp, 3, 1>(magno_reference[0], magno_reference[1], magno_reference[2]);
    Matrix<fp, 3, 1> unrotated_accel = Matrix<fp, 3, 1>(accel_reference[0], accel_reference[1], accel_reference[2]);

    std::default_random_engine generator(3452456);

    // TODO: get this from kalman
    std::normal_distribution<fp> distribution(0.0, 0.00001f);
    std::normal_distribution<fp> gyro_distribution(0, 0.00001f);

    Quaternion<fp> quat(-0.66327167417264843f, -0.34436319883409405f, 0.039508389760580714f, -0.66326748802235758f);
    Quaternion<fp> delta(AngleAxis<fp>(angle_increment.norm(), angle_increment.normalized()));


    const float time_increment = 1000;

    kalman_test_setup(quat, time_increment * angle_increment);

    std::vector<float> x_velocity;
    std::vector<float> y_velocity;
    std::vector<float> z_velocity;
    std::vector<float> timestamps;


    for (int i = 0; i < NUM_TESTS; i++) {
        Quaternion<fp> t = delta * quat;
        quat = t;

        Matrix<fp, 3, 1> rotated_magno = quat * unrotated_magno;
        Matrix<fp, 3, 1> rotated_accel = quat * unrotated_accel;
        Matrix<fp, 3, 1> rotated_gyro = quat * angle_increment;

        fp magno[3] = {rotated_magno.x() + distribution(generator), rotated_magno.y() + distribution(generator),
                       rotated_magno.z() + distribution(generator)};
        fp accel[3] = {rotated_accel.x() + distribution(generator), rotated_accel.y() + distribution(generator),
                       rotated_accel.z() + distribution(generator)};
        fp gyro[3] = {rotated_gyro.x() * time_increment + gyro_distribution(generator),
                      rotated_gyro.y() * time_increment + gyro_distribution(generator),
                      rotated_gyro.z() * time_increment + gyro_distribution(generator)};

        kalman_predict(1.0f / time_increment);

        kalman_new_magno(magno);
        kalman_new_accel(accel);
        kalman_new_gyro(gyro);

        kalman_get_state(&estimate);
        testEstimateStable(estimate);


        if (filename) {
            timestamps.push_back(i / time_increment);

            x_velocity.push_back(estimate.angular_velocity[0]);
            y_velocity.push_back(estimate.angular_velocity[1]);
            z_velocity.push_back(estimate.angular_velocity[2]);
        }
    }

    if (filename) {
        plt::figure();

        plt::named_plot("X Velocity", timestamps, x_velocity);
        plt::named_plot("Y Velocity", timestamps, y_velocity);
        plt::named_plot("Z Velocity", timestamps, z_velocity);

        plt::grid(true);
        plt::legend();
        plt::save(filename);
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

    Matrix<fp, 3, 1> rotated_magno = quat * unrotated_magno;
    Matrix<fp, 3, 1> rotated_accel = quat * unrotated_accel;

    // Clampf is necessary due to occasional rounding errors leading to results slight out of the range (-1, 1)
    fp angle =
            acos(clampf(magno_test.normalized().transpose() * rotated_magno.normalized(), -1, 1)) * 180.0f / (fp) M_PI;
    fp angle2 =
            acos(clampf(accel_test.normalized().transpose() * rotated_accel.normalized(), -1, 1)) * 180.0f / (fp) M_PI;

    // We expect accuracy to a degree - any less is unreasonable with 32-bit floating point
    EXPECT_LT(angle, 1.0f);
    EXPECT_LT(angle2, 1.0f);

    expect_fuzzy_eq(estimate.angular_velocity[0], angle_increment[0] * time_increment, 0.00005f, 0.05);
    expect_fuzzy_eq(estimate.angular_velocity[1], angle_increment[1] * time_increment, 0.00005f, 0.05);
    expect_fuzzy_eq(estimate.angular_velocity[2], angle_increment[2] * time_increment, 0.00005f, 0.05);
}

TEST(TestKalmanDynamic, TestGyro) {
    // 1 rad/s
    gyro_test(Matrix<fp, 3, 1>(1.0f / 1000, 0, 0));

    // 5 rad/s
    gyro_test(Matrix<fp, 3, 1>(5.0f / 1000, 0, 0));

    // 1 rad/s
    gyro_test(Matrix<fp, 3, 1>(0, 1.0f / 1000, 0));

    // 5 rad/s
    gyro_test(Matrix<fp, 3, 1>(0, 5.0f / 1000, 0));

    // 1 rad/s
    gyro_test(Matrix<fp, 3, 1>(0, 0, 1.0f / 1000));

    // 5 rad/s
    gyro_test(Matrix<fp, 3, 1>(0, 0, 5.0f / 1000));
}


TEST(TestKalmanDynamic, TestGyroComplex) {
    // 1 rad/s
    gyro_test(Matrix<fp, 3, 1>(1.0f / 1000, 1.0f / 1000, 0));

    // 5 rad/s
    gyro_test(Matrix<fp, 3, 1>(5.0f / 1000, 5.0f / 1000, 0));


    // 1 rad/s
    gyro_test(Matrix<fp, 3, 1>(0, 1.0f / 1000, 1.0f / 1000));

    // 5 rad/s
    gyro_test(Matrix<fp, 3, 1>(0, 5.0f / 1000, 5.0f / 1000));


    // 1 rad/s
    gyro_test(Matrix<fp, 3, 1>(1.0f / 1000, 0, 1.0f / 1000));

    // 5 rad/s
    gyro_test(Matrix<fp, 3, 1>(5.0f / 1000, 0, 5.0f / 1000));
}

TEST(TestKalmanDynamic, TestGyroFull) {
    // 1 rad/s
    gyro_test(Matrix<fp, 3, 1>(1.0f / 1000, 1.0f / 1000, 1.0f / 1000));

    // 5 rad/s
    gyro_test(Matrix<fp, 3, 1>(5.0f / 1000, 5.0f / 1000, 5.0f / 1000));
}

TEST(TestKalmanDynamic, TestAngularVelocity) {
    gyro_test(Matrix<fp, 3, 1>(1.0f / 1000, 1.0f / 1000, 1.0f / 1000));
}