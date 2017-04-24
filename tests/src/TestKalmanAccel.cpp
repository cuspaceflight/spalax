#include <gmock/gmock.h>
#include "math_debug_util.h"
#include <random>
#include "KalmanTestUtils.h"
#include <matplotlibcpp.h>

#define NUM_TESTS 5000

using namespace Eigen;
namespace plt = matplotlibcpp;


static void
accel_test(const Matrix<fp, 3, 1> &angle_increment, const Matrix<fp, 3, 1> &accel, const char *filename = nullptr) {
    state_estimate_t estimate;

    Matrix<fp, 3, 1> unrotated_magno = Matrix<fp, 3, 1>(magno_reference[0], magno_reference[1], magno_reference[2]);
    Matrix<fp, 3, 1> unrotated_accel = Matrix<fp, 3, 1>(accel_reference[0], accel_reference[1], accel_reference[2]);

    std::default_random_engine generator(3452456);

    // TODO: get this from kalman
    std::normal_distribution<fp> accel_distribution(0.0, kalman_accelerometer_cov);
    std::normal_distribution<fp> magno_distribution(0.0, kalman_magno_cov);
    std::normal_distribution<fp> gyro_distribution(0, kalman_gyro_cov);

    Quaternion<fp> quat(-0.66327167417264843f, -0.34436319883409405f, 0.039508389760580714f, -0.66326748802235758f);

    const float time_increment = 1000;

    kalman_test_setup(quat.inverse(), time_increment * angle_increment, Matrix<fp, 3, 1>::Zero(), Matrix<fp, 3, 1>::Zero(),
                      accel);


    std::vector<float> x_velocity;
    std::vector<float> y_velocity;
    std::vector<float> z_velocity;
    std::vector<float> timestamps;

    Quaternion<fp> delta(AngleAxis<fp>(angle_increment.norm(), angle_increment.normalized()));

    for (int i = 0; i < NUM_TESTS; i++) {
        Quaternion<fp> t = quat * delta.inverse();
        quat = t;

        Matrix<fp, 3, 1> rotated_magno = quat * unrotated_magno;
        Matrix<fp, 3, 1> rotated_accel = quat * (unrotated_accel + accel);
        Matrix<fp, 3, 1> rotated_gyro = quat * angle_increment;

        fp magno[3] = {rotated_magno.x() + magno_distribution(generator),
                       rotated_magno.y() + magno_distribution(generator),
                       rotated_magno.z() + magno_distribution(generator)};
        fp accel[3] = {rotated_accel.x() + accel_distribution(generator),
                       rotated_accel.y() + accel_distribution(generator),
                       rotated_accel.z() + accel_distribution(generator)};
        fp gyro[3] = {rotated_gyro.x() * time_increment + gyro_distribution(generator),
                      rotated_gyro.y() * time_increment + gyro_distribution(generator),
                      rotated_gyro.z() * time_increment + gyro_distribution(generator)};

        kalman_predict(1.0f / time_increment);

        kalman_new_gyro(gyro);
        kalman_new_magno(magno);
        kalman_new_accel(accel);

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


    Quaternion<fp> out(estimate.orientation_q[3], estimate.orientation_q[0], estimate.orientation_q[1],
                       estimate.orientation_q[2]);

    // The output should rotate the observations onto the references
    Matrix<fp, 3, 1> magno_test = out * (quat * unrotated_magno);
    Matrix<fp, 3, 1> accel_test = out * (quat * unrotated_accel);

    // Clampf is necessary due to occasional rounding errors leading to results slight out of the range (-1, 1)
    fp angle = acos(clampf(magno_test.normalized().transpose() * unrotated_magno.normalized(), -1, 1)) * 180.0f / (fp)M_PI;
    fp angle2 = acos(clampf(accel_test.normalized().transpose() * unrotated_accel.normalized(), -1, 1)) * 180.0f / (fp)M_PI;

    // We expect accuracy to a degree - any less is unreasonable with 32-bit fping point
    EXPECT_LT(angle, 1.f);
    EXPECT_LT(angle2, 1.f);

    float time = NUM_TESTS / time_increment;

    for (int i = 0; i < 3; i++) {
        expect_fuzzy_eq(estimate.angular_velocity[i], angle_increment[i] * time_increment, 0.00005f, 0.05);
        expect_fuzzy_eq(estimate.position[i], 0.5f * accel[i] * time * time, 0.00005f, kalman_accelerometer_cov * time * time);
        expect_fuzzy_eq(estimate.velocity[i], accel[i] * time, 0.00005f, kalman_accelerometer_cov * time);
        expect_fuzzy_eq(estimate.acceleration[i], accel[i], 0.00005f, kalman_accelerometer_cov);
    }
}

TEST(TestKalmanAccel, TestSimple) {
    accel_test(Matrix<fp, 3, 1>::Zero(), Matrix<fp, 3, 1>(1, 0, 0));
}

TEST(TestKalmanAccel, TestComplex) {
    accel_test(Matrix<fp, 3, 1>(1.0f / 1000, 0, 0), Matrix<fp, 3, 1>(1, 0, 0));

    accel_test(Matrix<fp, 3, 1>(1.0f / 1000, 0, 0), Matrix<fp, 3, 1>(0, 1, 0));

    accel_test(Matrix<fp, 3, 1>(1.0f / 1000, 0, 0), Matrix<fp, 3, 1>(0, 0, 1));

    accel_test(Matrix<fp, 3, 1>(1.0f / 1000, 0, 5.0f / 1000), Matrix<fp, 3, 1>(0, 0, 1));
}