#include <gmock/gmock.h>
#include "math_debug_util.h"
#include <random>
#include "KalmanTestUtils.h"
#include <matplotlibcpp.h>

static constexpr float time_increment = 1024;

using namespace Eigen;
namespace plt = matplotlibcpp;

static Matrix<fp, 3, 1> gyro_jerk_dependence(const Matrix<fp, 3, 1> &jerk) {
    return Matrix<fp, 3, 1>(2 * jerk[2], -4 * jerk[1], -10 * jerk[0]);
}

#define JERK_DEPENDENCE 0
#define ACCEL_MAGNO_ERROR 0

static void
accel_test(const Matrix<fp, 3, 1> &angle_increment, const Matrix<fp, 3, 1> &constant_accel, bool plot = false,
           int NUM_TESTS = 10000, float jerk_mult = 0) {
    state_estimate_t estimate;
    state_estimate_debug_t debug;

    Matrix<fp, 3, 1> unrotated_magno = Matrix<fp, 3, 1>(magno_reference[0], magno_reference[1], magno_reference[2]);
    Matrix<fp, 3, 1> unrotated_accel = Matrix<fp, 3, 1>(accel_reference[0], accel_reference[1], accel_reference[2]);

    std::default_random_engine generator(3452456);

    std::normal_distribution<fp> accel_distribution(0.0, kalman_accelerometer_cov);
    std::normal_distribution<fp> magno_distribution(0.0, kalman_magno_cov);
    std::normal_distribution<fp> gyro_distribution(0, kalman_gyro_cov);
    std::normal_distribution<fp> jerk_distribution(0, 1e-6f);

    Quaternion<fp> quat(-0.66327167417264843f, -0.34436319883409405f, 0.039508389760580714f, -0.66326748802235758f);
    //Quaternion<fp> quat(1,0,0,0);

    kalman_test_setup(quat.inverse(), time_increment * angle_increment, Matrix<fp, 3, 1>::Zero(),
                      Matrix<fp, 3, 1>::Zero(),
                      constant_accel);


    std::vector<float> ang_vel_err_x;
    std::vector<float> ang_vel_err_y;
    std::vector<float> ang_vel_err_z;

    std::vector<float> accel_err_x;
    std::vector<float> accel_err_y;
    std::vector<float> accel_err_z;

    std::vector<float> vel_err_x;
    std::vector<float> vel_err_y;
    std::vector<float> vel_err_z;

    std::vector<float> pos_err_x;
    std::vector<float> pos_err_y;
    std::vector<float> pos_err_z;

    std::vector<float> rot_err;
    std::vector<float> rot_axis_x;
    std::vector<float> rot_axis_y;
    std::vector<float> rot_axis_z;

    std::vector<float> accel_x;
    std::vector<float> accel_y;
    std::vector<float> accel_z;

    std::vector<float> gyro_x;
    std::vector<float> gyro_y;
    std::vector<float> gyro_z;

    std::vector<float> P_vector[KALMAN_NUM_STATES];

    std::vector<float> timestamps;

    Eigen::Matrix<fp, 3, 1> variable_accel = constant_accel;
    Eigen::Matrix<fp, 3, 1> current_velocity = Eigen::Matrix<fp, 3, 1>::Zero();
    Eigen::Matrix<fp, 3, 1> current_position = Eigen::Matrix<fp, 3, 1>::Zero();

    Quaternion<fp> delta(AngleAxis<fp>(angle_increment.norm(), angle_increment.normalized()));


    Matrix<fp, 3, 1> accel_magno_error_axis = unrotated_magno.normalized().cross(unrotated_accel.normalized());

    for (int i = 0; i < NUM_TESTS; i++) {
        Quaternion<fp> t = quat * delta.inverse();
        quat = t;


        Matrix<fp, 3, 1> jerk = Matrix<fp, 3, 1>(
                sin((i / time_increment * 20 / 60) * 6.28318530718) * 4e-4f + jerk_distribution(generator),
                sin((i / time_increment * 20 / 60 + 1.25f) * 6.28318530718) * 4e-4f + jerk_distribution(generator),
                sin((i / time_increment * 20 / 60 + 2.5f) * 6.28318530718) * 4e-4f + jerk_distribution(generator));

        variable_accel += jerk;


        Matrix<fp, 3, 1> rotated_accel = quat * (unrotated_accel + variable_accel);

#if ACCEL_MAGNO_ERROR
        fp accel_magno_error_angle =  sin((i / time_increment * 5 / 60) * 6.28318530718) * 0.1f;
        AngleAxis<fp> accel_magno_error_quat(accel_magno_error_angle, accel_magno_error_axis);
        Matrix<fp, 3, 1> rotated_magno = accel_magno_error_quat * quat * unrotated_magno;
#else
        Matrix<fp, 3, 1> rotated_magno =  quat * unrotated_magno;
#endif

#if JERK_DEPENDENCE
        Matrix<fp, 3, 1> rotated_gyro = quat * (time_increment * angle_increment + gyro_jerk_dependence(jerk));
#else
        Matrix<fp, 3, 1> rotated_gyro = quat * (time_increment * angle_increment);
#endif

        fp magno[3] = {rotated_magno.x() + magno_distribution(generator),
                       rotated_magno.y() + magno_distribution(generator),
                       rotated_magno.z() + magno_distribution(generator)};
        fp accel[3] = {rotated_accel.x() + accel_distribution(generator),
                       rotated_accel.y() + accel_distribution(generator),
                       rotated_accel.z() + accel_distribution(generator)};
        fp gyro[3] = {rotated_gyro.x() + gyro_distribution(generator),
                      rotated_gyro.y() + gyro_distribution(generator),
                      rotated_gyro.z() + gyro_distribution(generator)};

        kalman_predict(1.0f / time_increment);

        kalman_new_accel(accel);
        kalman_new_magno(magno);
        kalman_new_gyro(gyro);

        kalman_get_state(&estimate);
        kalman_get_state_debug(&debug);
        testEstimateStable(estimate);

#if SPALAX_TEST_PLOTS
        if (plot) {

            gyro_x.push_back(gyro[0]);
            gyro_y.push_back(gyro[1]);
            gyro_z.push_back(gyro[2]);

            current_position += 1.0 / time_increment * current_velocity;
            current_velocity += 1.0 / time_increment * (constant_accel + variable_accel);
            timestamps.push_back(i / time_increment);

            ang_vel_err_x.push_back(estimate.angular_velocity[0] - angle_increment[0] * time_increment);
            ang_vel_err_y.push_back(estimate.angular_velocity[1] - angle_increment[1] * time_increment);
            ang_vel_err_z.push_back(estimate.angular_velocity[2] - angle_increment[2] * time_increment);

            accel_err_x.push_back(estimate.acceleration[0] - constant_accel[0] - variable_accel[0]);
            accel_err_y.push_back(estimate.acceleration[1] - constant_accel[1] - variable_accel[1]);
            accel_err_z.push_back(estimate.acceleration[2] - constant_accel[2] - variable_accel[2]);

            vel_err_x.push_back(estimate.velocity[0] - current_velocity[0]);
            vel_err_y.push_back(estimate.velocity[1] - current_velocity[1]);
            vel_err_z.push_back(estimate.velocity[2] - current_velocity[2]);

            pos_err_x.push_back(estimate.position[0] - current_position[0]);
            pos_err_y.push_back(estimate.position[1] - current_position[1]);
            pos_err_z.push_back(estimate.position[2] - current_position[2]);

            accel_x.push_back(variable_accel[0]);
            accel_y.push_back(variable_accel[1]);
            accel_z.push_back(variable_accel[2]);


            Quaternion<fp> out(estimate.orientation_q[3], estimate.orientation_q[0], estimate.orientation_q[1],
                               estimate.orientation_q[2]);

            AngleAxis<fp> angleAxis = AngleAxis<fp>(quat * out);
            rot_axis_x.push_back((fp) angleAxis.axis().x());
            rot_axis_y.push_back((fp) angleAxis.axis().y());
            rot_axis_z.push_back((fp) angleAxis.axis().z());
            rot_err.push_back((float) angleAxis.angle());

            for (int j = 0; j < KALMAN_NUM_STATES; j++) {
                P_vector[j].push_back(debug.P[j]);
            }
        }
#endif
    }

#if SPALAX_TEST_PLOTS
    if (plot) {
        plt::figure();

        plt::named_plot("X Acceleration Error (m/s)", timestamps, accel_err_x);
        plt::named_plot("Y Acceleration Error (m/s)", timestamps, accel_err_y);
        plt::named_plot("Z Acceleration Error (m/s)", timestamps, accel_err_z);

        plt::grid(true);
        plt::legend();
        plt::save("Acceleration.png");

        plt::clf();
        plt::named_plot("X Angular Velocity Error (rad/s)", timestamps, ang_vel_err_x);
        plt::named_plot("Y Angular Velocity Error (rad/s)", timestamps, ang_vel_err_y);
        plt::named_plot("Z Angular Velocity Error (rad/s)", timestamps, ang_vel_err_z);

        plt::grid(true);
        plt::legend();
        plt::save("Angular Velocity.png");

        plt::clf();
        plt::named_plot("Rotation Error (rad)", timestamps, rot_err);

        plt::grid(true);
        plt::legend();
        plt::save("Rotation.png");

        plt::clf();
        plt::named_plot("Actual Acceleration X (m/s^2)", timestamps, accel_x);
        plt::named_plot("Actual Acceleration Y (m/s^2)", timestamps, accel_y);
        plt::named_plot("Actual Acceleration Z (m/s^2)", timestamps, accel_z);

        plt::grid(true);
        plt::legend();
        plt::save("Acceleration - Actual.png");


        plt::clf();
        plt::named_plot("Gyro X (rad/s)", timestamps, gyro_x);
        plt::named_plot("Gyro Y (rad/s)", timestamps, gyro_y);
        plt::named_plot("Gyro Z (rad/s)", timestamps, gyro_z);

        plt::grid(true);
        plt::legend();
        plt::save("Gyro - Actual.png");

        plt::clf();
        plt::named_plot("Rotation Axis X", timestamps, rot_axis_x);
        plt::named_plot("Rotation Axis Y", timestamps, rot_axis_y);
        plt::named_plot("Rotation Axis Z", timestamps, rot_axis_z);

        plt::grid(true);
        plt::legend();
        plt::save("Rotation Axis.png");

        plt::clf();
        plt::named_plot("X Velocity Error (m/s)", timestamps, vel_err_x);
        plt::named_plot("Y Velocity Error (m/s)", timestamps, vel_err_y);
        plt::named_plot("Z Velocity Error (m/s)", timestamps, vel_err_z);

        plt::grid(true);
        plt::legend();
        plt::save("Velocity.png");

        plt::clf();
        plt::named_plot("X Position Error (m)", timestamps, pos_err_x);
        plt::named_plot("Y Position Error (m)", timestamps, pos_err_y);
        plt::named_plot("Z Position Error (m)", timestamps, pos_err_z);

        plt::grid(true);
        plt::legend();
        plt::save("Position.png");


//        for (int i = 0; i < KALMAN_NUM_STATES; i++) {
//            std::string name = "P";
//            name += std::to_string(i);
//
//            plt::clf();
//            plt::named_plot(name, timestamps, P_vector[i]);
//
//            plt::grid(true);
//            plt::legend();
//            plt::save(name + ".png");
//        }

    }
#endif

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
    fp angle = acos(clampf(magno_test.normalized().transpose() * unrotated_magno.normalized(), -1, 1)) * 180.0f /
               (fp) M_PI;
    fp angle2 = acos(clampf(accel_test.normalized().transpose() * unrotated_accel.normalized(), -1, 1)) * 180.0f /
                (fp) M_PI;

    // We expect accuracy to a degree - any less is unreasonable with 32-bit fping point
    EXPECT_LT(angle, 1.f);
    EXPECT_LT(angle2, 1.f);

    float time = NUM_TESTS / time_increment;

    for (int i = 0; i < 3; i++) {
        expect_fuzzy_eq(estimate.angular_velocity[i], angle_increment[i] * time_increment, 0.00005f, 0.05);
    }
}

TEST(TestKalmanAccel, TestSimple) {
    accel_test(Matrix<fp, 3, 1>::Zero(), Matrix<fp, 3, 1>(1, 0, 0));
}

TEST(TestKalmanAccel, TestComplex) {
    accel_test(Matrix<fp, 3, 1>(1.0f / time_increment, 0, 0), Matrix<fp, 3, 1>(1, 0, 0));

    accel_test(Matrix<fp, 3, 1>(1.0f / time_increment, 0, 0), Matrix<fp, 3, 1>(0, 1, 0));

    accel_test(Matrix<fp, 3, 1>(1.0f / time_increment, 0, 0), Matrix<fp, 3, 1>(0, 0, 1));
}

TEST(TestKalmanAccel, TestPlot) {
    accel_test(Matrix<fp, 3, 1>(1 / time_increment, 1 / time_increment, 0 / time_increment), Matrix<fp, 3, 1>(0, 0, 0),
               true, 1 * 60 * time_increment, 2e-4f);
}