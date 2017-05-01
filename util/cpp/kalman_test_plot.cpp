#include <random>
#include <matplotlibcpp.h>
#include "state/kalman.h"
#include "Eigen/Core"
#include <Eigen/Geometry>


#define JERK_DEPENDENCE 0
#define ACCEL_MAGNO_ERROR 0
#define GYRO_SF_ERROR 0

static constexpr float time_increment = 1024;
static const int num_experiments = 50;
static const int simulation_minutes = 5;

using namespace Eigen;
namespace plt = matplotlibcpp;

static Matrix<fp, 3, 1> gyro_jerk_dependence(const Matrix<fp, 3, 1> &jerk) {
    return Matrix<fp, 3, 1>(2 * jerk[2], -4 * jerk[1], -10 * jerk[0]);
}

fp accel_reference[3] = {0, 0, 9.80665f};
fp magno_reference[3] = {0.70710678118f, 0.70710678118f, 0};


static void kalman_test_setup(Eigen::Quaternion<fp> quat, Eigen::Matrix<fp, 3, 1> angular_velocity,
                              Eigen::Matrix<fp, 3, 1> position,
                              Eigen::Matrix<fp, 3, 1> velocity, Eigen::Matrix<fp, 3, 1> acceleration) {
    fp quat_arr[4] = {quat.x(), quat.y(), quat.z(), quat.w()};
    fp ang_vel[3] = {angular_velocity.x(), angular_velocity.y(), angular_velocity.z()};
    fp pos[3] = {position.x(), position.y(), position.z()};
    fp vel[3] = {velocity.x(), velocity.y(), velocity.z()};
    fp accel[3] = {acceleration.x(), acceleration.y(), acceleration.z()};
    fp initial_gyro_bias[3] = {0, 0, 0};
    fp initial_accel_bias[3] = {0, 0, 0};
    fp initial_magno_bias[3] = {0, 0, 0};

    kalman_init(accel_reference, magno_reference, quat_arr, ang_vel, pos, vel, accel, initial_gyro_bias,
                initial_accel_bias, initial_magno_bias);
}

static std::vector<float> average(std::vector<float> input[num_experiments]) {
    size_t size = input[0].size();
    for (int i = 1; i < num_experiments; i++)
        assert(input[i].size() == size);

    std::vector<float> ret;
    ret.reserve(size);

    for (int i = 0; i < size; i++) {
        float sum = 0;
        for (int j = 0; j < num_experiments; j++)
            sum += input[j][i];
        ret.push_back(sum / num_experiments);
    }
    return ret;
}

static void
accel_test(const Matrix<fp, 3, 1> &angle_increment, const Matrix<fp, 3, 1> &constant_accel, bool plot = false,
           int NUM_TESTS = 10000, float jerk_mult = 0) {
    state_estimate_t estimate;
    state_estimate_debug_t debug;

    const Matrix<fp, 3, 1> unrotated_magno = Matrix<fp, 3, 1>(magno_reference[0], magno_reference[1],
                                                              magno_reference[2]);
    const Matrix<fp, 3, 1> unrotated_accel = Matrix<fp, 3, 1>(accel_reference[0], accel_reference[1],
                                                              accel_reference[2]);

    std::default_random_engine generator(3452456);

    std::normal_distribution<fp> accel_distribution(0.0, kalman_accelerometer_cov);
    std::normal_distribution<fp> magno_distribution(0.0, kalman_magno_cov);
    std::normal_distribution<fp> gyro_distribution(0, kalman_gyro_cov);
    std::normal_distribution<fp> jerk_distribution(0, 1e-6f);


    std::vector<float> ang_vel_err_x[num_experiments];
    std::vector<float> ang_vel_err_y[num_experiments];
    std::vector<float> ang_vel_err_z[num_experiments];

    std::vector<float> accel_err_x[num_experiments];
    std::vector<float> accel_err_y[num_experiments];
    std::vector<float> accel_err_z[num_experiments];

    std::vector<float> vel_err_x[num_experiments];
    std::vector<float> vel_err_y[num_experiments];
    std::vector<float> vel_err_z[num_experiments];

    std::vector<float> pos_err_x[num_experiments];
    std::vector<float> pos_err_y[num_experiments];
    std::vector<float> pos_err_z[num_experiments];

    std::vector<float> rot_err[num_experiments];

    std::vector<float> timestamps;


    for (int k = 0; k < num_experiments; k++) {
        printf("Simulation %i\n", k);

        Quaternion<fp> quat(-0.66327167417264843f, -0.34436319883409405f, 0.039508389760580714f, -0.66326748802235758f);

        kalman_test_setup(quat.inverse(), time_increment * angle_increment, Matrix<fp, 3, 1>::Zero(),
                          Matrix<fp, 3, 1>::Zero(),
                          constant_accel);

        Eigen::Matrix<fp, 3, 1> variable_accel = constant_accel;
        Eigen::Matrix<fp, 3, 1> current_velocity = Eigen::Matrix<fp, 3, 1>::Zero();
        Eigen::Matrix<fp, 3, 1> current_position = Eigen::Matrix<fp, 3, 1>::Zero();

        Quaternion<fp> delta(AngleAxis<fp>(angle_increment.norm(), angle_increment.normalized()));


        Matrix<fp, 3, 1> accel_magno_error_axis = unrotated_magno.normalized().cross(unrotated_accel.normalized());

        for (int i = 0; i < NUM_TESTS; i++) {
            Quaternion<fp> t = quat * delta.inverse();
            quat = t;


            Matrix<fp, 3, 1> jerk = Matrix<fp, 3, 1>(
                    sin((i / time_increment * 20 / 60) * 6.28318530718) * 8e-4f + jerk_distribution(generator),
                    sin((i / time_increment * 20 / 60 + 1.25f) * 6.28318530718) * 8e-4f + jerk_distribution(generator),
                    sin((i / time_increment * 20 / 60 + 2.5f) * 6.28318530718) * 8e-4f + jerk_distribution(generator));

            variable_accel += jerk;


            Matrix<fp, 3, 1> rotated_accel = quat * (unrotated_accel + variable_accel);

#if ACCEL_MAGNO_ERROR
            fp accel_magno_error_angle =  sin((i / time_increment * 5 / 60) * 6.28318530718) * 0.1f;
        AngleAxis<fp> accel_magno_error_quat(accel_magno_error_angle, accel_magno_error_axis);
        Matrix<fp, 3, 1> rotated_magno = accel_magno_error_quat * quat * unrotated_magno;
#else
            Matrix<fp, 3, 1> rotated_magno = quat * unrotated_magno;
#endif

#if JERK_DEPENDENCE
            Matrix<fp, 3, 1> rotated_gyro = quat * (time_increment * angle_increment + gyro_jerk_dependence(jerk));
#else
            Matrix<fp, 3, 1> rotated_gyro = quat * (time_increment * angle_increment);
#endif

#if GYRO_SF_ERROR
            rotated_gyro = 1.00001f * rotated_gyro;
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



            current_position += 1.0 / time_increment * current_velocity;
            current_velocity += 1.0 / time_increment * (constant_accel + variable_accel);


            if (k == 0)
                timestamps.push_back(i / time_increment);

            ang_vel_err_x[k].push_back(estimate.angular_velocity[0] - angle_increment[0] * time_increment);
            ang_vel_err_y[k].push_back(estimate.angular_velocity[1] - angle_increment[1] * time_increment);
            ang_vel_err_z[k].push_back(estimate.angular_velocity[2] - angle_increment[2] * time_increment);

            accel_err_x[k].push_back(estimate.acceleration[0] - constant_accel[0] - variable_accel[0]);
            accel_err_y[k].push_back(estimate.acceleration[1] - constant_accel[1] - variable_accel[1]);
            accel_err_z[k].push_back(estimate.acceleration[2] - constant_accel[2] - variable_accel[2]);

            vel_err_x[k].push_back(estimate.velocity[0] - current_velocity[0]);
            vel_err_y[k].push_back(estimate.velocity[1] - current_velocity[1]);
            vel_err_z[k].push_back(estimate.velocity[2] - current_velocity[2]);

            pos_err_x[k].push_back(estimate.position[0] - current_position[0]);
            pos_err_y[k].push_back(estimate.position[1] - current_position[1]);
            pos_err_z[k].push_back(estimate.position[2] - current_position[2]);

            Quaternion<fp> out(estimate.orientation_q[3], estimate.orientation_q[0], estimate.orientation_q[1],
                               estimate.orientation_q[2]);

            AngleAxis<fp> angleAxis = AngleAxis<fp>(quat * out);
            rot_err[k].push_back((float) angleAxis.angle());
        }
    }

    printf("Plotting Graphs\n");

    plt::figure();

    auto t = average(accel_err_x);

    plt::named_plot("X Acceleration Error (m/s)", timestamps, t);
    plt::named_plot("Y Acceleration Error (m/s)", timestamps, average(accel_err_y));
    plt::named_plot("Z Acceleration Error (m/s)", timestamps, average(accel_err_z));

    plt::grid(true);
    plt::legend();
    plt::save("Acceleration.png");

    plt::clf();
    plt::named_plot("X Angular Velocity Error (rad/s)", timestamps, average(ang_vel_err_x));
    plt::named_plot("Y Angular Velocity Error (rad/s)", timestamps, average(ang_vel_err_y));
    plt::named_plot("Z Angular Velocity Error (rad/s)", timestamps, average(ang_vel_err_z));

    plt::grid(true);
    plt::legend();
    plt::save("Angular Velocity.png");

    plt::clf();
    plt::named_plot("Rotation Error (rad)", timestamps, average(rot_err));

    plt::grid(true);
    plt::legend();
    plt::save("Rotation.png");

    plt::clf();
    plt::named_plot("X Velocity Error (m/s)", timestamps, average(vel_err_x));
    plt::named_plot("Y Velocity Error (m/s)", timestamps, average(vel_err_y));
    plt::named_plot("Z Velocity Error (m/s)", timestamps, average(vel_err_z));

    plt::grid(true);
    plt::legend();
    plt::save("Velocity.png");

    plt::clf();
    plt::named_plot("X Position Error (m)", timestamps, average(pos_err_x));
    plt::named_plot("Y Position Error (m)", timestamps, average(pos_err_y));
    plt::named_plot("Z Position Error (m)", timestamps, average(pos_err_z));

    plt::grid(true);
    plt::legend();
    plt::save("Position.png");


//        for (int i = 0; i < KALMAN_NUM_STATES; i++) {
//            std::string name = "P";
//            name += std::to_string(i);
//
//            plt::clf();
//            plt::named_plot(name, timestamps, average(P_vector[i]));
//
//            plt::grid(true);
//            plt::legend();
//            plt::save(name + ".png");
//        }
}


int main(int argc, char *argv[]) {
    accel_test(Matrix<fp, 3, 1>(1 / time_increment, 1 / time_increment, 0 / time_increment), Matrix<fp, 3, 1>(0, 0, 0),
               true, simulation_minutes * 60 * time_increment, 2e-4f);
}