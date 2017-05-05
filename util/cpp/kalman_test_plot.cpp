#include <random>
#include <matplotlibcpp.h>
#include "state/kalman.h"
#include "Eigen/Core"
#include <Eigen/Geometry>


#define JERK_DEPENDENCE 0
#define ACCEL_MAGNO_ERROR 0
#define GYRO_SF_ERROR 1
#define ANGULAR_ACCELERATION 0
#define JERK 0

static constexpr float time_increment = 1024;
static const int num_experiments = 50;
static const int simulation_minutes = 5;

using namespace Eigen;
namespace plt = matplotlibcpp;

static Matrix<fp, 3, 1> gyro_jerk_dependence(const Matrix<fp, 3, 1> &jerk) {
    return Matrix<fp, 3, 1>(2 * jerk[2], -4 * jerk[1], -10 * jerk[0]);
}

fp accel_reference[3] = {0, 0, 9.80665f};
fp magno_reference[3] = {1, 0, 0};//{0.39134267f, -0.00455851434f, -0.920233727f};


static void plot_error_bars(std::vector<float> vector, std::vector<float> pVector[], const std::string &fmt);

static float get_rand(float range = 20) {
    float divisor = RAND_MAX / 2 / range;
    return (float) (rand() % RAND_MAX) / divisor - range;
}

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

static void plot_error_bars(std::vector<float> timestamps, std::vector<float> input[num_experiments], const std::string &color) {
    float t = 0;
    size_t i = 0;
    std::vector<float> plot_timestamps;
    std::vector<float> plot_values;
    std::vector<float> plot_error_bars;

    while (i < timestamps.size()) {
        if (timestamps[i] >= t) {
            plot_timestamps.push_back(timestamps[i]);
            float sum = 0;
            float sum_sq = 0;

            for (int j = 0; j < num_experiments; j++) {
                sum += input[j][i];
                sum_sq += input[j][i] * input[j][i];
            }

            float avg = sum / num_experiments;
            float var = sum_sq / num_experiments - avg * avg;

            plot_values.push_back(avg);
            plot_error_bars.push_back(sqrtf(var));

            t += 10;
        }
        i++;
    }

    plt::errorbar(plot_timestamps, plot_values, plot_error_bars, "none", color);

}

static void
accel_test(const Matrix<fp, 3, 1> &const_angle_increment, const Matrix<fp, 3, 1> &constant_accel, bool plot = false,
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

    std::normal_distribution<fp> gyro_bias_distribution(0, gyro_bias_process_noise / time_increment);
    std::normal_distribution<fp> magno_bias_distribution(0, magno_bias_process_noise / time_increment);
    std::normal_distribution<fp> accel_bias_distribution(0, accel_bias_process_noise / time_increment);

    std::normal_distribution<fp> jerk_distribution(0, acceleration_process_noise / time_increment);
    std::normal_distribution<fp> angular_accel_distribution(0, angular_vel_process_noise / time_increment);

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

    std::vector<float> gyro_bias_err_x[num_experiments];
    std::vector<float> gyro_bias_err_y[num_experiments];
    std::vector<float> gyro_bias_err_z[num_experiments];

    std::vector<float> accel_bias_err_x[num_experiments];
    std::vector<float> accel_bias_err_y[num_experiments];
    std::vector<float> accel_bias_err_z[num_experiments];

    std::vector<float> rot_err[num_experiments];
    std::vector<float> rot_axis_x[num_experiments];
    std::vector<float> rot_axis_y[num_experiments];
    std::vector<float> rot_axis_z[num_experiments];

    std::vector<float> accel_x;
    std::vector<float> accel_y;
    std::vector<float> accel_z;

    std::vector<float> gyro_x;
    std::vector<float> gyro_y;
    std::vector<float> gyro_z;

    std::vector<float> timestamps;

    fp accel_magno_error_angle = get_rand(0.1f);


    for (int k = 0; k < num_experiments; k++) {
        printf("Simulation %i\n", k);

        Eigen::Quaternion<fp> quat(Eigen::AngleAxisf(get_rand(), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(get_rand(), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(get_rand(), Eigen::Vector3f::UnitX()));

        kalman_test_setup(quat.inverse(), const_angle_increment, Matrix<fp, 3, 1>::Zero(),
                          Matrix<fp, 3, 1>::Zero(),
                          constant_accel);

        Eigen::Matrix<fp, 3, 1> variable_accel = constant_accel;
        Eigen::Matrix<fp, 3, 1> current_velocity = Eigen::Matrix<fp, 3, 1>::Zero();
        Eigen::Matrix<fp, 3, 1> current_position = Eigen::Matrix<fp, 3, 1>::Zero();

        Eigen::Matrix<fp, 3, 1> gyro_bias = Eigen::Matrix<fp, 3, 1>::Zero();
        Eigen::Matrix<fp, 3, 1> magno_bias = Eigen::Matrix<fp, 3, 1>::Zero();
        Eigen::Matrix<fp, 3, 1> accel_bias = Eigen::Matrix<fp, 3, 1>::Zero();

        Eigen::Matrix<fp, 3, 1> angle_increment = const_angle_increment;

        Matrix<fp, 3, 1> accel_magno_error_axis = unrotated_magno.normalized().cross(unrotated_accel.normalized());

        for (int i = 0; i <= NUM_TESTS; i++) {

            gyro_bias += Eigen::Matrix<fp, 3, 1>(gyro_bias_distribution(generator), gyro_bias_distribution(generator), gyro_bias_distribution(generator));
            accel_bias += Eigen::Matrix<fp, 3, 1>(accel_bias_distribution(generator), accel_bias_distribution(generator), accel_bias_distribution(generator));

            Quaternion<fp> delta(AngleAxis<fp>(angle_increment.norm() / time_increment, angle_increment.normalized()));
            Quaternion<fp> t = quat * delta.inverse();
            quat = t;

#if ANGULAR_ACCELERATION
            angle_increment += Eigen::Matrix<fp, 3, 1>(angular_accel_distribution(generator), angular_accel_distribution(generator), angular_accel_distribution(generator));
#endif
#if JERK
            Matrix<fp, 3, 1> jerk = Matrix<fp, 3, 1>(
                    jerk_distribution(generator),
                    jerk_distribution(generator),
                    jerk_distribution(generator));

            variable_accel += jerk;
#endif

            Matrix<fp, 3, 1> rotated_accel = quat * (unrotated_accel + variable_accel) + accel_bias;

#if ACCEL_MAGNO_ERROR
        AngleAxis<fp> accel_magno_error_quat(accel_magno_error_angle, accel_magno_error_axis);
        Matrix<fp, 3, 1> rotated_magno = accel_magno_error_quat * quat * unrotated_magno + magno_bias;
#else
            Matrix<fp, 3, 1> rotated_magno = quat * unrotated_magno + magno_bias;
#endif

#if JERK_DEPENDENCE
            Matrix<fp, 3, 1> rotated_gyro = quat * (angle_increment + gyro_jerk_dependence(jerk)) + gyro_bias;
#elif GYRO_SF_ERROR
            Matrix<fp, 3, 1> rotated_gyro = 1.001f * (quat * (angle_increment)) + gyro_bias;
#else
            Matrix<fp, 3, 1> rotated_gyro = quat * (angle_increment) + gyro_bias;
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

            kalman_new_gyro(gyro);
            kalman_new_accel(accel);
            kalman_new_magno(magno);


            kalman_get_state(&estimate);
            kalman_get_state_debug(&debug);



            current_position += 1.0 / time_increment * current_velocity;
            current_velocity += 1.0 / time_increment * variable_accel;


            if (k == 0) {
                timestamps.push_back(i / time_increment);
                accel_x.push_back(accel[0]);
                accel_y.push_back(accel[1]);
                accel_z.push_back(accel[2]);

                gyro_x.push_back(gyro[0]);
                gyro_y.push_back(gyro[1]);
                gyro_z.push_back(gyro[2]);
            }

            ang_vel_err_x[k].push_back(abs(estimate.angular_velocity[0] - angle_increment[0]));
            ang_vel_err_y[k].push_back(abs(estimate.angular_velocity[1] - angle_increment[1]));
            ang_vel_err_z[k].push_back(abs(estimate.angular_velocity[2] - angle_increment[2]));

            accel_err_x[k].push_back(abs(estimate.acceleration[0] - variable_accel[0]));
            accel_err_y[k].push_back(abs(estimate.acceleration[1] - variable_accel[1]));
            accel_err_z[k].push_back(abs(estimate.acceleration[2] - variable_accel[2]));

            vel_err_x[k].push_back(abs(estimate.velocity[0] - current_velocity[0]));
            vel_err_y[k].push_back(abs(estimate.velocity[1] - current_velocity[1]));
            vel_err_z[k].push_back(abs(estimate.velocity[2] - current_velocity[2]));

            pos_err_x[k].push_back(abs(estimate.position[0] - current_position[0]));
            pos_err_y[k].push_back(abs(estimate.position[1] - current_position[1]));
            pos_err_z[k].push_back(abs(estimate.position[2] - current_position[2]));

            gyro_bias_err_x[k].push_back(abs(debug.gyro_bias[0] - gyro_bias[0]));
            gyro_bias_err_y[k].push_back(abs(debug.gyro_bias[1] - gyro_bias[1]));
            gyro_bias_err_z[k].push_back(abs(debug.gyro_bias[2] - gyro_bias[2]));

            accel_bias_err_x[k].push_back(abs(debug.accel_bias[0] - accel_bias[0]));
            accel_bias_err_y[k].push_back(abs(debug.accel_bias[1] - accel_bias[1]));
            accel_bias_err_z[k].push_back(abs(debug.accel_bias[2] - accel_bias[2]));

            Quaternion<fp> out(estimate.orientation_q[3], estimate.orientation_q[0], estimate.orientation_q[1],
                               estimate.orientation_q[2]);

            AngleAxis<fp> angleAxis = AngleAxis<fp>(quat * out);
            //printf("%f %f %f\n", angleAxis.axis().x(), angleAxis.axis().y(), angleAxis.axis().z());

            rot_axis_x[k].push_back((float) angleAxis.axis().x());
            rot_axis_y[k].push_back((float) angleAxis.axis().y());
            rot_axis_z[k].push_back((float) angleAxis.axis().z());
            rot_err[k].push_back((float) angleAxis.angle());
        }
    }

    printf("Plotting Graphs\n");

    plt::figure();

    auto t = average(accel_err_x);

    plt::named_plot("X Acceleration Error (m/s$^2$)", timestamps, t);
    plt::named_plot("Y Acceleration Error (m/s$^2$)", timestamps, average(accel_err_y));
    plt::named_plot("Z Acceleration Error (m/s$^2$)", timestamps, average(accel_err_z));

    plt::grid(true);
    plt::legend();
    plt::xlabel("Time (s)");
    plt::save("Acceleration.png");

    plt::clf();
    plt::named_plot("X Angular Velocity Error (rad/s)", timestamps, average(ang_vel_err_x));
    plt::named_plot("Y Angular Velocity Error (rad/s)", timestamps, average(ang_vel_err_y));
    plt::named_plot("Z Angular Velocity Error (rad/s)", timestamps, average(ang_vel_err_z));

    plt::grid(true);
    plt::legend();
    plt::xlabel("Time (s)");
    plt::save("Angular Velocity.png");

    plt::clf();
    plt::named_plot("Rotation Error (rad)", timestamps, average(rot_err));

    plt::grid(true);
    plt::legend();
    plt::xlabel("Time (s)");
    plt::save("Rotation.png");

    plt::clf();
    plt::named_plot("X Velocity Error (m/s)", timestamps, average(vel_err_x));
    plt::named_plot("Y Velocity Error (m/s)", timestamps, average(vel_err_y));
    plt::named_plot("Z Velocity Error (m/s)", timestamps, average(vel_err_z));

    plt::grid(true);
    plt::legend();
    plt::xlabel("Time (s)");
    plt::save("Velocity.png");

    plt::clf();
    plt::named_plot("X Position Error (m)", timestamps, average(pos_err_x));
    plot_error_bars(timestamps, pos_err_x, "C0");
    plt::named_plot("Y Position Error (m)", timestamps, average(pos_err_y));
    plot_error_bars(timestamps, pos_err_y, "C1");
    plt::named_plot("Z Position Error (m)", timestamps, average(pos_err_z));
    plot_error_bars(timestamps, pos_err_z, "C2");

    plt::grid(true);
    plt::legend("upper left");
    plt::xlabel("Time (s)");
    plt::save("Position.png");

    plt::clf();
    plt::named_plot("Accel X (m/s$^2$)", timestamps, accel_x);
    plt::named_plot("Accel Y (m/s$^2$)", timestamps, accel_y);
    plt::named_plot("Accel Z (m/s$^2$)", timestamps, accel_z);

    plt::grid(true);
    plt::legend();
    plt::xlabel("Time (s)");
    plt::save("Accel.png");


    plt::clf();
    plt::named_plot("Gyro X (rad/s)", timestamps, gyro_x);
    plt::named_plot("Gyro Y (rad/s)", timestamps, gyro_y);
    plt::named_plot("Gyro Z (rad/s)", timestamps, gyro_z);

    plt::grid(true);
    plt::legend();
    plt::xlabel("Time (s)");
    plt::save("Gyro.png");

    plt::clf();
    plt::named_plot("Gyro Bias Error X (rad/s)", timestamps, average(gyro_bias_err_x));
    plt::named_plot("Gyro Bias Error Y (rad/s)", timestamps, average(gyro_bias_err_y));
    plt::named_plot("Gyro Bias Error Z (rad/s)", timestamps, average(gyro_bias_err_z));

    plt::grid(true);
    plt::legend();
    plt::xlabel("Time (s)");
    plt::save("Gyro Bias.png");

    plt::clf();
    plt::named_plot("Accel Bias Error X (m/s$^2$)", timestamps, average(accel_bias_err_x));
    plt::named_plot("Accel Bias Error Y (m/s$^2$)", timestamps, average(accel_bias_err_y));
    plt::named_plot("Accel Bias Error Z (m/s$^2$)", timestamps, average(accel_bias_err_z));

    plt::grid(true);
    plt::legend();
    plt::xlabel("Time (s)");
    plt::save("Accel Bias.png");

    plt::clf();
    plt::named_plot("Rot Error Axis X", timestamps, average(rot_axis_x));
    plt::named_plot("Rot Error Axis Y", timestamps, average(rot_axis_y));
    plt::named_plot("Rot Error Axis Z", timestamps, average(rot_axis_z));

    plt::grid(true);
    plt::legend();
    plt::xlabel("Time (s)");
    plt::save("Rotation Axis.png");



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
    accel_test(Matrix<fp, 3, 1>(1, 1, 1), Matrix<fp, 3, 1>(0, 0, 0),
               true, (int) ((simulation_minutes * 60 + 5) * time_increment), 2e-4f);
}