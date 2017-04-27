#include <matplotlibcpp.h>
#include <sstream>
#include <algorithm>
#include "data_extractor.h"
#include "data_plotter.h"


namespace plt = matplotlibcpp;

static const char options_delimeter = ':';
static const char plot_delimimeter = ';';

int enable_streams(int argc, char **argv, DataExtractor *de) {
    for (int k = 0; k < argc; k += 2) {
        std::istringstream f(argv[k + 1]);
        std::string graph_constant;

        while (getline(f, graph_constant, plot_delimimeter)) {
            auto it = graph_constant.find_first_of(options_delimeter);
            std::string graph_variable = graph_constant.substr(0, it);

            if (graph_variable == "AX")
                de->accel_x.enabled = true;
            else if (graph_variable == "AY")
                de->accel_y.enabled = true;
            else if (graph_variable == "AZ")
                de->accel_z.enabled = true;

            else if (graph_variable == "GX")
                de->gyro_x.enabled = true;
            else if (graph_variable == "GY")
                de->gyro_y.enabled = true;
            else if (graph_variable == "GZ")
                de->gyro_z.enabled = true;

            else if (graph_variable == "MX")
                de->magno_x.enabled = true;
            else if (graph_variable == "MY")
                de->magno_y.enabled = true;
            else if (graph_variable == "MZ")
                de->magno_z.enabled = true;

            else if (graph_variable == "AMANGLE")
                de->accel_magno_angle.enabled = true;
            else if (graph_variable == "MNORM")
                de->magno_norm.enabled = true;
            else if (graph_variable == "ANORM")
                de->accel_norm.enabled = true;
            else if (graph_variable == "GNORM")
                de->gyro_norm.enabled = true;

            else if (graph_variable == "ADAX")
                de->adis_accel_x.enabled = true;
            else if (graph_variable == "ADAY")
                de->adis_accel_y.enabled = true;
            else if (graph_variable == "ADAZ")
                de->adis_accel_z.enabled = true;

            else if (graph_variable == "ADGX")
                de->adis_gyro_x.enabled = true;
            else if (graph_variable == "ADGY")
                de->adis_gyro_y.enabled = true;
            else if (graph_variable == "ADGZ")
                de->adis_gyro_z.enabled = true;

            else if (graph_variable == "ADMX")
                de->adis_magno_x.enabled = true;
            else if (graph_variable == "ADMY")
                de->adis_magno_y.enabled = true;
            else if (graph_variable == "ADMZ")
                de->adis_magno_z.enabled = true;

            else if (graph_variable == "ADAMANGLE")
                de->adis_accel_magno_angle.enabled = true;
            else if (graph_variable == "ADMNORM")
                de->adis_magno_norm.enabled = true;
            else if (graph_variable == "ADANORM")
                de->adis_accel_norm.enabled = true;
            else if (graph_variable == "ADGNORM")
                de->adis_gyro_norm.enabled = true;

            else if (graph_variable == "SEAX")
                de->se_accel_x.enabled = true;
            else if (graph_variable == "SEAY")
                de->se_accel_y.enabled = true;
            else if (graph_variable == "SEAZ")
                de->se_accel_z.enabled = true;
            else if (graph_variable == "SEANORM")
                de->se_accel_norm.enabled = true;

            else if (graph_variable == "SEVX")
                de->se_velocity_x.enabled = true;
            else if (graph_variable == "SEVY")
                de->se_velocity_y.enabled = true;
            else if (graph_variable == "SEVZ")
                de->se_velocity_z.enabled = true;

            else if (graph_variable == "SERX")
                de->se_rotation_x.enabled = true;
            else if (graph_variable == "SERY")
                de->se_rotation_y.enabled = true;
            else if (graph_variable == "SERZ")
                de->se_rotation_z.enabled = true;

            else if (graph_variable == "SEAVX")
                de->se_ang_velocity_x.enabled = true;
            else if (graph_variable == "SEAVY")
                de->se_ang_velocity_y.enabled = true;
            else if (graph_variable == "SEAVZ")
                de->se_ang_velocity_z.enabled = true;
            else if (graph_variable == "SEAV")
                de->se_ang_vel_norm.enabled = true;

            else if (graph_variable == "SEGBX")
                de->se_gyro_bias_x.enabled = true;
            else if (graph_variable == "SEGBY")
                de->se_gyro_bias_y.enabled = true;
            else if (graph_variable == "SEGBZ")
                de->se_gyro_bias_z.enabled = true;

            else if (graph_variable == "SEABX")
                de->se_accel_bias_x.enabled = true;
            else if (graph_variable == "SEABY")
                de->se_accel_bias_y.enabled = true;
            else if (graph_variable == "SEABZ")
                de->se_accel_bias_z.enabled = true;
            else if (graph_variable == "SEABNORM")
                de->se_accel_bias_norm.enabled = true;

            else if (graph_variable == "SEMBX")
                de->se_magno_bias_x.enabled = true;
            else if (graph_variable == "SEMBY")
                de->se_magno_bias_y.enabled = true;
            else if (graph_variable == "SEMBZ")
                de->se_magno_bias_z.enabled = true;
            else if (graph_variable == "SEMBNORM")
                de->se_magno_bias_norm.enabled = true;

            else if (graph_variable == "SEANEAV")
                de->se_accel_norm_exp_avg.enabled = true;
            else if (graph_variable == "SEGNEAV")
                de->se_gyro_norm_exp_avg.enabled = true;
            else if (graph_variable == "SEMNEAV")
                de->se_magno_norm_exp_avg.enabled = true;
            else if (graph_variable == "SEANEVAR")
                de->se_accel_exp_var.enabled = true;

            else if (graph_variable[0] == 'P') {
                int num;
                if (sscanf(graph_variable.c_str(), "P%d", &num) != 1) {
                    std::cerr << "Invalid Variable Format" << graph_variable << std::endl;
                    return 1;
                }
                assert(num >= 0 && num < KALMAN_NUM_STATES);

                de->P[num].enabled = true;
            } else if (graph_variable == "AMRANGLE")
                de->accel_magno_reference_angle.enabled = true;
            else {
                std::cerr << "Unrecognised Variable " << graph_variable << std::endl;
                return 1;
            }


        }
    }
    return 0;
}

static std::pair<std::vector<float>, std::vector<float>>
average_filter(const std::vector<float> &data, const std::vector<float> &timestamps, size_t n) {
    std::vector<float> filtered_data;
    std::vector<float> filtered_timestamps;
    for (auto i = 0; i < data.size(); i += n) {
        size_t samples = std::min(n, data.size() - i);

        filtered_data.push_back(std::accumulate(data.begin() + i, data.begin() + i + samples, 0.0f) / (float) samples);
        filtered_timestamps.push_back(
                std::accumulate(timestamps.begin() + i, timestamps.begin() + i + samples, 0.0f) / (float) samples);
    }

    return std::make_pair(filtered_data, filtered_timestamps);
}

static std::pair<std::vector<float>, std::vector<float>>
offset_filter(const std::vector<float> &data, const std::vector<float> &timestamps, float n) {
    std::vector<float> filtered_data;
    for (auto entry : data) {
        filtered_data.push_back(entry - n);
    }
    return std::make_pair(filtered_data, timestamps);
}

void plot_with_options(const std::string &name, const std::vector<float> &timestamps, const std::vector<float> &data,
                       const std::string &options, bool set_ylabel) {
    if (options == "") {
        plt::named_plot(name, timestamps, data);
        plt::xlabel("Time (s)");

        if (set_ylabel)
            plt::ylabel(name);

        return;
    }

    std::string plot_name = name;
    std::vector<float> plot_data;
    std::vector<float> plot_timestamps;

    if (options.compare(0, 2, "av") == 0) {
        int num;
        if (sscanf(options.c_str(), "av%d", &num) != 1) {
            std::cerr << "Invalid Option String " << options << std::endl;
            exit(1);
        }

        assert(num > 0);

        plot_name += " - ";
        plot_name += std::to_string(num);
        plot_name += " Sample Mean";

        auto pair = average_filter(data, timestamps, (size_t)num);
        plot_data = std::move(pair.first);
        plot_timestamps = std::move(pair.second);
    } else if (options.compare(0,2, "of") == 0) {
        float num;
        if (sscanf(options.c_str(), "of%f", &num) != 1) {
            std::cerr << "Invalid Option String " << options << std::endl;
            exit(1);
        }

        assert(num > 0);

        plot_name += " - ";
        plot_name += " offset by ";
        plot_name += std::to_string(num);

        auto pair = offset_filter(data, timestamps, num);
        plot_data = std::move(pair.first);
        plot_timestamps = std::move(pair.second);
    } else {
        std::cerr << "Unrecognised Options String " << options;
    }

    plt::named_plot(plot_name, plot_timestamps, plot_data);
    plt::xlabel("Time (s)");

    if (set_ylabel)
        plt::ylabel(plot_name);
}

int plot_data(int argc, char **argv, const DataExtractor *de) {
    printf("Plotting Graphs\n");

    for (int k = 0; k < argc; k += 2) {
        const char *output = argv[k];
        std::istringstream f(argv[k + 1]);
        std::string arg = argv[k + 1];

        auto line_count = std::count(arg.begin(), arg.end(), plot_delimimeter) + 1;

        std::string graph_constant;
        while (getline(f, graph_constant, plot_delimimeter)) {
            auto it = graph_constant.find_first_of(options_delimeter);
            std::string graph_variable = graph_constant.substr(0, it);
            std::string graph_options =
                    it == std::string::npos ? "" : graph_constant.substr(it + 1, graph_constant.size() - it - 1);

            if (graph_variable == "AX")
                plot_with_options("Accelerometer X $(m/s^2)$", de->mpu_timestamps, de->accel_x.data, graph_options, line_count == 1);
            else if (graph_variable == "AY")
                plot_with_options("Accelerometer Y $(m/s^2)$", de->mpu_timestamps, de->accel_y.data, graph_options, line_count == 1);
            else if (graph_variable == "AZ")
                plot_with_options("Accelerometer Z $(m/s^2)$", de->mpu_timestamps, de->accel_z.data, graph_options, line_count == 1);

            else if (graph_variable == "GX")
                plot_with_options("Gyroscope X (rad/s)", de->mpu_timestamps, de->gyro_x.data, graph_options, line_count == 1);
            else if (graph_variable == "GY")
                plot_with_options("Gyroscope Y (rad/s)", de->mpu_timestamps, de->gyro_y.data, graph_options, line_count == 1);
            else if (graph_variable == "GZ")
                plot_with_options("Gyroscope Z (rad/s)", de->mpu_timestamps, de->gyro_z.data, graph_options, line_count == 1);

            else if (graph_variable == "MX")
                plot_with_options("Magnetometer X", de->mpu_timestamps, de->magno_x.data, graph_options, line_count == 1);
            else if (graph_variable == "MY")
                plot_with_options("Magnetometer Y", de->mpu_timestamps, de->magno_y.data, graph_options, line_count == 1);
            else if (graph_variable == "MZ")
                plot_with_options("Magnetometer Z", de->mpu_timestamps, de->magno_z.data, graph_options, line_count == 1);

            else if (graph_variable == "AMANGLE")
                plot_with_options("Field Angle (rad)", de->mpu_timestamps, de->accel_magno_angle.data, graph_options,
                                  line_count == 1);
            else if (graph_variable == "MNORM")
                plot_with_options("Magnetometer Magnitude", de->mpu_timestamps, de->magno_norm.data, graph_options,
                                  line_count == 1);
            else if (graph_variable == "ANORM")
                plot_with_options("Accelerometer Magnitude $(m/s^2)$", de->mpu_timestamps, de->accel_norm.data, graph_options,
                                  line_count == 1);
            else if (graph_variable == "GNORM")
                plot_with_options("Gyro Magnitude (rad/s)", de->mpu_timestamps, de->gyro_norm.data, graph_options,
                                  line_count == 1);

            else if (graph_variable == "ADAX")
                plot_with_options("Accelerometer X $(m/s^2)$", de->adis_timestamps, de->adis_accel_x.data, graph_options, line_count == 1);
            else if (graph_variable == "ADAY")
                plot_with_options("Accelerometer Y $(m/s^2)$", de->adis_timestamps, de->adis_accel_y.data, graph_options, line_count == 1);
            else if (graph_variable == "ADAZ")
                plot_with_options("Accelerometer Z $(m/s^2)$", de->adis_timestamps, de->adis_accel_z.data, graph_options, line_count == 1);

            else if (graph_variable == "ADGX")
                plot_with_options("Gyroscope X (rad/s)", de->adis_timestamps, de->adis_gyro_x.data, graph_options, line_count == 1);
            else if (graph_variable == "ADGY")
                plot_with_options("Gyroscope Y (rad/s)", de->adis_timestamps, de->adis_gyro_y.data, graph_options, line_count == 1);
            else if (graph_variable == "ADGZ")
                plot_with_options("Gyroscope Z (rad/s)", de->adis_timestamps, de->adis_gyro_z.data, graph_options, line_count == 1);

            else if (graph_variable == "ADMX")
                plot_with_options("Magnetometer X", de->adis_timestamps, de->adis_magno_x.data, graph_options, line_count == 1);
            else if (graph_variable == "ADMY")
                plot_with_options("Magnetometer Y", de->adis_timestamps, de->adis_magno_y.data, graph_options, line_count == 1);
            else if (graph_variable == "ADMZ")
                plot_with_options("Magnetometer Z", de->adis_timestamps, de->adis_magno_z.data, graph_options, line_count == 1);

            else if (graph_variable == "ADAMANGLE")
                plot_with_options("Field Angle (rad)", de->adis_timestamps, de->adis_accel_magno_angle.data, graph_options,
                                  line_count == 1);
            else if (graph_variable == "ADMNORM")
                plot_with_options("Magnetometer Magnitude", de->adis_timestamps, de->adis_magno_norm.data, graph_options,
                                  line_count == 1);
            else if (graph_variable == "ADANORM")
                plot_with_options("Accelerometer Magnitude $(m/s^2)$", de->adis_timestamps, de->adis_accel_norm.data, graph_options,
                                  line_count == 1);
            else if (graph_variable == "ADGNORM")
                plot_with_options("Gyro Magnitude (rad/s)", de->adis_timestamps, de->adis_gyro_norm.data, graph_options,
                                  line_count == 1);

            else if (graph_variable == "SEAX")
                plot_with_options("Acceleration X $(m/s^2)$", de->state_timestamps, de->se_accel_x.data, graph_options,
                                  line_count == 1);
            else if (graph_variable == "SEAY")
                plot_with_options("Acceleration Y $(m/s^2)$", de->state_timestamps, de->se_accel_y.data, graph_options,
                                  line_count == 1);
            else if (graph_variable == "SEAZ")
                plot_with_options("Acceleration Z $(m/s^2)$", de->state_timestamps, de->se_accel_z.data, graph_options,
                                  line_count == 1);
            else if (graph_variable == "SEANORM")
                plot_with_options("Acceleration Magnitude $(m/s^2)$", de->state_timestamps, de->se_accel_norm.data, graph_options,
                                  line_count == 1);

            else if (graph_variable == "SEVX")
                plot_with_options("Velocity X (m/s)", de->state_timestamps, de->se_velocity_x.data, graph_options,
                                  line_count == 1);
            else if (graph_variable == "SEVY")
                plot_with_options("Velocity Y (m/s)", de->state_timestamps, de->se_velocity_y.data, graph_options,
                                  line_count == 1);
            else if (graph_variable == "SEVZ")
                plot_with_options("Velocity Z (m/s)", de->state_timestamps, de->se_velocity_z.data, graph_options,
                                  line_count == 1);

            else if (graph_variable == "SERX")
                plot_with_options("Rotation X ($^\\circ$)", de->state_timestamps, de->se_rotation_x.data, graph_options,
                                  line_count == 1);
            else if (graph_variable == "SERY")
                plot_with_options("Rotation Y ($^\\circ$)", de->state_timestamps, de->se_rotation_y.data, graph_options,
                                  line_count == 1);
            else if (graph_variable == "SERZ")
                plot_with_options("Rotation Z ($^\\circ$)", de->state_timestamps, de->se_rotation_z.data, graph_options,
                                  line_count == 1);

            else if (graph_variable == "SEAVX")
                plot_with_options("Angular Velocity X (rad/s)", de->state_timestamps, de->se_ang_velocity_x.data,
                                  graph_options, line_count == 1);
            else if (graph_variable == "SEAVY")
                plot_with_options("Angular Velocity Y (rad/s)", de->state_timestamps, de->se_ang_velocity_y.data,
                                  graph_options, line_count == 1);
            else if (graph_variable == "SEAVZ")
                plot_with_options("Angular Velocity Z (rad/s)", de->state_timestamps, de->se_ang_velocity_z.data,
                                  graph_options, line_count == 1);
            else if (graph_variable == "SEAV")
                plot_with_options("Angular Velocity Magnitude (rad/s)", de->state_timestamps, de->se_ang_vel_norm.data,
                                  graph_options, line_count == 1);

            else if (graph_variable == "SEGBX")
                plot_with_options("Gyroscope Bias X (rad/s)", de->state_debug_timestamps, de->se_gyro_bias_x.data, graph_options,
                                  line_count == 1);
            else if (graph_variable == "SEGBY")
                plot_with_options("Gyroscope Bias Y (rad/s)", de->state_debug_timestamps, de->se_gyro_bias_y.data, graph_options,
                                  line_count == 1);
            else if (graph_variable == "SEGBZ")
                plot_with_options("Gyroscope Bias Z (rad/s)", de->state_debug_timestamps, de->se_gyro_bias_z.data, graph_options,
                                  line_count == 1);

            else if (graph_variable == "SEABX")
                plot_with_options("Accelerometer Bias X $(m/s^2)$", de->state_debug_timestamps, de->se_accel_bias_x.data,
                                  graph_options, line_count == 1);
            else if (graph_variable == "SEABY")
                plot_with_options("Accelerometer Bias Y $(m/s^2)$", de->state_debug_timestamps, de->se_accel_bias_y.data,
                                  graph_options, line_count == 1);
            else if (graph_variable == "SEABZ")
                plot_with_options("Accelerometer Bias Z $(m/s^2)$", de->state_debug_timestamps, de->se_accel_bias_z.data,
                                  graph_options, line_count == 1);
            else if (graph_variable == "SEABNORM")
                plot_with_options("Accelerometer Bias Magnitude $(m/s^2)$", de->state_debug_timestamps, de->se_accel_bias_norm.data,
                                  graph_options, line_count == 1);

            else if (graph_variable == "SEMBX")
                plot_with_options("Magnetometer Bias X", de->state_debug_timestamps, de->se_magno_bias_x.data,
                                  graph_options, line_count == 1);
            else if (graph_variable == "SEMBY")
                plot_with_options("Magnetometer Bias Y", de->state_debug_timestamps, de->se_magno_bias_y.data,
                                  graph_options, line_count == 1);
            else if (graph_variable == "SEMBZ")
                plot_with_options("Magnetometer Bias Z", de->state_debug_timestamps, de->se_magno_bias_z.data,
                                  graph_options, line_count == 1);
            else if (graph_variable == "SEMBNORM")
                plot_with_options("Magnetometer Bias Magnitude", de->state_debug_timestamps, de->se_magno_bias_norm.data,
                                  graph_options, line_count == 1);

            else if (graph_variable == "AMRANGLE")
                plot_with_options("Accel Magno Reference Angle ($^\\circ$)", de->state_debug_timestamps,
                                  de->accel_magno_reference_angle.data, graph_options, line_count == 1);

            else if (graph_variable == "SEANEAV")
                plot_with_options("SE Accel Norm Exponential Average", de->state_debug_timestamps,
                                  de->se_accel_norm_exp_avg.data, graph_options, line_count == 1);
            else if (graph_variable == "SEGNEAV")
                plot_with_options("SE Gyro Norm Exponential Average", de->state_debug_timestamps,
                                  de->se_gyro_norm_exp_avg.data, graph_options, line_count == 1);
            else if (graph_variable == "SEMNEAV")
                plot_with_options("SE Magno Norm Exponential Average", de->state_debug_timestamps,
                                  de->se_magno_norm_exp_avg.data, graph_options, line_count == 1);
            else if (graph_variable == "SEANEVAR")
                plot_with_options("SE Accel Norm Exponential Variance", de->state_debug_timestamps,
                                  de->se_accel_exp_var.data, graph_options, line_count == 1);

            else if (graph_variable[0] == 'P') {
                int num;
                if (sscanf(graph_variable.c_str(), "P%d", &num) != 1) {
                    std::cerr << "Invalid Variable Format" << graph_variable << std::endl;
                    return 1;
                }
                assert(num >= 0 && num < KALMAN_NUM_STATES);

                plot_with_options("P" + std::to_string(num), de->state_debug_timestamps, de->P[num].data,
                                  graph_options, line_count == 1);
            } else {
                std::cerr << "Unrecognised Variable " << graph_variable << std::endl;
                return 1;
            }

        }

        plt::grid(true);

        if (line_count > 1)
            plt::legend();
        plt::save(output);
        plt::clf();
    }
    return 0;
}
