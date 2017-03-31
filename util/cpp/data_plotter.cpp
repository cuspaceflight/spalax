#include <matplotlibcpp.h>
#include <sstream>
#include "data_extractor.h"
#include "data_plotter.h"


namespace plt = matplotlibcpp;

void plot_data(int argc, char **argv) {
    printf("Plotting Graphs\n");

    for (int k = 2; k < argc; k+= 2) {
        const char *output = argv[k];
        std::istringstream f(argv[k+1]);
        std::string graph_variable;

        while (getline(f, graph_variable, ';')) {
            if (graph_variable == "AX")
                plt::named_plot("Accel X", mpu_timestamps, accel_x);
            else if (graph_variable == "AY")
                plt::named_plot("Accel Y", mpu_timestamps, accel_y);
            else if (graph_variable == "AZ")
                plt::named_plot("Accel Z", mpu_timestamps, accel_z);

            else if (graph_variable == "GX")
                plt::named_plot("Gyro X", mpu_timestamps, gyro_x);
            else if (graph_variable == "GY")
                plt::named_plot("Gyro Y", mpu_timestamps, gyro_y);
            else if (graph_variable == "GZ")
                plt::named_plot("Gyro Z", mpu_timestamps, gyro_z);

            else if (graph_variable == "MX")
                plt::named_plot("Magno X", mpu_timestamps, magno_x);
            else if (graph_variable == "MY")
                plt::named_plot("Magno Y", mpu_timestamps, magno_y);
            else if (graph_variable == "MZ")
                plt::named_plot("Magno Z", mpu_timestamps, magno_z);

            else if (graph_variable == "AMANGLE")
                plt::named_plot("Accel Magno Angle", mpu_timestamps, accel_magno_angle);
            else if (graph_variable == "MNORM")
                plt::named_plot("Magno Magnitude", mpu_timestamps, magno_norm);
            else if (graph_variable == "ANORM")
                plt::named_plot("Accel Magnitude", mpu_timestamps, accel_norm);
            else if (graph_variable == "GNORM")
                plt::named_plot("Gyro Magnitude", mpu_timestamps, gyro_norm);

            else if (graph_variable == "SEAX")
                plt::named_plot("SE Accel X", state_timestamps, se_accel_x);
            else if (graph_variable == "SEAY")
                plt::named_plot("SE Accel Y", state_timestamps, se_accel_y);
            else if (graph_variable == "SEAZ")
                plt::named_plot("SE Accel Z", state_timestamps, se_accel_z);
            else if (graph_variable == "SEANORM")
                plt::named_plot("SE Accel Magnitude", state_timestamps, se_accel_norm);

            else if (graph_variable == "SEVX")
                plt::named_plot("SE Velocity X", state_timestamps, se_velocity_x);
            else if (graph_variable == "SEVY")
                plt::named_plot("SE Velocity Y", state_timestamps, se_velocity_y);
            else if (graph_variable == "SEVZ")
                plt::named_plot("SE Velocity Z", state_timestamps, se_velocity_z);

            else if (graph_variable == "SERX")
                plt::named_plot("SE Rotation X", state_timestamps, se_rotation_x);
            else if (graph_variable == "SERY")
                plt::named_plot("SE Rotation Y", state_timestamps, se_rotation_y);
            else if (graph_variable == "SERZ")
                plt::named_plot("SE Rotation Z", state_timestamps, se_rotation_z);

            else if (graph_variable == "SEAVX")
                plt::named_plot("SE Angular Velocity X", state_timestamps, se_ang_velocity_x);
            else if (graph_variable == "SEAVY")
                plt::named_plot("SE Angular Velocity Y", state_timestamps, se_ang_velocity_y);
            else if (graph_variable == "SEAVZ")
                plt::named_plot("SE Angular Velocity Z", state_timestamps, se_ang_velocity_z);
            else if (graph_variable == "SEAV")
                plt::named_plot("SE Angular Velocity Magnitude", state_timestamps, se_ang_vel_norm);

            else if (graph_variable == "SEGBX")
                plt::named_plot("SE Gyro Bias X", state_debug_timestamps, se_gyro_bias_x);
            else if (graph_variable == "SEGBY")
                plt::named_plot("SE Gyro Bias Y", state_debug_timestamps, se_gyro_bias_y);
            else if (graph_variable == "SEGBZ")
                plt::named_plot("SE Gyro Bias Z", state_debug_timestamps, se_gyro_bias_z);

            else if (graph_variable == "SEABX")
                plt::named_plot("SE Accel Bias X", state_debug_timestamps, se_accel_bias_x);
            else if (graph_variable == "SEABY")
                plt::named_plot("SE Accel Bias Y", state_debug_timestamps, se_accel_bias_y);
            else if (graph_variable == "SEABZ")
                plt::named_plot("SE Accel Bias Z", state_debug_timestamps, se_accel_bias_z);
            else if (graph_variable == "SEABNORM")
                plt::named_plot("SE Accel Bias Magnitude", state_debug_timestamps, se_accel_bias_norm);

            else if (graph_variable == "SEMBX")
                plt::named_plot("SE Magno Bias X", state_debug_timestamps, se_magno_bias_x);
            else if (graph_variable == "SEMBY")
                plt::named_plot("SE Magno Bias Y", state_debug_timestamps, se_magno_bias_y);
            else if (graph_variable == "SEMBZ")
                plt::named_plot("SE Magno Bias Z", state_debug_timestamps, se_magno_bias_z);
            else if (graph_variable == "SEMBNORM")
                plt::named_plot("SE Magno Bias Magnitude", state_debug_timestamps, se_magno_bias_norm);

            else if (graph_variable == "AMRANGLE")
                plt::named_plot("Accel Magno Reference Angle", state_debug_timestamps, accel_magno_reference_angle);

        }

        plt::grid(true);
        plt::legend();
        plt::save(output);
        plt::clf();
    }

}
