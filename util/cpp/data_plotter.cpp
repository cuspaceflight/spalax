#include <matplotlibcpp.h>
#include <sstream>
#include "data_extractor.h"
#include "data_plotter.h"


namespace plt = matplotlibcpp;

void enable_streams(int argc, char **argv, DataExtractor *de) {
    for (int k = 2; k < argc; k+= 2) {
        std::istringstream f(argv[k+1]);
        std::string graph_variable;

        while (getline(f, graph_variable, ';')) {
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

            else if (graph_variable == "AMRANGLE")
                de->accel_magno_reference_angle.enabled = true;

        }
    }
}

void plot_data(int argc, char **argv, const DataExtractor* de) {
    printf("Plotting Graphs\n");

    for (int k = 2; k < argc; k+= 2) {
        const char *output = argv[k];
        std::istringstream f(argv[k+1]);
        std::string graph_variable;

        while (getline(f, graph_variable, ';')) {
            if (graph_variable == "AX")
                plt::named_plot("Accel X", de->mpu_timestamps, de->accel_x.data);
            else if (graph_variable == "AY")
                plt::named_plot("Accel Y", de->mpu_timestamps, de->accel_y.data);
            else if (graph_variable == "AZ")
                plt::named_plot("Accel Z", de->mpu_timestamps, de->accel_z.data);

            else if (graph_variable == "GX")
                plt::named_plot("Gyro X", de->mpu_timestamps, de->gyro_x.data);
            else if (graph_variable == "GY")
                plt::named_plot("Gyro Y", de->mpu_timestamps, de->gyro_y.data);
            else if (graph_variable == "GZ")
                plt::named_plot("Gyro Z", de->mpu_timestamps, de->gyro_z.data);

            else if (graph_variable == "MX")
                plt::named_plot("Magno X", de->mpu_timestamps, de->magno_x.data);
            else if (graph_variable == "MY")
                plt::named_plot("Magno Y", de->mpu_timestamps, de->magno_y.data);
            else if (graph_variable == "MZ")
                plt::named_plot("Magno Z", de->mpu_timestamps, de->magno_z.data);

            else if (graph_variable == "AMANGLE")
                plt::named_plot("Accel Magno Angle", de->mpu_timestamps, de->accel_magno_angle.data);
            else if (graph_variable == "MNORM")
                plt::named_plot("Magno Magnitude", de->mpu_timestamps, de->magno_norm.data);
            else if (graph_variable == "ANORM")
                plt::named_plot("Accel Magnitude", de->mpu_timestamps, de->accel_norm.data);
            else if (graph_variable == "GNORM")
                plt::named_plot("Gyro Magnitude", de->mpu_timestamps, de->gyro_norm.data);

            else if (graph_variable == "SEAX")
                plt::named_plot("SE Accel X", de->state_timestamps, de->se_accel_x.data);
            else if (graph_variable == "SEAY")
                plt::named_plot("SE Accel Y", de->state_timestamps, de->se_accel_y.data);
            else if (graph_variable == "SEAZ")
                plt::named_plot("SE Accel Z", de->state_timestamps, de->se_accel_z.data);
            else if (graph_variable == "SEANORM")
                plt::named_plot("SE Accel Magnitude", de->state_timestamps, de->se_accel_norm.data);

            else if (graph_variable == "SEVX")
                plt::named_plot("SE Velocity X", de->state_timestamps, de->se_velocity_x.data);
            else if (graph_variable == "SEVY")
                plt::named_plot("SE Velocity Y", de->state_timestamps, de->se_velocity_y.data);
            else if (graph_variable == "SEVZ")
                plt::named_plot("SE Velocity Z", de->state_timestamps, de->se_velocity_z.data);

            else if (graph_variable == "SERX")
                plt::named_plot("SE Rotation X", de->state_timestamps, de->se_rotation_x.data);
            else if (graph_variable == "SERY")
                plt::named_plot("SE Rotation Y", de->state_timestamps, de->se_rotation_y.data);
            else if (graph_variable == "SERZ")
                plt::named_plot("SE Rotation Z", de->state_timestamps, de->se_rotation_z.data);

            else if (graph_variable == "SEAVX")
                plt::named_plot("SE Angular Velocity X", de->state_timestamps, de->se_ang_velocity_x.data);
            else if (graph_variable == "SEAVY")
                plt::named_plot("SE Angular Velocity Y", de->state_timestamps, de->se_ang_velocity_y.data);
            else if (graph_variable == "SEAVZ")
                plt::named_plot("SE Angular Velocity Z", de->state_timestamps, de->se_ang_velocity_z.data);
            else if (graph_variable == "SEAV")
                plt::named_plot("SE Angular Velocity Magnitude", de->state_timestamps, de->se_ang_vel_norm.data);

            else if (graph_variable == "SEGBX")
                plt::named_plot("SE Gyro Bias X", de->state_debug_timestamps, de->se_gyro_bias_x.data);
            else if (graph_variable == "SEGBY")
                plt::named_plot("SE Gyro Bias Y", de->state_debug_timestamps, de->se_gyro_bias_y.data);
            else if (graph_variable == "SEGBZ")
                plt::named_plot("SE Gyro Bias Z", de->state_debug_timestamps, de->se_gyro_bias_z.data);

            else if (graph_variable == "SEABX")
                plt::named_plot("SE Accel Bias X", de->state_debug_timestamps, de->se_accel_bias_x.data);
            else if (graph_variable == "SEABY")
                plt::named_plot("SE Accel Bias Y", de->state_debug_timestamps, de->se_accel_bias_y.data);
            else if (graph_variable == "SEABZ")
                plt::named_plot("SE Accel Bias Z", de->state_debug_timestamps, de->se_accel_bias_z.data);
            else if (graph_variable == "SEABNORM")
                plt::named_plot("SE Accel Bias Magnitude", de->state_debug_timestamps, de->se_accel_bias_norm.data);

            else if (graph_variable == "SEMBX")
                plt::named_plot("SE Magno Bias X", de->state_debug_timestamps, de->se_magno_bias_x.data);
            else if (graph_variable == "SEMBY")
                plt::named_plot("SE Magno Bias Y", de->state_debug_timestamps, de->se_magno_bias_y.data);
            else if (graph_variable == "SEMBZ")
                plt::named_plot("SE Magno Bias Z", de->state_debug_timestamps, de->se_magno_bias_z.data);
            else if (graph_variable == "SEMBNORM")
                plt::named_plot("SE Magno Bias Magnitude", de->state_debug_timestamps, de->se_magno_bias_norm.data);

            else if (graph_variable == "AMRANGLE")
                plt::named_plot("Accel Magno Reference Angle", de->state_debug_timestamps, de->accel_magno_reference_angle.data);

        }

        plt::grid(true);
        plt::legend();
        plt::save(output);
        plt::clf();
    }

}
