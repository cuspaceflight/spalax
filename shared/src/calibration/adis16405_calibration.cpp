#include <util/board_config.h>
#include "adis16405_calibration.h"
#include "Eigen/Core"


void adis16405_calibrate_data(const adis16405_data_t* uncalibrated_data, adis16405_calibrated_data_t* calibrated_data) {
    auto config = getBoardConfig();

    Eigen::Matrix3f magno_transform = Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(
            config->adis16405_magno_transform);
    Eigen::Matrix3f accel_transform = Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(
            config->adis16405_accel_transform);

    Eigen::Vector3f magno_offset = Eigen::Vector3f(config->adis16405_magno_offset);
    Eigen::Vector3f accel_offset = Eigen::Vector3f(config->adis16405_accel_offset);

    Eigen::Vector3f magno_raw = Eigen::Vector3f(uncalibrated_data->magno[0], uncalibrated_data->magno[1], uncalibrated_data->magno[2]);
    Eigen::Vector3f accel_raw = Eigen::Vector3f(uncalibrated_data->accel[0], uncalibrated_data->accel[1], uncalibrated_data->accel[2]);

    Eigen::Vector3f calibrated_magno = magno_transform * (magno_raw - magno_offset);
    Eigen::Vector3f calibrated_accel = -accel_transform * (accel_raw - accel_offset);

    calibrated_data->magno[0] = calibrated_magno.x();
    calibrated_data->magno[1] = calibrated_magno.y();
    calibrated_data->magno[2] = calibrated_magno.z();

    calibrated_data->accel[0] = calibrated_accel.x() * 9.80665f;
    calibrated_data->accel[1] = calibrated_accel.y() * 9.80665f;
    calibrated_data->accel[2] = calibrated_accel.z() * 9.80665f;

    for (int i = 0; i < 3; i++) {
        calibrated_data->gyro[i] = (float)uncalibrated_data->gyro[i] * 0.05f*3.14159265359f/180.0f;
        calibrated_data->supply = (float)uncalibrated_data->supply * 2.418e-3f;
    }
}
