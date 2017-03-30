#include <util/board_config.h>
#include "mpu9250_calibration.h"
#include "Eigen/Core"

const float F_PI = 3.14159265359f;

void mpu9250_calibrate_data(const mpu9250_data_t *uncalibrated_data, mpu9250_calibrated_data_t *calibrated_data) {
    auto config = getBoardConfig();

    Eigen::Matrix3f magno_transform = Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(
            config->mpu9250_magno_transform);
    Eigen::Matrix3f accel_transform = Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(
            config->mpu9250_accel_transform);

    Eigen::Vector3f magno_offset = Eigen::Vector3f(config->mpu9250_magno_offset);
    Eigen::Vector3f accel_offset = Eigen::Vector3f(config->mpu9250_accel_offset);

    Eigen::Vector3f magno_raw = Eigen::Vector3f(uncalibrated_data->magno[1], uncalibrated_data->magno[0],
                                                -uncalibrated_data->magno[2]);
    Eigen::Vector3f accel_raw = Eigen::Vector3f(uncalibrated_data->accel[0], uncalibrated_data->accel[1],
                                                uncalibrated_data->accel[2]);

    Eigen::Vector3f calibrated_magno = magno_transform * (magno_raw - magno_offset);
    Eigen::Vector3f calibrated_accel = accel_transform * (accel_raw - accel_offset);

    calibrated_data->magno[0] = calibrated_magno.x();
    calibrated_data->magno[1] = calibrated_magno.y();
    calibrated_data->magno[2] = calibrated_magno.z();

    calibrated_data->accel[0] = calibrated_accel.x() * 9.80665f;
    calibrated_data->accel[1] = calibrated_accel.y() * 9.80665f;
    calibrated_data->accel[2] = calibrated_accel.z() * 9.80665f;

    calibrated_data->gyro[0] = uncalibrated_data->gyro[0] * config->mpu9250_gyro_sf;
    calibrated_data->gyro[1] = uncalibrated_data->gyro[1] * config->mpu9250_gyro_sf;
    calibrated_data->gyro[2] = uncalibrated_data->gyro[2] * config->mpu9250_gyro_sf;
}

float mpu9250_get_heading(mpu9250_calibrated_data_t *calibrated_data) {
    float value = atan2f(-calibrated_data->magno[0], calibrated_data->magno[1]) * 180.0f / F_PI;
    if (value < 0.0f)
        value += 360.0f;
    return value;
}
