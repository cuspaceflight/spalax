#include "mpu9250_config.h"

void mpu9250_calibrate_data(const mpu9250_config_t* config, const mpu9250_data_t* uncalibrated_data, mpu9250_calibrated_data_t* calibrated_data) {
    for (int i = 0; i < 3; i++) {
        calibrated_data->accel[i] = uncalibrated_data->accel[i] * config->accel_sf;
        calibrated_data->gyro[i] = uncalibrated_data->gyro[i] * config->gyro_sf;
    }

    // We correct for the magnetometer's strange orientation
    calibrated_data->magno[0] = (uncalibrated_data->magno[1] - config->magno_bias[0]) * config->magno_sf[0];
    calibrated_data->magno[1] = (uncalibrated_data->magno[0] - config->magno_bias[1]) * config->magno_sf[1];
    calibrated_data->magno[2] = -(uncalibrated_data->magno[2] + config->magno_bias[2]) * config->magno_sf[2];
}