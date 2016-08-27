#include "mpu9250_config.h"

void mpu9250_calibrate_data(const mpu9250_config_t* config, const mpu9250_data_t* uncalibrated_data, mpu9250_calibrated_data_t* calibrated_data) {
	// Correct for flight stack's upside down mpu9250

	calibrated_data->accel[0] = uncalibrated_data->accel[1] * config->accel_sf;
	calibrated_data->gyro[0] = uncalibrated_data->gyro[1] * config->gyro_sf;
	calibrated_data->accel[1] = uncalibrated_data->accel[0] * config->accel_sf;
	calibrated_data->gyro[1] = uncalibrated_data->gyro[0] * config->gyro_sf;
	calibrated_data->accel[2] = -uncalibrated_data->accel[2] * config->accel_sf;
	calibrated_data->gyro[2] = -uncalibrated_data->gyro[2] * config->gyro_sf;

    // We correct for the magnetometer's strange orientation
    calibrated_data->magno[1] = (uncalibrated_data->magno[1] - (float)config->magno_bias[1]) * (float)config->magno_sf[1] / 1000000.f;
    calibrated_data->magno[0] = (uncalibrated_data->magno[0] - (float)config->magno_bias[0]) * (float)config->magno_sf[0] / 1000000.f;
    calibrated_data->magno[2] = (uncalibrated_data->magno[2] - (float)config->magno_bias[2]) * (float)config->magno_sf[2] / 1000000.f;
}