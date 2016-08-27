#include "adis16405_config.h"

void adis16405_calibrate_data(const adis16405_config_t* config, const adis16405_data_t* uncalibrated_data, adis16405_calibrated_data_t* calibrated_data) {
	for (int i = 0; i < 3; i++) {
		// The ADIS's accelerometer has axis in the opposite direction to normal
		calibrated_data->accel[i] = -uncalibrated_data->accel[i] * config->accel_sf;
		calibrated_data->gyro[i] = uncalibrated_data->gyro[i] * config->gyro_sf;
	}

	calibrated_data->magno[0] = (uncalibrated_data->magno[0] - (float)config->magno_bias[0]) * (float)config->magno_sf[0] / 1000000.f;
	calibrated_data->magno[1] = (uncalibrated_data->magno[1] - (float)config->magno_bias[1]) * (float)config->magno_sf[1] / 1000000.f;
	calibrated_data->magno[2] = (uncalibrated_data->magno[2] - (float)config->magno_bias[2]) * (float)config->magno_sf[2] / 1000000.f;

	// These are fixed by the sensor and are not user configurable so we hard code them here
	calibrated_data->supply = uncalibrated_data->supply * 2.4180f / 1000.0f;
}
