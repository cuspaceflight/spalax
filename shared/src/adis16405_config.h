#ifndef ADIS16405_CONFIG_H
#define ADIS16405_CONFIG_H
#include "compilermacros.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

	typedef struct adis16405_config_t {
		// Converts to m/s^2
		float accel_sf;

		// Converts to rad/s
		float gyro_sf;

		uint16_t magno_sf[3];
		int16_t magno_bias[3];
	} adis16405_config_t;

	typedef struct adis16405_data_t {
		int16_t supply;
		int16_t gyro[3];
		int16_t accel[3];
		int16_t magno[3];
	} adis16405_data_t;

	typedef struct adis16405_calibrated_data_t {
		float supply;
		float gyro[3];
		float accel[3];
		float magno[3];
	} adis16405_calibrated_data_t;

	STATIC_ASSERT(sizeof(adis16405_config_t) == 20, adis16405_config_padded);
	STATIC_ASSERT(sizeof(adis16405_data_t) == 20, adis16405_data_padded);

	void adis16405_calibrate_data(const adis16405_config_t* config, const adis16405_data_t* uncalibrated_data, adis16405_calibrated_data_t* calibrated_data);

#ifdef __cplusplus
}
#endif

#endif /* ADIS16405_CONFIG_H */
