#ifndef ADIS16405_CONFIG_H
#define ADIS16405_CONFIG_H
#include "compilermacros.h"
#include <stdint.h>
#include "telemetry_packets.h"

#ifdef __cplusplus
extern "C" {
#endif

	typedef struct adis16405_calibrated_data_t {
		float supply;
		float gyro[3];
		float accel[3];
		float magno[3];
	} adis16405_calibrated_data_t;

	void adis16405_calibrate_data(const adis16405_config_t* config, const adis16405_data_t* uncalibrated_data, adis16405_calibrated_data_t* calibrated_data);

#ifdef __cplusplus
}
#endif

#endif /* ADIS16405_CONFIG_H */
