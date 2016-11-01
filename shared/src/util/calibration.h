#ifndef CALIBRATION_H
#define CALIBRATION_H
#include <stdint.h>
#include "compilermacros.h"

#ifdef __cplusplus
extern "C" {
#endif

    typedef enum {
        calibration_procedure_none,
        calibration_procedure_mpu9250_bias,
		calibration_producer_adis16405_bias,
    } calibration_procedure_t;

    typedef struct {
        uint8_t procedure;
    } calibration_control_t;


    void calibration_thread(void* arg);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_H */
