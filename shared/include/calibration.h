#ifndef CALIBRATION_H
#define CALIBRATION_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    typedef enum {
        calibration_procedure_none,
        calibration_procedure_mpu9250_bias,
    } calibration_procedure_t;

    typedef struct {
        calibration_procedure_t procedure;
        float data[3][3];
    } calibration_data_t;

    typedef struct {
        calibration_procedure_t procedure;
    } calibration_control_t;


    void calibration_thread(void* arg);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_H */
