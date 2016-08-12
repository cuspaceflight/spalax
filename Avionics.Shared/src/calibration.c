#include "calibration.h"
#include <platform.h>
#include <component_state.h>
#include "messaging.h"
#include <config/telemetry_config.h>
#include <mpu9250_config.h>
#include <stdlib.h>
#include <math.h>

static calibration_procedure_t current_producedure = calibration_procedure_none;
static calibration_data_t calibration_data;

static bool control_packet(const telemetry_t* packet, message_metadata_t metadata);
static bool data_packet(const telemetry_t* packet, message_metadata_t metadata);

MESSAGING_CONSUMER(consumer_control, telemetry_id_calibration_control, telemetry_source_packet_specific_mask, 0, 0, &control_packet, 5);
MESSAGING_CONSUMER(consumer_data, telemetry_source_imu_data, telemetry_source_imu_data_mask, 0, 0, data_packet, 1024);
MESSAGING_PRODUCER(data_producer, telemetry_id_calibration_data, sizeof(calibration_data_t), 2);

static bool control_packet(const telemetry_t* packet, message_metadata_t metadata) {
    (void)metadata;

    if (packet->header.id != telemetry_id_calibration_control)
        return true;
    
    calibration_control_t* control = (calibration_control_t*)packet->payload;

    if (current_producedure == calibration_procedure_none) {
        current_producedure = control->procedure;
    } else if (control->procedure == calibration_procedure_none) {
        calibration_data.procedure = current_producedure;
        current_producedure = calibration_procedure_none;
    }
    return true;
}

static bool mpu9250_bias_data(const telemetry_t* packet, uint16_t metadata) {
    (void)metadata;

    static bool has_config = false;
    static mpu9250_config_t config;
    static float min_accel[3];
    static float max_accel[3];
    static float min_magno[3];
    static float max_magno[3];
    
    if (packet->header.id == telemetry_id_mpu9250_data) {
        if (!has_config)
            return true;
        mpu9250_data_t* data = (mpu9250_data_t*)packet->payload;
        mpu9250_calibrated_data_t calibrated;
        mpu9250_calibrate_data(&config, data, &calibrated);

        for (int i = 0; i < 3; i++) {
            min_accel[i] = fmin(min_accel[i], calibrated.accel[i]);
            max_accel[i] = fmax(max_accel[i], calibrated.accel[i]);
            min_magno[i] = fmin(min_magno[i], calibrated.magno[i]);
            max_magno[i] = fmax(max_magno[i], calibrated.magno[i]);

            calibration_data.data[0][i] = (max_accel[i] + min_accel[i]) / 2.0f;
            calibration_data.data[1][i] = (max_magno[i] + min_magno[i]) / 2.0f;
        }

    } else if (packet->header.id == telemetry_id_mpu9250_config) {
        if (has_config)
            return true;
        has_config = true;
        config = *(mpu9250_config_t*)packet->payload;
        for (int i = 0; i < 3; i++) {
            config.accel_bias[i] = 0;
            config.magno_bias[i] = 0;
        }
    }
    return true;
}

static bool data_packet(const telemetry_t* packet, message_metadata_t metadata) {
    if (current_producedure == calibration_procedure_mpu9250_bias)
        return mpu9250_bias_data(packet, metadata);

    return true;
}


int32_t calibration_thread(void* arg) {
    (void)arg;

    platform_set_thread_name("Calibration");

    messaging_consumer_init(&consumer_control);
    messaging_consumer_init(&consumer_data);
    messaging_producer_init(&data_producer);
   

    while (true) {
        COMPONENT_STATE_UPDATE(avionics_component_calibration, state_ok);

        messaging_pause_consumer(&consumer_data, true);
        while (current_producedure == calibration_procedure_none) {
            messaging_consumer_receive(&consumer_control, true, false);
        }

        COMPONENT_STATE_UPDATE(avionics_component_calibration, state_initializing);
        messaging_resume_consumer(&consumer_data);


        calibration_data.procedure = current_producedure;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                calibration_data.data[i][j] = 0;
            }
        }

        while (true) {
            while (messaging_consumer_receive(&consumer_control, false, false) == messaging_receive_ok);
            if (current_producedure == calibration_procedure_none) {
                messaging_producer_send(&data_producer, 0, (const uint8_t*)&calibration_data);
                break;
            }

            while (messaging_consumer_receive(&consumer_data, false, false) == messaging_receive_ok);
            messaging_consumer_receive(&consumer_data, true, false);
        }
    }
}