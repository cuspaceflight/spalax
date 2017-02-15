#include <messaging.h>
#include <config/telemetry_packets.h>
#include <calibration/mpu9250_calibration.h>
#include <Eigen/Geometry>
#include <time_utils.h>
#include "state_estimate.h"
#include "Eigen/Core"
#include "quest.h"
#include "kalman.h"

// Forward Declarations
static void send_state_estimate();

bool has_gps = false;

state_estimate_t current_estimate;

uint32_t last_mpu_time = 0;

#define VEC3_NORM(name) sqrtf(name[0] * name[0] + name[1] * name[1] + name[2] * name[2]);

float reference_vectors[2][3];

static bool getPacket(const telemetry_t *packet, message_metadata_t metadata) {
    if (packet->header.id == ts_mpu9250_data) {
        if (last_mpu_time == 0) {
            last_mpu_time = packet->header.timestamp;
            return true;
        }
        last_mpu_time = packet->header.timestamp;

        if (!has_gps)
            return true;


        auto data = telemetry_get_payload<mpu9250_data_t>(packet);
        mpu9250_calibrated_data_t calibrated_data;
        mpu9250_calibrate_data(data, &calibrated_data);

//        auto mag_norm = VEC3_NORM(calibrated_data.magno);
//        auto accel_norm = VEC3_NORM(calibrated_data.accel);
//
//        const float observations[2][3] = {
//                {calibrated_data.accel[0] / accel_norm, calibrated_data.accel[1] / accel_norm,
//                                                                                             calibrated_data.accel[2] /
//                                                                                             accel_norm},
//                {calibrated_data.magno[0] / mag_norm,   calibrated_data.magno[1] / mag_norm, calibrated_data.magno[2] /
//                                                                                             mag_norm}
//        };
//
//        const float a[2] = {0.5f, 0.5f};


        //quest_estimate(observations, reference_vectors, a, current_estimate.orientation_q);

        float dt = (float)(clocks_between(last_mpu_time, packet->header.timestamp)) / CLOCK_FREQUENCY;
        kalman_predict(dt);

        kalman_new_accel(calibrated_data.accel);
        kalman_new_magno(calibrated_data.magno);
        kalman_new_gyro(calibrated_data.gyro);

        kalman_get_state(&current_estimate);
        send_state_estimate();

    } else if (packet->header.id == ts_ublox_nav) {
        auto data = telemetry_get_payload<ublox_nav_t>(packet);
        // Disabled for testing indoors
        //auto v_mag = sqrtf(data->velE * data->velE + data->velN * data->velN);
        //if (data->fix_type != 3 || data->num_sv < 8 || data->p_dop > 300 || data->v_acc > v_mag / 2.0f)
        //    return true;

        current_estimate.latitude = data->lat;
        current_estimate.longitude = data->lon;
        current_estimate.altitude = data->height;
        has_gps = true;
    }
    return true;
}

MESSAGING_PRODUCER(messaging_producer, ts_state_estimate_data, sizeof(state_estimate_t), 20)
MESSAGING_CONSUMER(messaging_consumer, ts_m3imu, ts_m3imu_mask, 0, 0, getPacket, 1024);

void state_estimate_thread(void *arg) {
    platform_set_thread_name("State Estimate");

    messaging_producer_init(&messaging_producer);
    messaging_consumer_init(&messaging_consumer);

    // Temporary For Testing
    has_gps = true;
    current_estimate.latitude = 52.2053f;
    current_estimate.longitude = 0.1218f;
    current_estimate.altitude = 60;

    reference_vectors[0][0] = 0;
    reference_vectors[0][1] = 0;
    reference_vectors[0][2] = 1;
    reference_vectors[1][0] = 0.39134267f;
    reference_vectors[1][1] = -0.00455851434f;
    reference_vectors[1][2] = -0.920233727f;

    kalman_init(reference_vectors[0], reference_vectors[1]);

    while (messaging_consumer_receive(&messaging_consumer, true, false) != messaging_receive_terminate);
}

static void send_state_estimate() {
    messaging_producer_send(&messaging_producer, 0, (const uint8_t *) &current_estimate);
}

#ifdef MESSAGING_OS_STD

void state_estimate_terminate() {
    messaging_consumer_terminate(&messaging_consumer);
}

#endif