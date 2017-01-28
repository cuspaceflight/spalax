#include <messaging.h>
#include <config/telemetry_packets.h>
#include <calibration/mpu9250_calibration.h>
#include <Eigen/Geometry>
#include "state_estimate.h"
#include "Eigen/Core"
#include "wmm_util.h"
#include "quest.h"

// Forward Declarations
static void send_state_estimate();

bool has_gps = false;

state_estimate_t current_estimate;


#define VEC3_NORM(name) sqrtf(name[0] * name[0] + name[1] * name[1] + name[2] * name[2]);

static bool getPacket(const telemetry_t* packet, message_metadata_t metadata) {
    if (packet->header.id == ts_mpu9250_data) {
        if (!has_gps)
            return true;

        auto data = telemetry_get_payload<mpu9250_data_t>(packet);
        mpu9250_calibrated_data_t calibrated_data;
        mpu9250_calibrate_data(data, &calibrated_data);

        MagneticFieldParams params;
        wmm_util_get_magnetic_field(current_estimate.latitude,current_estimate.longitude,current_estimate.altitude, &params);

        auto mag_ref_norm = VEC3_NORM(params.field_vector);
        auto mag_norm = VEC3_NORM(calibrated_data.magno);
        auto accel_norm = VEC3_NORM(calibrated_data.accel);

        const float observations[2][3] = {
                {calibrated_data.accel[0] / accel_norm, calibrated_data.accel[1]/accel_norm, calibrated_data.accel[2]/accel_norm},
                {calibrated_data.magno[0] / mag_norm, calibrated_data.magno[1]/mag_norm, calibrated_data.magno[2]/mag_norm}
        };

        const float references[2][3] = {
                {0.0, 0.0, 1.0f},
                {params.field_vector[0]/mag_ref_norm, params.field_vector[1]/mag_ref_norm, -params.field_vector[2]/mag_ref_norm}
        };

        const float a[2] = {0.5f, 0.5f};

        quest_estimate(observations, references, a, current_estimate.orientation_q);

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

    wmm_util_init(TIMESTAMP_YEAR + TIMESTAMP_WEEK / 52.0 + TIMESTAMP_DAY_OF_WEEK / (52.0 * 7.0));

    messaging_producer_init(&messaging_producer);
    messaging_consumer_init(&messaging_consumer);

    while (messaging_consumer_receive(&messaging_consumer, true, false) != messaging_receive_terminate);
}

static void send_state_estimate() {
    messaging_producer_send(&messaging_producer, message_flags_dont_send_over_usb, (const uint8_t *) &current_estimate);
}

#ifdef MESSAGING_OS_STD
void state_estimate_terminate() {
    messaging_consumer_terminate(&messaging_consumer);
}
#endif