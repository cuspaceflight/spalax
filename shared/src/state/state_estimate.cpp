#include <messaging.h>
#include <config/telemetry_packets.h>
#include <calibration/mpu9250_calibration.h>
#include <Eigen/Geometry>
#include <time_utils.h>
#include <component_state.h>
#include "state_estimate.h"
#include "Eigen/Core"
#include <Eigen/Geometry>
#include "quest.h"
#include "kalman.h"
#include "wmm_util.h"

using namespace Eigen;

// Forward Declarations
static bool getPacket(const telemetry_t *packet, message_metadata_t metadata);

enum class StateEstimatePhase {
    Init,
    Calibration,
    Estimation
};

static StateEstimatePhase state_estimate_phase = StateEstimatePhase::Init;

#define NUM_CALIBRATION_SAMPLES 1000
static int remaining_calibration_samples;
static Matrix<fp, 3, 1> magno_calibration;
static Matrix<fp, 3, 1> accel_calibration;
static Matrix<fp, 3, 1> gyro_calibration;

static const Vector3f accel_reference(0, 0, 1);
static const Vector3f magno_reference(0.39134267f, -0.00455851434f, -0.920233727f);

static uint32_t data_timestamp = 0;

MESSAGING_PRODUCER(messaging_producer, ts_state_estimate_data, sizeof(state_estimate_t), 20)
MESSAGING_PRODUCER(messaging_producer_debug, ts_state_estimate_debug, sizeof(state_estimate_debug_t), 20)
MESSAGING_CONSUMER(messaging_consumer, ts_raw_data, ts_raw_data, 0, 0, getPacket, 1024);

void state_estimate_init() {
    state_estimate_phase = StateEstimatePhase::Calibration;

    remaining_calibration_samples = NUM_CALIBRATION_SAMPLES;

    data_timestamp = 0;
    magno_calibration = Vector3f::Zero();
    accel_calibration = Vector3f::Zero();
    gyro_calibration = Vector3f::Zero();

//
//    wmm_util_init(TIMESTAMP_YEAR + TIMESTAMP_WEEK / 52.0 + TIMESTAMP_DAY_OF_WEEK / (52.0 * 7.0));
//
//    MagneticFieldParams params;
//    wmm_util_get_magnetic_field(52.2053, 0.1218, 0, &params);
//    wmm_util_get_magnetic_field(52.2053, 0.1218, 0, &params);
//    wmm_util_get_magnetic_field(52.2053, 0.1218, 0, &params);
//    wmm_util_get_magnetic_field(52.2053, 0.1218, 0, &params);
//    wmm_util_get_magnetic_field(52.2053, 0.1218, 0, &params);
//    wmm_util_get_magnetic_field(52.2053, 0.1218, 0, &params);
//    wmm_util_get_magnetic_field(52.2053, 0.1218, 0, &params);
//    wmm_util_get_magnetic_field(52.2053, 0.1218, 0, &params);

    messaging_producer_init(&messaging_producer);
    messaging_producer_init(&messaging_producer_debug);
    messaging_consumer_init(&messaging_consumer);
}

void state_estimate_thread(void *arg) {
    platform_set_thread_name("State Estimate");

    state_estimate_init();

    while (messaging_consumer_receive(&messaging_consumer, true, false) != messaging_receive_terminate);
}


static bool getPacket(const telemetry_t *packet, message_metadata_t metadata) {
    if (packet->header.id == ts_mpu9250_data) {
        if (data_timestamp == 0) {
            data_timestamp = packet->header.timestamp;
            return true;
        }
        float dt =
                (float) (clocks_between(data_timestamp, packet->header.timestamp)) / CLOCK_FREQUENCY;

        data_timestamp = packet->header.timestamp;

        auto data = telemetry_get_payload<mpu9250_data_t>(packet);
        mpu9250_calibrated_data_t calibrated_data;
        mpu9250_calibrate_data(data, &calibrated_data);


        if (state_estimate_phase == StateEstimatePhase::Calibration) {
            remaining_calibration_samples--;

            magno_calibration += Map<const Matrix<fp, 3, 1>>(calibrated_data.magno);
            accel_calibration += Map<const Matrix<fp, 3, 1>>(calibrated_data.accel);
            gyro_calibration += Map<const Matrix<fp, 3, 1>>(calibrated_data.gyro);

            if (remaining_calibration_samples <= 0) {

                accel_calibration /= (float) NUM_CALIBRATION_SAMPLES;
                magno_calibration /= (float) NUM_CALIBRATION_SAMPLES;
                gyro_calibration /= (float) NUM_CALIBRATION_SAMPLES;

                const float quest_reference_vectors[2][3] = {
                        {accel_reference.x(), accel_reference.y(), accel_reference.z()},
                        {magno_reference.x(), magno_reference.y(), magno_reference.z()}
                };

                Vector3f accel_calib_norm = accel_calibration.normalized();
                Vector3f magno_calib_norm = magno_calibration.normalized();

                const float quest_observations[2][3] = {
                        {accel_calib_norm[0], accel_calib_norm[1], accel_calib_norm[2]},
                        {magno_calib_norm[0], magno_calib_norm[1], magno_calib_norm[2]},
                };

                const float a[2] = {0.5f, 0.5f};

                float initial_orientation[4] = {0, 0, 0, 1};
                float initial_angular_velocity[3] = {0, 0, 0};
                float initial_position[3] = {0, 0, 0};
                float initial_velocity[3] = {0, 0, 0};
                float initial_acceleration[3] = {0, 0, 0};
                float initial_accel_bias[3] = {0, 0, 0};
                float initial_magno_bias[3] = {0, 0, 0};
                float initial_gyro_bias[3] = {
                        gyro_calibration.x(),
                        gyro_calibration.y(),
                        gyro_calibration.z()
                };

                if (quest_estimate(quest_observations, quest_reference_vectors, a, initial_orientation) == -1) {
                    COMPONENT_STATE_UPDATE(avionics_component_state_state_estimate, state_error);
                }

                const float kalman_reference_vectors[2][3] = {
                        {accel_reference.x() * 9.80665f, accel_reference.y() * 9.80665f,
                                                                              accel_reference.z() * 9.80665f},
                        {magno_reference.x(),            magno_reference.y(), magno_reference.z()}
                };

                kalman_init(kalman_reference_vectors[0], kalman_reference_vectors[1], initial_orientation,
                            initial_angular_velocity, initial_position, initial_velocity, initial_acceleration,
                            initial_gyro_bias, initial_accel_bias, initial_magno_bias);

                state_estimate_phase = StateEstimatePhase::Estimation;
            }
        } else if (state_estimate_phase == StateEstimatePhase::Estimation) {
            kalman_predict(dt);

            kalman_new_gyro(calibrated_data.gyro);
            kalman_new_magno(calibrated_data.magno);
            kalman_new_accel(calibrated_data.accel);


            state_estimate_t current_estimate;

            kalman_get_state(&current_estimate);

            messaging_producer_send_timestamp(&messaging_producer, 0, (const uint8_t *) &current_estimate,
                                              data_timestamp);

            state_estimate_debug_t debug;
            kalman_get_state_debug(&debug);

            messaging_producer_send_timestamp(&messaging_producer_debug, message_flags_dont_send_over_usb, (const uint8_t *) &debug,
                                              data_timestamp);
        }

    } else if (packet->header.id == ts_ublox_nav) {
        // auto data = telemetry_get_payload<ublox_nav_t>(packet);
        //auto v_mag = sqrtf(data->velE * data->velE + data->velN * data->velN);
        //if (data->fix_type != 3 || data->num_sv < 8 || data->p_dop > 300 || data->v_acc > v_mag / 2.0f)
        //    return true;

        // TODO: Complete me!

    }
    return true;
}

#ifdef MESSAGING_OS_STD

void state_estimate_terminate() {
    messaging_consumer_terminate(&messaging_consumer);
}

#endif