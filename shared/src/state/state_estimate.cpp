#include <messaging.h>
#include <config/telemetry_packets.h>
#include <calibration/mpu9250_calibration.h>
#include <Eigen/Geometry>
#include <time_utils.h>
#include <component_state.h>
#include "state_estimate.h"
#include "Eigen/Core"
#include <Eigen/Geometry>
#include <calibration/adis16405_calibration.h>
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

#define NUM_CALIBRATION_SAMPLES 10000
static int remaining_calibration_samples;
static Matrix<float, 3, 1> magno_calibration;
static Matrix<float, 3, 1> accel_calibration;
static Matrix<float, 3, 1> gyro_calibration;

static const Vector3f accel_reference(0, 0, 1);
static const Vector3f magno_reference(0.39134267f, -0.00455851434f, -0.920233727f);

static uint32_t data_timestamp = 0;

static float gyro_norm_exp_avg = 1;
static float accel_norm_exp_avg = 1;
static float magno_norm_exp_avg = 1;
static float accel_norm_exp_avg_sq = 1;
static float expected_accel_norm = 0;

static const float exp_avg_alpha = 0.99f;


#define USE_ADIS 1

MESSAGING_PRODUCER(messaging_producer, ts_state_estimate_data, sizeof(state_estimate_t), 100)
MESSAGING_PRODUCER(messaging_producer_debug, ts_state_estimate_debug, sizeof(state_estimate_debug_t), 100)
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
#if USE_ADIS
    if (packet->header.id == ts_adis16405_data) {


#else
        if (packet->header.id == ts_mpu9250_data) {

#endif

        if (data_timestamp == 0) {
            data_timestamp = packet->header.timestamp;
            return true;
        }
        float dt =
                (float) (clocks_between(data_timestamp, packet->header.timestamp)) / CLOCK_FREQUENCY;

        data_timestamp = packet->header.timestamp;

#if USE_ADIS
        auto data = telemetry_get_payload<adis16405_data_t>(packet);
        adis16405_calibrated_data_t calibrated_data;
        adis16405_calibrate_data(data, &calibrated_data);
#else
        auto data = telemetry_get_payload<mpu9250_data_t>(packet);
        mpu9250_calibrated_data_t calibrated_data;
        mpu9250_calibrate_data(data, &calibrated_data);
#endif
        if (state_estimate_phase == StateEstimatePhase::Calibration) {
            remaining_calibration_samples--;

            magno_calibration += Map<const Matrix<float, 3, 1>>(calibrated_data.magno);
            accel_calibration += Map<const Matrix<float, 3, 1>>(calibrated_data.accel);
            gyro_calibration += Map<const Matrix<float, 3, 1>>(calibrated_data.gyro);

            if (remaining_calibration_samples <= 0) {

                accel_calibration /= (float) NUM_CALIBRATION_SAMPLES;
                magno_calibration /= (float) NUM_CALIBRATION_SAMPLES;
                gyro_calibration /= (float) NUM_CALIBRATION_SAMPLES;

                // We adjust the magnetic reference vector so that the angle between the references
                // is the same as the angle between the observations
                Eigen::Vector3f rot_vector = accel_reference.cross(magno_reference).normalized();
                float angle = std::acos(accel_calibration.normalized().transpose() * magno_calibration.normalized());
                Eigen::Vector3f new_magno_reference =
                        Eigen::Quaternionf(Eigen::AngleAxisf(angle, rot_vector)) * accel_reference;

                const float quest_reference_vectors[2][3] = {
                        {accel_reference.x(),     accel_reference.y(),     accel_reference.z()},
                        {new_magno_reference.x(), new_magno_reference.y(), new_magno_reference.z()}
                };

                float accel_norm = accel_calibration.norm();
                float magno_norm = magno_calibration.norm();

                Vector3f accel_calib_norm = accel_calibration / accel_norm;
                Vector3f magno_calib_norm = magno_calibration / magno_norm;

                const float quest_observations[2][3] = {
                        {accel_calib_norm[0], accel_calib_norm[1], accel_calib_norm[2]},
                        {magno_calib_norm[0], magno_calib_norm[1], magno_calib_norm[2]},
                };

                const float a[2] = {0.5f, 0.5f};

                float initial_orientation[4] = {0, 0, 0, 1};
                fp initial_angular_velocity[3] = {0, 0, 0};
                fp initial_position[3] = {0, 0, 0};
                fp initial_velocity[3] = {0, 0, 0};
                fp initial_acceleration[3] = {0, 0, 0};
                fp initial_accel_bias[3] = {0, 0, 0};
                fp initial_magno_bias[3] = {0, 0, 0};
                fp initial_gyro_bias[3] = {
                        gyro_calibration.x(),
                        gyro_calibration.y(),
                        gyro_calibration.z()
                };

                if (quest_estimate(quest_observations, quest_reference_vectors, a, initial_orientation) == -1) {
                    COMPONENT_STATE_UPDATE(avionics_component_state_state_estimate, state_error);
                }

                const fp kalman_reference_vectors[2][3] = {
                        {accel_reference.x() * accel_norm,       accel_reference.y() * accel_norm,
                                accel_reference.z() * accel_norm},
                        {new_magno_reference.x() * magno_norm, new_magno_reference.y() * magno_norm,
                                new_magno_reference.z() * magno_norm}
                };

                const fp kalman_orientation[4] = {initial_orientation[0], initial_orientation[1], initial_orientation[2], initial_orientation[3]};

                expected_accel_norm = accel_norm;
                accel_norm_exp_avg = accel_norm;
                accel_norm_exp_avg_sq = accel_norm*accel_norm;

                kalman_init(kalman_reference_vectors[0], kalman_reference_vectors[1], kalman_orientation,
                            initial_angular_velocity, initial_position, initial_velocity, initial_acceleration,
                            initial_gyro_bias, initial_accel_bias, initial_magno_bias);

                state_estimate_phase = StateEstimatePhase::Estimation;
            }
        } else if (state_estimate_phase == StateEstimatePhase::Estimation) {
            kalman_predict(dt);

            fp gyro[3] = {calibrated_data.gyro[0], calibrated_data.gyro[1], calibrated_data.gyro[2]};
            fp magno[3] = {calibrated_data.magno[0], calibrated_data.magno[1], calibrated_data.magno[2]};
            fp accel[3] = {calibrated_data.accel[0], calibrated_data.accel[1], calibrated_data.accel[2]};

            kalman_new_gyro(gyro);
            kalman_new_magno(magno);
            kalman_new_accel(accel);


            state_estimate_t current_estimate;
            kalman_get_state(&current_estimate);
            state_estimate_debug_t debug;
            kalman_get_state_debug(&debug);

            gyro_norm_exp_avg = (1 - exp_avg_alpha) * (Map<const Matrix<float, 3, 1>>(calibrated_data.gyro) -
                                                       Map<const Matrix<float, 3, 1>>(debug.gyro_bias)).norm() +
                                exp_avg_alpha * gyro_norm_exp_avg;

            accel_norm_exp_avg = (1 - exp_avg_alpha) * (Map<const Matrix<float, 3, 1>>(calibrated_data.accel) -
                                                        Map<const Matrix<float, 3, 1>>(debug.accel_bias)).norm() +
                                 exp_avg_alpha * accel_norm_exp_avg;

            magno_norm_exp_avg = (1 - exp_avg_alpha) * (Map<const Matrix<float, 3, 1>>(calibrated_data.magno) -
                                                        Map<const Matrix<float, 3, 1>>(debug.magno_bias)).norm() +
                                 exp_avg_alpha * magno_norm_exp_avg;

            accel_norm_exp_avg_sq = (1 - exp_avg_alpha) * (Map<const Matrix<float, 3, 1>>(calibrated_data.accel) -
                                                        Map<const Matrix<float, 3, 1>>(debug.accel_bias)).squaredNorm() +
                                 exp_avg_alpha * accel_norm_exp_avg_sq;

            debug.gyro_norm_exp_avg = gyro_norm_exp_avg;
            debug.accel_norm_exp_avg = accel_norm_exp_avg;
            debug.magno_norm_exp_avg = magno_norm_exp_avg;

            debug.accel_exp_variance = accel_norm_exp_avg_sq - accel_norm_exp_avg * accel_norm_exp_avg;

            //if (accel_norm_exp_avg - expected_accel_norm < 0.01 && debug.accel_exp_variance < 0.5f) {
            //    kalman_zero_accel();
            //}

            messaging_producer_send_timestamp(&messaging_producer, message_flags_send_over_can, (const uint8_t *) &current_estimate,
                                              data_timestamp);

            messaging_producer_send_timestamp(&messaging_producer_debug, message_flags_dont_send_over_usb,
                                              (const uint8_t *) &debug,
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