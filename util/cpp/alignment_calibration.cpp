#include <messaging_all.h>
#include <file_telemetry.h>
#include <cpp_utils.h>
#include <config/component_state_config.h>
#include <component_state.h>
#include <iostream>
#include <vector>
#include <config/telemetry_packets.h>
#include <calibration/ms5611_calibration.h>
#include <time_utils.h>
#include <matplotlibcpp.h>
#include <chrono>
#include <thread>
#include <algorithm>
#include "Eigen/Core"
#include <Eigen/Geometry>
#include <state/quest.h>
#include <calibration/mpu9250_calibration.h>
#include <util/board_config.h>


namespace plt = matplotlibcpp;
using namespace Eigen;


static const int num_samples = 10000;


static uint64_t mpu_timestamp = 0;
static uint32_t last_mpu_timestamp = 0;

static std::vector<Vector3f> accel_all;
static std::vector<Vector3f> magno_all;
static std::vector<float> mpu_timestamps;

static bool getPacket(const telemetry_t *packet, message_metadata_t metadata) {
    if (packet->header.id == ts_mpu9250_data) {
        if (last_mpu_timestamp == 0) {
            last_mpu_timestamp = packet->header.timestamp;
        }

        auto delta = clocks_between(last_mpu_timestamp, packet->header.timestamp);
        mpu_timestamp += delta;
        last_mpu_timestamp = packet->header.timestamp;

        auto data = telemetry_get_payload<mpu9250_data_t>(packet);
        mpu9250_calibrated_data_t calibrated_data;
        mpu9250_calibrate_data(data, &calibrated_data);

        mpu_timestamps.push_back((float) mpu_timestamp / (float) platform_get_counter_frequency());
        accel_all.push_back(Eigen::Map<const Vector3f>(calibrated_data.accel));
        magno_all.push_back(Eigen::Map<const Vector3f>(calibrated_data.magno));
    }
    return true;
}

MESSAGING_CONSUMER(messaging_consumer, ts_all, ts_all_mask, 0, 0, getPacket, 1024);

void update_handler(avionics_component_t component, avionics_component_state_t state, int line) {
    if (state == state_error)
        fprintf(stderr, "Error in component %i with line %i\n", component, line);
}

template <int samples>
Quaternionf qmethod(const std::vector<Vector3f>& observations_eigen, const std::vector<Vector3f>& references_eigen) {
    float references[samples][3];
    float observations[samples][3];
    float a[samples];

    for (int i = 0; i < samples; i++) {
        a[i] = 1.0f / samples;
        for (int j = 0; j < 3; j++) {
            references[i][j] = references_eigen[i][j];
            observations[i][j] = observations_eigen[i][j];
        }
    }

    float q[4];
    davenport_q_method(observations, references, a, samples, q);
    return Quaternionf(q[3], q[0], q[1], q[2]);
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Invalid arguments - expected <Input>";
        return 1;
    }
    const char *input = argv[1];

    setBoardConfig(BoardConfigSpalax);

    component_state_start(update_handler, false);
    messaging_all_start_options(false, false);

    messaging_consumer_init(&messaging_consumer);

    file_telemetry_input_start(input);

    // The file_telemetry_input can disconnect but there may still be packets in the consumers buffer from when we slept
    while (file_telemetry_input_connected() ||
           messaging_consumer_receive(&messaging_consumer, false, false) == messaging_receive_ok) {
        // We have to "busy" wait as the consumer has no way to know when no further packets are going to arrive
        // A better solution would be to be able to specify a timeout but this hasn't been implemented
        while (messaging_consumer_receive(&messaging_consumer, false, false) == messaging_receive_ok);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


    Vector3f accel_reference(0, 0, 1);
    Vector3f magno_reference(0.39134267f, -0.00455851434f, -0.920233727f);


    Vector3f accel_calibration = Vector3f::Zero();
    Vector3f magno_calibration = Vector3f::Zero();


    // We initialize the references based on the first 100 samples
    for (int i = 0; i < 100; i++) {
        accel_calibration += accel_all[i];
        magno_calibration += magno_all[i];
    }
    accel_calibration.normalize();
    magno_calibration.normalize();


    Vector3f rot_vector = accel_reference.cross(magno_reference).normalized();
    float angle = std::acos(accel_calibration.normalized().transpose() * magno_calibration.normalized());
    magno_reference = Eigen::Quaternionf(Eigen::AngleAxisf(angle, rot_vector)) * accel_reference;



    Vector3f magno_s[num_samples];
    Vector3f accel_s[num_samples];


    // We select a portion of samples from those available
    for (int i = 0; i < num_samples; i++) {
        size_t stride = std::max<size_t>(magno_all.size() / num_samples, 1);
        size_t index = stride * i;
        accel_s[i] = accel_all[index];
        magno_s[i] = magno_all[index];
    }


    std::vector<Vector3f> BS_estimate;
    std::vector<Vector3f> BSE;

    int k;
    Matrix3f misalignment = Matrix3f::Identity();
    Matrix3f last_misalignment = misalignment;

    const int max_iterations = 1000;

    for (k = 0; k < max_iterations; k++) {
        BS_estimate.clear();
        BSE.clear();

        for (auto magno : magno_s)
            BS_estimate.push_back(misalignment * magno);

        for (int i = 0; i < num_samples; i++) {
            std::vector<Vector3f> observations;
            std::vector<Vector3f> references;

            observations.push_back(accel_s[i]);
            observations.push_back(BS_estimate[i]);

            references.push_back(accel_s[i].norm() * accel_reference);
            references.push_back(BS_estimate[i].norm() * magno_reference);

            // This is the quaternion that rotates the observations onto the references
            Quaternionf quat = qmethod<2>(observations, references);
            Vector3f BSEi = quat.inverse() * magno_reference;
            BSE.push_back(BSEi);
        }


        // This is the quaternion that rotates BSE onto BS_estimate
        Quaternionf quat = qmethod<num_samples>(BSE, BS_estimate);

        misalignment = Matrix3f(quat.inverse()) * misalignment;

        float err = (misalignment - last_misalignment).norm();

        last_misalignment = misalignment;

        // With single precision floating point we can't get better
        if (err < 1e-6f)
            break;

    }

    if (k == max_iterations)
        std::cout << "Failed to converge" << std::endl;

    AngleAxisf correction(misalignment);

    printf("Correction of %f radians\n",correction.angle());

    auto config = getBoardConfig();

    Eigen::Matrix3f magno_transform = Eigen::Map<Eigen::Matrix<float,3,3,Eigen::RowMajor>>(config->mpu9250_magno_transform);

    Eigen::Matrix3f new_transform = misalignment * magno_transform;

    IOFormat HeavyFmt(FullPrecision, 0, "f, ", "f,\n", "", "", "{", "f},");

    std::cout <<".mpu9250_magno_transform = "<< new_transform.format(HeavyFmt) << std::endl;

    return 0;
}
