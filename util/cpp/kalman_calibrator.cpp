#include <messaging_all.h>
#include <cpp_utils.h>
#include <config/component_state_config.h>
#include <component_state.h>
#include <matplotlibcpp.h>
#include "Eigen/Core"
#include <Eigen/Geometry>
#include <util/board_config.h>
#include "data_extractor.h"
#include "state/kalman_constants.h"
#include "data_plotter.h"

#define DEBUG_LOGGING 0

namespace plt = matplotlibcpp;
using namespace Eigen;

static const char *input;

void update_handler(avionics_component_t component, avionics_component_state_t state, int line) {
    if (state == state_error)
        fprintf(stderr, "Error in component %i with line %i\n", component, line);
}


template<typename T>
float mean(T a_begin, T a_end) {
    auto a_i = a_begin;

    float mean = 0;
    int i = 0;

    while (a_i != a_end) {
        mean += *a_i++;
        i++;
    }

    return mean / (float) i;
}


template<typename T>
float delta_mean(T a_begin, T a_end, T b_begin, T b_end) {
    auto a_i = a_begin;
    auto b_i = b_begin;

    float mean = 0;
    int i = 0;

    while (a_i != a_end && b_i != b_end) {
        mean += *a_i++ - *b_i++;
        i++;
    }

    return mean / (float) i;
}

template<typename T>
float angle_delta_mean(T a_begin, T a_end, T b_begin, T b_end) {
    auto a_i = a_begin;
    auto b_i = b_begin;

    float mean = 0;
    int i = 0;

    while (a_i != a_end && b_i != b_end) {
        mean += std::min(std::min(*a_i - *b_i, *a_i + 360.0f - *b_i), *a_i - *b_i - 360.0f);
        i++;
        a_i++;
        b_i++;
    }

    return mean / (float) i;
}


float compute_score() {
    run_data_extractor(input);

    float score = 0;

    // The length of time to take averages over
    const int average_length = 100;
    // The number of samples to allow filter to settle
    const int settle_time = 1000;

    // We want the acceleration at the start to be similar to the acceleration at the end

    score += std::abs(delta_mean(se_accel_x.begin() + settle_time, se_accel_x.begin() + settle_time + average_length,
                        se_accel_x.end() - average_length, se_accel_x.end()));

    score += std::abs(delta_mean(se_accel_y.begin() + settle_time, se_accel_y.begin() + settle_time + average_length,
                        se_accel_y.end() - average_length, se_accel_y.end()));

    score += std::abs(delta_mean(se_accel_z.begin() + settle_time, se_accel_z.begin() + settle_time + average_length,
                        se_accel_z.end() - average_length, se_accel_z.end()));

    // We want the acceleration at the start to be similar to the acceleration at 21 seconds

    score += std::abs(delta_mean(se_accel_x.begin() + settle_time, se_accel_x.begin() + settle_time + average_length,
                                 se_accel_x.begin() + 21000, se_accel_x.begin() + 21000 + average_length));

    score += std::abs(delta_mean(se_accel_y.begin() + settle_time, se_accel_y.begin() + settle_time + average_length,
                                 se_accel_y.begin() + 21000, se_accel_y.begin() + 21000 + average_length));

    score += std::abs(delta_mean(se_accel_z.begin() + settle_time, se_accel_z.begin() + settle_time + average_length,
                                 se_accel_z.begin() + 21000, se_accel_z.begin() + 21000 + average_length));

    // We want the rotation at the start to be similar to the acceleration at the end

    score += std::abs(angle_delta_mean(se_rotation_x.begin() + settle_time, se_rotation_x.begin() + settle_time + average_length,
                                 se_rotation_x.end() - average_length, se_rotation_x.end())) / 360.0f;

    score += std::abs(angle_delta_mean(se_rotation_y.begin() + settle_time, se_rotation_y.begin() + settle_time + average_length,
                                 se_rotation_y.end() - average_length, se_rotation_y.end())) / 360.0f;

    score += std::abs(angle_delta_mean(se_rotation_z.begin() + settle_time, se_rotation_z.begin() + settle_time + average_length,
                                 se_rotation_z.end() - average_length, se_rotation_z.end())) / 360.0f;


    // We want the acceleration at the end to be roughly constant

    score += std::abs(delta_mean(se_accel_x.end() - 2000, se_accel_x.end() - 2000 + average_length,
                                 se_accel_x.end() - average_length, se_accel_x.end()));

    score += std::abs(delta_mean(se_accel_y.end() - 2000, se_accel_y.end() - 2000 + average_length,
                                 se_accel_y.end() - average_length, se_accel_y.end()));

    score += std::abs(delta_mean(se_accel_z.end() - 2000, se_accel_z.end() - 2000 + average_length,
                                 se_accel_z.end() - average_length, se_accel_z.end()));

    return score;
}

#define NUM_TUNING_CONSTANTS 8

float *tuning_constants[NUM_TUNING_CONSTANTS] = {
        &kalman_magno_cov,
        &kalman_accelerometer_cov,
        &kalman_gyro_cov,
        &angular_vel_process_noise,
        &attitude_err_process_noise,
        &gyro_bias_process_noise,
        &accel_bias_process_noise,
        &magno_bias_process_noise
};

const float initial_delta_mult = 1.f;
const float min_delta_mult = 1 / 128.f;
const float delta_mult_factor = 0.5f;

float delta_mult = initial_delta_mult;

float compute_derivatives(float pos_derivatives[NUM_TUNING_CONSTANTS], float neg_derivatives[NUM_TUNING_CONSTANTS]) {
    float original_score = compute_score();

    for (int i = 0; i < NUM_TUNING_CONSTANTS; i++) {
        float delta = std::max(delta_mult * *tuning_constants[i], 1e-9f * delta_mult);

        float original = *tuning_constants[i];
        *tuning_constants[i] += delta;

        float score1 = compute_score();

        *tuning_constants[i] = std::max(original - delta, 0.f);

        float score2 = compute_score();

        pos_derivatives[i] = score1 - original_score;

        neg_derivatives[i] = score2 - original_score;

        *tuning_constants[i] = original;
    }

    return original_score;
}

int min_index(float derivative[NUM_TUNING_CONSTANTS]) {
    float min = std::numeric_limits<float>::max();
    int index = 0;

    for (int i = 0; i < NUM_TUNING_CONSTANTS; i++) {
        if (!isnan(derivative[i]) && derivative[i] < min) {
            index = i;
            min = derivative[i];
        }
    }
    return index;
}

void print_constants() {
    std::cout << "Constants: ";
    for (int i = 0; i < NUM_TUNING_CONSTANTS; i++) {
        std::cout << *tuning_constants[i] << ", ";
    }
    std::cout << std::endl;
}

int main(int argc, char *argv[]) {
    if (argc < 2 || argc % 2 != 0) {
        std::cerr << "Invalid arguments - expected <Input> [<Output> <Format>]...";
        return 1;
    }
    input = argv[1];

    setBoardConfig(BoardConfigSpalax);

    component_state_start(update_handler, false);
    messaging_all_start_options(false, false);

    float reset = -1;
    int delta_mult_index = 0;

    while (true) {
        float pos_derivative[NUM_TUNING_CONSTANTS];
        float neg_derivative[NUM_TUNING_CONSTANTS];


        float score = compute_derivatives(pos_derivative, neg_derivative);
        std::cout << "Score: " << score << std::endl;

#if DEBUG_LOGGING
        std::cout << "Pos Derivates: ";
        for (int i = 0; i < NUM_TUNING_CONSTANTS; i++) {
            std::cout << pos_derivative[i] << ", ";
        }
        std::cout << std::endl;

        std::cout << "Neg Derivates: ";
        for (int i = 0; i < NUM_TUNING_CONSTANTS; i++) {
            std::cout << neg_derivative[i] << ", ";
        }
        std::cout << std::endl;
#endif

        auto min_i_pos = min_index(pos_derivative);
        auto min_i_neg = min_index(neg_derivative);

        if (neg_derivative[min_i_neg] < -1e-3f || pos_derivative[min_i_pos] < -1e-3f) {
            reset = -1;
            if (neg_derivative[min_i_neg] < pos_derivative[min_i_pos]) {
                float delta = std::max(delta_mult * *tuning_constants[min_i_neg], 1e-9f * delta_mult);
                *tuning_constants[min_i_neg] = std::max(*tuning_constants[min_i_neg] -delta, 0.f);
#if DEBUG_LOGGING
                std::cout << "Subtracting Index: " << min_i_neg << std::endl;
#endif
            } else {
                float delta = std::max(delta_mult * *tuning_constants[min_i_pos], 1e-9f * delta_mult);
                *tuning_constants[min_i_pos] += delta;
#if DEBUG_LOGGING
                std::cout << "Adding Index: " << min_i_pos << std::endl;
#endif
            }
        } else if (reset == delta_mult_index) {
            print_constants();
            break;
        } else {
            if (reset == -1)
                reset = delta_mult_index;

            if (delta_mult < min_delta_mult) {
                delta_mult = initial_delta_mult;
                delta_mult_index = 0;
            } else {
                delta_mult *= delta_mult_factor;
                delta_mult_index++;
            }
        }

        print_constants();
    }

    plot_data(argc, argv);
    return 0;
}
