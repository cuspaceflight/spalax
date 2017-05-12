#include <messaging_all.h>
#include <cpp_utils.h>
#include <config/component_state_config.h>
#include <component_state.h>
#include <matplotlibcpp.h>
#include "Eigen/Core"
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
fp mean(T a_begin, T a_end) {
    auto a_i = a_begin;

    fp mean = 0;
    int i = 0;

    while (a_i != a_end) {
        mean += *a_i++;
        i++;
    }

    return mean / (fp) i;
}


template<typename T>
fp delta_mean(T a_begin, T a_end, T b_begin, T b_end) {
    auto a_i = a_begin;
    auto b_i = b_begin;

    fp mean = 0;
    int i = 0;

    while (a_i != a_end && b_i != b_end) {
        mean += *a_i++ - *b_i++;
        i++;
    }

    return mean / (fp) i;
}

template<typename T>
fp angle_delta_mean(T a_begin, T a_end, T b_begin, T b_end) {
    auto a_i = a_begin;
    auto b_i = b_begin;

    fp mean = 0;
    int i = 0;

    while (a_i != a_end && b_i != b_end) {
        mean += std::min(std::min(*a_i - *b_i, *a_i + 360.0f - *b_i), *a_i - *b_i - 360.0f);
        i++;
        a_i++;
        b_i++;
    }

    return mean / (fp) i;
}


fp compute_score() {
    DataExtractor de;

    de.se_rotation_x.enabled = true;
    de.se_rotation_y.enabled = true;
    de.se_rotation_z.enabled = true;

    de.se_accel_x.enabled = true;
    de.se_accel_y.enabled = true;
    de.se_accel_z.enabled = true;
    de.se_accel_norm.enabled = true;

    run_data_extractor(input, true, &de);

    fp score = 0;

    // The length of time to take averages over
    const int average_length = 100;
    // The number of samples to allow filter to settle
    const int settle_time = 1000;

    // We want the acceleration at the start to be similar to the acceleration at the end

    score += std::abs(delta_mean(de.se_accel_x.begin() + settle_time, de.se_accel_x.begin() + settle_time + average_length,
                                 de.se_accel_x.end() - average_length, de.se_accel_x.end()));

    score += std::abs(delta_mean(de.se_accel_y.begin() + settle_time, de.se_accel_y.begin() + settle_time + average_length,
                                 de.se_accel_y.end() - average_length, de.se_accel_y.end()));

    score += std::abs(delta_mean(de.se_accel_z.begin() + settle_time, de.se_accel_z.begin() + settle_time + average_length,
                                 de.se_accel_z.end() - average_length, de.se_accel_z.end()));

    // We want the mean acceleration to be as small as possible
    score += mean(de.se_accel_norm.begin(), de.se_accel_norm.end()) * 2;

    return score;
}

#define NUM_TUNING_CONSTANTS 2

fp *tuning_constants[NUM_TUNING_CONSTANTS] = {
        &magno_bias_process_noise,
        &accel_bias_process_noise
};

const fp initial_delta_mult = 1.f;
const fp min_delta_mult = 1 / 128.f;
const fp delta_mult_factor = 0.5f;

fp delta_mult = initial_delta_mult;

fp compute_derivatives(fp pos_derivatives[NUM_TUNING_CONSTANTS], fp neg_derivatives[NUM_TUNING_CONSTANTS]) {
    fp original_score = compute_score();

    for (int i = 0; i < NUM_TUNING_CONSTANTS; i++) {
        fp delta = std::max(delta_mult * *tuning_constants[i], 1e-8f * delta_mult);

        fp original = *tuning_constants[i];
        *tuning_constants[i] += delta;

        fp score1 = compute_score();

        *tuning_constants[i] = std::max<fp>(original - delta, 0.f);

        fp score2 = compute_score();

        pos_derivatives[i] = score1 - original_score;

        neg_derivatives[i] = score2 - original_score;

        *tuning_constants[i] = original;
    }

    return original_score;
}

int min_index(fp derivative[NUM_TUNING_CONSTANTS]) {
    fp min = std::numeric_limits<fp>::max();
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

    fp reset = -1;
    int delta_mult_index = 0;

    while (true) {
        fp pos_derivative[NUM_TUNING_CONSTANTS];
        fp neg_derivative[NUM_TUNING_CONSTANTS];


        fp score = compute_derivatives(pos_derivative, neg_derivative);
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
                fp delta = std::max(delta_mult * *tuning_constants[min_i_neg], 1e-9f * delta_mult);
                *tuning_constants[min_i_neg] = std::max<fp>(*tuning_constants[min_i_neg] -delta, 0.f);
#if DEBUG_LOGGING
                std::cout << "Subtracting Index: " << min_i_neg << std::endl;
#endif
            } else {
                fp delta = std::max(delta_mult * *tuning_constants[min_i_pos], 1e-9f * delta_mult);
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

    std::cout << "Generating Graph Data" << std::endl;

    DataExtractor extractor;

    enable_streams(argc, argv, &extractor);

    run_data_extractor(input, true, &extractor);

    plot_data(argc, argv, &extractor);
    return 0;
}
