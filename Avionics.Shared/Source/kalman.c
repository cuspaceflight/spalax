#include "kalman.h"
#include "math_utils.h"

typedef struct {
    union {
        struct {
            float attitude_err[3];
            float ang_vel[3];
            float ang_accel[3];
            float gyro_bias[3];
        };
        float state_vector[12];
    };
    
    float covariance_diag[12];
} kalman_state;

void print_kalman_state(kalman_state* state) {
    PRINT("State = {");
    for (int i = 0; i < 12; i++) {
        PRINT("%.2f, ", state->state_vector[i]);
    }
    PRINT("} Variance = {");
    for (int i = 0; i < 12; i++) {
        PRINT("%.2f, ", state->covariance_diag[i]);
    }
    PRINT("}\n");
}

void reset_kalman_state(kalman_state* state) {
    for (int i = 0; i < 3; i++) {
        state->attitude_err[i] = 0;
        state->ang_vel[i] = 0;
        state->ang_accel[i] = 0;
        state->gyro_bias[i] = 0;
    }

    // attitude error
    state->covariance_diag[0] = 0.616850275068084f;
    state->covariance_diag[1] = 0.616850275068084f;
    state->covariance_diag[2] = 0.616850275068084f;

    //angular velocity
    state->covariance_diag[3] = 4;
    state->covariance_diag[4] = 4;
    state->covariance_diag[5] = 4;

    //angular acceleration
    state->covariance_diag[6] = 25;
    state->covariance_diag[7] = 25;
    state->covariance_diag[8] = 25;

    //gyro bias
    state->covariance_diag[9] = 1;
    state->covariance_diag[10] = 1;
    state->covariance_diag[11] = 1;
}

void reset_quaternion(float q[4]) {
    q[0] = 0;
    q[1] = 0;
    q[2] = 0;
    q[3] = 1;
}

kalman_state prior_state;
float prior_attitude[4];

float g_reference[3];
float b_reference[3];
const float process_noise_diag[12] = {
    1e-5f, 1e-5f, 1e-5f, // attitude
    3e-5f, 3e-5f, 3e-5f, // angular velocity
    1e-5f, 1e-5f, 1e-5f, // angular acceleration roll, pitch, yaw
    1e-7f, 1e-7f, 1e-7f // gyro bias 
};

float accelerometer_covariance[3][3] = {
    {0.25f, 0, 0},
    {0, 0.25f, 0},
    {0,0,0.25f}
};

void kalman_init(const float g_ref[3], const float b_ref[3]) {
    kalman_set_reference_vectors(g_ref, b_ref);
    
    reset_kalman_state(&prior_state);
    reset_quaternion(prior_attitude);
}

void kalman_set_reference_vectors(const float accel_ref[3], const float mag_ref[3]) {
    for (int i = 0; i < 3; i++) {
        g_reference[i] = accel_ref[i];
        b_reference[i] = mag_ref[i];
    }
}

void predict_attitude(float dt, kalman_state* state, float attitude[4]) {
    float omega_mag = vector_mag(state->ang_vel);
    
    if (omega_mag > 1e-8f) {
        float omega_norm[3];
        float omega_q[4];
        for (int i = 0; i < 3; i++)
            omega_norm[i] = state->ang_vel[i] / omega_mag;
        axis_angle_to_quat(omega_norm, omega_mag*dt, omega_q);

        float attitude_temp[4];

        quat_mult(omega_q, attitude, attitude_temp);

        for (int i = 0; i < 4; i++)
            attitude[i] = attitude_temp[i];
    }
}

void update_attitude(kalman_state* state, const float attitude[4], float attitude_out[4]) {
    float q_temp[4];
    rodrigues_to_quaternion(state->attitude_err, q_temp);
    quat_mult(q_temp, attitude, attitude_out);
}

void mrp_application_jacobian(const float mrp_vector[3], const float target_vector[3], float J[3][3]) {
    // This isn't numerically stable
    // TODO: do this properly
    float v0[3];
    float v1[3];
    float mrp_q[4];
    rodrigues_to_quaternion(mrp_vector, mrp_q);
    apply_q(mrp_q, target_vector, v0);
    const float epsilon = 1e-3f;

    for (int i = 0; i < 3; i++) {
        float altered_mrp_vector[3];
        for (int j = 0; j < 3; j++)
            altered_mrp_vector[j] = mrp_vector[j];
        altered_mrp_vector[i] += epsilon;
        float altered_mrp_q[4];
        rodrigues_to_quaternion(altered_mrp_vector, altered_mrp_q);

        apply_q(altered_mrp_q, target_vector, v1);
        for (int j = 0; j < 3; j++)
            J[i][j] = (v1[j] - v0[j]) / epsilon;
    }
}

void q_target_jacobian(const float q[4], const float target_vector[3], float J[3][3]) {
    // This isn't numerically stable
    // TODO: do this properly
    float v0[3];
    float v1[3];
    apply_q(q, target_vector, v0);
    const float epsilon = 1e-3f;

    for (int i = 0; i < 3; i++) {
        float altered_target_vector[3];
        for (int j = 0; j < 3; j++)
            altered_target_vector[j] = target_vector[j];
        altered_target_vector[i] += epsilon;
        apply_q(q, altered_target_vector, v1);
        for (int j = 0; j < 3; j++)
            J[i][j] = (v1[j] - v0[j]) / epsilon;
    }
}

void h_accel(kalman_state* state, const float q[4], float h_accel[3]) {
    float q_temp[4];
    float q_actual[4];
    rodrigues_to_quaternion(state->attitude_err, q_temp);
    quat_mult(q_temp, q, q_actual);
    apply_q(q_actual, g_reference, h_accel);
}

void h_accel_jacobian(kalman_state* state, const float q[4], float J[3][12]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 12; j++)
            J[i][j] = 0;

    float J_3x3[3][3];
    float g_prime[3];
    apply_q(q, g_reference, g_prime);
    mrp_application_jacobian(state->attitude_err, g_prime, J_3x3);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            J[i][j] = J_3x3[i][j];
}

void h_gyro(kalman_state* state, const float q[4], float h_gyro[3]) {
    float q_temp[4];
    float q_prime[4];
    rodrigues_to_quaternion(state->attitude_err, q_temp);
    quat_mult(q_temp, q, q_prime);

    apply_q(q_prime, state->ang_vel, h_gyro);
    for (int i = 0; i < 3; i++)
        h_gyro[i] += state->gyro_bias[i];
}

void h_gyro_jacobian(kalman_state* state, const float q[4], float J[3][12]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 12; j++)
            J[i][j] = 0;

    float J_3x3[3][3];
    float omega_prime[3];
    apply_q(q, state->ang_vel, omega_prime);
    
    float q_temp[4];
    float q_actual[4];

    rodrigues_to_quaternion(state->attitude_err, q_temp);
    quat_mult(q_temp, q, q_actual);
    q_target_jacobian(q_actual, state->ang_vel, J_3x3);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            J[i][j+3] = J_3x3[i][j];

    mrp_application_jacobian(state->attitude_err, omega_prime, J_3x3);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            J[i][j] = J_3x3[i][j];

    for (int i = 0; i < 3; i++)
        for (int j = 9; j < 12; j++)
            J[i][j] = 0;
    J[0][9] = 1;
    J[1][10] = 1;
    J[2][11] = 1;
}

void h_mag(kalman_state* state, const float q[4], float h_mag[3]) {
    float q_temp[4];
    float q_prime[4];
    rodrigues_to_quaternion(state->attitude_err, q_temp);
    quat_mult(q_temp, q, q_prime);

    apply_q(q_prime, b_reference, h_mag);
}

void h_mag_jacobian(kalman_state* state, const float q[4], float J[3][12]) {
    float J_3x3[3][3];
    float b_prime[3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 12; j++)
            J[i][j] = 0;

    apply_q(q, b_reference, b_prime);
    mrp_application_jacobian(state->attitude_err, b_prime, J_3x3);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            J[i][j] = J_3x3[i][j];

}



void kalman_predict(state_estimate_t* next_estimate, float dt) {
    kalman_state post_state = prior_state;
    float dt2 = dt * dt;

    for (int i = 0; i < 3; i++) {
        post_state.ang_vel[i] += dt * prior_state.ang_accel[i];
        post_state.attitude_err[i] = 0;
        post_state.ang_accel[i] = 0;
    }

    for (int i = 0; i < 12; i++) {
        post_state.covariance_diag[i] += dt * process_noise_diag[i];
    }

    post_state.covariance_diag[3] += dt2 * prior_state.covariance_diag[6];
    post_state.covariance_diag[4] += dt2 * prior_state.covariance_diag[7];
    post_state.covariance_diag[5] += dt2 * prior_state.covariance_diag[8];
    
    predict_attitude(dt, &prior_state, prior_attitude);
    update_attitude(&prior_state, prior_attitude, next_estimate->orientation_q);
    for (int i = 0; i < 4; i++) {
        prior_attitude[i] = next_estimate->orientation_q[i];
    }


    prior_state = post_state;
    
    for (int i = 0; i < 3; i++) {
        next_estimate->angular_velocity[i] = prior_state.ang_vel[i];
    }
}

void do_update(const float y[3], float J[3][12], float sensor_covariance[3][3]) {
    float S[3][3];

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = sensor_covariance[i][j];

    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 3; j++) {
            S[j][0] += J[j][i] * prior_state.covariance_diag[i] * J[0][i];
            S[j][1] += J[j][i] * prior_state.covariance_diag[i] * J[1][i];
            S[j][2] += J[j][i] * prior_state.covariance_diag[i] * J[2][i];
        }
    }

    float K[12][3];

    // TODO use method that isn't vulnerable to singularities
    float s_inverse[3][3];
    mat3x3_inv(S, s_inverse);

    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = prior_state.covariance_diag[i] * (J[0][i] * s_inverse[0][j] + J[1][i] * s_inverse[1][j] + J[2][i] * s_inverse[2][j]);
        }
    }


    float post_attitude[4];
    kalman_state post_state = prior_state;
    for (int i = 0; i < 12; i++) {
        post_state.covariance_diag[i] -= prior_state.covariance_diag[i] * (J[0][i] * K[i][0] + J[1][i] * K[i][1] + J[2][i] * K[i][2]);
        post_state.state_vector[i] += K[i][0] * y[0] + K[i][1] * y[1] + K[i][2] * y[2];
    }

    update_attitude(&post_state, prior_attitude, post_attitude);

    for (int i = 0; i < 3; i++)
        post_state.attitude_err[i] = 0;

    prior_state = post_state;
    for (int i = 0; i < 4; i++)
        prior_attitude[i] = post_attitude[i];
}

void kalman_new_accel(const float accel[3]) {
    float predicted_measurement[3];
    h_accel(&prior_state, prior_attitude, predicted_measurement);
    float y[3];
    for (int i = 0; i < 3; i++)
        y[i] = accel[i] - predicted_measurement[i];

    float J[3][12];
    h_accel_jacobian(&prior_state, prior_attitude, J);
    do_update(y, J, accelerometer_covariance);
}

void kalman_new_pressure(float pressure) {
    // Pure orientation filter atm - so not used
}

void kalman_new_mag(const float mag[3]) {
    float predicted_measurement[3];
    h_mag(&prior_state, prior_attitude, predicted_measurement);
    float y[3];
    for (int i = 0; i < 3; i++)
        y[i] = mag[i] - predicted_measurement[i];

    float J[3][12];
    h_mag_jacobian(&prior_state, prior_attitude, J);
    do_update(y, J, accelerometer_covariance);
}

void kalman_new_gyro(const float gyro[3]) {
    float predicted_measurement[3];
    h_gyro(&prior_state, prior_attitude, predicted_measurement);
    float y[3];
    for (int i = 0; i < 3; i++)
        y[i] = gyro[i] - predicted_measurement[i];

    float J[3][12];
    h_gyro_jacobian(&prior_state, prior_attitude, J);
    do_update(y, J, accelerometer_covariance);
}