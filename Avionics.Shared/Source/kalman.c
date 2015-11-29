#include "kalman.h"
#include "math_utils.h"

typedef struct {
    float attitude_err[3];
    float ang_vel[3];
    float ang_accel[3];
    float gyro_bias[3];
} kalman_state;

// TODO: Set these
float g_reference[3];
float b_reference[3];

kalman_state predict_forward(float dt, kalman_state* prev) {
    kalman_state ret = *prev;
    
    for (int i = 0; i < 3; i++) {
        ret.ang_vel[i] += dt * ret.ang_accel[i];
        ret.attitude_err[i] = 0;
        ret.ang_accel[i] = 0;
    }

    return ret;
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

        apply_q(altered_mrp_vector, target_vector, v1);
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
    float J_3x3[3][3];
    float omega_prime[3];
    apply_q(q, state->ang_vel, omega_prime);
    
    float q_temp[4];
    float q_actual[4];

    rodrigues_to_quaternion(state->attitude_err, q_temp);
    quat_mult(q_temp, q, q_actual);
    q_target_jacobian(q_actual, state->attitude_err, J_3x3);
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
    apply_q(q, b_reference, b_prime);
    mrp_application_jacobian(state->attitude_err, b_prime, J_3x3);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            J[i][j] = J_3x3[i][j];

}


void kalman_new_accel(const float accel[3]) {
}

void kalman_new_pressure(float pressure) {
}

void kalman_new_mag(const float mag[3]) {
}

void kalman_new_gyro(const float gyro[3]) {
}

void kalman_predict(state_estimate_t* next_estimate, float dt) {
}
