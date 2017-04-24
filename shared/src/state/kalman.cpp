#include "kalman.h"
#include <cfloat>
#include "math_util.h"

using namespace Eigen;

static Matrix<fp, KALMAN_NUM_STATES, 1> prior_state_vector;
static Matrix<fp, KALMAN_NUM_STATES, KALMAN_NUM_STATES> P;
static Quaternion<fp> prior_attitude;

#define ATTITUDE_ERROR prior_state_vector.block<3,1>(KALMAN_ATTITUDE_ERR_IDX,0)
#define ANGULAR_VELOCITY prior_state_vector.block<3,1>(KALMAN_ANGULAR_VEL_IDX,0)
#define GYRO_BIAS prior_state_vector.block<3,1>(KALMAN_GYRO_BIAS_IDX,0)
#define POSITION prior_state_vector.block<3,1>(KALMAN_POSITION_IDX,0)
#define VELOCITY prior_state_vector.block<3,1>(KALMAN_VELOCITY_IDX,0)
#define ACCELERATION prior_state_vector.block<3,1>(KALMAN_ACCELERATION_IDX,0)
#define ACCEL_BIAS prior_state_vector.block<3,1>(KALMAN_ACCEL_BIAS_IDX,0)
#define MAGNO_BIAS prior_state_vector.block<3,1>(KALMAN_MAGNO_BIAS_IDX, 0)

static Matrix<fp, 3, 1> g_reference;
static Matrix<fp, 3, 1> b_reference;

static DiagonalMatrix<fp, 3> accelerometer_covariance;
static DiagonalMatrix<fp, 3> magno_covariance;
static DiagonalMatrix<fp, 3> gyro_covariance;

static DiagonalMatrix<fp, KALMAN_NUM_STATES> process_noise;

void kalman_init(const fp accel_reference[3], const fp magno_reference[3], const fp initial_orientation[4],
                 const fp initial_angular_velocity[3], const fp initial_position[3], const fp initial_velocity[3],
                 const fp initial_acceleration[3], const fp initial_gyro_bias[3], const fp initial_accel_bias[3],
                 const fp initial_magno_bias[3]) {
    for (int i = 0; i < KALMAN_NUM_STATES; i++) {
        prior_state_vector(i) = 0;
    }

    ANGULAR_VELOCITY = Map<const Matrix<fp, 3, 1>>(initial_angular_velocity);
    POSITION = Map<const Matrix<fp, 3, 1>>(initial_position);
    VELOCITY = Map<const Matrix<fp, 3, 1>>(initial_velocity);
    ACCELERATION = Map<const Matrix<fp, 3, 1>>(initial_acceleration);
    GYRO_BIAS = Map<const Matrix<fp, 3, 1>>(initial_gyro_bias);
    ACCEL_BIAS = Map<const Matrix<fp, 3, 1>>(initial_accel_bias);
    MAGNO_BIAS = Map<const Matrix<fp, 3, 1>>(initial_magno_bias);

    prior_attitude.x() = initial_orientation[0];
    prior_attitude.y() = initial_orientation[1];
    prior_attitude.z() = initial_orientation[2];
    prior_attitude.w() = initial_orientation[3];

    P = Matrix<fp, KALMAN_NUM_STATES, KALMAN_NUM_STATES>::Zero();

    for (int i = 0; i < 3; i++) {
        accelerometer_covariance.diagonal()[i] = kalman_accelerometer_cov;
        magno_covariance.diagonal()[i] = kalman_magno_cov;
        gyro_covariance.diagonal()[i] = kalman_gyro_cov;

        // Attitude Error
        P.diagonal()[i + KALMAN_ATTITUDE_ERR_IDX] = initial_attitude_err_cov;
        process_noise.diagonal()[i + KALMAN_ATTITUDE_ERR_IDX] = attitude_err_process_noise;
        // Angular Velocity
        P.diagonal()[i + KALMAN_ANGULAR_VEL_IDX] = initial_angular_vel_cov;
        process_noise.diagonal()[i + KALMAN_ANGULAR_VEL_IDX] = angular_vel_process_noise;
        // Position
        P.diagonal()[i + KALMAN_POSITION_IDX] = initial_position_cov;
        process_noise.diagonal()[i + KALMAN_POSITION_IDX] = position_process_noise;
        // Velocity
        P.diagonal()[i + KALMAN_VELOCITY_IDX] = initial_velocity_cov;
        process_noise.diagonal()[i + KALMAN_VELOCITY_IDX] = velocity_process_noise;
        // Acceleration
        P.diagonal()[i + KALMAN_ACCELERATION_IDX] = initial_acceleration_cov;
        process_noise.diagonal()[i + KALMAN_ACCELERATION_IDX] = acceleration_process_noise;
        // Accel Bias
        P.diagonal()[i + KALMAN_ACCEL_BIAS_IDX] = initial_accel_bias_cov;
        process_noise.diagonal()[i + KALMAN_ACCEL_BIAS_IDX] = accel_bias_process_noise;
        // Gyro Bias
        P.diagonal()[i + KALMAN_GYRO_BIAS_IDX] = initial_gyro_bias_cov;
        process_noise.diagonal()[i + KALMAN_GYRO_BIAS_IDX] = gyro_bias_process_noise;
        // Magno Bias
        P.diagonal()[i + KALMAN_MAGNO_BIAS_IDX] = initial_magno_bias_cov;
        process_noise.diagonal()[i + KALMAN_MAGNO_BIAS_IDX] = magno_bias_process_noise;

        g_reference[i] = accel_reference[i];
        b_reference[i] = magno_reference[i];
    }
}

// Moves the MRP attitude error into the prior_attitude quaternion
inline void update_attitude() {
    Quaternion<fp> delta = mrpToQuat(ATTITUDE_ERROR);
    Quaternion<fp> t = prior_attitude * delta;
    prior_attitude = t;

    ATTITUDE_ERROR = Matrix<fp, 3, 1>::Zero();
}

void kalman_get_state(state_estimate_t *state) {
    for (int i = 0; i < 3; i++) {
        state->angular_velocity[i] = ANGULAR_VELOCITY[i];
        state->position[i] = POSITION[i];
        state->velocity[i] = VELOCITY[i];
        state->acceleration[i] = ACCELERATION[i];
    }

    state->orientation_q[0] = prior_attitude.x();
    state->orientation_q[1] = prior_attitude.y();
    state->orientation_q[2] = prior_attitude.z();
    state->orientation_q[3] = prior_attitude.w();
}

static_assert(KALMAN_NUM_STATES == (sizeof(((state_estimate_debug_t *) 0)->P) / sizeof(float)),
              "KALMAN_NUM_STATES incorrect");

void kalman_get_state_debug(state_estimate_debug_t *state) {
    for (int i = 0; i < 3; i++) {
        state->magno_ref[i] = b_reference[i];
        state->accel_ref[i] = g_reference[i];
        state->magno_bias[i] = MAGNO_BIAS[i];
        state->accel_bias[i] = ACCEL_BIAS[i];
        state->gyro_bias[i] = GYRO_BIAS[i];
    }

    for (int i = 0; i < KALMAN_NUM_STATES; i++) {
        state->P[i] = P.diagonal()[i];
    }
}

void kalman_get_covariance(fp covar[KALMAN_NUM_STATES]) {
    for (int i = 0; i < KALMAN_NUM_STATES; i++)
        covar[i] = P.diagonal()[i];
}

// Most updates only touch a subset of the states. Testing showed that GCC isn't very good at eliminating
// the redundant multiplications by 0 and so we give it a hand by eliminating some of them for it
template<int N, int I = 0>
inline void do_update_t(const Matrix<fp, 3, 1> &y, const Matrix<fp, 3, N> &H,
                        const DiagonalMatrix<fp, 3> &sensor_covariance) {
    auto Pt = P.block<N, N>(I, I);

    Matrix<fp, 3, 3> S = (H * Pt * H.transpose());
    S.diagonal() += sensor_covariance.diagonal();

    Matrix<fp, 3, 3> inverse = S.inverse();
    Matrix<fp, N, 3> K = Pt * H.transpose() * inverse;

    Matrix<fp, N, 1> t1 = K * y;

    for (int i = 0; i < N; i++)
        prior_state_vector[i + I] += t1[i];


    Matrix<fp, N, N> t2 = (K * H * Pt);

    Pt -= t2;

    update_attitude();
}


void kalman_predict(fp dt) {
    //
    // Predict next state
    //

    // We apply rotation directly to prior_attitude instead of
    // ATTITUDE_ERROR = ANGULAR_VELOCITY.normalized() * std::tan(ANGULAR_VELOCITY.norm() * dt / 4);
    // update_attitude();
    // As it yields better precision

    Quaternion<fp> delta = Quaternion<fp>(
            AngleAxis<fp>(AngleAxis<fp>(ANGULAR_VELOCITY.norm() * dt, ANGULAR_VELOCITY.normalized())));
    Quaternion<fp> t = delta * prior_attitude; // Not sure whether quaternions have aliasing protection
    prior_attitude = t;

    POSITION += VELOCITY * dt;
    VELOCITY += ACCELERATION * dt;


    //
    // Update Covariance
    //

    // Whilst this matrix is very large, GCC does a good job of removing the redundant
    // multiplications by zero and so no further manual optimisation is needed
    Matrix<fp, KALMAN_NUM_STATES, KALMAN_NUM_STATES> F = Matrix<fp, KALMAN_NUM_STATES, KALMAN_NUM_STATES>::Identity();

    F.block<3, 3>(KALMAN_ATTITUDE_ERR_IDX, KALMAN_ANGULAR_VEL_IDX) = angular_velocity_jacobian(ANGULAR_VELOCITY, dt);

    // Latitude
    F(KALMAN_POSITION_IDX + 0, KALMAN_VELOCITY_IDX + 0) = dt;
    // Longitude
    F(KALMAN_POSITION_IDX + 1, KALMAN_VELOCITY_IDX + 1) = dt;
    // Elevation
    F(KALMAN_POSITION_IDX + 2, KALMAN_VELOCITY_IDX + 2) = dt;

    F(KALMAN_VELOCITY_IDX + 0, KALMAN_ACCELERATION_IDX + 0) = dt;
    F(KALMAN_VELOCITY_IDX + 1, KALMAN_ACCELERATION_IDX + 1) = dt;
    F(KALMAN_VELOCITY_IDX + 2, KALMAN_ACCELERATION_IDX + 2) = dt;

    P = (F * P * F.transpose());
    P.diagonal() += dt * process_noise.diagonal();
}

void kalman_new_accel(const fp accel[3]) {
    // The attitude rotates the observations onto the references
    // Attitude error is always 0 on entry
    Matrix<fp, 3, 1> a_prime = prior_attitude.inverse() * (g_reference + ACCELERATION);
    Matrix<fp, 3, 1> predicted_measurement = a_prime + ACCEL_BIAS;
    Matrix<fp, 3, 1> y = Map<const Matrix<fp, 3, 1>>(accel) - predicted_measurement;

    Matrix<fp, 3, 3> H1;
    Matrix<fp, 3, 3> H2;
    Matrix<fp, 3, 3> H3;

    H1 = q_target_jacobian(g_reference + ACCELERATION, prior_attitude.inverse());

    H2 = Matrix3f::Identity();

    H3 = mrp_application_jacobian(ATTITUDE_ERROR, a_prime) * -1;

    do_update_t<3, KALMAN_ACCELERATION_IDX>(y, H1, accelerometer_covariance);
    do_update_t<3, KALMAN_ACCEL_BIAS_IDX>(y, H2, accelerometer_covariance);
    do_update_t<3, KALMAN_ATTITUDE_ERR_IDX>(y, H3, accelerometer_covariance);
}

void kalman_new_magno(const fp magno[3]) {
    // The attitude rotates the observations onto the references
    // Attitude error is always 0 on entry
    Matrix<fp, 3, 1> b_prime = prior_attitude.inverse() * b_reference;
    Matrix<fp, 3, 1> predicted_measurement = b_prime + MAGNO_BIAS;
    Matrix<fp, 3, 1> y = Map<const Matrix<fp, 3, 1>>(magno) - predicted_measurement;

    Matrix<fp, 3, 3> H = mrp_application_jacobian(ATTITUDE_ERROR, b_prime) * -1;

    Matrix<fp, 3, 3> H_prime = Matrix<fp, 3, 3>::Identity();

    do_update_t<3, KALMAN_ATTITUDE_ERR_IDX>(y, H, magno_covariance);

    do_update_t<3, KALMAN_MAGNO_BIAS_IDX>(y, H_prime, magno_covariance);
}

void kalman_new_gyro(const fp gyro[3]) {
    // The attitude rotates the observations onto the references
    // Attitude error is always 0 on entry
    Matrix<fp, 3, 1> g_prime = prior_attitude.inverse() * ANGULAR_VELOCITY;
    Matrix<fp, 3, 1> predicted_measurement = g_prime + GYRO_BIAS;

    Matrix<fp, 3, 1> y = Map<const Matrix<fp, 3, 1>>(gyro) - predicted_measurement;
    Matrix<fp, 3, 3> H1;
    Matrix<fp, 3, 3> H2;
    Matrix<fp, 3, 3> H3;

    H1 = mrp_application_jacobian(ATTITUDE_ERROR, g_prime) * -1;
    H2 = q_target_jacobian(ANGULAR_VELOCITY, prior_attitude.inverse());
    H3 = Matrix<fp, 3, 3>::Identity();

    do_update_t<3, KALMAN_ATTITUDE_ERR_IDX>(y, H1, gyro_covariance);

    do_update_t<3, KALMAN_ANGULAR_VEL_IDX>(y, H2, gyro_covariance);

    do_update_t<3, KALMAN_GYRO_BIAS_IDX>(y, H3, gyro_covariance);
}

void kalman_new_gps(const fp *r, const fp *v) {
    //TODO: Implement Me
}


