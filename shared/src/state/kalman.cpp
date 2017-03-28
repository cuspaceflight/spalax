#include "kalman.h"
#include <cfloat>
#include "math_util.h"

using namespace Eigen;

static Matrix<fp, KALMAN_NUM_STATES, 1> prior_state_vector;
static DiagonalMatrix<fp, KALMAN_NUM_STATES> P;
static Eigen::Quaternion<fp> prior_attitude;

#define ATTITUDE_ERROR prior_state_vector.block<3,1>(KALMAN_ATTITUDE_ERR_IDX,0)
#define ANGULAR_VELOCITY prior_state_vector.block<3,1>(KALMAN_ANGULAR_VEL_IDX,0)
#define GYRO_BIAS prior_state_vector.block<3,1>(KALMAN_GYRO_BIAS_IDX,0)
#define POSITION prior_state_vector.block<3,1>(KALMAN_POSITION_IDX,0)
#define VELOCITY prior_state_vector.block<3,1>(KALMAN_VELOCITY_IDX,0)
#define ACCELERATION prior_state_vector.block<3,1>(KALMAN_ACCELERATION_IDX,0)

static Eigen::Matrix<fp, 3, 1> g_reference;
static Eigen::Matrix<fp, 3, 1> b_reference;

static Eigen::DiagonalMatrix<fp, 3> accelerometer_covariance;
static Eigen::DiagonalMatrix<fp, 3> magno_covariance;
static Eigen::DiagonalMatrix<fp, 3> gyro_covariance;

static DiagonalMatrix<fp, KALMAN_NUM_STATES> process_noise;

fp kalman_magno_cov = 0.003f;
fp kalman_accelerometer_cov = 0.001f;
fp kalman_gyro_cov = 1e-6f;

void kalman_init(const fp accel_reference[3], const fp magno_reference[3], const fp initial_orientation[4],
                 const fp initial_angular_velocity[3], const fp initial_position[3], const fp initial_velocity[3],
                 const fp initial_acceleration[3], const fp initial_gyro_bias[3]) {
    for (int i = 0; i < KALMAN_NUM_STATES; i++) {
        prior_state_vector(i) = 0;
    }

    ANGULAR_VELOCITY = Eigen::Map<const Matrix<fp, 3, 1>>(initial_angular_velocity);
    POSITION = Eigen::Map<const Matrix<fp, 3, 1>>(initial_position);
    VELOCITY = Eigen::Map<const Matrix<fp, 3, 1>>(initial_velocity);
    ACCELERATION = Eigen::Map<const Matrix<fp, 3, 1>>(initial_acceleration);
    GYRO_BIAS = Eigen::Map<const Matrix<fp, 3, 1>>(initial_gyro_bias);

    prior_attitude.x() = initial_orientation[0];
    prior_attitude.y() = initial_orientation[1];
    prior_attitude.z() = initial_orientation[2];
    prior_attitude.w() = initial_orientation[3];

    for (int i = 0; i < 3; i++) {
        accelerometer_covariance.diagonal()[i] = kalman_accelerometer_cov;
        magno_covariance.diagonal()[i] = kalman_magno_cov;
        gyro_covariance.diagonal()[i] = kalman_gyro_cov;

        // Attitude Error
        P.diagonal()[i + KALMAN_ATTITUDE_ERR_IDX] = 1e-5f;
        process_noise.diagonal()[i + KALMAN_ATTITUDE_ERR_IDX] = 1e-6f;
        // Angular Velocity
        P.diagonal()[i + KALMAN_ANGULAR_VEL_IDX] = 1e-5f;
        process_noise.diagonal()[i + KALMAN_ANGULAR_VEL_IDX] = 1e2f;
        // Gyro Bias
        P.diagonal()[i + KALMAN_GYRO_BIAS_IDX] = 1e-3f;
        process_noise.diagonal()[i + KALMAN_GYRO_BIAS_IDX] = 1e-6f;
        // Position
        P.diagonal()[i + KALMAN_POSITION_IDX] = 1e-4;
        process_noise.diagonal()[i + KALMAN_POSITION_IDX] = 1e-7f;
        // Velocity
        P.diagonal()[i + KALMAN_VELOCITY_IDX] = 1e-4;
        process_noise.diagonal()[i + KALMAN_VELOCITY_IDX] = 1e-7f;
        // Acceleration
        P.diagonal()[i + KALMAN_ACCELERATION_IDX] = 1e-6f;
        process_noise.diagonal()[i + KALMAN_ACCELERATION_IDX] = 1e2f;

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

void kalman_get_covariance(fp covar[KALMAN_NUM_STATES]) {
    for (int i = 0; i < KALMAN_NUM_STATES; i++)
        covar[i] = P.diagonal()[i];
}

// Most updates only touch a subset of the states. Testing showed that GCC isn't very good at eliminating
// the redundant multiplications by 0 and so we give it a hand by eliminating some of them for it
template<int N, int I = 0>
inline void do_update_t(const Matrix<fp, 3, 1> &y, const Matrix<fp, 3, N> &H,
                        const DiagonalMatrix<fp, 3> &sensor_covariance) {
    DiagonalMatrix<fp, N, N> Pt;
    for (int i = 0; i < N; i++)
        Pt.diagonal()[i] = P.diagonal()[i + I];

    Matrix<fp, 3, 3> S = (H * Pt * H.transpose());
    S.diagonal() += sensor_covariance.diagonal();

    Matrix<fp, 3, 3> inverse = S.inverse();
    Matrix<fp, N, 3> K = Pt * H.transpose() * inverse;

    auto t1 = K * y;

    for (int i = 0; i < N; i++)
        prior_state_vector[i + I] += t1[i];


    Matrix<fp, N, 1> t2 = (K * H * Pt).diagonal();

    for (int i = 0; i < N; i++)
        P.diagonal()[i + I] -= t2[i];

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

    Quaternion<fp> delta = Quaternion<fp>(AngleAxis<fp>(AngleAxis<fp>(ANGULAR_VELOCITY.norm() * dt, ANGULAR_VELOCITY.normalized())));
    Quaternion<fp> t = delta * prior_attitude; // Not sure whether quaternions have aliasing protection
    prior_attitude = t;

    POSITION += VELOCITY * dt;
    VELOCITY += ACCELERATION * dt;


    //
    // Update Covariance
    //

    // Whilst this matrix is very large, GCC does a good job of removing the redundant
    // multiplications by zero and so no further manual optimisation is needed
    Eigen::Matrix<fp, KALMAN_NUM_STATES, KALMAN_NUM_STATES> F = Eigen::Matrix<fp, KALMAN_NUM_STATES, KALMAN_NUM_STATES>::Identity();

    F.block<3,3>(KALMAN_ATTITUDE_ERR_IDX, KALMAN_ANGULAR_VEL_IDX) = angular_velocity_jacobian(ANGULAR_VELOCITY, dt);

    // Latitude
    F(KALMAN_POSITION_IDX + 0, KALMAN_VELOCITY_IDX + 0) = dt;
    // Longitude
    F(KALMAN_POSITION_IDX + 1, KALMAN_VELOCITY_IDX + 1) = dt;
    // Elevation
    F(KALMAN_POSITION_IDX + 2, KALMAN_VELOCITY_IDX + 2) = dt;

    F(KALMAN_VELOCITY_IDX + 0, KALMAN_ACCELERATION_IDX + 0) = dt;
    F(KALMAN_VELOCITY_IDX + 1, KALMAN_ACCELERATION_IDX + 1) = dt;
    F(KALMAN_VELOCITY_IDX + 2, KALMAN_ACCELERATION_IDX + 2) = dt;

    P.diagonal() = (F * P * F.transpose()).diagonal() + dt * process_noise.diagonal();
}

void kalman_new_accel(const fp accel[3]) {
    // The attitude rotates the observations onto the references
    // Attitude error is always 0 on entry
    Matrix<fp, 3, 1> predicted_measurement = prior_attitude.inverse() * (g_reference + ACCELERATION);

    Matrix<fp, 3, 1> y = Eigen::Map<const Matrix<fp, 3, 1>>(accel) - predicted_measurement;

    Matrix<fp, 3, 3> H = mrp_application_jacobian(ATTITUDE_ERROR, predicted_measurement) * -1;

    Matrix<fp, 3, 3> H_prime = q_target_jacobian(ACCELERATION,
                                                 prior_attitude.inverse()
    );

    do_update_t<3, KALMAN_ATTITUDE_ERR_IDX>(y, H, accelerometer_covariance);

    do_update_t<3, KALMAN_ACCELERATION_IDX>(y, H_prime, accelerometer_covariance);
}

void kalman_new_magno(const fp magno[3]) {
    // The attitude rotates the observations onto the references
    // Attitude error is always 0 on entry
    Matrix<fp, 3, 1> predicted_measurement = prior_attitude.inverse() * b_reference;

    Matrix<fp, 3, 1> y = Eigen::Map<const Matrix<fp, 3, 1>>(magno) - predicted_measurement;

    Matrix<fp, 3, 3> H = mrp_application_jacobian(ATTITUDE_ERROR, predicted_measurement) * -1;

    do_update_t<3, KALMAN_ATTITUDE_ERR_IDX>(y, H, magno_covariance);
}

void kalman_new_gyro(const fp gyro[3]) {
    // The attitude rotates the observations onto the references
    // Attitude error is always 0 on entry
    Matrix<fp, 3, 1> v_prime = prior_attitude.inverse() * ANGULAR_VELOCITY;
    Matrix<fp, 3, 1> predicted_measurement = (mrpToQuat(ATTITUDE_ERROR) * v_prime);
    predicted_measurement += GYRO_BIAS;

    Matrix<fp, 3, 1> y = Eigen::Map<const Matrix<fp, 3, 1>>(gyro) - predicted_measurement;
    Matrix<fp, 3, 9> H;
    H.block<3, 3>(0, 0) = mrp_application_jacobian(ATTITUDE_ERROR, v_prime) * -1;

    H.block<3, 3>(0, KALMAN_ANGULAR_VEL_IDX) = q_target_jacobian(ANGULAR_VELOCITY,
                                                                 prior_attitude.inverse()
    );
    H.block<3, 3>(0, KALMAN_GYRO_BIAS_IDX) = Matrix<fp, 3, 3>::Identity();

    do_update_t<9>(y, H, gyro_covariance);
}

void kalman_get_gyro_bias(fp bias[3]) {
    for (int i = 0; i < 3; i++)
        bias[i] = prior_state_vector[KALMAN_GYRO_BIAS_IDX + i];
}


