#include "kalman.h"
#include <cfloat>
#include "math_util.h"

using namespace Eigen;

static Matrix<fp, KALMAN_NUM_STATES, 1> prior_state_vector;
static DiagonalMatrix<fp, KALMAN_NUM_STATES> P;
static Eigen::Quaternion<fp> prior_attitude;

#define ATTITUDE_ERROR(state_vector) state_vector.block<3,1>(KALMAN_ATTITUDE_ERR_IDX,0)
#define ANGULAR_VELOCITY(state_vector) state_vector.block<3,1>(KALMAN_ANGULAR_VEL_IDX,0)
#define GYRO_BIAS(state_vector) state_vector.block<3,1>(KALMAN_GYRO_BIAS_IDX,0)

static Eigen::Matrix<fp, 3, 1> g_reference;
static Eigen::Matrix<fp, 3, 1> b_reference;

static Eigen::DiagonalMatrix<fp, 3> accelerometer_covariance;
static Eigen::DiagonalMatrix<fp, 3> magno_covariance;
static Eigen::DiagonalMatrix<fp, 3> gyro_covariance;

static DiagonalMatrix<fp, KALMAN_NUM_STATES> process_noise;

fp kalman_magno_cov = 0.1f;
fp kalman_accelerometer_cov = 0.1f;
fp kalman_gyro_cov = 0.1f;

void kalman_init(fp accel_reference[3], fp magno_reference[3],
                 fp initial_orientation[4], fp initial_angular_velocity[3]) {
    for (int i = 0; i < KALMAN_NUM_STATES; i++) {
        prior_state_vector(i) = 0;
    }

    ANGULAR_VELOCITY(prior_state_vector) = Eigen::Map<const Matrix<fp, 3, 1>>(initial_angular_velocity);

    prior_attitude.x() = initial_orientation[0];
    prior_attitude.y() = initial_orientation[1];
    prior_attitude.z() = initial_orientation[2];
    prior_attitude.w() = initial_orientation[3];

    for (int i = 0; i < 3; i++) {
        accelerometer_covariance.diagonal()[i] = kalman_accelerometer_cov;
        magno_covariance.diagonal()[i] = kalman_magno_cov;
        gyro_covariance.diagonal()[i] = kalman_gyro_cov;

        // Attitude Error
        P.diagonal()[i + KALMAN_ATTITUDE_ERR_IDX] = 10;
        process_noise.diagonal()[i + KALMAN_ATTITUDE_ERR_IDX] = 1e-2f;
        // Angular Velocity
        P.diagonal()[i + KALMAN_ANGULAR_VEL_IDX] = 10;
        process_noise.diagonal()[i + KALMAN_ANGULAR_VEL_IDX] = 1e-1f;
        // Gyro Bias
        P.diagonal()[i + KALMAN_GYRO_BIAS_IDX] = 1;
        process_noise.diagonal()[i + KALMAN_GYRO_BIAS_IDX] = 1e-3f;

        g_reference[i] = accel_reference[i];
        b_reference[i] = magno_reference[i];
    }
}

// Moves the MRP attitude error into the prior_attitude quaternion
inline void update_attitude() {
    Quaternion<fp> t = mrpToQuat(ATTITUDE_ERROR(prior_state_vector)) * prior_attitude;
    prior_attitude = t;

    ATTITUDE_ERROR(prior_state_vector) = Matrix<fp, 3, 1>::Zero();
}

void kalman_get_state(state_estimate_t *state) {
    for (int i = 0; i < 3; i++)
        state->angular_velocity[i] = ANGULAR_VELOCITY(prior_state_vector)[i];

    state->orientation_q[0] = prior_attitude.x();
    state->orientation_q[1] = prior_attitude.y();
    state->orientation_q[2] = prior_attitude.z();
    state->orientation_q[3] = prior_attitude.w();

    state->altitude = 0;
    state->latitude = 0;
    state->longitude = 0;
}

void kalman_get_covariance(fp covar[KALMAN_NUM_STATES]) {
    for (int i = 0; i < KALMAN_NUM_STATES; i++)
        covar[i] = P.diagonal()[i];
}


// TODO: Optimise this - large portions of H and K are zero
inline void do_update(const Matrix<fp, 3, 1> &y, const Matrix<fp, 3, KALMAN_NUM_STATES> &H,
                      const DiagonalMatrix<fp, 3> &sensor_covariance) {
    Matrix<fp, 3, 3> S = (H * P * H.transpose());
    S.diagonal() += sensor_covariance.diagonal();

    Matrix<fp, 3, 3> inverse = S.inverse();
    Matrix<fp, KALMAN_NUM_STATES, 3> K = P * H.transpose() * inverse;

    prior_state_vector += K * y;

    DiagonalMatrix<fp, KALMAN_NUM_STATES>::DiagonalVectorType t = (K * H * P).diagonal();
    P.diagonal() -= t;

    update_attitude();
}

inline void predict_attitude(fp dt) {
    fp omega_mag = ANGULAR_VELOCITY(prior_state_vector).norm();
    if (omega_mag > 1e-8f) {
        Matrix<fp, 3, 1> axis = ANGULAR_VELOCITY(prior_state_vector).normalized();
        Quaternionf delta = Quaternion<fp>(AngleAxis<fp>(omega_mag * dt, axis));

        Quaternion<fp> t = prior_attitude * delta;
        prior_attitude = t;
    }
}


void kalman_predict(fp dt) {
    predict_attitude(dt);

    // TODO: Should there be some relationship between attitude error covariance and angular velocity covariance?
    // i.e Should F be something other than the identity?

    for (int i = 0; i < KALMAN_NUM_STATES; i++) {
        P.diagonal()[i] += dt * process_noise.diagonal()[i];
    }
    //update_attitude();
}

void kalman_new_accel(const fp accel[3]) {
    Matrix<fp, 3, 1> g_prime = prior_attitude * g_reference;
    Matrix<fp, 3, 1> predicted_measurement = (mrpToQuat(ATTITUDE_ERROR(prior_state_vector)) * g_prime);

    Matrix<fp, 3, 1> y = Eigen::Map<const Matrix<fp, 3, 1>>(accel) - predicted_measurement;

    Matrix<fp, 3, KALMAN_NUM_STATES> H = Matrix<fp, 3, KALMAN_NUM_STATES>::Zero();
    H.block<3, 3>(0, 0) = mrp_application_jacobian_numerical(ATTITUDE_ERROR(prior_state_vector), g_prime);

    do_update(y, H, accelerometer_covariance);
}

void kalman_new_magno(const fp magno[3]) {
    Matrix<fp, 3, 1> b_prime = prior_attitude * b_reference;
    Matrix<fp, 3, 1> predicted_measurement = (mrpToQuat(ATTITUDE_ERROR(prior_state_vector)) * b_prime);

    Matrix<fp, 3, 1> y = Eigen::Map<const Matrix<fp, 3, 1>>(magno) - predicted_measurement;

    Matrix<fp, 3, KALMAN_NUM_STATES> H = Matrix<fp, 3, KALMAN_NUM_STATES>::Zero();
    H.block<3, 3>(0, 0) = mrp_application_jacobian_numerical(ATTITUDE_ERROR(prior_state_vector), b_prime);

    do_update(y, H, magno_covariance);
}

void kalman_new_gyro(const fp gyro[3]) {
    Matrix<fp, 3, 1> v_prime = prior_attitude * ANGULAR_VELOCITY(prior_state_vector);
    Matrix<fp, 3, 1> predicted_measurement = (mrpToQuat(ATTITUDE_ERROR(prior_state_vector)) * v_prime);
    predicted_measurement += GYRO_BIAS(prior_state_vector);

    Matrix<fp, 3, 1> y = Eigen::Map<const Matrix<fp, 3, 1>>(gyro) - predicted_measurement;
    Matrix<fp, 3, KALMAN_NUM_STATES> H;
    H.block<3, 3>(0, 0) = mrp_application_jacobian_numerical(ATTITUDE_ERROR(prior_state_vector), v_prime);

    H.block<3, 3>(0, KALMAN_ANGULAR_VEL_IDX) = q_target_jacobian(ANGULAR_VELOCITY(prior_state_vector),
                                                                 prior_attitude *
                                                                 mrpToQuat(ATTITUDE_ERROR(prior_state_vector))
    );
    H.block<3, 3>(0, KALMAN_GYRO_BIAS_IDX) = Matrix<fp, 3, 3>::Identity();

    do_update(y, H, gyro_covariance);
}


