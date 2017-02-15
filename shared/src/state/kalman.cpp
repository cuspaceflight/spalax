#include "kalman.h"
#include "Eigen/Core"
#include <Eigen/Geometry>
#include <cfloat>

using namespace Eigen;

static Matrix<float, KALMAN_NUM_STATES, 1> prior_state_vector;
static DiagonalMatrix<float, KALMAN_NUM_STATES> P;
static Eigen::Quaternionf prior_attitude;

#define ATTITUDE_ERROR(state_vector) state_vector.block<3,1>(KALMAN_ATTITUDE_ERR_IDX,0)
#define ANGULAR_VELOCITY(state_vector) state_vector.block<3,1>(KALMAN_ANGULAR_VEL_IDX,0)
#define GYRO_BIAS(state_vector) state_vector.block<3,1>(KALMAN_GYRO_BIAS_IDX,0)

static Eigen::Vector3f g_reference;
static Eigen::Vector3f b_reference;

static Eigen::DiagonalMatrix<float, 3> accelerometer_covariance;
static Eigen::DiagonalMatrix<float, 3> magno_covariance;
static Eigen::DiagonalMatrix<float, 3> gyro_covariance;

static DiagonalMatrix<float, KALMAN_NUM_STATES> process_noise;

inline Quaternionf mrpToQuat(const Vector3f &mrp) {
    Quaternionf q;
    float mag_mrp_squared = mrp[0] * mrp[0] + mrp[1] * mrp[1] + mrp[2] * mrp[2];
    float mag_mrp = sqrtf(mag_mrp_squared);

    if (mag_mrp < FLT_EPSILON) {
        q.x() = 0;
        q.y() = 0;
        q.z() = 0;
        q.w() = 1;
        return q;
    }

    float sin_theta_2 = 2 * mag_mrp / (mag_mrp_squared + 1);
    float cos_theta_2 = (1 - mag_mrp_squared) / (mag_mrp_squared + 1);

    q.x() = sin_theta_2 * mrp[0] / mag_mrp;
    q.y() = sin_theta_2 * mrp[1] / mag_mrp;
    q.z() = sin_theta_2 * mrp[2] / mag_mrp;
    q.w() = cos_theta_2;
    return q;
}

inline Quaternionf mrpToQuat(const float mrp[3]) {
    return mrpToQuat(Vector3f(mrp[0], mrp[1], mrp[2]));
}

inline Vector3f quatToMrp(const Quaternionf &q) {
    Vector3f mrp;
    float div = 1 / (1 + q.w());
    mrp[0] = q.x() * div;
    mrp[1] = q.y() * div;
    mrp[2] = q.z() * div;
    return mrp;
}

void kalman_init(float accel_reference[3], float magno_reference[3]) {
    for (int i = 0; i < KALMAN_NUM_STATES; i++) {
        prior_state_vector(i) = 0;
    }

    prior_attitude = Quaternionf(1, 0, 0, 0);

    for (int i = 0; i < 3; i++) {
        accelerometer_covariance.diagonal()[i] = 0.001f;
        magno_covariance.diagonal()[i] = 0.001f;
        gyro_covariance.diagonal()[i] = 0.00001f;

        // Attitude Error
        P.diagonal()[i + KALMAN_ATTITUDE_ERR_IDX] = 0.616850275068084f;
        process_noise.diagonal()[i + KALMAN_ATTITUDE_ERR_IDX] = 1e-5f;
        // Angular Velocity
        P.diagonal()[i + KALMAN_ANGULAR_VEL_IDX] = 4;
        process_noise.diagonal()[i + KALMAN_ANGULAR_VEL_IDX] = 3e-3f;
        // Gyro Bias
        P.diagonal()[i + KALMAN_GYRO_BIAS_IDX] = 1;
        process_noise.diagonal()[i + KALMAN_GYRO_BIAS_IDX] = 1e-5f;

        g_reference[i] = accel_reference[i];
        b_reference[i] = magno_reference[i];
    }
}

inline Matrix3f mrp_application_jacobian(const Vector3f &mrp, const Vector3f &target_vector) {
    // TODO: Verify this is numerically stable
    Matrix3f ret;
    Vector3f v0 = mrpToQuat(mrp) * target_vector;
    const double epsilon = 1e-5f;
    for (int i = 0; i < 3; i++) {
        Vector3f altered_mrp = mrp;
        altered_mrp[i] += epsilon;
        Vector3f v1 = mrpToQuat(altered_mrp) * target_vector;

        ret.block<3, 1>(0, i) = (v1 - v0) / epsilon;
    }
    return ret;
}

inline Matrix3f q_target_jacobian(const Vector3f &target_vector, const Quaternionf &quat) {
    Matrix3f ret;
    Vector3f v0 = quat * target_vector;
    const float epsilon = 1e-5f;
    for (int i = 0; i < 3; i++) {
        Vector3f altered_target = target_vector;
        altered_target[i] += epsilon;
        Vector3f v1 = quat * altered_target;

        ret.block<3, 1>(0, i) = (v1 - v0) / epsilon;
    }
    return ret;
}

// Moves the MRP attitude error into the prior_attitude quaternion
inline void update_attitude() {
    // TODO: Determine whether temporary needed
    Quaternionf t = mrpToQuat(ATTITUDE_ERROR(prior_state_vector)) * prior_attitude;
    prior_attitude = t;

    ATTITUDE_ERROR(prior_state_vector) = Vector3f::Zero();
}

void kalman_get_state(state_estimate_t *state) {
    Eigen::Map<Vector3f>(state->angular_velocity) = ANGULAR_VELOCITY(prior_state_vector);

    state->orientation_q[0] = prior_attitude.x();
    state->orientation_q[1] = prior_attitude.y();
    state->orientation_q[2] = prior_attitude.z();
    state->orientation_q[3] = prior_attitude.w();

    state->altitude = 0;
    state->latitude = 0;
    state->longitude = 0;
}

void kalman_get_covariance(float covar[KALMAN_NUM_STATES]) {
    for (int i = 0; i < KALMAN_NUM_STATES; i++)
        covar[i] = P.diagonal()[i];
}


// TODO: Optimise this - large portions of H and K are zero
static void
do_update(const Vector3f &y, const Matrix<float, 3, KALMAN_NUM_STATES> &H, const DiagonalMatrix<float, 3> &sensor_covariance) {
    Matrix3f S = (H * P * H.transpose());
    S.diagonal() += sensor_covariance.diagonal();

    Matrix3f inverse = S.inverse();

    Matrix<float, KALMAN_NUM_STATES, 3> K = P * H.transpose() * inverse;

    prior_state_vector += K * y;

    DiagonalMatrix<float, KALMAN_NUM_STATES>::DiagonalVectorType t = (K * H * P).diagonal();
    P.diagonal() -= t;

    update_attitude();
}

inline void predict_attitude(float dt) {
    float omega_mag = ANGULAR_VELOCITY(prior_state_vector).norm();
    if (omega_mag > 1e-8f) {
        // TODO: Determine whether temporary needed to avoid aliasing

        // The orientation stored in the kalman state is actually inverted
        // To understand why imagine a ship travelling north. North will be at a reference of 0
        // If it rotates 5 degrees clockwise, north will now be at a reference of 355
        // This estimator tracks reference vectors which as shown above rotate in the opposite direction
        // To the actual rotation of the body
        Quaternionf t = (prior_attitude *
                         Quaternionf(AngleAxisf(-omega_mag * dt, ANGULAR_VELOCITY(prior_state_vector).normalized())));
        prior_attitude = t;
    }
}


void kalman_predict(float dt) {
    float dt2 = dt * dt;

    predict_attitude(dt);

    for (int i = 0; i < KALMAN_NUM_STATES; i++) {
        P.diagonal()[i] += dt * process_noise.diagonal()[i];
    }

    P.diagonal()[3] += dt2 * P.diagonal()[3];
    P.diagonal()[4] += dt2 * P.diagonal()[4];
    P.diagonal()[5] += dt2 * P.diagonal()[5];

    //update_attitude();
}

void kalman_new_accel(const float accel[3]) {
    Vector3f g_prime = prior_attitude * g_reference;
    Vector3f predicted_measurement = (mrpToQuat(ATTITUDE_ERROR(prior_state_vector)) * g_prime);

    Vector3f y = Eigen::Map<const Vector3f>(accel) - predicted_measurement;

    Matrix<float, 3, KALMAN_NUM_STATES> H = Matrix<float, 3, KALMAN_NUM_STATES>::Zero();
    H.block<3, 3>(0, 0) = mrp_application_jacobian(ATTITUDE_ERROR(prior_state_vector), g_prime);

    do_update(y, H, accelerometer_covariance);
}

void kalman_new_magno(const float magno[3]) {
    Vector3f b_prime = prior_attitude * b_reference;
    Vector3f predicted_measurement = (mrpToQuat(ATTITUDE_ERROR(prior_state_vector)) * b_prime);

    Vector3f y = Eigen::Map<const Vector3f>(magno) - predicted_measurement;

    Matrix<float, 3, KALMAN_NUM_STATES> H = Matrix<float, 3, KALMAN_NUM_STATES>::Zero();
    H.block<3, 3>(0, 0) = mrp_application_jacobian(ATTITUDE_ERROR(prior_state_vector), b_prime);

    do_update(y, H, magno_covariance);
}

void kalman_new_gyro(const float gyro[3]) {
    Vector3f v_prime = prior_attitude * ANGULAR_VELOCITY(prior_state_vector);
    Vector3f predicted_measurement = (mrpToQuat(ATTITUDE_ERROR(prior_state_vector)) * v_prime);
    predicted_measurement += GYRO_BIAS(prior_state_vector);

    Vector3f y = Eigen::Map<const Vector3f>(gyro) - predicted_measurement;
    Matrix<float, 3, KALMAN_NUM_STATES> H;
    H.block<3, 3>(0, 0) = mrp_application_jacobian(ATTITUDE_ERROR(prior_state_vector), v_prime);

    H.block<3, 3>(0, KALMAN_ANGULAR_VEL_IDX) = q_target_jacobian(ATTITUDE_ERROR(prior_state_vector),
                                                                 mrpToQuat(ATTITUDE_ERROR(prior_state_vector)) *
                                                                prior_attitude);
    H.block<3,3>(0, KALMAN_GYRO_BIAS_IDX) = Matrix3f::Identity();

    do_update(y, H, gyro_covariance);
}


