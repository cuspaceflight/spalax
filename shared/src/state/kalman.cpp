#include "kalman.h"
#include "Eigen/Core"
#include <Eigen/Geometry>
#include <cfloat>
#include <config/telemetry_packets.h>

using namespace Eigen;

static constexpr int num_states = 12;
static Matrix<float, num_states, 1> state_vector;
static DiagonalMatrix<float, num_states> P;
static Eigen::Quaternionf prior_attitude;

#define ATTITUDE_ERROR(state_vector) state_vector.block<3,1>(0,0)
#define ANGULAR_VELOCITY(state_vector) state_vector.block<3,1>(3,0)
#define ANGULAR_ACCELERATION(state_vector) state_vector.block<3,1>(6,0)
#define GYRO_BIAS(state_vector) state_vector.block<3,1>(9,0)

static Eigen::Vector3f g_reference;
static Eigen::Vector3f b_reference;

static Eigen::DiagonalMatrix<float, 3> accelerometer_covariance;
static Eigen::DiagonalMatrix<float, 3> magno_covariance;
static Eigen::DiagonalMatrix<float, 3> gyro_covariance;

static DiagonalMatrix<float, num_states> process_noise;

inline Quaternionf mrpToQuat(const Vector3f& mrp) {
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

inline Vector3f quatToMrp(const Quaternionf& q) {
    Vector3f mrp;
    float div = 1 / (1 + q.w());
    mrp[0] = q.x() * div;
    mrp[1] = q.y() * div;
    mrp[2] = q.z() * div;
    return mrp;
}

void kalman_init(float accel_reference[3], float magno_reference[3]) {
    for (int i = 0; i < 12; i++) {
        state_vector(i) = 0;
    }

    prior_attitude = Quaternionf(1, 0, 0, 0);

    for (int i = 0; i < 3; i++) {
        accelerometer_covariance.diagonal()[i] = 0.001f;
        magno_covariance.diagonal()[i] = 0.00001f;
        gyro_covariance.diagonal()[i] = 0.00001f;

        // Attitude Error
        P.diagonal()[i] = 0.616850275068084f;
        process_noise.diagonal()[i] = 1e-5f;
        // Angular Velocity
        P.diagonal()[i+3] = 4;
        process_noise.diagonal()[i+3] = 3e-5f;
        // Angular Acceleration
        P.diagonal()[i+6] = 25;
        process_noise.diagonal()[i+6] = 1e-5f;
        // Gyro Bias
        P.diagonal()[i+9] = 1;
        process_noise.diagonal()[i+9] = 1e-7f;

        g_reference[i] = accel_reference[i];
        b_reference[i] = magno_reference[i];
    }
}

static void mrp_application_jacobian(const Vector3f& mrp, const Vector3f& target_vector, Matrix<float, 3, num_states>& J) {
    // TODO: Verify this is numerically stable
    Vector3f v0 = mrpToQuat(mrp) * target_vector;
    const float epsilon = 1e-3f;
    for (int i = 0; i < 3; i++) {
        Vector3f altered_mrp = mrp;
        altered_mrp[i] += epsilon;
        Vector3f v1 =  mrpToQuat(altered_mrp) * target_vector;

        J.block<3,1>(0, i) = (v1 - v0) / epsilon;
    }
}

static void q_target_jacobian(const Vector3f& target_vector, const Quaternionf& quat, Matrix<float, 3, num_states>& J) {
    Vector3f v0 = quat * target_vector;
    const float epsilon = 1e-3f;
    for (int i = 0; i < 3; i++) {
        Vector3f altered_target = target_vector;
        altered_target[i] += epsilon;
        Vector3f v1 = quat * altered_target;

        // TODO: Verify this is correct
        J.block<3,1>(0, i) = (v1 - v0) / epsilon;
    }
}

// TODO: Optimise this - large portions of H and K are zero
static void do_update(const Vector3f& y, const Matrix<float, 3, num_states>& H, const DiagonalMatrix<float, 3>& sensor_covariance) {
    Matrix3f S = (H * P * H.transpose());
    S.diagonal() += sensor_covariance.diagonal();

    auto K = P * H.transpose() * S.inverse();

    state_vector += K * y;
    P.diagonal() += (K * H * P).diagonal();
}

void kalman_new_accel(const float accel[3]) {
    Vector3f g_prime = prior_attitude * g_reference;
    Vector3f predicted_measurement = (mrpToQuat(ATTITUDE_ERROR(state_vector)) * g_prime);

    Vector3f y = Eigen::Map<const Vector3f>(accel) - predicted_measurement;

    Matrix<float, 3, num_states> H = Matrix<float, 3, num_states>::Zero();
    mrp_application_jacobian(ATTITUDE_ERROR(state_vector), g_prime, H);

    do_update(y, H, accelerometer_covariance);
}

void kalman_predict(state_estimate_t next_estimate, float dt) {
    Matrix<float, num_states, 1> post_state_vector;

}