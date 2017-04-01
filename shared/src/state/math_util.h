#ifndef MATH_UTIL_H
#define MATH_UTIL_H

#ifdef __cplusplus

#include "Eigen/Core"
#include <Eigen/Geometry>
#include <cfloat>

using namespace Eigen;

inline Quaternion<fp> mrpToQuat(const Matrix<fp, 3, 1> &mrp) {
    Quaternion<fp> q;
    fp mag_mrp_squared = mrp[0] * mrp[0] + mrp[1] * mrp[1] + mrp[2] * mrp[2];
    fp mag_mrp = sqrtf(mag_mrp_squared);

    if (mag_mrp < FLT_EPSILON) {
        q.x() = 0;
        q.y() = 0;
        q.z() = 0;
        q.w() = 1;
        return q;
    }

    fp sin_theta_2 = 2 * mag_mrp / (mag_mrp_squared + 1);
    fp cos_theta_2 = (1 - mag_mrp_squared) / (mag_mrp_squared + 1);

    q.x() = sin_theta_2 * mrp[0] / mag_mrp;
    q.y() = sin_theta_2 * mrp[1] / mag_mrp;
    q.z() = sin_theta_2 * mrp[2] / mag_mrp;
    q.w() = cos_theta_2;
    return q;
}

inline Quaternion<fp> mrpToQuat(const fp mrp[3]) {
    return mrpToQuat(Matrix<fp, 3, 1>(mrp[0], mrp[1], mrp[2]));
}

inline Matrix<fp, 3, 1> quatToMrp(const Quaternion<fp> &q) {
    Matrix<fp, 3, 1> mrp;
    fp div = 1 / (1 + q.w());
    mrp[0] = q.x() * div;
    mrp[1] = q.y() * div;
    mrp[2] = q.z() * div;
    return mrp;
}

inline Matrix<fp, 3, 1> angleAxisToMrp(const AngleAxis<fp> &a) {
    return a.axis() * std::tan(a.angle() / 4);
};

// The 3x3 Jacobian of the rotation the target vector by the MRP rotatation w.r.t the MRP rotation
// Assumes an MRP of zero
inline Matrix<fp, 3, 3>
mrp_application_jacobian(const Matrix<fp, 3, 1> &mrp, const Matrix<fp, 3, 1> &target_vector) {
    Matrix<fp, 3, 3> ret;
    ret << 0, target_vector.z() * 4, -target_vector.y() * 4,
            -target_vector.z() * 4, 0, target_vector.x() * 4,
            target_vector.y() * 4, -target_vector.x() * 4, 0;

    return ret;
}

// mrp = velocity.normalized() * tan(velocity.norm() * dt / 4)
inline Matrix<fp, 3, 1> angular_velocity_to_mrp(const Matrix<fp, 3, 1> &velocity, float dt) {
    fp omega_mag = velocity.norm();
    if (omega_mag > 1e-8f) {
        Matrix<fp, 3, 1> axis = velocity.normalized();

        // The attitude error on entry is 0 on entry
        return angleAxisToMrp(AngleAxis<fp>(omega_mag * dt, axis));
    }
    return Matrix<fp, 3, 1>::Zero();
};

// The 3x3 Jacobian of the conversion of an angular velocity to mrp
// For small angles this is mrp = velocity.normalized() * velocity.norm() / 4
inline Matrix<fp, 3, 3>
angular_velocity_jacobian(const Matrix<fp, 3, 1> &velocity, float dt) {
    // TODO: if velocity magnitude small can use small angle approximation
    Matrix<fp, 3, 3> ret;
    Matrix<fp, 3, 1> v0 = angular_velocity_to_mrp(velocity, dt);
    const fp epsilon = 1e-6f;
    for (int i = 0; i < 3; i++) {
        Matrix<fp, 3, 1> altered_velocity = velocity;
        altered_velocity[i] += epsilon;
        Matrix<fp, 3, 1> v1 = angular_velocity_to_mrp(altered_velocity, dt);

        ret.block<3, 1>(0, i) = (v1 - v0) / epsilon;
    }
    return ret;
}

// The 3x3 Jacobian of the rotation of the target vector by the quaternion w.r.t the target vector
inline Matrix<fp, 3, 3> q_target_jacobian(const Matrix<fp, 3, 1> &target_vector, const Quaternion<fp> &quat) {
    (void) target_vector;

    Matrix3f ret;
    ret.block<3, 1>(0, 0) = quat * Vector3f(1, 0, 0);
    ret.block<3, 1>(0, 1) = quat * Vector3f(0, 1, 0);
    ret.block<3, 1>(0, 2) = quat * Vector3f(0, 0, 1);
    return ret;
}

inline Matrix<float, 3, 1> geodetic_to_ecef(float latitude, float longitude, float elevation) {
    const float a = 6378137.0f; // Semi-major axis
    // const float b = 6356752.314245f; // Semi-minor axis
    const float f = 1.0f / 298.257223563f;
    const float e2 = (2 - f) * f;

    float rlat = latitude / 180.0f * 3.14159265358979323846f;
    float rlon = longitude / 180.0f * 3.14159265358979323846f;

    float slat = sin(rlat);
    float clat = cos(rlat);

    float N = a / sqrtf(1 - e2 * slat * slat);

    float x = (N + elevation) * clat * cos(rlon);
    float y = (N + elevation) * clat * sin(rlon);
    float z = (N * (1 - e2) + elevation) * slat;

    return Matrix<float, 3, 1>(x, y, z);
};

const float wgm84_re = 6378137;
const float wgm84_rp = 6356752.3142f;
const float wgm84_eccentricity = 0.0818191908426f;
const float wgm84_eccentricity2 = wgm84_eccentricity * wgm84_eccentricity;

inline void compute_radii_of_curvature(float latitude, float *r_meridian, float *r_normal) {
    // This method is too inaccurate at extreme latitudes (we use the arctic circles as limits)
    assert (latitude < 66 && latitude > -66);

    float rlat = latitude / 180.0f * 3.14159265358979323846f;
    float slat = sin(rlat);

    float t = sqrtf(1 - wgm84_eccentricity2 * slat * slat);

    *r_normal = wgm84_re / t;
    *r_meridian = wgm84_re * (1 - wgm84_eccentricity2) / (t * t * t);
}

inline Vector3f quat_to_euler(const Quaternionf &quat) {
    return quat.toRotationMatrix().eulerAngles(0, 1, 2);
}

#endif
#endif
