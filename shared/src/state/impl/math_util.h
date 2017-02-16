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

// The 3x3 Jacobian of the rotation the target vector by the MRP rotatation w.r.t the MRP rotation
inline Matrix<fp, 3, 3> mrp_application_jacobian_numerical(const Matrix<fp, 3, 1> &mrp, const Matrix<fp, 3, 1> &target_vector) {
    // TODO: Verify this is numerically stable
    Matrix<fp, 3, 3> ret;
    Matrix<fp, 3, 1> v0 = mrpToQuat(mrp) * target_vector;
    const fp epsilon = 7.8125e-3f;
    for (int i = 0; i < 3; i++) {
        Matrix<fp, 3, 1> altered_mrp = mrp;
        altered_mrp[i] += epsilon;
        Matrix<fp, 3, 1> v1 = mrpToQuat(altered_mrp) * target_vector;

        ret.block<3, 1>(0, i) = (v1 - v0) / epsilon;
    }
    return ret;
}

inline Matrix<fp, 3, 3> q_target_jacobian(const Matrix<fp, 3, 1> &target_vector, const Quaternion<fp> &quat) {
    Matrix<fp, 3, 3> ret;
    Matrix<fp, 3, 1> v0 = quat * target_vector;
    const fp epsilon = 7.8125e-3f;
    for (int i = 0; i < 3; i++) {
        Matrix<fp, 3, 1> altered_target = target_vector;
        altered_target[i] += epsilon;
        Matrix<fp, 3, 1> v1 = quat * altered_target;

        ret.block<3, 1>(0, i) = (v1 - v0) / epsilon;
    }
    return ret;
}

#endif
#endif
