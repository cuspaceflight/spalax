#include "quest.h"
#include "Eigen/Core"
#include <Eigen/Geometry>

// Based on paper at - http://arc.aiaa.org/doi/pdf/10.2514/3.19717
void quest_estimate(const float observations[2][3], const float references[2][3], const float a[2], float *q_out) {
    Eigen::Vector3f v1(references[0][0], references[0][1], references[0][2]);
    Eigen::Vector3f v2(references[1][0], references[1][1], references[1][2]);

    Eigen::Vector3f w1(observations[0][0], observations[0][1], observations[0][2]);
    Eigen::Vector3f w2(observations[1][0], observations[1][1], observations[1][2]);

    float lambda_max = v1.dot(v2) * w1.dot(w2) + v1.cross(v2).norm() * w1.cross(w2).norm();
    Eigen::Matrix3f B = a[0] * (w1 * v1.transpose()) + a[1] * (w2 * v2.transpose());

    Eigen::Matrix3f S = B + B.transpose();
    float sigma = B.trace();
    Eigen::Vector3f Z = a[0] * w1.cross(v1) + a[1] * w2.cross(v2);

    auto delta = S.determinant();
    auto k = S.adjoint().trace();

    auto alpha = lambda_max * lambda_max - sigma * sigma + k;
    auto beta = lambda_max - sigma;
    auto gamma = (lambda_max + sigma) * alpha - delta;

    Eigen::Vector3f X = (alpha * Eigen::Matrix3f::Identity() + beta * S + S * S) * Z;
    auto x_norm = X.norm();

    auto q_mult = 1 / sqrtf(gamma* gamma + x_norm * x_norm);

    q_out[0] = q_mult * X[0];
    q_out[1] = q_mult * X[1];
    q_out[2] = q_mult * X[2];
    q_out[3] = q_mult * gamma;
}
