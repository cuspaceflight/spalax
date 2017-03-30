#include "quest.h"
#include "Eigen/Core"
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace Eigen;

bool try_estimate(const Vector3f& v1, const Vector3f& v2, const Vector3f& w1, const Vector3f& w2, const float a[2], float *q_out) {
    float cos = v1.dot(v2) * w1.dot(w2) + v1.cross(v2).norm() * w1.cross(w2).norm();
    float lambda_max = sqrtf(a[0] * a[0] + 2 * a[0] * a[1] * cos + a[1] * a[1]);

    Matrix3f B = a[0] * (w1 * v1.transpose()) + a[1] * (w2 * v2.transpose());
    Matrix3f S = B + B.transpose();
    Vector3f Z(B(1,2) - B(2,1), B(2,0) - B(0,2), B(0,1) - B(1,0));

    float sigma = B.trace();
    Matrix3f to_invert = (lambda_max + sigma) * Matrix3f::Identity() - S;
    if (to_invert.determinant() < 0.01f)
        return false;

    Vector3f Y = to_invert.inverse() * Z;

    float q_mult = 1 / sqrtf(1 + Y.squaredNorm());
    q_out[0] = Y[0] * q_mult;
    q_out[1] = Y[1] * q_mult;
    q_out[2] = Y[2] * q_mult;
    q_out[3] = q_mult;


    // Alternative Method

//    float delta = S.determinant();
//    float k = B.adjoint().trace();
//
//    float alpha = lambda_max * lambda_max - sigma * sigma + k;
//    float beta = lambda_max - sigma;
//    float gamma = (lambda_max + sigma)* alpha - delta;
//
//    Vector3f X = (alpha * Matrix3f::Identity() + beta * S + S * S) * Z;
//
//    float q_new[4];
//    float q_div = sqrtf(gamma* gamma + X.squaredNorm());
//    q_new[0] = X.x() / q_div;
//    q_new[1] = X.y() / q_div;
//    q_new[2] = X.z() / q_div;
//    q_new[3] = gamma / q_div;


//    Matrix4f K;
//    Matrix3f d = S - sigma * Matrix3f::Identity();
//    for (int i = 0; i < 3; i++) {
//        for (int j = 0; j < 3; j++) {
//            K(i,j) = d(i,j);
//        }
//        K(3,i) = Z[i];
//        K(i,3) = Z[i];
//        K(3,3) = sigma;
//    }
//
//    // This should be very close to 0
//    float test = (lambda_max * Matrix4f::Identity() - K).determinant();


    return true;
}

// Based on papers at -
// http://arc.aiaa.org/doi/pdf/10.2514/3.19717
// http://malcolmdshuster.com/FC_MarkleyMortari_Girdwood_1999_AAS.pdf
int quest_estimate(const float observations[2][3], const float references[2][3], const float a[2], float *q_out) {
    Vector3f v1(references[0][0], references[0][1], references[0][2]);
    Vector3f v2(references[1][0], references[1][1], references[1][2]);
    { // No sequential rotation
        Vector3f w1(observations[0][0], observations[0][1], observations[0][2]);
        Vector3f w2(observations[1][0], observations[1][1], observations[1][2]);
        if (try_estimate(v1, v2, w1, w2, a, q_out))
            return 1;
    }
    { // Initial rotation through pi about the x axis
        Vector3f w1(observations[0][0], -observations[0][1], -observations[0][2]);
        Vector3f w2(observations[1][0], -observations[1][1], -observations[1][2]);
        float q_temp[4];
        if (try_estimate(v1, v2, w1, w2, a, q_temp)) {
            q_out[0] = -q_temp[3];
            q_out[1] = -q_temp[2];
            q_out[2] = q_temp[1];
            q_out[3] = q_temp[0];
            return 2;
        }
    }
    { // Initial rotation through pi about the y axis
        Vector3f w1(-observations[0][0], observations[0][1], -observations[0][2]);
        Vector3f w2(-observations[1][0], observations[1][1], -observations[1][2]);
        float q_temp[4];
        if (try_estimate(v1, v2, w1, w2, a, q_temp)) {
            q_out[0] = q_temp[2];
            q_out[1] = -q_temp[3];
            q_out[2] = -q_temp[0];
            q_out[3] = q_temp[1];
            return 3;
        }
    }
    { // Initial rotation through pi about the z axis
        Vector3f w1(-observations[0][0], -observations[0][1], observations[0][2]);
        Vector3f w2(-observations[1][0], -observations[1][1], observations[1][2]);
        float q_temp[4];
        if (try_estimate(v1, v2, w1, w2, a, q_temp)) {
            q_out[0] = -q_temp[1];
            q_out[1] = q_temp[0];
            q_out[2] = -q_temp[3];
            q_out[3] = q_temp[2];
            return 4;
        }
    }
    return -1;
}

// Davenport's Q Method
int davenport_q_method(const float observations[][3], const float references[][3], const float a[], int n, float q_out[4]) {
    Vector3f v1(references[0][0], references[0][1], references[0][2]);
    Vector3f v2(references[1][0], references[1][1], references[1][2]);
    Vector3f w1(observations[0][0], observations[0][1], observations[0][2]);
    Vector3f w2(observations[1][0], observations[1][1], observations[1][2]);

    Matrix3f B = Matrix3f::Zero();

    for (int i = 0; i < n; i++) {
        Vector3f w = Map<const Vector3f>(observations[i]);
        Vector3f v = Map<const Vector3f>(references[i]);
        B += a[i] * (w * v.transpose());
    }

    Matrix3f S = B + B.transpose();
    Vector3f Z(B(1,2) - B(2,1), B(2,0) - B(0,2), B(0,1) - B(1,0));

    float sigma = B.trace();

    Matrix4f K;
    Matrix3f t = S - sigma *  Matrix3f::Identity();
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            K(i,j) = t(i,j);

    for (int i = 0; i < 3; i++) {
        K(3, i) = Z(i);
        K(i, 3) = Z(i);
    }
    K(3,3) = sigma;

    SelfAdjointEigenSolver<Matrix4f> solver;
    solver.compute(K);

    auto values = solver.eigenvalues();

    float max = -INFINITY;
    int max_i = -1;
    for (int i = 0; i < 4; i++) {
        if (values(i) > max) {
            max = values(i);
            max_i = i;
        }
    }

    auto vector = solver.eigenvectors().col(max_i);

    q_out[0] = vector(0);
    q_out[1] = vector(1);
    q_out[2] = vector(2);
    q_out[3] = vector(3);

    return 1;
}