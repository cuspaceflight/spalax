#include "math_utils.h"

float mat2x2_det(float mat[2][2]) {
    return mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
}

void mat2x2_inv(float mat[2][2], float out[2][2]) {
    out[0][0] = mat[1][1];
    out[1][1] = mat[0][0];
    out[0][1] = -mat[0][1];
    out[1][0] = -mat[1][0];
}

float mat3x3_det(float mat[3][3]) {
    return mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2])
        - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
        + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
}

void mat3x3_inv(float m[3][3], float out[3][3]) {
    float invdet = 1 / mat3x3_det(m);
    if (invdet != invdet) {
        PRINT("3x3 inverse Nan Error!\n");
        return;
    }
    out[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invdet;
    out[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;
    out[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;
    out[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;
    out[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
    out[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * invdet;
    out[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invdet;
    out[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * invdet;
    out[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invdet;
}

void mat3x3_mult(float a[3][3], float b[3][3], float out[3][3]) {
    out[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0];
    out[0][1] = a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1];
    out[0][2] = a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2];
    out[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0];
    out[1][1] = a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1];
    out[1][2] = a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2];
    out[2][0] = a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0];
    out[2][1] = a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1];
    out[2][2] = a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2];
}

float mat4x4_det(float m[4][4]) {
    return m[0][3] * m[1][2] * m[2][1] * m[3][0] - m[0][2] * m[1][3] * m[2][1] * m[3][0] - m[0][3] * m[1][1] * m[2][2] * m[3][0] + m[0][1] * m[1][3] * m[2][2] * m[3][0] +
        m[0][2] * m[1][1] * m[2][3] * m[3][0] - m[0][1] * m[1][2] * m[2][3] * m[3][0] - m[0][3] * m[1][2] * m[2][0] * m[3][1] + m[0][2] * m[1][3] * m[2][0] * m[3][1] +
        m[0][3] * m[1][0] * m[2][2] * m[3][1] - m[0][0] * m[1][3] * m[2][2] * m[3][1] - m[0][2] * m[1][0] * m[2][3] * m[3][1] + m[0][0] * m[1][2] * m[2][3] * m[3][1] +
        m[0][3] * m[1][1] * m[2][0] * m[3][2] - m[0][1] * m[1][3] * m[2][0] * m[3][2] - m[0][3] * m[1][0] * m[2][1] * m[3][2] + m[0][0] * m[1][3] * m[2][1] * m[3][2] +
        m[0][1] * m[1][0] * m[2][3] * m[3][2] - m[0][0] * m[1][1] * m[2][3] * m[3][2] - m[0][2] * m[1][1] * m[2][0] * m[3][3] + m[0][1] * m[1][2] * m[2][0] * m[3][3] +
        m[0][2] * m[1][0] * m[2][1] * m[3][3] - m[0][0] * m[1][2] * m[2][1] * m[3][3] - m[0][1] * m[1][0] * m[2][2] * m[3][3] + m[0][0] * m[1][1] * m[2][2] * m[3][3];
}

void mat4x4_inv(float m[4][4], float out[4][4]) {
    float invdet = 1 / mat4x4_det(m);
    if (invdet != invdet) {
        PRINT("4x4 inverse Nan Error!\n");
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
            PRINT("%f\n", m[i][j]);
        return;
    } else if (invdet == FP_INFINITE) {
        PRINT("4x4 inverse Infinity Error!\n");
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
            PRINT("%f\n", m[i][j]);
        return;
    }

    PRINT("%f\n", invdet);

    out[0][0] = (m[1][2] * m[2][3] * m[3][1] - m[1][3] * m[2][2] * m[3][1] + m[1][3] * m[2][1] * m[3][2] - m[1][1] * m[2][3] * m[3][2] - m[1][2] * m[2][1] * m[3][3] + m[1][1] * m[2][2] * m[3][3]) * invdet;
    out[0][1] = (m[0][3] * m[2][2] * m[3][1] - m[0][2] * m[2][3] * m[3][1] - m[0][3] * m[2][1] * m[3][2] + m[0][1] * m[2][3] * m[3][2] + m[0][2] * m[2][1] * m[3][3] - m[0][1] * m[2][2] * m[3][3]) * invdet;
    out[0][2] = (m[0][2] * m[1][3] * m[3][1] - m[0][3] * m[1][2] * m[3][1] + m[0][3] * m[1][1] * m[3][2] - m[0][1] * m[1][3] * m[3][2] - m[0][2] * m[1][1] * m[3][3] + m[0][1] * m[1][2] * m[3][3]) * invdet;
    out[0][3] = (m[0][3] * m[1][2] * m[2][1] - m[0][2] * m[1][3] * m[2][1] - m[0][3] * m[1][1] * m[2][2] + m[0][1] * m[1][3] * m[2][2] + m[0][2] * m[1][1] * m[2][3] - m[0][1] * m[1][2] * m[2][3]) * invdet;
    out[1][0] = (m[1][3] * m[2][2] * m[3][0] - m[1][2] * m[2][3] * m[3][0] - m[1][3] * m[2][0] * m[3][2] + m[1][0] * m[2][3] * m[3][2] + m[1][2] * m[2][0] * m[3][3] - m[1][0] * m[2][2] * m[3][3]) * invdet;
    out[1][1] = (m[0][2] * m[2][3] * m[3][0] - m[0][3] * m[2][2] * m[3][0] + m[0][3] * m[2][0] * m[3][2] - m[0][0] * m[2][3] * m[3][2] - m[0][2] * m[2][0] * m[3][3] + m[0][0] * m[2][2] * m[3][3]) * invdet;
    out[1][2] = (m[0][3] * m[1][2] * m[3][0] - m[0][2] * m[1][3] * m[3][0] - m[0][3] * m[1][0] * m[3][2] + m[0][0] * m[1][3] * m[3][2] + m[0][2] * m[1][0] * m[3][3] - m[0][0] * m[1][2] * m[3][3]) * invdet;
    out[1][3] = (m[0][2] * m[1][3] * m[2][0] - m[0][3] * m[1][2] * m[2][0] + m[0][3] * m[1][0] * m[2][2] - m[0][0] * m[1][3] * m[2][2] - m[0][2] * m[1][0] * m[2][3] + m[0][0] * m[1][2] * m[2][3]) * invdet;
    out[2][0] = (m[1][1] * m[2][3] * m[3][0] - m[1][3] * m[2][1] * m[3][0] + m[1][3] * m[2][0] * m[3][1] - m[1][0] * m[2][3] * m[3][1] - m[1][1] * m[2][0] * m[3][3] + m[1][0] * m[2][1] * m[3][3]) * invdet;
    out[2][1] = (m[0][3] * m[2][1] * m[3][0] - m[0][1] * m[2][3] * m[3][0] - m[0][3] * m[2][0] * m[3][1] + m[0][0] * m[2][3] * m[3][1] + m[0][1] * m[2][0] * m[3][3] - m[0][0] * m[2][1] * m[3][3]) * invdet;
    out[2][2] = (m[0][1] * m[1][3] * m[3][0] - m[0][3] * m[1][1] * m[3][0] + m[0][3] * m[1][0] * m[3][1] - m[0][0] * m[1][3] * m[3][1] - m[0][1] * m[1][0] * m[3][3] + m[0][0] * m[1][1] * m[3][3]) * invdet;
    out[2][3] = (m[0][3] * m[1][1] * m[2][0] - m[0][1] * m[1][3] * m[2][0] - m[0][3] * m[1][0] * m[2][1] + m[0][0] * m[1][3] * m[2][1] + m[0][1] * m[1][0] * m[2][3] - m[0][0] * m[1][1] * m[2][3]) * invdet;
    out[3][0] = (m[1][2] * m[2][1] * m[3][0] - m[1][1] * m[2][2] * m[3][0] - m[1][2] * m[2][0] * m[3][1] + m[1][0] * m[2][2] * m[3][1] + m[1][1] * m[2][0] * m[3][2] - m[1][0] * m[2][1] * m[3][2]) * invdet;
    out[3][1] = (m[0][1] * m[2][2] * m[3][0] - m[0][2] * m[2][1] * m[3][0] + m[0][2] * m[2][0] * m[3][1] - m[0][0] * m[2][2] * m[3][1] - m[0][1] * m[2][0] * m[3][2] + m[0][0] * m[2][1] * m[3][2]) * invdet;
    out[3][2] = (m[0][2] * m[1][1] * m[3][0] - m[0][1] * m[1][2] * m[3][0] - m[0][2] * m[1][0] * m[3][1] + m[0][0] * m[1][2] * m[3][1] + m[0][1] * m[1][0] * m[3][2] - m[0][0] * m[1][1] * m[3][2]) * invdet;
    out[3][3] = (m[0][1] * m[1][2] * m[2][0] - m[0][2] * m[1][1] * m[2][0] + m[0][2] * m[1][0] * m[2][1] - m[0][0] * m[1][2] * m[2][1] - m[0][1] * m[1][0] * m[2][2] + m[0][0] * m[1][1] * m[2][2]) * invdet;
}

void mat4x4_mult(float a[4][4], float b[4][4], float out[4][4]) {
    out[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0] + a[0][3] * b[3][0];
    out[0][1] = a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1] + a[0][3] * b[3][1];
    out[0][2] = a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2] + a[0][3] * b[3][2];
    out[0][3] = a[0][0] * b[0][3] + a[0][1] * b[1][3] + a[0][2] * b[2][3] + a[0][3] * b[3][3];
    out[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0] + a[1][3] * b[3][0];
    out[1][1] = a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1] + a[1][3] * b[3][1];
    out[1][2] = a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2] + a[1][3] * b[3][2];
    out[1][3] = a[1][0] * b[0][3] + a[1][1] * b[1][3] + a[1][2] * b[2][3] + a[1][3] * b[3][3];
    out[2][0] = a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0] + a[2][3] * b[3][0];
    out[2][1] = a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1] + a[2][3] * b[3][1];
    out[2][2] = a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2] + a[2][3] * b[3][2];
    out[2][3] = a[2][0] * b[0][3] + a[2][1] * b[1][3] + a[2][2] * b[2][3] + a[2][3] * b[3][3];
    out[3][0] = a[3][0] * b[0][0] + a[3][1] * b[1][0] + a[3][2] * b[2][0] + a[3][3] * b[3][0];
    out[3][1] = a[3][0] * b[0][1] + a[3][1] * b[1][1] + a[3][2] * b[2][1] + a[3][3] * b[3][1];
    out[3][2] = a[3][0] * b[0][2] + a[3][1] * b[1][2] + a[3][2] * b[2][2] + a[3][3] * b[3][2];
    out[3][3] = a[3][0] * b[0][3] + a[3][1] * b[1][3] + a[3][2] * b[2][3] + a[3][3] * b[3][3];
}

float vector_dot(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_cross(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

float vector_cross_mag(const float a[3], const float b[3]) {
    float x = a[1] * b[2] - a[2] * b[1];
    float y = a[2] * b[0] - a[0] * b[2];
    float z = a[0] * b[1] - a[1] * b[0];

    return sqrtf(x * x + y * y + z * z);
}

void axis_angle_to_euler(const float axis[3], const float angle, float out[3]) {
    float s = sinf(angle);
    float c = cosf(angle);
    float t = 1 - c;

    float x = axis[0];
    float y = axis[1];
    float z = axis[2];

    float singularity_test = (x * y * t - z * s);

    if (singularity_test > 0.998) {
        return;
    }
    if (singularity_test < -0.998) {
        return;
    }

    out[0] = atan2f(y * s - x * z * t, 1 - (y * y + z * z) * t);
    out[1] = asinf(x * y * t + z * s);
    out[2] = atan2f(x * s - y * z * t, 1 - (x * x + z * z) * t);
}

void quat_to_euler(const float q[4], float xyz[3]) {
    xyz[0] = -atan2f(2 * (q[1] * q[2] - q[0] * q[3]), 1 - 2 * (q[0] * q[0] + q[1] * q[1]));
    xyz[1] = asinf(2 * (q[0] * q[2] + q[1] * q[3]));
    xyz[2] = -atan2f(2 * (q[0] * q[1] - q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));


    /*xyz[0] = atan2f(2.0f * (q[1] * q[2] + q[3] * q[0]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2]);
    xyz[1] = asinf(-2.0f * (q[0] * q[2] - q[3] * q[1]));
    xyz[2] = atan2f(2.0f * (q[0] * q[1] + q[3] * q[2]), q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2]);*/
}

void axis_angle_to_quat(const float axis[3], const float angle, float q[4]) {
    float s = sinf(angle / 2.0f);
    float x = axis[0];
    float y = axis[1];
    float z = axis[2];
    float w = cosf(angle / 2.0f);

    float axis_norm = sqrtf(x * x + y * y + z * z);
    float mult = s / axis_norm;

    x *= mult;
    y *= mult;
    z *= mult;

    float norm = sqrtf(x * x + y * y + z * z + w * w);
    q[0] = x / norm;
    q[1] = y / norm;
    q[2] = z / norm;
    q[3] = w / norm;
}

void quaternion_to_rodrigues(const float q[4], float mrp[3]) {
    mrp[0] = q[0] / (1 + q[3]);
    mrp[1] = q[1] / (1 + q[3]);
    mrp[2] = q[2] / (1 + q[3]);
}

void rodrigues_to_quaternion(const float mrp[3], float q[4]) {
    //
    //  mrp[0-2] = n * tan(theta/4) = 1/(1+q[3])q[0-2] = 1/(1+cos(theta/2)) * (n * sin(theta/2));
    //
    // sin(theta/2) = sin(2*atan(mag(mrp))) = 2 * mag(mrp) / (mag(mrp)*mag(mrp) + 1)
    // cos(theta/2) = cos(2*atan(mag(mrp))) = (1 - mag(mrp)*mag(mrp)) / (a + mag(mrp)*mag(mrp))

    float mag_mrp_squared = mrp[0] * mrp[0] + mrp[1] * mrp[1] + mrp[2] * mrp[2];
    float mag_mrp = sqrtf(mag_mrp_squared);

    if (mag_mrp < FLT_EPSILON) {
        q[0] = 0;
        q[1] = 0;
        q[2] = 0;
        q[3] = 1;
        return;
    }

    float sin_theta_2 = 2 * mag_mrp / (mag_mrp_squared + 1);
    float cos_theta_2 = (1 - mag_mrp_squared) / (mag_mrp_squared + 1);

    q[0] = sin_theta_2 * mrp[0] / mag_mrp;
    q[1] = sin_theta_2 * mrp[1] / mag_mrp;
    q[2] = sin_theta_2 * mrp[2] / mag_mrp;
    q[3] = cos_theta_2;
}

void quat_mult(const float q1[4], const float q2[4], float out[4]) {
    out[0] = q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1];
    out[1] = q1[3] * q2[1] - q1[0] * q2[2] + q1[1] * q2[3] + q1[2] * q2[0];
    out[2] = q1[3] * q2[2] + q1[0] * q2[1] - q1[1] * q2[0] + q1[2] * q2[3];
    out[3] = q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2];
}

float vector_mag(const float v[3]) {
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void quat_rotate(const float q[4], const float v[3], float out[3]) {
    float uv[3];
    float uuv[3];

    vector_cross(q, v, uv);
    vector_cross(q, uv, uuv);

    out[0] = v[0] + ((uv[0] * q[3]) + uuv[0]) * 2.0f;
    out[1] = v[1] + ((uv[1] * q[3]) + uuv[1]) * 2.0f;
    out[2] = v[2] + ((uv[2] * q[3]) + uuv[2]) * 2.0f;
}

void apply_q(const float q[4], const float v[3], float out[3]) {
    float v_mag = vector_mag(v);
    if (v_mag < 1e-10f) {
        for (int i = 0; i < 3; i++)
            out[i] = v[i];
        return;
    }
    quat_rotate(q, v, out);
}

void normalize(const float v[3], float out[3]) {
    float v_mag = vector_mag(v);
    for (int i = 0; i < 3; i++)
        out[i] = v[i] / v_mag;
}
