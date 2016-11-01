#include "math_utils.h"
#include "math.h"
#include <float.h>

float mat2x2_det(float mat[2][2]) {
    return mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
}

void mat2x2_inv(float mat[2][2], float out[2][2]) {
    float invdet = 1 / mat2x2_det(mat);
    if (invdet != invdet) {
        return;
    }
    out[0][0] = mat[1][1] * invdet;
    out[1][1] = mat[0][0] * invdet;
    out[0][1] = -mat[0][1] * invdet;
    out[1][0] = -mat[1][0] * invdet;
}

float mat3x3_det(float mat[3][3]) {
    return mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2])
        - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
        + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
}

void mat3x3_inv(float m[3][3], float out[3][3]) {
    float invdet = 1 / mat3x3_det(m);
    if (invdet != invdet) {
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
    // Based on code from glm
    float SubFactor00 = m[2][2] * m[3][3] - m[2][3] * m[3][2];
    float SubFactor01 = m[1][2] * m[3][3] - m[1][3] * m[3][2];
    float SubFactor02 = m[1][2] * m[2][3] - m[1][3] * m[2][2];
    float SubFactor03 = m[0][2] * m[3][3] - m[0][3] * m[3][2];
    float SubFactor04 = m[0][2] * m[2][3] - m[0][3] * m[2][2];
    float SubFactor05 = m[0][2] * m[1][3] - m[0][3] * m[1][2];

    float DetCof[4] = {
        +(m[1][1] * SubFactor00 - m[2][1] * SubFactor01 + m[3][1] * SubFactor02),
        -(m[0][1] * SubFactor00 - m[2][1] * SubFactor03 + m[3][1] * SubFactor04),
        +(m[0][1] * SubFactor01 - m[1][1] * SubFactor03 + m[3][1] * SubFactor05),
        -(m[0][1] * SubFactor02 - m[1][1] * SubFactor04 + m[2][1] * SubFactor05)
    };

    return m[0][0] * DetCof[0] + m[1][0] * DetCof[1] + m[2][0] * DetCof[2] + m[3][0] * DetCof[3];
}

void mat4x4_inv(float m[4][4], float out[4][4]) {
    // Based on code from glm
    float Coef00 = m[2][2] * m[3][3] - m[2][3] * m[3][2];
    float Coef02 = m[2][1] * m[3][3] - m[2][3] * m[3][1];
    float Coef03 = m[2][1] * m[3][2] - m[2][2] * m[3][1];

    float Coef04 = m[1][2] * m[3][3] - m[1][3] * m[3][2];
    float Coef06 = m[1][1] * m[3][3] - m[1][3] * m[3][1];
    float Coef07 = m[1][1] * m[3][2] - m[1][2] * m[3][1];

    float Coef08 = m[1][2] * m[2][3] - m[1][3] * m[2][2];
    float Coef10 = m[1][1] * m[2][3] - m[1][3] * m[2][1];
    float Coef11 = m[1][1] * m[2][2] - m[1][2] * m[2][1];

    float Coef12 = m[0][2] * m[3][3] - m[0][3] * m[3][2];
    float Coef14 = m[0][1] * m[3][3] - m[0][3] * m[3][1];
    float Coef15 = m[0][1] * m[3][2] - m[0][2] * m[3][1];

    float Coef16 = m[0][2] * m[2][3] - m[0][3] * m[2][2];
    float Coef18 = m[0][1] * m[2][3] - m[0][3] * m[2][1];
    float Coef19 = m[0][1] * m[2][2] - m[0][2] * m[2][1];

    float Coef20 = m[0][2] * m[1][3] - m[0][3] * m[1][2];
    float Coef22 = m[0][1] * m[1][3] - m[0][3] * m[1][1];
    float Coef23 = m[0][1] * m[1][2] - m[0][2] * m[1][1];

    float Fac0[4] = { Coef00, Coef00, Coef02, Coef03 };
    float Fac1[4] = { Coef04, Coef04, Coef06, Coef07 };
    float Fac2[4] = { Coef08, Coef08, Coef10, Coef11 };
    float Fac3[4] = { Coef12, Coef12, Coef14, Coef15 };
    float Fac4[4] = { Coef16, Coef16, Coef18, Coef19 };
    float Fac5[4] = { Coef20, Coef20, Coef22, Coef23 };

    float Vec0[4] = { m[0][1], m[0][0], m[0][0], m[0][0] };
    float Vec1[4] = { m[1][1], m[1][0], m[1][0], m[1][0] };
    float Vec2[4] = { m[2][1], m[2][0], m[2][0], m[2][0] };
    float Vec3[4] = { m[3][1], m[3][0], m[3][0], m[3][0] };

    float Inv0[4];
    for (int i = 0; i < 4; i++)
        Inv0[i] = Vec1[i] * Fac0[i] - Vec2[i] * Fac1[i] + Vec3[i] * Fac2[i];

    float Inv1[4];
    for (int i = 0; i < 4; i++)
        Inv1[i] = Vec0[i] * Fac0[i] - Vec2[i] * Fac3[i] + Vec3[i] * Fac4[i];
    
    float Inv2[4];
    for (int i = 0; i < 4; i++)
        Inv2[i] = Vec0[i] * Fac1[i] - Vec1[i] * Fac3[i] + Vec3[i] * Fac5[i];

    float Inv3[4];
    for (int i = 0; i < 4; i++)
        Inv3[i] = (Vec0[i] * Fac2[i] - Vec1[i] * Fac4[i] + Vec2[i] * Fac5[i]);

    float SignA[4] = { +1, -1, +1, -1 };
    float SignB[4] = { -1, +1, -1, +1 };

    float OneOverDeterminant = 1 / mat4x4_det(m);
    
    for (int i = 0; i < 4; i++) {
        out[i][0] = Inv0[i] * SignA[i] * OneOverDeterminant;
        out[i][1] = Inv1[i] * SignB[i] * OneOverDeterminant;
        out[i][2] = Inv2[i] * SignA[i] * OneOverDeterminant;
        out[i][3] = Inv3[i] * SignB[i] * OneOverDeterminant;
    }
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

float vector_mag(const float v[3]) {
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

float vector_cross_mag(const float a[3], const float b[3]) {
    float v[3];
    vector_cross(a, b, v);
    return vector_mag(v);
}

float vector_normalize(const float v[3], float out[3]) {
    float v_mag = vector_mag(v);
    for (int i = 0; i < 3; i++)
        out[i] = v[i] / v_mag;
    return v_mag;
}

void axis_angle_to_quat(const float axis[3], const float angle, float q[4]) {
    float s = sinf(angle / 2.0f);

    q[0] = axis[0] * s;
    q[1] = axis[1] * s;
    q[2] = axis[2] * s;
    q[3] = cosf(angle / 2.0f);
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

void quat_invert(const float q[4], float out[4]) {
    float dot = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    out[0] = -q[0] / dot;
    out[1] = -q[1] / dot;
    out[2] = -q[2] / dot;
    out[3] = q[3] / dot;
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