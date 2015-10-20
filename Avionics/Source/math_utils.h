#ifndef MATH_UTILS_H
#define MATH_UTILS_H
#include <float.h>
#include <math.h>
#include <logging.h>

#define PI 3.14159265358979323846f

static float mat2x2_det(const float mat[2][2]) {
	return mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
}

static void mat2x2_inv(const float mat[2][2], float out[2][2]) {
	out[0][0] = mat[1][1];
	out[1][1] = mat[0][0];
	out[0][1] = -mat[0][1];
	out[1][0] = -mat[1][0];
}

static float mat3x3_det(const float mat[3][3]) {
	return mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2])
		- mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
		+ mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
}

static void mat3x3_inv(const float m[3][3], float out[3][3]) {
	float invdet = 1 / mat3x3_det(m);
	if (invdet != invdet) {
		PRINT("4x4 inverse Nan Error!\n");
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

static void mat3x3_mult(const float a[3][3], const float b[3][3], float out[3][3]) {
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

static float mat4x4_det(const float m[4][4]) {
	return m[0][3] * m[1][2] * m[2][1] * m[3][0] - m[0][2] * m[1][3] * m[2][1] * m[3][0] - m[0][3] * m[1][1] * m[2][2] * m[3][0] + m[0][1] * m[1][3] * m[2][2] * m[3][0] +
		m[0][2] * m[1][1] * m[2][3] * m[3][0] - m[0][1] * m[1][2] * m[2][3] * m[3][0] - m[0][3] * m[1][2] * m[2][0] * m[3][1] + m[0][2] * m[1][3] * m[2][0] * m[3][1] +
		m[0][3] * m[1][0] * m[2][2] * m[3][1] - m[0][0] * m[1][3] * m[2][2] * m[3][1] - m[0][2] * m[1][0] * m[2][3] * m[3][1] + m[0][0] * m[1][2] * m[2][3] * m[3][1] +
		m[0][3] * m[1][1] * m[2][0] * m[3][2] - m[0][1] * m[1][3] * m[2][0] * m[3][2] - m[0][3] * m[1][0] * m[2][1] * m[3][2] + m[0][0] * m[1][3] * m[2][1] * m[3][2] +
		m[0][1] * m[1][0] * m[2][3] * m[3][2] - m[0][0] * m[1][1] * m[2][3] * m[3][2] - m[0][2] * m[1][1] * m[2][0] * m[3][3] + m[0][1] * m[1][2] * m[2][0] * m[3][3] +
		m[0][2] * m[1][0] * m[2][1] * m[3][3] - m[0][0] * m[1][2] * m[2][1] * m[3][3] - m[0][1] * m[1][0] * m[2][2] * m[3][3] + m[0][0] * m[1][1] * m[2][2] * m[3][3];
}

static void mat4x4_inv(const float m[4][4], float out[4][4]) {
	float invdet = 1 / mat4x4_det(m);
	if (invdet != invdet) {
		PRINT("4x4 inverse Nan Error!\n"); 
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				PRINT("%f\n", m[i][j]);
		return;
	}
	else if (invdet == FP_INFINITE) {
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

static void mat4x4_mult(const float a[4][4], const float b[4][4], float out[4][4]) {
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

static float vector_dot(const float a[3], const float b[3]) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void vector_cross(const float a[3], const float b[3], float out[3]) {
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}

static float vector_cross_mag(const float a[3], const float b[3]) {
	float x = a[1] * b[2] - a[2] * b[1];
	float y = a[2] * b[0] - a[0] * b[2];
	float z = a[0] * b[1] - a[1] * b[0];

	return sqrtf(x*x + y*y + z*z);
}

static void axis_angle_to_euler(const float axis[3], const float angle, float out[3]) {
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

	out[0] = atan2f(y * s - x * z * t, 1 - (y*y + z*z) * t);
	out[1] = asinf(x * y * t + z * s);
	out[2] = atan2f(x * s - y * z * t, 1 - (x*x + z*z) * t);
}

// Warning: This code does not handle gimbal lock and so should not be used for anything crucial
// It will return incorrect data y = +-pi/2
// It is meant as a function to help debug quaternions
// It uses xyz ordering (ie x rotation followed by y rotation followed by z rotation)
static void quat_to_euler(const float q[4], float xyz[3]) {
	xyz[0] = -atan2f(2 * (q[1] * q[2] - q[0] * q[3]), 1 - 2 * (q[0] * q[0] + q[1] * q[1]));
	xyz[1] = asinf(2 * (q[0] * q[2] + q[1] * q[3]));
	xyz[2] = -atan2f(2 * (q[0] * q[1] - q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));


	/*xyz[0] = atan2f(2.0f * (q[1] * q[2] + q[3] * q[0]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2]);
	xyz[1] = asinf(-2.0f * (q[0] * q[2] - q[3] * q[1]));
	xyz[2] = atan2f(2.0f * (q[0] * q[1] + q[3] * q[2]), q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2]);*/
}

static void axis_angle_to_quat(const float axis[3], const float angle, float q[4]) {
	float s = sinf(angle / 2.0f);
	float x = axis[0];
	float y = axis[1];
	float z = axis[2];
	float w = cosf(angle / 2.0f);

	float axis_norm = sqrtf(x*x + y*y + z*z);
	float mult = s / axis_norm;

	x *= mult;
	y *= mult;
	z *= mult;

	float norm = sqrtf(x*x + y*y + z*z + w*w);
	q[0] = x / norm;
	q[1] = y / norm;
	q[2] = z / norm;
	q[3] = w / norm;
}

static void quat_rotate(const float q[4], float v[3], float out[3]) {
	float uv[3];
	float uuv[3];

	vector_cross(q, v, uv);
	vector_cross(q, uv, uuv);

	out[0] = v[0] + ((uv[0] * q[3]) + uuv[0])*2.0f;
	out[1] = v[1] + ((uv[1] * q[3]) + uuv[1])*2.0f;
	out[2] = v[2] + ((uv[2] * q[3]) + uuv[2])*2.0f;
}
#endif /* MATH_UTILS_H */