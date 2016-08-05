#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#define PI 3.14159265358979323846f
#define PI_2 1.57079632679489661923f

float mat2x2_det(float mat[2][2]);

void mat2x2_inv(float mat[2][2], float out[2][2]);

float mat3x3_det(float mat[3][3]);

void mat3x3_inv(float m[3][3], float out[3][3]);

void mat3x3_mult(float a[3][3], float b[3][3], float out[3][3]);

float mat4x4_det(float m[4][4]);

void mat4x4_inv(float m[4][4], float out[4][4]);

void mat4x4_mult(float a[4][4], float b[4][4], float out[4][4]);

float vector_dot(const float a[3], const float b[3]);

void vector_cross(const float a[3], const float b[3], float out[3]);

float vector_cross_mag(const float a[3], const float b[3]);

void axis_angle_to_euler(const float axis[3], const float angle, float out[3]);

// Warning: This code does not handle gimbal lock and so should not be used for anything crucial
// It will return incorrect data y = +-pi/2
// It is meant as a function to help debug quaternions
// It uses xyz ordering (ie x rotation followed by y rotation followed by z rotation)
void quat_to_euler(const float q[4], float xyz[3]);

void axis_angle_to_quat(const float axis[3], const float angle, float q[4]);

void quaternion_to_rodrigues(const float q[4], float mrp[3]);

void rodrigues_to_quaternion(const float mrp[3], float q[4]);

void quat_mult(const float q1[4], const float q2[4], float out[4]);

void quat_inverse(const float q[4], float out[4]);

float vector_mag(const float v[3]);

void quat_rotate(const float q[4], const float v[3], float out[3]);

// Wrapper around quaternion multiplication which handles zero length vectors
void apply_q(const float q[4], const float v[3], float out[3]);

void normalize(const float v[3], float out[3]);

#ifdef __cplusplus
}
#endif

#endif /* MATH_UTILS_H */
