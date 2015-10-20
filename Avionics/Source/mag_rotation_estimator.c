#include "mag_rotation_estimator.h"
#include <math_utils.h>
#include <logging.h>

float MO[3];
float MR[3];

void mag_rotation_estimator_set_reference_vector(float mag_reference[3]) {
	float norm = sqrtf(mag_reference[0] * mag_reference[0] + mag_reference[1] * mag_reference[1] + mag_reference[2] * mag_reference[2]);

	MR[0] = mag_reference[0] / norm;
	MR[1] = mag_reference[1] / norm;
	MR[2] = mag_reference[2] / norm;
}

void mag_rotation_estimator_new_mag(float mag[3]) {
	float norm = sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
	MO[0] = mag[0] / norm;
	MO[1] = mag[1] / norm;
	MO[2] = mag[2] / norm;
}

void mag_rotation_estimator_update(float euler_angles[3], float quaternion[4]) {
	float cross[3];
	float v[3];
	v[0] = (MO[0] + MR[0])/2.0f;
	v[1] = (MO[1] + MR[1])/2.0f;
	v[2] = (MO[2] + MR[2])/2.0f;

	vector_cross(v, MR, cross);
	float dot = vector_dot(v, MR);
	quaternion[0] = cross[0];
	quaternion[1] = cross[1];
	quaternion[2] = cross[2];
	quaternion[3] = dot;
}