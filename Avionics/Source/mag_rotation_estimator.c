#include "mag_rotation_estimator.h"
#include <math_utils.h>
#include <logging.h>

// Whether to print the magnetometer data rotated by the computed quaternion
// This is usefull for determining the accuracy of the orientation
//#define PRINT_ROTATED_MAG

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

    PRINT("MO: (%f, %f, %f)\n", MO[0], MO[1], MO[2]);
}

void mag_rotation_estimator_update(float quaternion[4]) {
    // MO and MR are guaranteed to be normalized
	float cross[3];
	vector_cross(MO, MR, cross);

    float cos_angle = vector_dot(MO, MR);
    
    // cos(theta/2) = sqrt((1+cos(theta))/ 2)
    // sin(theta/2) = sqrt((1-cos(theta))/ 2)
    float cos_half_angle = sqrt(0.5f * (1.f + cos_angle));
    float sin_half_angle = sqrt(0.5f * (1.f - cos_angle));

    // cross_mag = sin(theta) = 2 * sin(theta/2) * cos(theta/2)
    // The sin(theta/2) cancels with the sin(theta/2) below and so can be removed
    float cross_mag = 2 * cos_half_angle;

    quaternion[0] = cross[0] / cross_mag;
    quaternion[1] = cross[1] / cross_mag;
    quaternion[2] = cross[2] / cross_mag;
    quaternion[3] = cos_half_angle;

#ifdef PRINT_ROTATED_MAG
    float rotatedMag[3];
    float inverseQuat[4];

    inverseQuat[0] = -quaternion[0];
    inverseQuat[1] = -quaternion[1];
    inverseQuat[2] = -quaternion[2];
    inverseQuat[3] = quaternion[3];


    quat_rotate(quaternion, MO, rotatedMag);

    // These should be approximately equal
    PRINT("Reference (%f,%f,%f) Rotated (%f,%f,%f)\n", MR[0], MR[1], MR[2], rotatedMag[0], rotatedMag[1], rotatedMag[2]);
#endif
}