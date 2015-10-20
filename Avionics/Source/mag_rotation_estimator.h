#ifndef MAG_ROTATION_ESTIMATOR_H
#define MAG_ROTATION_ESTIMATOR_H

void mag_rotation_estimator_set_reference_vector(float mag_reference[3]);

void mag_rotation_estimator_update(float euler_angles[3], float q[4]);

void mag_rotation_estimator_new_mag(float mag[3]);

#endif /* MAG_ROTATION_ESTIMATOR_H */