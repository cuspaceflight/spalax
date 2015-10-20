#ifndef QUEST_ESTIMATOR_H
#define QUEST_ESTIMATOR_H

// The calibrated Accelerometer reading in local coordinates - magnitude doesn't matter only the direction is important
void quest_estimator_new_accel(const float accel[3]);

// The calibrated magnetometer reading in local coordinates - magnitude doesn't matter only the direction is important
void quest_estimator_new_mag(const float mag[3]);

// Compute the current quaternion based on the current data
void quest_estimator_update(float q[4]);

void quest_estimator_set_reference_vectors(float accel_reference[3], float mag_reference[3]);

#endif /* QUEST_ESTIMATOR_H */