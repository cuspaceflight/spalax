#include "quest.h"
#include "Eigen/Core"


void quest_estimate(const float observations[2][3], const float references[2][3], float *q_out) {
    Eigen::Vector3f observation1(observations[0][0], observations[0][1], observations[0][2]);
    Eigen::Vector3f observation2(observations[1][0], observations[1][1], observations[1][2]);

}
