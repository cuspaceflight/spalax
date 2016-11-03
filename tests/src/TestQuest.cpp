#include <gmock/gmock.h>
#include "state/quest.h"


TEST(TestQuest, TestSimple) {
    const float observations[2][3] = {
            {1.0, 0.0, 0.0f},
            {0.0, 1.0, 0.0f}
    };

    const float references[2][3] = {
            {1.0, 0.0, 0.0f},
            {0.0, 1.0, 0.0f}
    };


    float q_out[4];

    quest_estimate(observations, references, q_out);


}