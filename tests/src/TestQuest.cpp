#include <gmock/gmock.h>
#include "state/quest.h"
#include "Eigen/Core"
#include <Eigen/Geometry>

bool fuzzy_eq(float A, float B) {
    const float maxRelativeError = 0.00005f;
    const float maxAbsoluteError = 1e-5f;

    if (fabs(A - B) < maxAbsoluteError)
        return true;
    float relativeError;
    if (fabs(B) > fabs(A))
        relativeError = fabs((A - B) / B);
    else
        relativeError = fabs((A - B) / A);
    return relativeError <= maxRelativeError;
}

void expect_fuzzy_eq(float A, float B) {
    EXPECT_TRUE(fuzzy_eq(A, B));
}

TEST(TestQuest, TestIdentity) {
    const float observations[2][3] = {
            {1.0, 0.0, 0.0f},
            {0.0, 1.0, 0.0f}
    };

    const float references[2][3] = {
            {1.0, 0.0, 0.0f},
            {0.0, 1.0, 0.0f}
    };

    const float a[2] = {0.5f, 0.5f};


    float q_out[4];

    quest_estimate(observations, references, a, q_out);

    expect_fuzzy_eq(q_out[0], 0);
    expect_fuzzy_eq(q_out[1], 0);
    expect_fuzzy_eq(q_out[2], 0);
    expect_fuzzy_eq(q_out[3], 1);
}

TEST(TestQuest, TestIdentity2) {
    float observations[2][3] = {
            {1.0, 0.0, 0.0f},
            {0.0, 1.0, 0.0f}
    };

    float references[2][3] = {
            {1.0, 0.0, 0.0f},
            {0.0, 1.0, 0.0f}
    };

    Eigen::Quaternionf rotation(Eigen::AngleAxisf(0.5f, Eigen::Vector3f::UnitZ()));

    for (int i = 0; i < 2; i++) {
        Eigen::Vector3f orig(observations[i][0], observations[i][1], observations[i][2]);
        auto new_vec = rotation * orig;
        observations[i][0] = new_vec[0];
        observations[i][1] = new_vec[1];
        observations[i][2] = new_vec[2];
    }

    for (int i = 0; i < 2; i++) {
        Eigen::Vector3f orig(references[i][0], references[i][1], references[i][2]);
        auto new_vec = rotation * orig;
        references[i][0] = new_vec[0];
        references[i][1] = new_vec[1];
        references[i][2] = new_vec[2];
    }

    const float a[2] = {0.5f, 0.5f};

    float q_out[4];

    quest_estimate(observations, references, a, q_out);

    expect_fuzzy_eq(q_out[0], 0);
    expect_fuzzy_eq(q_out[1], 0);
    expect_fuzzy_eq(q_out[2], 0);
    expect_fuzzy_eq(q_out[3], 1);
}

TEST(TestQuest, TestIdentity3) {
    float observations[2][3] = {
            {1.0, 0.0, 0.0f},
            {0.0, 1.0, 0.0f}
    };

    float references[2][3] = {
            {1.0, 0.0, 0.0f},
            {0.0, 1.0, 0.0f}
    };

    Eigen::Quaternionf rotation(Eigen::AngleAxisf(0.25f, Eigen::Vector3f::UnitZ()));

    for (int i = 0; i < 1; i++) {
        Eigen::Vector3f orig(observations[i][0], observations[i][1], observations[i][2]);
        auto new_vec = rotation * orig;
        observations[i][0] = new_vec[0];
        observations[i][1] = new_vec[1];
        observations[i][2] = new_vec[2];
    }

    for (int i = 0; i < 1; i++) {
        Eigen::Vector3f orig(references[i][0], references[i][1], references[i][2]);
        auto new_vec = rotation * orig;
        references[i][0] = new_vec[0];
        references[i][1] = new_vec[1];
        references[i][2] = new_vec[2];
    }

    const float a[2] = {0.5f, 0.5f};

    float q_out[4];

    quest_estimate(observations, references, a, q_out);

    expect_fuzzy_eq(q_out[0], 0);
    expect_fuzzy_eq(q_out[1], 0);
    expect_fuzzy_eq(q_out[2], 0);
    expect_fuzzy_eq(q_out[3], 1);
}

TEST(TestQuest, TestRotation) {
    float observations[2][3] = {
            {1.0, 0.0, 0.0f},
            {0.0, 1.0, 0.0f}
    };

    float references[2][3] = {
            {1.0, 0.0, 0.0f},
            {0.0, 1.0, 0.0f}
    };

    Eigen::Quaternionf rotation(Eigen::AngleAxisf(0.3f, Eigen::Vector3f::UnitZ()));

    for (int i = 0; i < 2; i++) {
        Eigen::Vector3f orig(observations[i][0], observations[i][1], observations[i][2]);
        auto new_vec = rotation * orig;
        observations[i][0] = new_vec[0];
        observations[i][1] = new_vec[1];
        observations[i][2] = new_vec[2];
    }

    const float a[2] = {0.5f, 0.5f};

    float q_out[4];

    quest_estimate(observations, references, a, q_out);

    // QUEST returns the quaternion to rotate the observations onto the references
    // This should be the reverse of the rotation above
    auto inv_rotation = rotation.inverse();

    expect_fuzzy_eq(q_out[0], inv_rotation.x());
    expect_fuzzy_eq(q_out[1], inv_rotation.y());
    expect_fuzzy_eq(q_out[2], inv_rotation.z());
    expect_fuzzy_eq(q_out[3], inv_rotation.w());
}