#include <gmock/gmock.h>
#include "state/quest.h"
#include "Eigen/Core"
#include <Eigen/Geometry>

#define NUM_ITERATIONS 1000

float get_rand(float range = 20.0f) {
    float divisor = RAND_MAX / 2 / range;
    return (float)(rand() % RAND_MAX) / divisor - range;
}

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

bool quat_eq(const Eigen::Quaternionf& q, float q_arr[4]) {
    if (fuzzy_eq(q.x(), q_arr[0]) && fuzzy_eq(q.y(), q_arr[1]) && fuzzy_eq(q.z(), q_arr[2]) && fuzzy_eq(q.w(), q_arr[3]))
        return true;
    return fuzzy_eq(-q.x(), q_arr[0]) && fuzzy_eq(-q.y(), q_arr[1]) && fuzzy_eq(-q.z(), q_arr[2]) && fuzzy_eq(-q.w(), q_arr[3]);
}

void expect_quat_eq(const Eigen::Quaternionf& q, float q_arr[4]) {
    EXPECT_TRUE(quat_eq(q, q_arr));
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
        Eigen::Vector3f new_vec = rotation * orig;
        observations[i][0] = new_vec[0];
        observations[i][1] = new_vec[1];
        observations[i][2] = new_vec[2];
    }

    for (int i = 0; i < 2; i++) {
        Eigen::Vector3f orig(references[i][0], references[i][1], references[i][2]);
        Eigen::Vector3f new_vec = rotation * orig;
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

    Eigen::Quaternionf rotation(Eigen::AngleAxisf(get_rand(), Eigen::Vector3f::UnitZ()));

    for (int i = 0; i < 1; i++) {
        Eigen::Vector3f orig(observations[i][0], observations[i][1], observations[i][2]);
        Eigen::Vector3f new_vec = rotation * orig;
        observations[i][0] = new_vec[0];
        observations[i][1] = new_vec[1];
        observations[i][2] = new_vec[2];
    }

    for (int i = 0; i < 1; i++) {
        Eigen::Vector3f orig(references[i][0], references[i][1], references[i][2]);
        Eigen::Vector3f new_vec = rotation * orig;
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
        Eigen::Vector3f new_vec = rotation * orig;
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


    expect_quat_eq(inv_rotation, q_out);
}

TEST(TestQuest, TestRotation2) {
    float observations[2][3] = {
            {1.0, 0.0, 0.0f},
            {0.0, 1.0, 0.0f}
    };

    float references[2][3] = {
            {1.0, 0.0, 0.0f},
            {0.0, 1.0, 0.0f}
    };

    Eigen::Quaternionf rotation(Eigen::AngleAxisf(3.14159265359f, Eigen::Vector3f::UnitZ()));

    for (int i = 0; i < 2; i++) {
        Eigen::Vector3f orig(observations[i][0], observations[i][1], observations[i][2]);
        Eigen::Vector3f new_vec = rotation * orig;
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

    expect_quat_eq(inv_rotation, q_out);
}

TEST(TestQuest, TestRotation3) {
    float observations[2][3] = {
            {1.0, 0.0, 0.0f},
            {0.0, 1.0, 0.0f}
    };

    float references[2][3] = {
            {1.0, 0.0, 0.0f},
            {0.0, 1.0, 0.0f}
    };

    Eigen::Quaternionf rotation(Eigen::AngleAxisf(1.2f, Eigen::Vector3f::UnitY()));

    for (int i = 0; i < 2; i++) {
        Eigen::Vector3f orig(observations[i][0], observations[i][1], observations[i][2]);
        Eigen::Vector3f new_vec = rotation * orig;
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

    expect_quat_eq(inv_rotation, q_out);
}

TEST(TestQuest, TestRotation4) {
    for (int iter_i = 0; iter_i < NUM_ITERATIONS; iter_i++) {
        float observations[2][3] = {
                {1.0, 0.0, 0.0f},
                {0.0, 1.0, 0.0f}
        };

        float references[2][3] = {
                {1.0, 0.0, 0.0f},
                {0.0, 1.0, 0.0f}
        };

        Eigen::Quaternionf rotation(Eigen::AngleAxisf(3.14159265359f, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(get_rand(0.1f), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(get_rand(0.1f), Eigen::Vector3f::UnitX()));

        for (int i = 0; i < 2; i++) {
            Eigen::Vector3f orig(observations[i][0], observations[i][1], observations[i][2]);
            Eigen::Vector3f new_vec = rotation * orig;
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

        expect_quat_eq(inv_rotation, q_out);
    }
}

TEST(TestQuest, TestRotation5) {
    for (int iter_i = 0; iter_i < NUM_ITERATIONS; iter_i++) {
        float observations[2][3] = {
                {1.0, 0.0, 0.0f},
                {0.0, 1.0, 0.0f}
        };

        float references[2][3] = {
                {1.0, 0.0, 0.0f},
                {0.0, 1.0, 0.0f}
        };

        Eigen::Quaternionf rotation(Eigen::AngleAxisf(3.14159265359f, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(get_rand(0.1f), Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(get_rand(0.1f), Eigen::Vector3f::UnitZ()));

        for (int i = 0; i < 2; i++) {
            Eigen::Vector3f orig(observations[i][0], observations[i][1], observations[i][2]);
            Eigen::Vector3f new_vec = rotation * orig;
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

        expect_quat_eq(inv_rotation, q_out);
    }
}

TEST(TestQuest, TestRotation6) {
    for (int iter_i = 0; iter_i < NUM_ITERATIONS; iter_i++) {
        float observations[2][3] = {
                {1.0, 0.0, 0.0f},
                {0.0, 1.0, 0.0f}
        };

        float references[2][3] = {
                {1.0, 0.0, 0.0f},
                {0.0, 1.0, 0.0f}
        };

        Eigen::Quaternionf rotation(Eigen::AngleAxisf(3.14159265359f, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(get_rand(0.1f), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(get_rand(0.1f), Eigen::Vector3f::UnitZ()));

        for (int i = 0; i < 2; i++) {
            Eigen::Vector3f orig(observations[i][0], observations[i][1], observations[i][2]);
            Eigen::Vector3f new_vec = rotation * orig;
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

        expect_quat_eq(inv_rotation, q_out);
    }
}

TEST(TestQuest, TestRotationRandom) {
    for (int rand_i = 0; rand_i < NUM_ITERATIONS; rand_i++) {
        float observations[2][3] = {
                {1.0, 0.0, 0.0f},
                {0.0, 1.0, 0.0f}
        };

        float references[2][3] = {
                {1.0, 0.0, 0.0f},
                {0.0, 1.0, 0.0f}
        };

        Eigen::Quaternionf rotation(Eigen::AngleAxisf(get_rand(), Eigen::Vector3f::UnitZ()));

        for (int i = 0; i < 2; i++) {
            Eigen::Vector3f orig(observations[i][0], observations[i][1], observations[i][2]);
            Eigen::Vector3f new_vec = rotation * orig;
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

        expect_quat_eq(inv_rotation, q_out);
    }
}

TEST(TestQuest, TestRotationRandom2) {
    for (int rand_i = 0; rand_i < NUM_ITERATIONS; rand_i++) {
        float observations[2][3] = {
                {1.0, 0.0, 0.0f},
                {0.0, 1.0, 0.0f}
        };

        float references[2][3] = {
                {1.0, 0.0, 0.0f},
                {0.0, 1.0, 0.0f}
        };

        Eigen::Quaternionf rotation(Eigen::AngleAxisf(get_rand(), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(get_rand(), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(get_rand(), Eigen::Vector3f::UnitX()));

        for (int i = 0; i < 2; i++) {
            Eigen::Vector3f orig(observations[i][0], observations[i][1], observations[i][2]);
            Eigen::Vector3f new_vec = rotation * orig;
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

        expect_quat_eq(inv_rotation, q_out);
    }
}

TEST(TestQuest, TestRotationDisturbed) {
    for (int rand_i = 0; rand_i < 1; rand_i++) {
        float observations[2][3] = {
                {1.0, 0.0, 0.0f},
                {0.0, 1.0, 0.0f}
        };

        float references[2][3] = {
                {1.0, 0.0, 0.0f},
                {0.0, 1.0, 0.0f}
        };

        Eigen::Quaternionf rotation(Eigen::AngleAxisf(get_rand(), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(get_rand(), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(get_rand(), Eigen::Vector3f::UnitX()));

        Eigen::Quaternionf disturb(Eigen::AngleAxisf(get_rand(0.5f), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(get_rand(0.1f), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(get_rand(0.1f), Eigen::Vector3f::UnitX()));
        Eigen::Quaternionf disturb_inv = disturb.inverse();
        // We disturb the vectors equally in opposite directions so the result should be halfway between them - i.e unchanged

        Eigen::Quaternionf disturbed = disturb ;//* rotation;
        Eigen::Quaternionf disturbed_inv = disturb_inv;// * rotation;

        Eigen::Vector3f orig(observations[0][0], observations[0][1], observations[0][2]);
        Eigen::Vector3f new_vec = disturbed * orig;
        observations[0][0] = new_vec[0];
        observations[0][1] = new_vec[1];
        observations[0][2] = new_vec[2];

        Eigen::Vector3f orig1(observations[1][0], observations[1][1], observations[1][2]);
        Eigen::Vector3f new_vec1 = disturbed_inv * orig1;
        observations[1][0] = new_vec1[0];
        observations[1][1] = new_vec1[1];
        observations[1][2] = new_vec1[2];



        const float a[2] = {0.5f, 0.5f};

        float q_out[4];

        quest_estimate(observations, references, a, q_out);

        // QUEST returns the quaternion to rotate the observations onto the references
        // This should be the reverse of the rotation above
        auto inv_rotation = rotation.inverse();

        expect_quat_eq(inv_rotation, q_out);
    }
}