#include <gmock/gmock.h>
#include "state/quest.h"
#include "math_debug_util.h"

#define NUM_ITERATIONS 1000

using namespace Eigen;

TEST(TestQMethod, TestIdentity) {
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

    EXPECT_GT(davenport_q_method(observations, references, a, 2, q_out), 0);

    expect_fuzzy_eq(q_out[0], 0);
    expect_fuzzy_eq(q_out[1], 0);
    expect_fuzzy_eq(q_out[2], 0);
    expect_fuzzy_eq(q_out[3], 1);
}

TEST(TestQMethod, TestIdentity2) {
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

    EXPECT_GT(davenport_q_method(observations, references, a, 2, q_out), 0);

    expect_fuzzy_eq(q_out[0], 0);
    expect_fuzzy_eq(q_out[1], 0);
    expect_fuzzy_eq(q_out[2], 0);
    expect_fuzzy_eq(q_out[3], 1);
}

TEST(TestQMethod, TestIdentity3) {
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

    EXPECT_GT(davenport_q_method(observations, references, a, 2, q_out), 0);

    expect_fuzzy_eq(q_out[0], 0);
    expect_fuzzy_eq(q_out[1], 0);
    expect_fuzzy_eq(q_out[2], 0);
    expect_fuzzy_eq(q_out[3], 1);
}

TEST(TestQMethod, TestRotation) {
    float observations[2][3];

    const float references[2][3] = {
            {1.0, 0.0, 0.0f},
            {0.0, 1.0, 0.0f}
    };

    Eigen::Quaternionf rotation(Eigen::AngleAxisf(0.3f, Eigen::Vector3f::UnitZ()));

    for (int i = 0; i < 2; i++) {
        Eigen::Vector3f orig(references[i][0], references[i][1], references[i][2]);
        Eigen::Vector3f new_vec = rotation * orig;
        observations[i][0] = new_vec[0];
        observations[i][1] = new_vec[1];
        observations[i][2] = new_vec[2];
    }

    const float a[2] = {0.5f, 0.5f};



    float q_out[4];

    EXPECT_GT(davenport_q_method(observations, references, a, 2, q_out), 0);

    Eigen::Quaternionf out;
    out.x() = q_out[0];
    out.y() = q_out[1];
    out.z() = q_out[2];
    out.w() = q_out[3];


    for (int i = 0; i < 2; i++) {
        Eigen::Vector3f test = out * Eigen::Vector3f(observations[i][0], observations[i][1], observations[i][2]) -
                               Eigen::Vector3f(references[i][0], references[i][1], references[i][2]);
        expect_fuzzy_eq(test.norm(), 0);
    }
}

TEST(TestQMethod, TestRotation2) {
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

    EXPECT_GT(davenport_q_method(observations, references, a, 2, q_out), 0);

    Eigen::Quaternionf out;
    out.x() = q_out[0];
    out.y() = q_out[1];
    out.z() = q_out[2];
    out.w() = q_out[3];


    for (int i = 0; i < 2; i++) {
        Eigen::Vector3f test = out * Eigen::Vector3f(observations[0][0], observations[0][1], observations[0][2]) -
                               Eigen::Vector3f(references[0][0], references[0][1], references[0][2]);
        expect_fuzzy_eq(test.norm(), 0);
    }
}

TEST(TestQMethod, TestRotation3) {
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

    EXPECT_GT(davenport_q_method(observations, references, a, 2, q_out), 0);

    Eigen::Quaternionf out;
    out.x() = q_out[0];
    out.y() = q_out[1];
    out.z() = q_out[2];
    out.w() = q_out[3];

    for (int i = 0; i < 2; i++) {
        Eigen::Vector3f test = out * Eigen::Vector3f(observations[0][0], observations[0][1], observations[0][2]) -
                               Eigen::Vector3f(references[0][0], references[0][1], references[0][2]);
        expect_fuzzy_eq(test.norm(), 0);
    }
}

TEST(TestQMethod, TestRotation4) {
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

        EXPECT_GT(davenport_q_method(observations, references, a, 2, q_out), 0);

        Eigen::Quaternionf out;
        out.x() = q_out[0];
        out.y() = q_out[1];
        out.z() = q_out[2];
        out.w() = q_out[3];

        for (int i = 0; i < 2; i++) {
            Eigen::Vector3f test = out * Eigen::Vector3f(observations[0][0], observations[0][1], observations[0][2]) -
                                   Eigen::Vector3f(references[0][0], references[0][1], references[0][2]);
            expect_fuzzy_eq(test.norm(), 0);
        }
    }
}

TEST(TestQMethod, TestRotation5) {
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

        EXPECT_GT(davenport_q_method(observations, references, a, 2, q_out), 0);

        Eigen::Quaternionf out;
        out.x() = q_out[0];
        out.y() = q_out[1];
        out.z() = q_out[2];
        out.w() = q_out[3];

        for (int i = 0; i < 2; i++) {
            Eigen::Vector3f test = out * Eigen::Vector3f(observations[0][0], observations[0][1], observations[0][2]) -
                                   Eigen::Vector3f(references[0][0], references[0][1], references[0][2]);
            expect_fuzzy_eq(test.norm(), 0);
        }
    }
}

TEST(TestQMethod, TestRotation6) {
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

        EXPECT_GT(davenport_q_method(observations, references, a, 2, q_out), 0);

        Eigen::Quaternionf out;
        out.x() = q_out[0];
        out.y() = q_out[1];
        out.z() = q_out[2];
        out.w() = q_out[3];

        for (int i = 0; i < 2; i++) {
            Eigen::Vector3f test = out * Eigen::Vector3f(observations[0][0], observations[0][1], observations[0][2]) -
                                   Eigen::Vector3f(references[0][0], references[0][1], references[0][2]);
            expect_fuzzy_eq(test.norm(), 0);
        }
    }
}

TEST(TestQMethod, TestRotationRandom) {
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

        EXPECT_GT(davenport_q_method(observations, references, a, 2, q_out), 0);

        Eigen::Quaternionf out;
        out.x() = q_out[0];
        out.y() = q_out[1];
        out.z() = q_out[2];
        out.w() = q_out[3];

        for (int i = 0; i < 2; i++) {
            Eigen::Vector3f test = out * Eigen::Vector3f(observations[0][0], observations[0][1], observations[0][2]) -
                                   Eigen::Vector3f(references[0][0], references[0][1], references[0][2]);
            expect_fuzzy_eq(test.norm(), 0);
        }
    }
}

TEST(TestQMethod, TestRotationRandom2) {
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

        EXPECT_GT(davenport_q_method(observations, references, a, 2, q_out), 0);

        Eigen::Quaternionf out;
        out.x() = q_out[0];
        out.y() = q_out[1];
        out.z() = q_out[2];
        out.w() = q_out[3];

        for (int i = 0; i < 2; i++) {
            Eigen::Vector3f test = out * Eigen::Vector3f(observations[0][0], observations[0][1], observations[0][2]) -
                                   Eigen::Vector3f(references[0][0], references[0][1], references[0][2]);
            expect_fuzzy_eq(test.norm(), 0);
        }
    }
}


TEST(TestQMethod, TestRotationRandom3) {
    const int num_observations = 100;

    for (int rand_i = 0; rand_i < NUM_ITERATIONS; rand_i++) {
        Eigen::Quaternionf rotation(Eigen::AngleAxisf(get_rand(), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(get_rand(), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(get_rand(), Eigen::Vector3f::UnitX()));

        float references[num_observations][3];
        float observations[num_observations][3];
        float a[num_observations];

        for (int i = 0; i < num_observations; i++) {
            Eigen::Vector3f reference = Eigen::Vector3f(get_rand(), get_rand(), get_rand()).normalized();
            Eigen::Vector3f observation = rotation * reference;

            a[i] = 1.0f / num_observations;
            for (int j = 0; j < 3; j++) {
                references[i][j] = reference[j];
                observations[i][j] = observation[j];
            }
        }



        float q_out[4];

        EXPECT_GT(davenport_q_method(observations, references, a, num_observations, q_out), 0);

        Eigen::Quaternionf out;
        out.x() = q_out[0];
        out.y() = q_out[1];
        out.z() = q_out[2];
        out.w() = q_out[3];

        for (int i = 0; i < num_observations; i++) {
            Vector3f test = out * Map<const Vector3f>(observations[i]) - Map<const Vector3f>(references[i]);
            expect_fuzzy_eq(test.norm(), 0);
        }
    }
}