#include <gmock/gmock.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <math_utils.h>
#include <Util/FTLog.h>

#define NUM_ITERATIONS 10000

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

void expect_mat_eq(const glm::mat2 a, float b[2][2]) {
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
            expect_fuzzy_eq(a[i][j], b[j][i]); // GLM uses column major whereas math utils uses row major indexing
}

void expect_mat_eq(const glm::mat3 a, float b[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            expect_fuzzy_eq(a[i][j], b[j][i]); // GLM uses column major whereas math utils uses row major indexing
}

void expect_mat_eq(const glm::mat4 a, float b[4][4]) {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            expect_fuzzy_eq(a[i][j], b[j][i]); // GLM uses column major whereas math utils uses row major indexing
}

void expect_vector_eq(const glm::vec3 a, float b[3]) {
    for (int i = 0; i < 3; i++)
        expect_fuzzy_eq(a[i], b[i]);
}

bool quat_eq(const glm::quat& q, float q_arr[4]) {
    if (fuzzy_eq(q.x, q_arr[0]) && fuzzy_eq(q.y, q_arr[1]) && fuzzy_eq(q.z, q_arr[2]) && fuzzy_eq(q.w, q_arr[3]))
        return true;
    return fuzzy_eq(-q.x, q_arr[0]) && fuzzy_eq(-q.y, q_arr[1]) && fuzzy_eq(-q.z, q_arr[2]) && fuzzy_eq(-q.w, q_arr[3]);
}

void expect_quat_eq(const glm::quat& q, float q_arr[4]) {
    EXPECT_TRUE(quat_eq(q, q_arr));
}

TEST(TestMathUtils, TestMat2x2) {
    for (int iter = 0; iter < NUM_ITERATIONS; iter++) {
        glm::mat2 mat(get_rand(), get_rand(), get_rand(), get_rand());

        float mat_arr[2][2];
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                mat_arr[i][j] = mat[j][i];

        expect_mat_eq(mat, mat_arr);

        float temp[2][2];

        expect_fuzzy_eq(glm::determinant(mat), mat2x2_det(mat_arr));

        mat2x2_inv(mat_arr, temp);

        expect_mat_eq(glm::inverse(mat), temp);
    }
}

TEST(TestMathUtils, TestMat3x3) {
    for (int iter = 0; iter < NUM_ITERATIONS; iter++) {
        glm::mat3 mat(get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand());
        glm::mat3 mat2(get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand());

        float mat_arr[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                mat_arr[i][j] = mat[j][i];

        float mat2_arr[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                mat2_arr[i][j] = mat2[j][i];

        expect_mat_eq(mat, mat_arr);

        float temp[3][3];

        expect_fuzzy_eq(glm::determinant(mat), mat3x3_det(mat_arr));

        mat3x3_inv(mat_arr, temp);

        expect_mat_eq(glm::inverse(mat), temp);

        mat3x3_mult(mat_arr, mat2_arr, temp);

        expect_mat_eq(mat * mat2, temp);
    }
}

TEST(TestMathUtils, TestMat4x4) {
    for (int iter = 0; iter < NUM_ITERATIONS; iter++) {
        glm::mat4 mat(get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand());
        glm::mat4 mat2(get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand(), get_rand());

        float mat_arr[4][4];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                mat_arr[i][j] = mat[j][i];

        float mat2_arr[4][4];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                mat2_arr[i][j] = mat2[j][i];


        expect_mat_eq(mat, mat_arr);

        float temp[4][4];

        expect_fuzzy_eq(glm::determinant(mat), mat4x4_det(mat_arr));

        mat4x4_inv(mat_arr, temp);

        expect_mat_eq(glm::inverse(mat), temp);

        mat4x4_mult(mat_arr, mat2_arr, temp);

        expect_mat_eq(mat * mat2, temp);
    }
}

TEST(TestMathUtils, TestVector) {
    for (int iter = 0; iter < NUM_ITERATIONS; iter++) {
        glm::vec3 a(get_rand(), get_rand(), get_rand());
        glm::vec3 b(get_rand(), get_rand(), get_rand());

        float a_arr[3];
        float b_arr[3];
        for (int i = 0; i < 3; i++) {
            a_arr[i] = a[i];
            b_arr[i] = b[i];
        }

        expect_vector_eq(a, a_arr);
        expect_vector_eq(b, b_arr);


        expect_fuzzy_eq(glm::length(a), vector_mag(a_arr));
        expect_fuzzy_eq(glm::dot(a, b), vector_dot(a_arr, b_arr));
        expect_fuzzy_eq(glm::length(glm::cross(a, b)), vector_cross_mag(a_arr, b_arr));

        float temp[3];

        vector_cross(a_arr, b_arr, temp);
        expect_vector_eq(glm::cross(a, b), temp);

        vector_normalize(a_arr, temp);
        expect_vector_eq(glm::normalize(a), temp);
    }
}

TEST(TestMathUtils, TestAxisAngle) {
    for (int iter = 0; iter < NUM_ITERATIONS; iter++) {
        glm::vec3 axis(get_rand(), get_rand(), get_rand());
        axis = glm::normalize(axis);
        float angle = get_rand(4);
        glm::quat q = glm::angleAxis(angle, axis);

        float axis_arr[3];
        for (int i = 0; i < 3; i++)
            axis_arr[i] = axis[i];

        float q_arr[4];
        axis_angle_to_quat(axis_arr, angle, q_arr);

        expect_quat_eq(q, q_arr);
    }
}

TEST(TestMathUtils, TestRodrigues) {
    for (int iter = 0; iter < NUM_ITERATIONS; iter++) {
        float axis[3] = { get_rand(), get_rand(), get_rand() };
        float axis_norm[3];
        vector_normalize(axis, axis_norm);

        float angle = get_rand(1.0f);

        float q[4];
        axis_angle_to_quat(axis_norm, angle, q);

        float mrp[3];
        quaternion_to_rodrigues(q, mrp);

        float q_out[4];
        rodrigues_to_quaternion(mrp, q_out);
        for (int i = 0; i < 4; i++)
            expect_fuzzy_eq(q[i], q_out[i]);

        glm::tvec3<float> v(mrp[0], mrp[1], mrp[2]);
        auto mag_v = glm::length(v);
        auto n = v / mag_v;
        auto angle_t = 4.0f * atanf(mag_v);

        auto tan = tanf(angle / 4.0f);
        auto correct_mrp = glm::vec3(axis_norm[0] * tan, axis_norm[1] * tan, axis_norm[2] * tan);

        if (!fuzzy_eq(angle, 0)) {
            // We skip if close to 0 as this breaks the glm code
            auto test = glm::angleAxis(angle_t, n);
            expect_quat_eq(test, q_out);
        }
    }
}

TEST(TestMathUtils, TestQuaternion) {
    for (int iter = 0; iter < NUM_ITERATIONS; iter++) {
        glm::quat q(get_rand(), get_rand(), get_rand(), get_rand());
        q = glm::normalize(q);

        glm::quat q2(get_rand(), get_rand(), get_rand(), get_rand());
        q2 = glm::normalize(q2);

        float q_arr[4];
        q_arr[0] = q.x;
        q_arr[1] = q.y;
        q_arr[2] = q.z;
        q_arr[3] = q.w;


        float q2_arr[4];
        q2_arr[0] = q2.x;
        q2_arr[1] = q2.y;
        q2_arr[2] = q2.z;
        q2_arr[3] = q2.w;
        
        float temp[4];

        quat_mult(q_arr, q2_arr, temp);
        expect_quat_eq(q * q2, temp);

        quat_invert(q_arr, temp);
        expect_quat_eq(glm::inverse(q), temp);

        glm::vec3 vector(get_rand(), get_rand(), get_rand());
        float vector_arr[3];
        for (int i = 0; i < 3; i++)
            vector_arr[i] = vector[i];

        float vector_out[3];

        quat_rotate(q_arr, vector_arr, vector_out);

        expect_vector_eq(q * vector, vector_out);
    }
}

TEST(TestMathUtils, TestAngleAxis) {
    for (int iter = 0; iter < NUM_ITERATIONS; iter++) {
        glm::vec3 axis = glm::normalize(glm::vec3(get_rand(), get_rand(), get_rand()));
        float angle = get_rand(6);
        float axis_arr[3];
        for (int i = 0; i < 3; i++)
            axis_arr[i] = axis[i];

        glm::quat quat = glm::angleAxis(angle, axis);
        float quat_arr[4];
        axis_angle_to_quat(axis_arr, angle, quat_arr);

        expect_quat_eq(quat, quat_arr);
    }
}