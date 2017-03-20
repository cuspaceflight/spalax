#include <gmock/gmock.h>
#include "Eigen/Core"

TEST(TestEigen, TestMemory) {
#ifndef EIGEN_NO_DEBUG
    EXPECT_DEATH(Eigen::internal::check_that_malloc_is_allowed(), ".*");
#endif
}