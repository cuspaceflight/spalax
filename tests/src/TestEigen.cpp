#include <gmock/gmock.h>
#include "Eigen/Core"

TEST(TestEigen, TestMemory) {
    EXPECT_DEATH(Eigen::internal::check_that_malloc_is_allowed(), ".*");
}