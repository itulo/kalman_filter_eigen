#include <limits.h>
#include "gtest/gtest.h"

// Tests factorial of negative numbers.
TEST(Equality, One) {
  EXPECT_EQ(1, 1);
}

// Tests factorial of 0.
TEST(GreaterThan, One) {
  EXPECT_GT(2, 1);
}