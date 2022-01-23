#include "gtest/gtest.h"

TEST(MyGreatFixture, AlwaysPass) {
    SUCCEED();
}

// Comment this out permanently.
// It is only useful to verify that the testsuite is actually running.
#if 0
TEST(MyGreatFixture, AlwaysFail) {
    FAIL();
}
#endif