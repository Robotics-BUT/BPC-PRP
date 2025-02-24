#include <gtest/gtest.h>
#include "example.hpp"

// Basic test case that just calls the function
TEST(ExampleLibTest, CallFunction) {
    printMessage();
}

// Main function for running tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
