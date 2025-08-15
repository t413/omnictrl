#include <gtest/gtest.h>

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS(); //zero exit-code may be required for PlatformIO to parse results
}
