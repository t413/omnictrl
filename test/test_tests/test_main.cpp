#include <gtest/gtest.h>

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
	RUN_ALL_TESTS();
	return 0; //zero exit-code required for PlatformIO to parse results
}
