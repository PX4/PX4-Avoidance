#include <gtest/gtest.h>
#include <ros/console.h>

int main(int argc, char **argv) {
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Error);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
