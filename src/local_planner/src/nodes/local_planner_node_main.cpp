#include <chrono>
#include <local_planner/local_planner.h>

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlanner>());

  rclcpp::shutdown();
  return 0;
}
