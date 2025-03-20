#include "rclcpp/rclcpp.hpp"
#include "vff_avoidance/AvoidanceNode.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vff_avoidance::AvoidanceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
