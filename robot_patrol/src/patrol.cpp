#include "rclcpp/executors.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/utilities.hpp"
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <unistd.h>

using std::placeholders::_1;

class Patrol : public rclcpp::Node {
public:
  Patrol(std::string laser_topic) : Node("patrol_bot_node") {
    subs_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
        laser_topic, 10, std::bind(&Patrol::laser_callback, this, _1));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subs_laser;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Basic laser info
    RCLCPP_INFO(this->get_logger(), "Laser data received:");
    RCLCPP_INFO(this->get_logger(), "Number of ranges: %zu",
                msg->ranges.size());
    RCLCPP_INFO(this->get_logger(), "Angle min: %.2f, Angle max: %.2f",
                msg->angle_min, msg->angle_max);

    // Display first few ranges
    RCLCPP_INFO(this->get_logger(), "First 5 ranges:");
    for (size_t i = 0; i < 5 && i < msg->ranges.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "Range[%zu]: %.2f", i, msg->ranges[i]);
    }

    // Display middle few ranges (front area)
    size_t middle = msg->ranges.size() / 2;
    RCLCPP_INFO(this->get_logger(), "Middle ranges (front area):");
    for (size_t i = middle - 2; i <= middle + 2 && i < msg->ranges.size();
         i++) {
      RCLCPP_INFO(this->get_logger(), "Range[%zu]: %.2f", i, msg->ranges[i]);
    }

    RCLCPP_INFO(this->get_logger(), "------------------------");
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::string laser_scn_topic = "/scan";
  std::shared_ptr<Patrol> patrol_node =
      std::make_shared<Patrol>(laser_scn_topic);

  RCLCPP_INFO(patrol_node->get_logger(), "Patrol Node INFO...");

  //   rclcpp::spin(patrol_node);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(patrol_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}