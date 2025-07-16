#include "rclcpp/executors.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/utilities.hpp"
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
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

    cmd_vel_pub =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100), // 10 Hz = 100ms
                                std::bind(&Patrol::control_loop, this));
  }

private:
  float direction_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subs_laser;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::TimerBase::SharedPtr timer_;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Get 180° front rays
    size_t total = msg->ranges.size();
    size_t front_start = total / 4;   // Skip right 90 degree
    size_t front_end = 3 * total / 4; // Skip left 90 degree

    // Find max distance in front 180° (excluding inf values)
    float max_distance = 0.0;
    size_t max_index = front_start;

    // Find min distance in front area (for obstacle detection)
    float min_front_distance = std::numeric_limits<float>::max();

    for (size_t i = front_start; i < front_end; i++) {
      // Check for minimum distance (obstacle detection)
      if (std::isfinite(msg->ranges[i]) &&
          msg->ranges[i] < min_front_distance) {
        min_front_distance = msg->ranges[i];
      }

      // Check for maximum distance (safest direction)
      if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > max_distance) {
        max_distance = msg->ranges[i];
        max_index = i;
      }
    }

    // // Calculate angle of the safest direction
    // float angle_increment = msg->angle_increment;
    // float angle_min = msg->angle_min;
    // float safest_angle = angle_min + (max_index * angle_increment);

    // // Store in class variable
    // direction_ = safest_angle;

    // Obstacle avoidance logic
    if (min_front_distance < 0.35) {
      // Use safest direction - calculate angle
      float angle_increment = msg->angle_increment;
      float angle_min = msg->angle_min;
      direction_ = angle_min + (max_index * angle_increment);
    } else {
      direction_ = 0.0; // Go straight
    }

    // Debug output
    RCLCPP_INFO(this->get_logger(),
                "Max distance: %.2f at index %zu, angle: %.2f", max_distance,
                max_index, direction_);
  }

  void control_loop() {
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = 0.1; // Always 0.1 m/s forward

    // Angular velocity based on safest direction
    cmd.angular.z = direction_ / 2;

    cmd_vel_pub->publish(cmd);

    // Reset the gazebo through
    // ros2 service call /reset_world std_srvs/srv/Empty
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