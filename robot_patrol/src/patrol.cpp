#include "rclcpp/executors.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/utilities.hpp"
#include <chrono>
#include <cstddef>
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
    // Get 180degree front rays
    size_t total = msg->ranges.size();
    size_t front_start = total / 4;   // Skip right 90 degree
    size_t front_end = 3 * total / 4; // Skip left 90 degree

    // Check obstacle in FRONT (narrow range around center)
    size_t side_band = 39; // Small range around front
    size_t center = total / 2;
    size_t front_check_start = center - side_band;
    size_t front_check_end = center + side_band;

    float min_front_distance = std::numeric_limits<float>::max();
    size_t max_index;

    // Front collision tetection
    for (size_t i = front_check_start; i < front_check_end; i++) {
      if (std::isfinite(msg->ranges[i]) &&
          msg->ranges[i] < min_front_distance) {
        min_front_distance = msg->ranges[i];
      }
    }

    // Case when all front rays are inf/nan
    if (min_front_distance == std::numeric_limits<float>::max()) {
      min_front_distance = 999.0; // Just to avoid printing huge nos on the logs
    }

    // If obstacle detected in front, find safest direction in 180degree
    // detecting at 35 cm - as mentioned in the task
    float obj_detect_threshold = 0.35;
    if (min_front_distance < obj_detect_threshold) {
      float max_distance = 0.0;
      max_index = front_start;

      for (size_t i = front_start; i < front_end; i++) {
        if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > max_distance) {
          max_distance = msg->ranges[i];
          max_index = i;
        }
      }

      // Calculate angle from front X axis
      float angle_increment = msg->angle_increment;
      float angle_min = msg->angle_min;

      float base_angle = angle_min + (max_index * angle_increment);
      // Just for testing -  Boost the turn based on how close the obstacle is
      float boost_factor = 1.0;
      if (min_front_distance < 0.25) {        // Very close
        boost_factor = 2.0;                   // Double the turn
      } else if (min_front_distance < 0.30) { // Close
        boost_factor = 1.5;                   // 50% more turn
      }
      direction_ = base_angle * boost_factor; // Apply boost

      RCLCPP_INFO(this->get_logger(),
                  "Obstacle at %.2f! Base angle: %.2f, Boosted angle: %.2f "
                  "(factor: %.1f)",
                  min_front_distance, base_angle, direction_, boost_factor);

      direction_ = angle_min + (max_index * angle_increment); // Take a turn
      RCLCPP_INFO(this->get_logger(),
                  "Obstacle at %.2f! Turning to angle: %.2f",
                  min_front_distance, direction_);
    } else {
      direction_ = 0.0; // Go straight
      RCLCPP_INFO(this->get_logger(), "Path clear (%.2f), going straight",
                  min_front_distance);
    }

    // Side collision detection - 30 degrees on either sides
    size_t left_ray = center + (total / 12);
    size_t right_ray = center - (total / 12);

    float side_collision_threshold = 0.20; // distance close to the sides
    float left_distance = msg->ranges[left_ray];
    float right_distance = msg->ranges[right_ray];

    // Side collision avoidance
    if (std::isfinite(left_distance) &&
        left_distance < side_collision_threshold) {
      direction_ = -0.5; // Turn right (negative angle)
      RCLCPP_WARN(this->get_logger(),
                  "LEFT COLLISION! Distance: %.2f, turning right",
                  left_distance);
      return;
    }

    if (std::isfinite(right_distance) &&
        right_distance < side_collision_threshold) {
      direction_ = 0.5; // Turn left (positive angle)
      RCLCPP_WARN(this->get_logger(),
                  "RIGHT COLLISION! Distance: %.2f, turning left",
                  right_distance);
      return;
    }
  }

  void control_loop() {
    auto cmd = geometry_msgs::msg::Twist();

    cmd.linear.x = 0.1; // Always 0.1 m/s forward

    // Angular velocity based on safest direction
    cmd.angular.z = direction_ / 2;

    cmd_vel_pub->publish(cmd);

    // Reset the gazebo through cmd if it goes crazy
    // ros2 service call /reset_world std_srvs/srv/Empty
    // ros2 run teleop_twist_keyboard teleop_twist_keyboard
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