//          Copyright Emil Fresk 2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <algorithm>

constexpr const double start_detection = 0.1;
constexpr const double goal_detection = 0.3;

enum class timer_state
{
  MEASURING_STARTING_POSITION,
  WAITING_FOR_START,
  RUNNING,
  GOAL_DETECTED,
  WAITING_FOR_RESTART
};

struct position
{
  double x;
  double y;
};

void callback_odom(const nav_msgs::OdometryConstPtr& msg)
{
  // Local variables
  static timer_state state = timer_state::MEASURING_STARTING_POSITION;
  static double max_speed;
  static position staring_pos;
  static ros::Time start_time;

  // State machine
  if (state == timer_state::MEASURING_STARTING_POSITION)
  {
    max_speed = 0;
    staring_pos = {msg->pose.pose.position.x, msg->pose.pose.position.y};
    state = timer_state::WAITING_FOR_START;
    ROS_INFO("Waiting for car to start...");
  }
  else if (state == timer_state::WAITING_FOR_START)
  {
    // Check for starting condition
    const double dx = msg->pose.pose.position.x - staring_pos.x;
    const double dy = msg->pose.pose.position.y - staring_pos.y;
    const double distance_from_start = std::sqrt(dx * dx + dy * dy);

    if (distance_from_start > start_detection)
    {
      ROS_INFO("START!!!!!");
      state = timer_state::RUNNING;
      start_time = ros::Time::now();
    }
  }
  else if (state == timer_state::RUNNING)
  {
    const auto time_now = ros::Time::now();
    const auto race_duration = time_now - start_time;

    if (race_duration > ros::Duration(5.0))  // Do not check goal for 5s
    {
      // Update max speed
      max_speed = std::max(
          max_speed,
          std::sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x +
                    msg->twist.twist.linear.y * msg->twist.twist.linear.y +
                    msg->twist.twist.linear.z * msg->twist.twist.linear.z));

      // Check if goal is reached
      const double dx = msg->pose.pose.position.x - staring_pos.x;
      const double dy = msg->pose.pose.position.y - staring_pos.y;
      const double distance_from_start = std::sqrt(dx * dx + dy * dy);

      if (distance_from_start < goal_detection)
      {
        ROS_INFO("GOAL!!!!!");
        ROS_INFO_STREAM("Time: " << race_duration.toSec()
                                 << ", max speed = " << max_speed << " m/s");
        state = timer_state::WAITING_FOR_RESTART;

        ROS_WARN("Restart the node to time again...");
      }
    }
  }
}

int main(int argc, char* argv[])
{
  /*
   * Initializing ROS
   */
  ROS_INFO("Initializing Mechatronics Timer...");
  ros::init(argc, argv, "mechatronics_timer");

  ros::NodeHandle nh;

  ros::Subscriber odom_sub_ =
      nh.subscribe("vicon/mech_target/mech_target/odom", 5, callback_odom,
                   ros::TransportHints().tcpNoDelay());

  /* Let ROS run. */
  ros::spin();

  return 0;
}
