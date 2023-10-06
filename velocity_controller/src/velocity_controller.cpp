
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <algorithm>

//global publishers
ros::Publisher throttle_pub;
ros::Publisher err_pub;
// gobal variables
double desired_velocity = 0;
double current_velocity = 0;
ros::Time last_timer_time;

const double max_throttle = 400.0;
const double max_velocity = 10.0;

// velocity command callback
void on_command_velocity(const std_msgs::Float32& msg) {
  desired_velocity = msg.data;
  std_msgs::Float32 err;
  err.data = desired_velocity - current_velocity;
  // ROS_INFO_STREAM_COND(cmd_velocity > 0.1, "current velocity " << current_velocity << " err = " << err.data);
  err_pub.publish(err);
}

/**
 * @brief odometry callback function
*/
void on_odo(const nav_msgs::Odometry& odom)
{
  current_velocity = odom.twist.twist.linear.x;
  // ROS_INFO_STREAM("current velocity " << current_velocity << " err = " << cmd_velocity - current_velocity);
}

/**
 * @brief Timer callback function
 * 
 * Using PID control
 * 
 * @author Dknt
 * @date 2023.10.6
*/
void on_timer(const ros::TimerEvent& event) {
  auto t = ros::Time::now();
  auto dt = (t - last_timer_time).toSec();
  last_timer_time = t;
  std_msgs::Float32 throttle_cmd;

  /* Using PID controller */
  // coefficient
  const double p_control = 200.0, i_control = 40.0, d_control = 2.0;

  static double error_i = 0.0;
  static double error_last = 0.0;
  double error_d = 0.0;

  auto clip = [](double _d1, double _d2) {
    if (_d1 > _d2) return _d2;
    if (_d1 < -_d2) return -_d2;
    return _d1;
  };

  desired_velocity = clip(desired_velocity, max_velocity);
  double error = desired_velocity - current_velocity;
  if (abs(error) < 1) error_i += error * dt;
  else error_i = 0;
  error_d = (error - error_last) / dt;
  
  std::cout << "\rerror: " << error << " error_i: " << error_i << " error_d " << error_d << "     " << std::flush;

  

  throttle_cmd.data = clip(p_control * error + i_control * error_i + d_control * error_d, max_throttle);

  error_last = error;
  throttle_pub.publish(throttle_cmd);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "velocity_controller");
  ros::NodeHandle nh("~");
  throttle_pub = nh.advertise<std_msgs::Float32>("throttle", 1);
  auto odo_sub = nh.subscribe("odom", 1, on_odo);
  auto cmd_sub = nh.subscribe("velocity", 1, on_command_velocity);
  err_pub = nh.advertise<std_msgs::Float32>("velocity_err", 1);
  if (nh.param("/use_sim_time", false)) {
    while(ros::ok()) {
      ros::spinOnce();
    
      last_timer_time = ros::Time::now();
      if (!last_timer_time.isZero()) {
        break;
      } 
    }
  }
  auto timer = nh.createTimer(ros::Duration(0.1), on_timer);
  last_timer_time = ros::Time::now();
  ros::spin();
  return 0;
}


