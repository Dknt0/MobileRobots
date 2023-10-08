/*
 * Controller.cpp
 *
 *  Created on: 30 апр. 2017 г.
 *      Author: aminin
 */

#include "mpccontroller.h"

#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <tf/transform_datatypes.h>
namespace mpc_controller
{

// 限位函数
template <class T>
T clip(T val, T max)
{
  if ( val > max )
    return max;
  if ( val < -max)
    return -max;
  return val;
}

/**
 * @brief 更新与机器人当前坐标位置距离最近的路径段，以及对应这段路径上的位置
*/
void MPCController::update_trajectory_segment()
{
  current_segment_length = (*current_segment)->get_point_length(robot_x, robot_y);

  // 上一条轨迹
  while( current_segment_length < 0.0 )
  {
    if (current_segment == trajectory.begin())
      current_segment = trajectory.end();

    --current_segment;
    current_segment_length = (*current_segment)->get_point_length(robot_x, robot_y);
  }
  // 下一条轨迹
  while (current_segment_length > (*current_segment)->get_length())
  {
    ++current_segment;
    if (current_segment == trajectory.end())
      current_segment = trajectory.begin();
    current_segment_length = (*current_segment)->get_point_length(robot_x, robot_y);
  }
//  ROS_DEBUG_STREAM("current segment length "<<current_segment_length);
}

/**
 * @brief 更新控制点
 * 
 * 从当前机器人位置对应的最近轨迹点开始，按照 control_points_dl 间隔，取若干轨迹点，组成控制点向量，用于计算控制多项式
*/
void MPCController::update_control_points() {
  control_points.resize(control_points_num);
  Trajectory::iterator segment = current_segment; // 当前轨迹段迭代器
  tf::Vector3 pose(robot_x, robot_y, 0);
  ROS_DEBUG_STREAM("control points");
  // 从当前位置对应最近轨迹点开始
  double control_point_distance = (*segment)->get_point_length(pose.x(), pose.y());
  for(std::size_t i = 0; i<control_points_num; ++i) {
    // 增加若干距离
    control_point_distance += i*control_points_dl;
    // 若超出当前轨迹段范围，则在下一条轨迹上寻找
    while (control_point_distance > (*segment)->get_length()) {
      control_point_distance -= (*segment)->get_length();
      ++segment;
      if (segment == trajectory.end())
        segment = trajectory.begin();
    }
    // 获取轨迹点坐标
    control_points[i] = (*segment)->get_point(control_point_distance);
    ROS_DEBUG_STREAM(i<<": "<<control_points[i].x()<<" "<<control_points[i].y());
  }
}

/**
 * @brief 变换控制点坐标到局部坐标系
 * 
*/
void MPCController::convert_control_points() {
  tf::Transform world2robot = robot2world.inverse();
  ROS_DEBUG_STREAM("control points in robot coordinates ");
  for (auto& point: control_points) {
    point = world2robot(point);
    ROS_DEBUG_STREAM(point.x()<<" "<<point.y());
  }

}

// calculates polynom coefficients
/**
 * @brief 计算控制多项式参数
 * 
 * 三次多项式插值，用多项式来近似轨迹
*/
void MPCController::calculate_control_coefs() {
  const int order = 3; // we have 4 coefficients
  assert(order <= control_points.size() - 1); // 断言
  Eigen::MatrixXd A(control_points.size(), order + 1); // 6 * 4 矩阵

  // 第一列全1
  for (int i = 0; i < control_points.size(); ++i) {
    A(i,0) = 1.0;
  }
  Eigen::VectorXd yvals(control_points.size()); // 6 * 1 向量
  for (int j = 0; j < control_points.size(); j++) {
    yvals(j) = control_points[j].y(); // y 坐标
    for (int i = 0; i < order; i++)
    {
      A(j, i + 1) = A(j, i) * control_points[j].x(); // 依次为 x, x^2, x^3
    }
  }
  // 使用 QR 分解法  求最小二乘解
  auto Q = A.householderQr();
  Eigen::VectorXd result = Q.solve(yvals);
  // 赋值  参数  指向第一个元素的迭代器 指向末尾的迭代器
  control_coefs.assign(result.data(), result.data() + result.size());
  ROS_DEBUG_STREAM("coefs: "<<control_coefs[0]<<" "<<control_coefs[1]<<" "<<control_coefs[2]<<" "<<control_coefs[3]);
}

/**
 * @brief 按照控制多项式计算点位置
*/
double MPCController::polyeval(double x) {
  double result = control_coefs[0];
  double ax = 1.0;
  for (int i = 1; i<control_coefs.size(); ++i) {
    ax *= x;
    result += ax*control_coefs[i];
  }
  return result;
}

/**
 * @brief 更新机器人位置，位姿话题发布较慢，需要在控制器内部实现位置的预测
*/
void MPCController::update_robot_pose(double dt)
{
//  ROS_DEBUG_STREAM("update_robot_pose "<<dt<<" v = "<<current_linear_velocity );
  robot_x += current_linear_velocity * dt * cos(robot_theta);
  robot_y += current_linear_velocity * dt * sin(robot_theta);
  robot_theta = angles::normalize_angle(robot_theta + current_angular_velocity * dt);
  robot_time += ros::Duration(dt);
  robot2world.setOrigin(tf::Vector3(robot_x, robot_y, 0));
  robot2world.setRotation(tf::createQuaternionFromYaw(robot_theta));
}

/**
 * @brief 实现控制    计算并发布发布转弯曲率、速度
*/
void MPCController::apply_control() {
  cmd_vel += cmd_acc * control_dt; // 由加速度计算速度
  cmd_steer_angle += cmd_steer_rate * control_dt; // 由转向速率计算转向角度
  cmd_steer_angle = clip<double>(cmd_steer_angle, max_steer_angle);
  //send curvature as command to drives
  std_msgs::Float32 cmd;
  cmd.data = cmd_steer_angle;
  steer_pub.publish(cmd); // 发布转向角度
  //send velocity as command to drives
  cmd.data = cmd_vel;
  vel_pub.publish(cmd); // 发布速度
  ROS_DEBUG_STREAM("cmd v = "<<cmd_vel<<" angle = "<<cmd_steer_angle);
}

/**
 * @brief 定时器回调函数
*/
void MPCController::on_timer(const ros::TimerEvent& event)
{
  apply_control(); // 发布控制指令

  //  ROS_INFO_STREAM("on_timer");
  // calculate robot pose to next cycle
  update_robot_pose((event.current_expected - robot_time).toSec() + control_dt ); // 更新机器人位姿
  update_trajectory_segment(); // 更新轨迹段

  update_control_points(); // 更新控制点
  convert_control_points(); // 变换控制点坐标到局部坐标系
  calculate_control_coefs(); // 计算多项式参数

  double error = control_coefs[0]; // 起点误差
  ROS_DEBUG_STREAM("error from coef[0] = "<<error);

  // 求解优化问题，并计时
  const auto start_solve = ros::WallTime::now();
  mpc.solve(current_linear_velocity, cmd_steer_angle, control_coefs, cmd_steer_rate, cmd_acc, mpc_x, mpc_y);
  double solve_time = (ros::WallTime::now() - start_solve).toSec();
  ROS_DEBUG_STREAM("solve time = "<<solve_time);
  ROS_ERROR_STREAM_COND(solve_time > 0.08, "Solve time too big "<<solve_time);
  
  // 发布轨迹
  publish_trajectory();
  publish_poly();
  //send error for debug proposes
  publish_error(cross_track_error());
//  ROS_DEBUG_STREAM("angular_rate cmd = "<<angular_rate);
  publish_mpc_traj(mpc_x, mpc_y);
}

/**
 * @brief 机器人位姿回调函数   更新机器人位置、姿态
*/
void MPCController::on_pose(const nav_msgs::OdometryConstPtr& odom)
{
  robot_x = odom->pose.pose.position.x;
  robot_y = odom->pose.pose.position.y;
  robot_theta = 2*atan2(odom->pose.pose.orientation.z,
                        odom->pose.pose.orientation.w);

  world_frame_id = odom->header.frame_id;
  robot_time = odom->header.stamp;

//  current_velocity = odom->twist.twist.linear.x;
//  ROS_DEBUG_STREAM("x = "<<robot_x<<" "<<" y = "<<robot_y<<" "<<robot_theta);

//  ROS_DEBUG_STREAM("truth vel = "<<odom->twist.twist.linear.x);
}

/**
 * @brief 里程计回调函数   更新机器人速度、角速度、前轮转角
*/
void MPCController::on_odo(const nav_msgs::OdometryConstPtr& odom)
{
  current_linear_velocity = odom->twist.twist.linear.x;
  current_angular_velocity = odom->twist.twist.angular.z;
  if ( std::abs(current_linear_velocity) < 0.01 ) {
    current_curvature = current_angular_velocity/ current_linear_velocity;
    current_angle = atan(current_curvature*wheel_base);
  }
  // else left the same as before
  ROS_DEBUG_STREAM("odom vel = "<<current_linear_velocity<<" w = "<<current_angular_velocity<<" angle = "<<current_angle);
}

/**
 * @brief 发布跟踪误差
*/
void MPCController::publish_error(double error)
{
  std_msgs::Float32 err_msg;
  err_msg.data = error;
  err_pub.publish(err_msg);
}

/**
 * @brief 计算跟踪误差
*/
double MPCController::cross_track_error()
{
  double error = 0.0;
  if (robot_y < radius)
  {
    double rx = robot_x;
    double ry = robot_y - radius;
    error = sqrt(rx*rx + ry*ry) - radius;
  }
  else if ( robot_y > cy)
  {
    double rx = robot_x;
    double ry = robot_y - cy;
    error = sqrt(rx*rx + ry*ry) - radius;
  }
  else if ( robot_x > 0 )
  {
    error = robot_x - radius;
  }
  else if ( robot_x < 0 )
  {
    error = -radius - robot_x;
  }
  return error;
}

/**
 * 这个函数没有用到
*/
void MPCController::get_segment(std::list<TrajPtr>::iterator&  traj_it, double& len)
{
  traj_it = trajectory.end();

  if (robot_y < radius )
  {
    if ( robot_x >= 0 )
    {
      traj_it = trajectory.begin();
    }
  }
}

/**
 * @brief 向点云中添加点
*/
void add_point(sensor_msgs::PointCloud& msg, const tf::Vector3& point)
{
  geometry_msgs::Point32 p;
  p.x = point.x();
  p.y = point.y();
  p.z = point.z();
  msg.points.push_back(p);
}

/**
 * @brief 发布轨迹点云  这个是真实轨迹
*/
void MPCController::publish_trajectory()
{
  //prepare pointcloud message
  sensor_msgs::PointCloud msg;
  msg.header.frame_id = world_frame_id;
  msg.header.stamp = robot_time;
  static int seq(0);
  msg.header.seq = seq++;

  int trajectory_points_quantity = traj_length / traj_dl  + 1;
  int points_left = trajectory_points_quantity;
  msg.points.reserve( trajectory_points_quantity );
  double publish_len = 0;
  Trajectory::iterator it = current_segment;
  double start_segment_length = current_segment_length;
//  ROS_DEBUG_STREAM("start from "<<start_segment_length);

  while ( points_left )
  {
    double segment_length = (*it)->get_length();
    //add points from the segment
    int segment_points_quantity = std::min<int>( points_left, floor((segment_length - start_segment_length)/traj_dl) );
//    ROS_DEBUG_STREAM("segment points "<<segment_points_quantity);
    for ( int i = 0; i<=segment_points_quantity; ++i)
    {
      add_point(msg, (*it)->get_point(start_segment_length + i * traj_dl) );
    }
    points_left -= segment_points_quantity;
    //switch to next segment
    if (points_left)
    {
      //start point for next segment
      start_segment_length += (segment_points_quantity + 1)* traj_dl - segment_length;
      //ROS_DEBUG_STREAM("start segment length = "<<start_segment_length);
      ++it;
      if ( it == trajectory.end() )
        it = trajectory.begin();
    }
  }
//  ROS_DEBUG_STREAM("publish trajectory");
  traj_pub.publish(msg);
}

/**
 * @brief 发布控制多项式点云
*/
void MPCController::publish_poly() {
  //prepare pointcloud message
  sensor_msgs::PointCloud msg;
  msg.header.frame_id = world_frame_id;
  msg.header.stamp = robot_time;
  static int seq(0);
  msg.header.seq = seq++;
  double xrange = control_points_dl * control_points_num * 1.5;
  int trajectory_points_quantity = xrange / traj_dl;
  msg.points.reserve( trajectory_points_quantity );

  for(int i = 0; i<trajectory_points_quantity; ++i) {
    double x = i*traj_dl;
    tf::Vector3 point = robot2world(tf::Vector3(x, polyeval(x), 0));
    add_point(msg, point);
  }
  poly_pub.publish(msg);
}

/**
 * @brief 发布 MPC 预测轨迹点云
*/
void MPCController::publish_mpc_traj(std::vector<double>& x, std::vector<double>& y) {
  if (x.empty())
    return;
  sensor_msgs::PointCloud msg;
  msg.header.frame_id = world_frame_id;
  msg.header.stamp = robot_time;
  static int seq(0);
  msg.header.seq = seq++;
  msg.points.reserve( x.size() );
  for( int i = 0; i < x.size(); ++i) {
    add_point(msg, robot2world(tf::Vector3(x[i], y[i], 0)));
  }
  mpc_traj_pub.publish(msg);
}

/*!
 * \brief constructor
 * loads parameters from ns
 * proportional, differential , integral - pid factors
 * max_antiwindup_error - max error for using integral component
 * trajectory consists of two circle segments connected with two lines
 * first circle center is (0, radius), second circle center is (0, cy)
 * radius - radius of circular parts
 * cy - center of second circle
 * traj_dl - discrete of published trajectory
 * traj_length - length of published trajectory
 * timer_period - timer discrete
 */
MPCController::MPCController(const std::string& ns):
    nh("~/" + ns),
    radius( nh.param("radius", 10.0) ),
    cy( nh.param("cy", 2*radius) ), //default circle is in center (0,radius)
    wheel_base( nh.param("wheel_base", 1.88) ),
    max_steer_angle( nh.param("max_steer_angle", 0.3)),
    max_steer_rate( nh.param("max_steer_rate", 0.3) ),
    max_velocity( nh.param("max_velocity", 5.0) ),
    max_acc( nh.param("max_acc", 0.8) ),
    control_dt(nh.param("timer_period", 0.1)),
    traj_dl( nh.param("traj_dl", 0.2) ),
    traj_length( nh.param("traj_length", 5.0) ),
    pose_sub(nh.subscribe("ground_truth", 1, &MPCController::on_pose, this)),
    timer( nh.createTimer( ros::Duration(nh.param("timer_period", 0.1)), &MPCController::on_timer, this ) ),
    err_pub( nh.advertise<std_msgs::Float32>("error", 10) ),
    steer_pub( nh.advertise<std_msgs::Float32>("/steering", 1) ),
    vel_pub( nh.advertise<std_msgs::Float32>("/velocity", 1) ),
    odo_sub( nh.subscribe("odom", 1, &MPCController::on_odo, this)),
    traj_pub( nh.advertise<sensor_msgs::PointCloud>("trajectory", 1) ), // 一段真实轨迹
    poly_pub( nh.advertise<sensor_msgs::PointCloud>("poly", 1) ), // 多项式拟合轨迹结果
    mpc_traj_pub( nh.advertise<sensor_msgs::PointCloud>("mpc_traj", 1) ), // 发布 MPC 预估轨迹点
    mpc_steps( nh.param("mpc_steps", 4) ),
    mpc_dt( nh.param("mpc_dt", 0.5) ),
    mpc(mpc_steps, mpc_dt, max_velocity, max_acc, max_steer_angle, max_steer_rate, wheel_base,
        nh.param("kcte", 1.0),
        nh.param("kepsi", 1.0),
        nh.param("kev", 1.0),
        nh.param("ksteer_cost", 1.0))
{
  //counter clock
  trajectory.emplace_back( std::make_shared<trajectory::CircularSegment>( 1.0 / radius,    0,       0,    1.0,   0,   M_PI/2*radius) );
  trajectory.emplace_back( std::make_shared<trajectory::LinearSegment>  (        radius, radius, 0.0,   1.0,  cy - radius) );
  trajectory.emplace_back( std::make_shared<trajectory::CircularSegment>( 1.0 / radius,   radius,   cy,   0.0,   1.0, M_PI/2*radius ) );
  trajectory.emplace_back( std::make_shared<trajectory::CircularSegment>( 1.0 / radius,   0, radius + cy,   -1.0, 0.0, M_PI/2*radius ) );
  trajectory.emplace_back( std::make_shared<trajectory::LinearSegment>  (         -radius, cy,   0.0,   -1.0, cy - radius) );
  trajectory.emplace_back( std::make_shared<trajectory::CircularSegment>( 1.0/ radius,   -radius, radius, 0.0,  -1.0,  M_PI/2*radius) );

  //clock wise track
//  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0 / radius,    0,       0,    1.0,   0,   M_PI/2*radius) );
//  trajectory.emplace_back( std::make_shared<trajectory::LinearTrajectory>  (        radius, -radius, 0.0,   -1.0,  cy - radius) );
//  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0 / radius,   radius,   -cy,   0.0,   -1.0, M_PI/2*radius ) );
//  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0 / radius,   0, -radius - cy,   -1.0,  0.0, M_PI/2*radius ) );
//  trajectory.emplace_back( std::make_shared<trajectory::LinearTrajectory>  (       -radius, -cy,   0.0,   1.0, cy - radius) );
//  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0/ radius,   -radius, -radius, 0.0,  1.0,  M_PI/2*radius) );


  current_segment = trajectory.begin();
}


MPCController::~MPCController()
{
  // TODO Auto-generated destructor stub
}

} /* namespace simple_controller */
