#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>

const std::size_t ROBOT_STATE_SIZE = 3; // 机器人状态数，位置2，姿态1
const std::size_t NUMBER_LANDMARKS = 12; // 路标数
const double HUGE_COVARIANCE = 1e10; // 大协方差

class Slam {
private:
  ros::NodeHandle nh;
  ros::Subscriber odo_sub;
  ros::Subscriber scan_sub;
  // публикатор положения робота
  ros::Publisher pose_pub; // 发布机器人位置
  // публикатор положений маяков
  ros::Publisher landmark_pub[NUMBER_LANDMARKS]; // 发布路标位置，27维
  void on_odo(const nav_msgs::Odometry& odom);
  void on_scan(const sensor_msgs::LaserScan& scan);
  void publish_results(const std::string& frame, const ros::Time& time);
  void predict(double dt);
  void advertize_landmark_publishers();
  // поиск координад маяков по скану
  void detect_landmarks(const sensor_msgs::LaserScan& scan);
  void add_landmark(const sensor_msgs::LaserScan& scan, std::size_t start, std::size_t finish);
  // поиск индекса маяка в векторе состояния подходящего для измерения, -1 - новый маяк
  int associate_measurement(const Eigen::Vector2d& landmark_measuriment);
  int add_landmark_to_state(const Eigen::Vector2d& landmark_measuriment);
  void correct(int landmarkIndex, int measurementIndex);
  // публикуем результаты
  void publish_transform(const std_msgs::Header& scan_header);
  // velocity, angle velocity
  double v = 0;
  double w = 0;

  // state vector
  // вектор состояния
  Eigen::VectorXd X; // 状态向量，机器人位姿、路标位置，共
  // system Jacobi
  // линеаризованная матрица системы
  Eigen::Matrix3d A; // 雅可比
  // covariation of system
  // матрица ковариации ошибок оценок
  Eigen::MatrixXd P; // 信念协方差
  // covariation of measurement errors for each landmark
  // матрица ковариации ошибок измерения
  Eigen::Matrix2d Q; // 观测方程噪声协方差
  // covariation of system vulnerability for x y fi
  // матрица ковариации возмущений системы
  Eigen::Matrix3d R; // 运动方程测量协方差

  ros::Time last_time = ros::Time::now();

  std::size_t landmarks_found_quantity = 0; // 已经找到的路标总数

  std::vector<Eigen::Vector2d> new_landmarks; // 当前帧路标在机器人系下的坐标
  std::vector<Eigen::Vector2d> new_landmarks_measurement; // 当前帧路标测量结果

  tf::TransformBroadcaster br;

  const std::string map_frame = nh.param<std::string>("map_frame", "map"); // 地图坐标系名称
  double feature_rad = nh.param<double>("feature_radius", 0.55); // 特征点半径
public:
  Slam();
};
