#include "slam.h"
#include <angles/angles.h>
#include <sstream>
#include <math.h>

/**
 * @brief 里程计回调函数
 * 
 * 获取机器人速度，机器人速度作为运动模型的输入
*/
void Slam::on_odo(const nav_msgs::Odometry& odom)
{
  v = odom.twist.twist.linear.x;
  w = odom.twist.twist.angular.z;
}

/**
 * @brief 向当前帧路标集合中添加路标
 * 
 * Dknt 2023.10.15
*/
void Slam::add_landmark(const sensor_msgs::LaserScan& scan, std::size_t start, std::size_t finish) {
  // добавляем только особенные  точки на которые попало более 2 лучей
  if (finish - start < 2) {
    return;
  }
  // ROS_INFO_STREAM("Add landmark between " << start << " " << finish);
  // TODO Здесь должен быть код определения координаты центра круглого препятствия

  double x = 0.0, y = 0.0;
  // 根据扫描中点确定特征点中心
  double minDist = 1e100;
  size_t minIdx;
  for (size_t idx = start; idx < finish; ++idx) {
    if (scan.ranges[idx] < minDist) {
      minDist = scan.ranges[idx];
      minIdx = idx;
    }
  }
  double pointDist = minDist + feature_rad;
  double pointAngle = minIdx * scan.angle_increment - M_PI_2;
  // Feature position relative to laser
  x = pointDist * cos(pointAngle);
  y = pointDist * sin(pointAngle);

  // 添加相对于激光雷达的坐标
  // добавляем в вектор особенных точек
  new_landmarks.push_back(Eigen::Vector2d(x, y));
  new_landmarks_measurement.push_back(Eigen::Vector2d(pointDist, pointAngle));
}

/**
 * @brief 检测路标
 * 
 * Dknt 2023.10.15
*/
void Slam::detect_landmarks(const sensor_msgs::LaserScan& scan)
{
  new_landmarks.clear();
  new_landmarks_measurement.clear();
  // TODO Здесь должен быть код для пределения особенные точек скана
  // ищем начальный и конечный индексы лучей, падающих на одно препятствие
  // и вызываем add_landmark
  size_t index = 0;
  while (true) {
    if (scan.ranges[index] > scan.range_max - 1.0) {
      ++index;
    }
    else if (abs(scan.ranges[index] - scan.ranges[index + 1]) < feature_rad) {
      size_t end_index = index + 1;
      while (abs(scan.ranges[end_index] - scan.ranges[end_index + 1]) < feature_rad && end_index < scan.ranges.size() - 1)
        ++end_index;
      add_landmark(scan, index, end_index);
      index = end_index + 1;
    }
    else ++index;
    if (index >= scan.ranges.size()) break;
  }
}

/**
 * @brief 路标匹配
*/
int Slam::associate_measurement(const Eigen::Vector2d& landmark_measurement) {
  double nearest_distance = 1e10;
  int nearest_index = -1;
  // преобразование от СК карты к СК робота (дальномера)
  // 机器人到地图的变换，上面的描述不对
  Eigen::Isometry2d robot_to_map = Eigen::Translation2d(X.segment(0, 2))
                                 * Eigen::Rotation2Dd(X(2));
  // 寻找最小偏差
  for (std::size_t i = 0; i < landmarks_found_quantity; ++i) {
    // 按照预测的机器人位姿，将检测到的路标点变换到地图系下，与已知的路标进行匹配
    double distance = (robot_to_map * landmark_measurement - X.segment(ROBOT_STATE_SIZE + i*2, 2)).norm();
    if (distance < nearest_distance) {
      nearest_index = i;
      nearest_distance = distance;
    }
  }
  // naive association
  // 误差最大限度为 5 m
  const double kAssocThreshold = 5.0;
  if (nearest_index >= 0 && nearest_distance < kAssocThreshold) {
    return nearest_index;
  }
  // 没有找到匹配，返回-1
  return -1;
}

/**
 * @brief 添加新路标到状态中
 * 
 * Dknt 2023.10.15
*/
int Slam::add_landmark_to_state(const Eigen::Vector2d& landmark_measurement)
{
  ++landmarks_found_quantity;
  // TODO init landmark in state
  // Здесь должен быть код по инициализации части вектора состояния, соответствующей 
  // маяку с индексом last_found_landmark_index
  Eigen::Vector2d newLandmark;
  // 机器人到地图的坐标变换
  Eigen::Isometry2d robot_to_map = Eigen::Translation2d(X.segment(0, 2))
                                 * Eigen::Rotation2Dd(X(2));
  newLandmark = robot_to_map * landmark_measurement;
  X[ROBOT_STATE_SIZE + 2 * landmarks_found_quantity - 2] = newLandmark[0];
  X[ROBOT_STATE_SIZE + 2 * landmarks_found_quantity - 1] = newLandmark[1];

  // 更新协方差
  P.block(ROBOT_STATE_SIZE + 2 * landmarks_found_quantity - 2, 
          ROBOT_STATE_SIZE + 2 * landmarks_found_quantity - 2, 2, 2) = Q;

  ROS_WARN("Adding landmark to state, x: %f y: %f", newLandmark[0], newLandmark[1]);
  return landmarks_found_quantity;
}

/**
 * @brief EKF 校正
 * 
 * @param landmarkIndex 路标集合中的下标
 * @param measurementIndex 当前测量结果中的下标
*/
void Slam::correct(int landmarkIndex, int measurementIndex) {
  /**
   * Measurement model
   * 
   * r[i,t] = ((Yi0 - X0[t])^2 - (Yi1 - X1[t])^2)^0.5 + e
   * phi[i,t] = atan((Yi1 - X1[t])/(Yi0 - X0[t])) - X2[t] + e
   *
   * Y - landmark position
  */
  // TODO 
  // Здесь должен быть код для обновления состояния по измерению iого маяка
  ROS_INFO("correct landmarkIndex: %d, measurementIndex: %d", landmarkIndex, measurementIndex);
  // robot pose predict
  double x = X(0);
  double y = X(1);
  double theta = X(2);

  // landmark position prior
  double mi_x = X(ROBOT_STATE_SIZE + landmarkIndex * 2);
  double mi_y = X(ROBOT_STATE_SIZE + landmarkIndex * 2 + 1);

  // measurement result
  double zi_r = new_landmarks_measurement[measurementIndex](0);
  double zi_phi = new_landmarks_measurement[measurementIndex](1);

  double dist2 = pow(mi_x - x, 2.0) + pow(mi_y - y, 2.0);
  double dist = sqrt(pow(mi_x - x, 2.0) + pow(mi_y - y, 2.0));

  double x_mi_x = x - mi_x;
  double y_mi_y = y - mi_y;

  std::cout << "mi posit x: " << mi_x << " y: " << mi_y <<
               " -- x: " << x << " y: " << y << " - dist " 
               << dist << " measurement: " <<  zi_r << std::endl;

  double zi_r_pred = dist;
  double zi_phi_pred = atan2(-y_mi_y, -x_mi_x);
  Eigen::Vector2d measurementPred(zi_r_pred, zi_phi_pred);

  // Jacobian for robot pose
  Eigen::Matrix<double, 2, 3> Gi_x;
  Gi_x << (x_mi_x / dist), (y_mi_y / dist), 0,
          (-y_mi_y / dist2), (x_mi_x / dist2), -1;
  // std::cout << "Gi_x:\n" << Gi_x << std::endl;

  // Jacobian for landmark position
  Eigen::Matrix<double, 2, 2> Gi_m;
  Gi_m << (-x_mi_x / dist), (-y_mi_y / dist),
          (y_mi_y / dist2), (-x_mi_x / dist2);
  // std::cout << "Gi_m:\n" << Gi_m << std::endl;

  Eigen::Matrix<double, 2, ROBOT_STATE_SIZE + NUMBER_LANDMARKS * 2> Gi;
  Gi.setZero();
  Gi.block(0, 0, 2, 3) = Gi_x;
  Gi.block(0, ROBOT_STATE_SIZE + landmarkIndex * 2, 2, 2) = Gi_m;
  // std::cout << "Gi:\n" << Gi << std::endl;

  // Kalman gain matrix
  Eigen::Matrix<double, 2, 2> tempMat;
  tempMat = Gi * P * Gi.transpose() + Q;
  Eigen::Matrix<double, ROBOT_STATE_SIZE + NUMBER_LANDMARKS * 2, 2> K;
  K.setZero();
  K = P * Gi.transpose() * tempMat.inverse();
  // std::cout << "K:\n" << K << std::endl;

  // update posterior covariance
  P = (Eigen::Matrix<double, ROBOT_STATE_SIZE + NUMBER_LANDMARKS * 2, ROBOT_STATE_SIZE + NUMBER_LANDMARKS * 2>::Identity() - K * Gi) * P;

  // update posterior mean
  // X = X + K * (new_landmarks_measurement[measurementIndex] - measurementPred);
  std::cout << "pred err: " << (new_landmarks_measurement[measurementIndex] - measurementPred).transpose() << std::endl;
  
}

/**
 * @brief 激光雷达扫描回调函数
 * 
*/
void Slam::on_scan(const sensor_msgs::LaserScan& scan) {
  ROS_INFO("on_scan");
  detect_landmarks(scan); // 检测路标
  predict((scan.header.stamp - last_time).toSec()); // 预测
  last_time = scan.header.stamp;
  for (std::size_t i = 0; i < new_landmarks.size(); ++i) {
    const auto landmark_index = associate_measurement(new_landmarks[i]); // 路标匹配
    if (landmark_index >= 0) {
      correct(landmark_index, i); // 矫正
    } else {
        if (landmarks_found_quantity < NUMBER_LANDMARKS) { // 检测到新路标，加入状态
          add_landmark_to_state(new_landmarks[i]); // 将新路标加入系统状态
        } else { // 已经达到最大目标数
            // ROS_ERROR_STREAM("can not associate new landmark with any existing one");
        }
    }
  }
  publish_results("map", scan.header.stamp); // 发布
  publish_transform(scan.header); // 发布
}

/**
 * @brief 填充路标位置信息，带不确定性
*/
void fill_pose_msg(geometry_msgs::PoseWithCovariance& pose,
                   double x, double y, double fi,
                   const Eigen::Matrix2d& cov_matr) {
  // 3位置 3姿态  协方差阵为 6*6
  pose.covariance.assign(0);
  pose.covariance[0] = cov_matr(0,0); pose.covariance[1] = cov_matr(0,1);
  pose.covariance[6] = cov_matr(1,0); pose.covariance[7] = cov_matr(1,1);
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.w = cos(fi/2);
  pose.pose.orientation.z = sin(fi/2);
}

/**
 * @brief 填充机器人位姿信息
*/
void fill_pose_msg(geometry_msgs::Pose& pose,
                   double x, double y, double fi) {
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.w = cos(fi/2);
    pose.orientation.z = sin(fi/2);
}

/**
 * @brief 
*/
void Slam::publish_results(const std::string& frame, const ros::Time& time) {
  // 发布机器人位姿
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp = time;
  // публикуем сообщение с позицией робота
  fill_pose_msg(pose.pose, X(0), X(1), X(2));
  pose_pub.publish(pose);

  // 发布路标位置
  // публикуем сообщения с положениями маяков
  for (int i = 0; i < landmarks_found_quantity; ++i)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.header.stamp = time;
    fill_pose_msg(pose.pose, X(ROBOT_STATE_SIZE + i * 2), X(ROBOT_STATE_SIZE + i * 2 + 1), 0);
    landmark_pub[i].publish(pose);
  }
}

/**
 * @brief 发布坐标变换
*/
void Slam::publish_transform(const std_msgs::Header& scan_header)
{  
  // публикуем трансформ от скана до карты, 
  // не наоборот, так как дерево tf - однонаправленное
  tf::Transform tf_transform;

  // 机器人相对于地图
  double angle = X(2);
  Eigen::Matrix2d R;
  R (0, 0) = R (1, 1) = cos (angle);
  R (0, 1) = -sin (angle);
  R (1, 0) = sin (angle);
  Eigen::Vector2d t = X.head(2);
  Eigen::Isometry2d transform = Eigen::Translation2d(t) * Eigen::Isometry2d(R);

  // 地图相对于机器人
  Eigen::Isometry2d inverted_transform = transform.inverse();

  tf_transform.setOrigin( tf::Vector3(inverted_transform.translation().x(), 
                      inverted_transform.translation().y(), 
                      0.0) );
  tf::Quaternion inv_q;
  const auto& inv_matrix = inverted_transform.matrix();
  double inv_yaw = atan2(inv_matrix(1, 0), inv_matrix(0, 0));
  inv_q.setRPY(0, 0, inv_yaw);
  tf_transform.setRotation(inv_q);

  // 发布地图相对于机器人的变换
  br.sendTransform(tf::StampedTransform(tf_transform, 
                                        scan_header.stamp, 
                                        scan_header.frame_id, 
                                        map_frame));
}

/**
 * @brief EKF 预测
 * 
 * 预测机器人位姿，更新协方差矩阵
*/
void Slam::predict(double dt)
{
  /**
   * Simplified Motion Model
   * 
   * X0[t+1] = X0[t] + v[t+1]*cos(X2[t])*dt + e0
   * X1[t+1] = X1[t] + v[t+1]*sin(X2[t])*dt + e1
   * X2[t+1] = X2[t] + w[t+1]*dt + e2
  */
  // 预测机器人位姿
  // X(t+1) = g(t)
  X(0) += v * cos(X(2)) * dt;
  X(1) += v * sin(X(2)) * dt;
  X(2) += w * dt;
  X(2) = angles::normalize_angle(X(2));

  // 计算运动方程雅可比，在预测点线性化
  // вычисляем якобиан
  A = Eigen::Matrix3d::Identity();
  A(0,0) = 1.0; A(0,1) = 0; A(0,2) = -v * sin(X(2)) * dt;
  A(1,0) = 0.0; A(1,1) = 1.0; A(1,2) = v * cos(X(2)) * dt;
  A(2,0) = 0.0; A(2,1) = 0.0; A(2,2) = 1.0;

  // 镐形矩阵
  // P = A*P*AT + R для блока соответствующего роботу
  P.topLeftCorner(ROBOT_STATE_SIZE, ROBOT_STATE_SIZE) =
      A * P.topLeftCorner(ROBOT_STATE_SIZE, ROBOT_STATE_SIZE) * A.transpose() + R;
  // для остальных блоков
  // 右上
  P.topRightCorner(ROBOT_STATE_SIZE, NUMBER_LANDMARKS * 2) = 
    A * P.topRightCorner(ROBOT_STATE_SIZE, NUMBER_LANDMARKS * 2);
  // 右下
  P.bottomLeftCorner(NUMBER_LANDMARKS * 2, ROBOT_STATE_SIZE) = 
    P.topRightCorner(ROBOT_STATE_SIZE, NUMBER_LANDMARKS * 2).transpose();
}

/**
 * @brief 创建路标发布者
*/
void Slam::advertize_landmark_publishers()
{
  std::string landmark("landmark");
  for (int i = 0; i < NUMBER_LANDMARKS; ++i)
  {
    std::stringstream stream;
    // landmark0 landmark1 ...
    stream << landmark << i;
    landmark_pub[i] = nh.advertise<geometry_msgs::PoseStamped>(stream.str(), 1);
  }
}

/**
 * @brief 构造函数
*/
Slam::Slam():
    nh("~"),
    odo_sub(nh.subscribe("/odom", 1, &Slam::on_odo, this)), // 里程计收听者
    scan_sub(nh.subscribe("/scan", 1, &Slam::on_scan, this)), // 激光雷达扫描收听者
    pose_pub(nh.advertise<geometry_msgs::PoseStamped>("slam_pose", 1)), // 位姿发布者
    X(ROBOT_STATE_SIZE + 2*NUMBER_LANDMARKS), // 27 维状态向量
    A(Eigen::Matrix3d::Identity()), // 雅可比，初值为单位矩阵
    P(Eigen::MatrixXd::Zero(X.size(), X.size())) // 信念协方差，初值为 0
{
  // начальный вектор состояния заполняем нулями
  // 初始状态为 0
  X = Eigen::VectorXd::Zero(X.size());
  // записываем огромное значение начальной ковариации для маяков
  // 路标先验为均匀分布
  P.bottomRightCorner(NUMBER_LANDMARKS * 2, NUMBER_LANDMARKS * 2) =
      HUGE_COVARIANCE * Eigen::MatrixXd::Identity(NUMBER_LANDMARKS * 2, NUMBER_LANDMARKS * 2);

  advertize_landmark_publishers();

  // 观测方程误差协方差
  Q = Eigen::Matrix2d::Zero();
  Q(0, 0) = nh.param<double>("range_sigma_sqr", 0.01);
  Q(1, 1) = nh.param<double>("angle_sigma_sqr", 0.001);

  // 运动方程误差协方差
  R = Eigen::Matrix3d::Zero();
  R(0, 0) = nh.param<double>("x_sigma_sqr", 0.01);
  R(1, 1) = nh.param<double>("y_sigma_sqr", 0.01);
  R(2, 2) = nh.param<double>("angle_sigma_sqr", 0.001);

  std::cout.precision(4);
}
