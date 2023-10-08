#ifndef SRC_SIMPLE_PLANNER_SRC_PLANNER_H_
#define SRC_SIMPLE_PLANNER_SRC_PLANNER_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
// #include <ceres/ceres.h>

namespace simple_planner
{


// структура, описывающая узел поиска
// 搜索结构
struct SearchNode{
  enum State {
    CLOSE, OPEN, UNDEFINED
  }; // 闭，开，未定义
  State state = UNDEFINED;
  // значение функции оптимальной стоимости достижения узла
  double g = std::numeric_limits<double>::max(); // 距离起点代价，初始至为最大
  // значение функции эвристики
  double h = 0; // 启发式
  // 前一个点
  int pre_i = -1, pre_j = -1;
};

// 地图点序号结构
struct MapIndex {
  int i;
  int j;
};

class Planner
{
public:
  Planner(ros::NodeHandle& nh);

private:
  friend class CompareSearchNodes;
  // обновление положения робота
  void on_pose(const nav_msgs::Odometry& odom);
  // колбек целевой точки
  void on_target(const geometry_msgs::PoseStamped& pose);
  // функция обновления карты (map_)
  bool update_static_map();
  // функция расширения карты препятствий (obstacle_map_)
  void increase_obstacles(std::size_t cells);
  // функция вычисления пути в заданную точку
  void calculate_path_astar();
  void calculate_path_lee();
  void trajectorySmoothing();


  double heuristic(int i, int j);

  // функции для работы с картами и индексами
  // Проверка индексов на нахождение в карте
  bool indices_in_map(int i, int j);

  // Возвращает ссылку на значение в карте
  /**
   * @brief 从一维向量获取地图 i, j 位置上的元素
  */
  template <class T>
  T& map_value(std::vector<T>& data, int i, int j)
  {
    int index = j * map_.info.width + i;
    ROS_ASSERT(index < data.size() && index >= 0);
    return data[index];
  }

  /**
   * @brief 获取空间位置 x, y 对应地图中的像素位置
  */
  MapIndex point_index(double x, double y) {
    return {
     static_cast<int>(floor((x - map_.info.origin.position.x)/ map_.info.resolution)),
     static_cast<int>(floor((y - map_.info.origin.position.y)/ map_.info.resolution))
    };
  }

  /**
   * @brief 获取地图点 x, y 对应空间位置
  */
  geometry_msgs::Point32 point_position(int i, int j) {
    geometry_msgs::Point32 point;
    point.x = (i * map_.info.resolution) + map_.info.origin.position.x;
    point.y = (j * map_.info.resolution) + map_.info.origin.position.y;
    point.z = 0.0;
    return point;
  }

private:
  ros::NodeHandle nh_;

  nav_msgs::OccupancyGrid map_; // 原始静态地图
  nav_msgs::OccupancyGrid obstacle_map_; // 扩张地图
  nav_msgs::OccupancyGrid cost_map_; // 代价地图
  std::vector<SearchNode> search_map_; // 搜索地图
  cv::Mat searchmap_img;

  ros::Publisher obstacle_map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("obstacle_map", 1);
  ros::Publisher cost_map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("cost_map", 1);
  ros::Publisher path_publisher_ = nh_.advertise<nav_msgs::Path>("path", 1);
  ros::Publisher searchmap_img_pubilsher = nh_.advertise<sensor_msgs::Image>("searchmap_img", 1);
  ros::Publisher controllerPath_publisher = nh_.advertise<sensor_msgs::Image>("searchmap_img", 1);

  ros::ServiceClient map_server_client_ =  nh_.serviceClient<nav_msgs::GetMap>("/static_map");

  ros::Subscriber pose_sub_ = nh_.subscribe("ground_truth", 1, &Planner::on_pose, this);
  ros::Subscriber target_sub_ = nh_.subscribe("target_pose", 1, &Planner::on_target, this);

  geometry_msgs::Pose start_pose_;
  MapIndex targetPoint;
  geometry_msgs::Pose target_pose_;

  sensor_msgs::PointCloud path_msg_; // 这个不是用来发送的消息，轨迹暂存在这里
  

  double robot_radius_ = nh_.param("robot_radius", 0.5);

};

} /* namespace simple_planner */

#endif /* SRC_SIMPLE_PLANNER_SRC_PLANNER_H_ */
