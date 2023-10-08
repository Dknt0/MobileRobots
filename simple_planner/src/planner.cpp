#include "planner.h"

#include <cstddef>
#include <queue>
#include <set>
#include <utility>

namespace simple_planner
{

const MapIndex neighbors[8] = { {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
const int8_t kObstacleValue = 100; // 地图中障碍物的值

/**
 * 构造函数  等待地图服务
*/
Planner::Planner(ros::NodeHandle& nh) :
 nh_(nh)
{
    while(!map_server_client_.waitForExistence(ros::Duration(1))) {
        ROS_INFO_STREAM("Wait map server");
    }
    ROS_INFO_STREAM("Service connected");
}

/**
 * @brief 位姿话题回调函数   起点为当前点
*/
void Planner::on_pose(const nav_msgs::Odometry& odom)
{
    start_pose_ = odom.pose.pose;
}

/**
 * @brief 目标点话题回调函数
*/
void Planner::on_target(const geometry_msgs::PoseStamped& pose)
{
    ROS_INFO_STREAM("Get goal " << pose.pose.position.x << " " << pose.pose.position.y);
    ROS_INFO_STREAM("Start is " << start_pose_.position.x << " " << start_pose_.position.y);
    target_pose_ = pose.pose;

    // 更新地图
    if (!update_static_map() )
    {
        ROS_ERROR_STREAM("Can not receive map");
        return ;
    }

    increase_obstacles(ceil(robot_radius_/map_.info.resolution));
    obstacle_map_publisher_.publish(obstacle_map_);

    // A*
    calculate_path_astar();
    // Lee
    // calculate_path_lee();
    cost_map_publisher_.publish(cost_map_);
    
    // 发布规划地图
    searchmap_img = cv::Mat(cv::Size(map_.info.width, map_.info.height), CV_8UC3);
    cv::MatIterator_<cv::Vec3b> itr = searchmap_img.begin<cv::Vec3b>();
    for (size_t index = 0; index < search_map_.size(); ++index) {
        if (search_map_[index].state == SearchNode::UNDEFINED) {
            *itr = cv::Vec3b(255, 0, 0); // Blue unpassed area
        }
        else {
            *itr = cv::Vec3b(0, 255, 0); // green passed area
        }
        if (cost_map_.data[index] == kObstacleValue) {
            *itr = cv::Vec3b(0, 0, 255); // Red obstacle
        }
        itr++;
    }
    cv::cvtColor(searchmap_img, searchmap_img, cv::COLOR_BGR2RGB);
    cv::flip(searchmap_img, searchmap_img, 0);
    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", searchmap_img).toImageMsg();
    searchmap_img_pubilsher.publish(imgMsg);


    if (!path_msg_.points.empty()) {
        // path_msg_.header.stamp = ros::Time::now();
        // path_msg_.header.frame_id = pose.header.frame_id;
        nav_msgs::Path navPath_msg;
        navPath_msg.header.frame_id = pose.header.frame_id;
        navPath_msg.header.stamp = ros::Time::now();
        for (size_t index = 0; index < path_msg_.points.size(); ++index) {
            geometry_msgs::PoseStamped poseTemp;
            poseTemp.header.frame_id = pose.header.frame_id;
            poseTemp.pose.position.x = path_msg_.points[index].x;
            poseTemp.pose.position.y = path_msg_.points[index].y;
            // TODO: 姿态
            navPath_msg.poses.push_back(poseTemp);
        }
        path_publisher_.publish(navPath_msg);
    } else {
        ROS_WARN_STREAM("Path not found!");
    }
}

/**
 * @brief 更新静态地图
 * 
 * 向 mapserver 获取地图
*/
bool Planner::update_static_map()
{
    nav_msgs::GetMap service;
    if (!map_server_client_.call(service))
    {
        ROS_ERROR_STREAM("Failed to receive a map");
        return false;
    }
    map_ = service.response.map;
    
    ROS_INFO_STREAM("Map received : " << map_.info.width << " " << map_.info.height);
    return true;
}

/**
 * @brief 判断给定序号是否合理
*/
bool Planner::indices_in_map(int i, int j)
{
    return i >= 0 && j >= 0 && i < map_.info.width && j < map_.info.height;
}

/**
 * @brief 扩张障碍范围  Lee Algorithm (волновой алгоритм)
 * 
 * @param cells 扩张半径 像素为单位
*/
void Planner::increase_obstacles(std::size_t cells)
{
    // 复制原始地图信息
    obstacle_map_.info = map_.info;
    obstacle_map_.header = map_.header;
    obstacle_map_.data.resize(map_.data.size());
    obstacle_map_.data = map_.data;

    // 重置代价地图
    cost_map_.info = map_.info;
    cost_map_.header = map_.header;
    cost_map_.data.resize(map_.data.size());
    
    std::queue<MapIndex> wave; // 波队列
    // 遍历原始地图，将所有障碍物放入波队列中
    for (int i = 0; i < map_.info.width; ++i)
    {
        for (int j = 0; j < map_.info.height; ++j)
        {
            // 如果不是障碍物，跳过
            if (map_value(map_.data, i, j) != kObstacleValue)
            {
                continue;
            }
            // else - obstacle
            // check neighbors
            // 如果是障碍点，查询近邻点中是否存在可达点
            for(const auto& shift : neighbors)
            {
                int neighbor_i = i + shift.i;
                int neighbor_j = j + shift.j;
                // 如果点在地图外，跳过
                if (!indices_in_map(neighbor_i, neighbor_j))
                {
                    continue;
                }
                // if neighbor is not obstacle - add i, j to wave
                // 如果存在一个近邻点可达，则将障碍加入波队列
                if (map_value(map_.data, neighbor_i, neighbor_j) != kObstacleValue)
                {
                    wave.push({i, j});
                    break;
                }
            }
        }
    }
    ROS_INFO_STREAM("Start wave size = " << wave.size());

    // 进行 n 次扩张
    for(std::size_t step = 0; step < cells; ++step)
    {
        std::queue<MapIndex> next_wave; // 下一次扩张的波队列
        while(!wave.empty()) {
            auto indices = wave.front(); // 当前
            wave.pop(); // 弹出队首
            // 遍历近邻  8 连通
            for(const auto& shift : neighbors)
            {
                auto neightbor_index = indices;
                neightbor_index.i += shift.i;
                neightbor_index.j += shift.j;
                // 如果点在地图外 跳过
                if (!indices_in_map(neightbor_index.i, neightbor_index.j))
                {
                    continue;
                }
                // 如果该近邻点不是障碍，则变为障碍，并放入下一次扩张队列
                if (map_value(obstacle_map_.data, neightbor_index.i, neightbor_index.j) != kObstacleValue)
                {
                    map_value(obstacle_map_.data, neightbor_index.i, neightbor_index.j) = kObstacleValue;
                    next_wave.push(neightbor_index);
                }
            }
        } // wave empty
        std::swap(wave, next_wave);
        // ROS_INFO_STREAM("Wave size = " << wave.size());
    }

    ROS_INFO("increase_obstacles Finished.");

    for (size_t index = 0; index < cost_map_.data.size(); ++index) {
        if (obstacle_map_.data[index] == kObstacleValue) {
            cost_map_.data[index] = kObstacleValue;
        }
        else {
            cost_map_.data[index] = 1;
        }
    }
}

/**
 * @brief 启发函数  Heruistic function
 * 
 * If heuristic is always 0, A* degenerates to Dijkstra.
 * Since our car can move in any direction, using Euclidean distance here.
 * 
 * Dknt 2023.10.8
*/
double Planner::heuristic(int i, int j) {
    double h = sqrt(pow(targetPoint.i - i, 2) + pow(targetPoint.j - j, 2));

    return h * 1.5;
}

/**
 * 谓词  比较搜索节点
*/
class CompareSearchNodes {
public:
    explicit CompareSearchNodes(Planner& planner): planner_(planner) {}
    bool operator () (const MapIndex& left_index, const MapIndex& right_index) const {
        SearchNode& left = planner_.map_value(planner_.search_map_, left_index.i, left_index.j);
        SearchNode& right = planner_.map_value(planner_.search_map_, right_index.i, right_index.j);
        // 处理相等的情况
        if (left.g + left.h == right.g + right.h) {
            if (left_index.i == right_index.i) {
                return left_index.j < right_index.j;
            }
            return left_index.i < right_index.i;
        }
        return left.g + left.h < right.g + right.h;
    }
private:
    Planner& planner_;
};

/**
 * @param 计算路径 A* algorithm
 * 
 * See https://www.redblobgames.com/pathfinding/a-star/introduction.html
 * 
 * Dknt 2023.10.8
*/
void Planner::calculate_path_astar() {
    // 重置搜索地图
    search_map_.resize(map_.data.size());
    std::fill(search_map_.begin(), search_map_.end(), SearchNode());
    path_msg_.points.clear();

    // 创建集合  集合存放搜索节点位置
    std::set<MapIndex, CompareSearchNodes> queue(CompareSearchNodes(*this));

    MapIndex startPoint = point_index(start_pose_.position.x, start_pose_.position.y);
    targetPoint = point_index(target_pose_.position.x, target_pose_.position.y);

    // 查询起点是否为障碍
    auto& start_obstacle_value = map_value(obstacle_map_.data, startPoint.i, startPoint.j);
    if (start_obstacle_value == kObstacleValue) {
        ROS_WARN_STREAM("Start is in obstacle!");
        return;
    }

    queue.insert(startPoint);
    map_value(search_map_, startPoint.i, startPoint.j).g = 0; // 起点代价
    map_value(search_map_, startPoint.i, startPoint.j).h = heuristic(startPoint.i, startPoint.j);
    map_value(search_map_, startPoint.i, startPoint.j).state = SearchNode::OPEN;

    bool foundTarget = false;
    double maxG = 0.0;
    auto getNeighborDistance = [](int index) {
        if (index % 2 == 0) return 1.0;
        else return 1.414;
    };

    // 遍历，直到寻找到目标点
    while (!foundTarget && !queue.empty()) {
        // std::cout << "Current queue size: " << queue.size() << std::endl;

        // 取队首目标点
        auto currentPointItr = queue.begin();
        auto currentPoint = *(currentPointItr);
        queue.erase(currentPointItr);
        auto& currentNode = map_value(search_map_, currentPoint.i, currentPoint.j);
        currentNode.state = SearchNode::CLOSE;
        
        // std::cout << "Wave size: " << wave.size() << std::endl;

        // 遍历近邻点    8 近邻
        for (size_t index = 0; index < 8; ++index) {
            const auto& shift = neighbors[index];
            auto neighborPoint = currentPoint;
            neighborPoint.i += shift.i;
            neighborPoint.j += shift.j;

            // 判断点是否在地图外
            if (!indices_in_map(neighborPoint.i, neighborPoint.j)) {
                continue;
            }

            // 判断近邻点是否为障碍
            if (map_value(obstacle_map_.data, neighborPoint.i, neighborPoint.j) == kObstacleValue) {
                map_value(search_map_, neighborPoint.i, neighborPoint.j).state = SearchNode::CLOSE;
                continue;
            }

            auto& neighborNode = map_value(search_map_, neighborPoint.i, neighborPoint.j);

            // 判断近邻点是否为目标点
            if (neighborPoint.i == targetPoint.i && neighborPoint.j == targetPoint.j) {
                foundTarget = true;
                neighborNode.g = currentNode.g + getNeighborDistance(index);
                neighborNode.pre_i = currentPoint.i;
                neighborNode.pre_j = currentPoint.j;
                neighborNode.state = SearchNode::CLOSE;
                break;
            }

            double tempG = currentNode.g + getNeighborDistance(index);
            if (tempG > maxG) maxG = tempG;

            // 如果近邻点在波队列中，或被处理过，跳过   考虑对角线可能会使距离更近，进行一次比较处理
            if (neighborNode.state == SearchNode::CLOSE || neighborNode.state == SearchNode::OPEN) {
                if (tempG < neighborNode.g) {
                    neighborNode.g = tempG;
                    neighborNode.pre_i = currentPoint.i;
                    neighborNode.pre_j = currentPoint.j;
                }
                continue;
            }

            // 更新搜索地图信息
            neighborNode.pre_i = currentPoint.i;
            neighborNode.pre_j = currentPoint.j;
            neighborNode.state = SearchNode::OPEN;
            neighborNode.g = tempG;
            neighborNode.h = heuristic(neighborPoint.i, neighborPoint.j);
            // 将近邻点加入到集合中
            queue.insert(neighborPoint);
        }
    }
    
    // 判断是否存在轨迹
    if (foundTarget) {
        ROS_INFO("Found path to target.");
    }
    else {
        ROS_INFO("Target cann't be reached.");
        return;
    }

    // 计算轨迹
    path_msg_.points.clear();
    std::vector<geometry_msgs::Point32, std::allocator<geometry_msgs::Point32>> tempPath;
    auto pathPoint = targetPoint;

    while (true) {
        geometry_msgs::Point32 tempPoint = point_position(pathPoint.i, pathPoint.j);
        tempPath.push_back(tempPoint);

        auto &node = map_value(search_map_, pathPoint.i, pathPoint.j);
        if (node.g == 0) break;
        pathPoint.i = node.pre_i;
        pathPoint.j = node.pre_j;
    }

    // 反向
    while (!tempPath.empty()) {
        auto &point = tempPath.back();
        path_msg_.points.push_back(point);
        tempPath.pop_back();
    }

    
}

/**
 * @param 计算路径  Lee algorithm  волновой алгоритм поиска
 * 
 * 类内共四张地图，将代价地图 obstacle_map_ 作为已知条件
 * 维护 search_map_, cost_map_
 * 
 * 遍历通过索引进行，通过集合中的索引可以直接访问值
 * 
 * Dknt 2023.10.7
*/
void Planner::calculate_path_lee() {
    // 重置搜索地图
    search_map_.resize(map_.data.size());
    std::fill(search_map_.begin(), search_map_.end(), SearchNode());
    path_msg_.points.clear();

    // 创建波队列  不需要排序，所以使用 queue
    std::queue<MapIndex> wave;

    MapIndex startPoint = point_index(start_pose_.position.x, start_pose_.position.y);
    MapIndex targetPoint = point_index(target_pose_.position.x, target_pose_.position.y);

    // 查询起点是否为障碍
    auto& start_obstacle_value = map_value(obstacle_map_.data, startPoint.i, startPoint.j);
    if (start_obstacle_value == kObstacleValue) {
        ROS_WARN_STREAM("Start is in obstacle!");
        return;
    }

    wave.push(startPoint);
    map_value(search_map_, startPoint.i, startPoint.j).g = 0; // 起点代价
    map_value(search_map_, startPoint.i, startPoint.j).state = SearchNode::OPEN;

    bool foundTarget = false;
    double maxG = 0.0;
    auto getNeighborDistance = [](int index) {
        if (index % 2 == 0) return 1.0;
        else return 1.414;
    };

    // 遍历，直到寻找到目标点
    while (!foundTarget && !wave.empty()) {
        std::queue<MapIndex> nextWave;
        // std::cout << "Current wave queue size: " << wave.size() << std::endl;

        // 遍历当前波队列
        while (!wave.empty()) {
            auto currentPoint = wave.front();
            wave.pop();
            auto& currentNode = map_value(search_map_, currentPoint.i, currentPoint.j);
            currentNode.state = SearchNode::CLOSE;
            
            // std::cout << "Wave size: " << wave.size() << std::endl;

            // 遍历近邻点    8 近邻
            for (size_t index = 0; index < 8; ++index) {
                const auto& shift = neighbors[index];
                auto neighborPoint = currentPoint;
                neighborPoint.i += shift.i;
                neighborPoint.j += shift.j;

                // 判断点是否在地图外
                if (!indices_in_map(neighborPoint.i, neighborPoint.j)) {
                    continue;
                }

                // 判断近邻点是否为障碍
                if (map_value(obstacle_map_.data, neighborPoint.i, neighborPoint.j) == kObstacleValue) {
                    map_value(search_map_, neighborPoint.i, neighborPoint.j).state = SearchNode::CLOSE;
                    continue;
                }

                auto& neighborNode = map_value(search_map_, neighborPoint.i, neighborPoint.j);

                // 判断近邻点是否为目标点
                if (neighborPoint.i == targetPoint.i && neighborPoint.j == targetPoint.j) {
                    foundTarget = true;
                    neighborNode.g = currentNode.g + getNeighborDistance(index);
                    neighborNode.pre_i = currentPoint.i;
                    neighborNode.pre_j = currentPoint.j;
                    neighborNode.state = SearchNode::CLOSE;
                    break;
                }

                double tempG = currentNode.g + getNeighborDistance(index);
                if (tempG > maxG) maxG = tempG;

                // 如果近邻点在波队列中，或被处理过，跳过   考虑对角线可能会使距离更近，进行一次比较处理
                if (neighborNode.state == SearchNode::CLOSE || neighborNode.state == SearchNode::OPEN) {
                    if (tempG < neighborNode.g) {
                        neighborNode.g = tempG;
                        neighborNode.pre_i = currentPoint.i;
                        neighborNode.pre_j = currentPoint.j;
                    }
                    continue;
                }

                // 更新搜索地图信息
                neighborNode.pre_i = currentPoint.i;
                neighborNode.pre_j = currentPoint.j;
                neighborNode.state = SearchNode::OPEN;
                neighborNode.g = tempG;
                // 将近邻点加入到下一次搜索中
                nextWave.push(neighborPoint);
            }
        }
        std::swap(wave, nextWave);
    }

    // 判断是否存在轨迹
    if (foundTarget) {
        ROS_INFO("Found path to target.");
    }
    else {
        ROS_INFO("Target cann't be reached.");
        return;
    }

    // 计算轨迹
    path_msg_.points.clear();
    std::vector<geometry_msgs::Point32, std::allocator<geometry_msgs::Point32>> tempPath;
    auto pathPoint = targetPoint;

    while (true) {
        geometry_msgs::Point32 tempPoint = point_position(pathPoint.i, pathPoint.j);
        tempPath.push_back(tempPoint);

        auto &node = map_value(search_map_, pathPoint.i, pathPoint.j);
        if (node.g == 0) break;
        pathPoint.i = node.pre_i;
        pathPoint.j = node.pre_j;
    }

    // 反向
    while (!tempPath.empty()) {
        auto &point = tempPath.back();
        path_msg_.points.push_back(point);
        tempPath.pop_back();
    }

}
} /* namespace simple_planner */
