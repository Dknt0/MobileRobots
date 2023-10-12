#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

//глобальная переменная - публикатор сообщения карты
ros::Publisher mapPub;
ros::Publisher ProbMapPub;

//глоабльный указатель на tfListener, который будет проинициализирован в main
tf::TransformListener *tfListener;

//имя для СК карты
std::string map_frame; // 地图坐标系名称

//разрешение карты
double map_resolution = 0.1;
//размер карты в клетках
int map_width = 2000;
int map_height = 2000;

//создаем сообщение карты
nav_msgs::OccupancyGrid map_msg;
// 几率图
std::vector<double> oddMap;
// 概率图
nav_msgs::OccupancyGrid ProbMap_msg;

/**
 * @brief 初始化栅格地图
*/
void prepareMapMessage(nav_msgs::OccupancyGrid& map_msg)
{
    map_msg.header.frame_id = map_frame;
    
    map_msg.info.height = map_height;
    map_msg.info.width = map_width;
    map_msg.info.resolution = map_resolution;
    // фиксированное положение начала карты (левый нижний угол)
    map_msg.info.origin.position.x = - map_width * map_resolution /2.0;
    map_msg.info.origin.position.y = - 10.0;
    // изменяем размер вектора, который является хранилищем данных карты, и заполняем его значением (-1) - неизвестное значение
    map_msg.data.resize(map_height*map_width, -1); // -1 是未探索

    ProbMap_msg.header.frame_id = map_frame;
    ProbMap_msg.info.height = map_height;
    ProbMap_msg.info.width = map_width;
    ProbMap_msg.info.resolution = map_resolution;
    ProbMap_msg.info.origin.position.x = - map_width * map_resolution /2.0;
    ProbMap_msg.info.origin.position.y = - 10.0;
    ProbMap_msg.data.resize(map_height*map_width, 50); // 初值无意义

    oddMap.resize(map_height*map_width, 0.6020599913279624); // log10(0.8 / 0.2)
}

/**
 * @brief 确定坐标变换
 * 
 * @param scanTransform 坐标变换
 * @param stamp 时间戳
 * @param laser_frame 激光雷达坐标系
*/
bool determineScanTransform(tf::StampedTransform& scanTransform,
                            const ros::Time& stamp,
                            const std::string& laser_frame)
{
    try
    {
        if ( ! tfListener->waitForTransform(map_frame,
                                            laser_frame,
                                            stamp,
                                            ros::Duration(0.1)) )
        {
          ROS_WARN_STREAM("no transform to scan "<<laser_frame);
          return false;
        }
        tfListener->lookupTransform(map_frame,
                                    laser_frame,
                                    stamp,
                                    scanTransform);

    }
    catch (tf::TransformException& e)
    {
        ROS_ERROR_STREAM("got tf exception "<<e.what());
        return false;
    }
    return true;
}


/**
 * Функция, которая будет вызвана
 * при получении данных от лазерного дальномера
 */
/**
 * @brief 激光雷达回调函数
 * 
 * 扫描范围 -1.57 ~ 1.57
 * 激光条数 180
 * 
 * Dknt 2023.10.12
*/
void laserCallback(const sensor_msgs::LaserScan& scan)
{
    tf::StampedTransform scanTransform; // 激光相对于地图 TF
    const std::string& laser_frame = scan.header.frame_id;
    const ros::Time& laser_stamp = scan.header.stamp;
    if (!determineScanTransform(scanTransform, laser_stamp, laser_frame)) {
        return;
    }

    map_msg.header.stamp = laser_stamp;

    //положение центра дальномера в СК дальномера
    tf::Vector3 zero_pose(0, 0, 0);
    //положение дальномера в СК карты
    // 地图坐标系中激光雷达位置
    tf::Vector3 scan_pose = scanTransform(zero_pose);
    // ROS_INFO_STREAM("scan pose "<<scan_pose.x()<<" "<<scan_pose.y());

    //индексы карты, соответствующие положению центра лазера
    // 雷达在地图中的位置
    int y = (scan_pose.y() - map_msg.info.origin.position.y ) / map_resolution;
    int x = (scan_pose.x() - map_msg.info.origin.position.x ) / map_resolution;
    // ROS_INFO_STREAM("publish map " << x << " " << y);

    // 机器人经过的位置无障碍  没有考虑机器人半径
    map_msg.data[ y* map_width + x] = 0;

    /**
     * Direct Transform Mapping
    */
    auto directTF = [&]() {
        for (size_t index = 0; index < scan.ranges.size(); ++index) {
            double theta = scanTransform.getRotation().getAngle(); // robot orientation
            tf::Vector3 axis = scanTransform.getRotation().getAxis();
            if (axis[2] < 0) theta = -theta;
            double beamDirection = index * scan.angle_increment - M_PI_2 + theta;
            // std::cout << "robot ori: " << theta << " beamDirection: " << beamDirection << std::endl;
            
            double distance = scan.ranges[index];
            double endX = (distance * cos(beamDirection)) / map_resolution + x;
            double endY = (distance * sin(beamDirection)) / map_resolution + y;
            // std::cout << "distance: " << distance << " endX: " << endX << " endY: " << endY << std::endl;

            // 占据
            if (distance < scan.range_max) {
                map_msg.data[(static_cast<int>(endY) * map_width + static_cast<int>(endX))] = 100;
                map_msg.data[(static_cast<int>(endY + 1) * map_width + static_cast<int>(endX + 1))] = 100;
                map_msg.data[(static_cast<int>(endY + 1) * map_width + static_cast<int>(endX - 1))] = 100;
                map_msg.data[(static_cast<int>(endY - 1) * map_width + static_cast<int>(endX + 1))] = 100;
                map_msg.data[(static_cast<int>(endY - 1) * map_width + static_cast<int>(endX - 1))] = 100;
            }
            double currentEnd = std::min<double>(distance, scan.range_max);
            // 空闲
            for (double distanceTemp = 0.0; distanceTemp < currentEnd; distanceTemp += map_resolution) {
                double tempX = (distanceTemp * cos(beamDirection)) / map_resolution + x;
                double tempY = (distanceTemp * sin(beamDirection)) / map_resolution + y;
                map_msg.data[(static_cast<int>(tempY) * map_width + static_cast<int>(tempX))] = 0;
            }
        }
    };

    /**
     * Occupancy Grid Mapping (Binary Bayes Filter)
    */
    auto bayesFilter = [&]() {
        const double locc = 1.6787536009528289; //
        const double locc_n = 0.8020599913279624; //
        const double locc_nn = 0.1020599913279624; //
        const double locc_nnn = -0.80020599913279624; //
        const double lfree = -1.2679767852945944; //

        for (size_t index = 0; index < scan.ranges.size(); ++index) {
            double theta = scanTransform.getRotation().getAngle(); // robot orientation
            tf::Vector3 axis = scanTransform.getRotation().getAxis();
            if (axis[2] < 0) theta = -theta;
            double beamDirection = index * scan.angle_increment - M_PI_2 + theta;
            // std::cout << "robot ori: " << theta << " beamDirection: " << beamDirection << std::endl;
            
            double distance = scan.ranges[index];
            double endX = (distance * cos(beamDirection)) / map_resolution + x;
            double endY = (distance * sin(beamDirection)) / map_resolution + y;
            // std::cout << "distance: " << distance << " endX: " << endX << " endY: " << endY << std::endl;

            // 占据
            if (distance < scan.range_max) {
                oddMap[(static_cast<int>(endY) * map_width + static_cast<int>(endX))] += locc;
                {
                    double region = map_resolution * 1;
                    double tempX = ((distance - region - 0.3) * cos(beamDirection)) / map_resolution + x;
                    double tempY = ((distance - region - 0.3) * sin(beamDirection)) / map_resolution + y;
                    oddMap[(static_cast<int>(round(tempY)) * map_width + static_cast<int>(round(tempX)))] += locc_n;
                    tempX = ((distance + region + 0.3) * cos(beamDirection)) / map_resolution + x;
                    tempY = ((distance + region + 0.3) * sin(beamDirection)) / map_resolution + y;
                    oddMap[(static_cast<int>(round(tempY)) * map_width + static_cast<int>(round(tempX)))] += locc_n;
                }
                {
                    double region = map_resolution * 2;
                    double tempX = ((distance - region - 0.3) * cos(beamDirection)) / map_resolution + x;
                    double tempY = ((distance - region - 0.3) * sin(beamDirection)) / map_resolution + y;
                    oddMap[(static_cast<int>(round(tempY)) * map_width + static_cast<int>(round(tempX)))] += locc_nn;
                    tempX = ((distance + region + 0.3) * cos(beamDirection)) / map_resolution + x;
                    tempY = ((distance + region + 0.3) * sin(beamDirection)) / map_resolution + y;
                    oddMap[(static_cast<int>(round(tempY)) * map_width + static_cast<int>(round(tempX)))] += locc_nn;
                }
                {
                    double region = map_resolution * 3;
                    double tempX = ((distance - region - 0.3) * cos(beamDirection)) / map_resolution + x;
                    double tempY = ((distance - region - 0.3) * sin(beamDirection)) / map_resolution + y;
                    oddMap[(static_cast<int>(round(tempY)) * map_width + static_cast<int>(round(tempX)))] += locc_nnn;
                    tempX = ((distance + region + 0.3) * cos(beamDirection)) / map_resolution + x;
                    tempY = ((distance + region + 0.3) * sin(beamDirection)) / map_resolution + y;
                    oddMap[(static_cast<int>(round(tempY)) * map_width + static_cast<int>(round(tempX)))] += locc_nnn;
                }
            }
            double currentEnd = std::min<double>(distance, scan.range_max) - map_resolution;
            // 空闲
            for (double distanceTemp = 0.0; distanceTemp < currentEnd; distanceTemp += map_resolution) {
                double tempX = (distanceTemp * cos(beamDirection)) / map_resolution + x;
                double tempY = (distanceTemp * sin(beamDirection)) / map_resolution + y;
                oddMap[(static_cast<int>(round(tempY)) * map_width + static_cast<int>(round(tempX)))] += lfree;
            }
            
        }

        // 从几率图恢复概率图
        for (size_t index = 0; index < oddMap.size(); ++index) {
            ProbMap_msg.data[index] = static_cast<int8_t>(99 * (1. - 1. / (1. + exp(oddMap[index])))) + 1;
            if (ProbMap_msg.data[index] > 80) map_msg.data[index] = 99;
            else if (ProbMap_msg.data[index] < 5) map_msg.data[index] = 1;
            else map_msg.data[index] = -1;
        }

        ProbMapPub.publish(ProbMap_msg);
    };

    // directTF();
    bayesFilter();

    // публикуем сообщение с построенной картой
    mapPub.publish(map_msg);
}


int main(int argc, char **argv)
{
  /**
   * Инициализация системы сообщений ros
   * Регистрация node с определенным именем (третий аргумент функции)
   * Эта функция должна быть вызвана в первую очередь
   */
  ros::init(argc, argv, "control_node");

  /**
   * NodeHandle  - объект через который осуществляется взаимодействие с ROS:
   * передача сообщений
   * регистрация коллбаков (функций обработки сообщений)
   */
  ros::NodeHandle node("~");

  //читаем параметры
  map_frame = node.param<std::string>("map_frame", "odom"); // 地图坐标系名称
  map_resolution = node.param("map_resolution", map_resolution); // 地图分辨率
  map_width = node.param("map_width", map_width); // 地图宽度
  map_height = node.param("map_height", map_height); // 地图高度

  //создание объекта tf Listener
  tfListener = new tf::TransformListener; // TF 收听者

  // Подписываемся на данные дальномера
  ros::Subscriber laser_sub = node.subscribe("/scan", 100, laserCallback); // 激光雷达收听者

  //объявляем публикацию сообщений карты
  //Используем глобальную переменную, так как она понядобится нам внутр функции - обработчика данных лазера

  mapPub = node.advertise<nav_msgs::OccupancyGrid>("/simple_map", 10); // 地图发布
  ProbMapPub = node.advertise<nav_msgs::OccupancyGrid>("/probability_map", 10);

  //заполняем информацию о карте - готовим сообщение
  prepareMapMessage(map_msg);
   /**
   * ros::spin() функция внутри которой происходит вся работа по приему сообщений
   * и вызову соответствующих обработчиков . Вся обработка происходит из основного потока
   * (того, который вызвал ros::spin(), то есть основного в данном случае)
   * Функция будет завершена, когда подьзователь прервет выполнение процесса с Ctrl-C
   *
   */
  ros::spin();

  return 0;
}
