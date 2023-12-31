# MobileRobots

See https://github.com/AndreyMinin/MobileRobots

---

# Макет системы управления мобильным роботом

Является ROS пакетом, предназначенным для разработки системы управления мобильным роботом.

Использует модель робота в симуляторе GAZEBO (www.gazebosim.org)
либо модель робота в симуляторе Stage

## Включает в себя следующие пакеты

**cart_launch** - проект с описанием сцены для Gazebo с картом и объектами сцены
* cart_launch/stage_worlds - описание сцены для stage
* cart_launch/launch/cart_launch - файл запуска модели в среде gazebo
* cart_launch/cart_stage.launch - файл для запуска модели в среде stage

> 场景，主要用 Gazebo

**polaris_ranger_ev** - проект, содержащий модель робота с датчиками для Gazebo
* model/simple_cart.sdf - робот без датчиков,
* model/scan_cart.sdf - робот с плоским дальномером и камерой

> 带传感器的机器人模型，cart_launch/launch/cart_launch 会从导入这里的模型

**vehicle_ros_plugin** - плагин для Gazebo, позволяющий управлять моделью из ROS.
* robot/velocity - топик для задания скорости движения
* robot/steering - топик для задания кривизны траектории (поворот руля)
* robot/odom   -   топик с текущими данными одометрии от робота

> 控制器插件，这个控制器是作者自己写的
> 实现了 ROS 和 GZ 的消息互传

**stage_controller** - модуль управления моделью робота в stage, преобразует команды управления поворотом руля (топик /steering), команды управления акселератором в команды управления моделью в stage (cmd_vel) 

> stage 中的控制器，用不到

**simple_controller** - простейший ROS контроллер, управляющий движением карта по овальной траектории, с помощью пид регулятора, в качестве ошибки используется расстояние от траектории, вычисляемое на основе абсолютного положения робота. Модуль управляет поворотом руля карта

> 轨迹跟踪

**odo2tf** - модуль преобразования данных о реальном положении модели робота (Odometry) в сообщения tf для формирования преобразования систем координат world->base_link
Пописывается на топики:
* /odo - топик с данными одометрии ([nav_msgs/Odometry])
Публикует преобразование tf

> 里程计转 tf

**laser2d_map** - модуль построения карты препятствий по данным сканирующего двумерного дальномера
Подписывается на топик
* /laser_scan - данные сканирующего дальномера (sensor_msgs/LaserScan)
* /tf - преобразование СК от лазера до world
Публикует
* laser_map - карта препятствий в системе координат world

> 2d 建图

Установка

Необходимо, чтобы были установлены ROS (Тестировалось на [melodic](http://wiki.ros.org/melodic/Installation)) и Gazebo (для управления моделью в Gazebo) или Stage (для управления моделью в среде stage), а также некоторые пакеты ROS (про которые будет указано внутри заданий)

Возможна разработка из [докер образа](https://github.com/AndreyMinin/MobileRobots#%D0%B8%D1%81%D0%BF%D0%BE%D0%BB%D1%8C%D0%B7%D0%BE%D0%B2%D0%B0%D0%BD%D0%B8%D0%B5-docker), где всё необходимое уже установлено.

1. Нужно склонировать репозиторий:
```bash
git clone https://github.com/AndreyMinin/MobileRobots.git <папка для размещения проекта>
```

1. Перейти в рабочую папку
```bash
cd ros_ws
```

1. собрать проект
```bash
catkin_make
```
или воспользоваться скриптом **build.sh** (из папки src)
```bash
./build.sh
```
Если сборка прошла без ошибок можно переходить к запуску

Для первого запуска может потребоваться выход в интернет- чтобы закачать некторые модели с репозитария Gazebo

4. запуск
  команды выполняются из директории рабочей области (ros_ws)
инициализация рабочей области ROS
```bash
source devel/setup.bash
```

5. запуск модели в среде Gazebo и контроллера 
```bash
roslaunch simple_controller controller.launch
```
модель стартует, затем запускаетя интерфейс gzclient, rviz и rqt
 Либо запуск модели в среде stage(предпочтительный вариант) и контроллера 
```bash
roslaunch simple_controller controller_stage.launch
```
модель стартует, затем запускаетя интерфейс  rqt

В rqt в плагине publish можно отправить сообщение с заданной скоростью в топик /robot/velocity

  Должно появиться окно симулятора (GAZEBO либо  stage), в котором робот ездит в некоторой сцене, отрабатывая заданную траекторию
  г) управление скоростью можно осуществлять с помощью rqt (publish_message) публикуя сообщение со значением скорости в топик /robot/velocity

  Также можно осуществить запуск с помощью скрипта start.sh, находящегося в папке src
```bash
./start.sh
```
	Остановить проект можно с помощью ctrl-C в консоли

## Практические задания курса

1. [Контроллер управления движением по траектории](https://github.com/AndreyMinin/MobileRobots/tree/master/mr_ws/src/simple_controller)

> 实现了轨迹跟踪，横向控制

2. [Контроллер управления скоростью](https://github.com/AndreyMinin/MobileRobots/tree/master/mr_ws/src/velocity_controller)

> 实现了速度伺服，纵向控制

3. [MPC контроллер](https://github.com/AndreyMinin/MobileRobots/tree/master/mr_ws/src/mpc_controller)

> MPC 控制，用多多项式拟合当前路径，输入 MPC 求解器计算最优控制，控制量为加速度和转向角速度

4. [Планирование траектории](https://github.com/AndreyMinin/MobileRobots/tree/master/mr_ws/src/simple_planner)

> 轨迹规划

5. [Построение карты](https://github.com/AndreyMinin/MobileRobots/tree/master/mr_ws/src/simple_map)

> 建图

6. [Локализация по дальномеру](https://github.com/AndreyMinin/MobileRobots/tree/master/mr_ws/src/feature_matcher)

> 基于特征的定位

7. [EKF SLAM](https://github.com/AndreyMinin/MobileRobots/tree/master/mr_ws/src/barrel_slam)

> 扩展卡尔曼滤波器 SLAM

