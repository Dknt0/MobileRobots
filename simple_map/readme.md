## Практическое задание №5  
### Построение карты по данным дальномера  
В этой задаче мы будем строить карту для мобильного робота, оснащенного лазерным дальномером, одним из алгоритмов, изложенных на лекции (Рекурсивный фильтр Байеса или модель подсчетов)
При решении этой задачи считаем, что положение МР известно. Мы можем получить его в топике с ground_truth или с помощью tf 

(команды выполняются при нахождении в директории рабочего пространства)  
0. Скачиваем проект:
```bash
git clone <адрес репозитария> src
```
или обновляем исходные файлы (если проект уже скачан):
```bash
cd src  
git pull  
cd ..  
```
В папке `src` должен находиться проект `simple_map`. Его будем использовать за основу.
1. Сборка проекта (из рабочей директории):
```bash
catkin_make
```
2. Не забываем в каждом терминале, где планируем запускать бинарные файлы проекта, инициализировать рабочее пространство:
```bash
source devel/setup.bash
```
3. Запуск модели робота и управляющего кода:  
```bash
roslaunch simple_map map.launch
```
Должно открыться окно симулятора Stage с роботом в мире с препятствиями, а также окно программы для визуализации данных rviz. В rviz должна отображаться серая карта (occupancy grid), публикуемая модулем simple_map. На карте белой точкой остается след центральной точки робота. Робот движется под управлением simple_controller с небольшой скоростью. Также в rviz отображаются текущие данные лидара - красные точки.

4. Модуль simple_map реализован в исходном файле [simple_map.cpp](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/simple_map/src/simple_map.cpp)  Он получает данные дальномера. Определяет с помощью tf преобразование СК от СК лидара (laser_frame) до неподвижной СК(odom), в которой двигается робот. Готовит сообщение с картой, Карта строится в неподвижных координатах, координаты левого нижнего угла выбраны таким образом, чтобы вся рабочая зона помещалась на карту. При движении робота координаты левого нижнего угла (origin) не меняются.

## Задача
1. Реализовать построение карты по данным лидара, таким образом, чтобы на публикуемой карте ячейки, соответствующие препятствиям, отображались как черные (100) пиксели, а свободные области белыми (0). На этом этапе решения адачи не требется использовать алгоритмы с лекции - просто, используя имеющийся трансформ, в ячейки карты, соответствующие препятствиям (концам лучей) записать значение 100 препятствие, а все ячейки, которые пересекаются лучами пометить как свободную зону (значение 0). 
Для вычисления индексов ячеек, через которые проходит луч можно использовать трассировку луча: пробежать по лучу с маленьким шагом(меньшим размера ячейки) и вычислить индексы ячеек в которые попадут промежуточные точки. В результате в rviz мы должны увидеть карту рабочей зоны.
2. Реализовать построение вероятностной карты с помощью рекурсивного алгоритма Байеса(подсчет логарифмического соотношения шансов) или с помощью модели подсчетов (из лекции про mapping). В результате в rviz мы должны увидеть вероятностную карту рабочей зоны.
3. Включить шум дальномера. Для этого в файле [src/cart_launch/stage_worlds/sick20.inc](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/cart_launch/stage_worlds/sick20.inc) раскомментировать строчку [#noise [0.1 0 0]](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/cart_launch/stage_worlds/sick20.inc#L11]). Настроить параметры вероятностного алгоритма построения карты, чтобы получить оптимальный результат (карту).