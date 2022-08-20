# ant_colony_path_planner
基于ROS的全球路径规划器，使用蚁群优化
# 基本说明：
将“ant_colony_path_planner”包放在工作区的源“src”文件夹中。
使用“catkin_make”构建，或者使用“catkin_make --pkg aco_ros”来单独构建此包。
创建自己的启动文件以启动机器人。
请确保在运行导航包之前实现以下已经完成以下功能
1.预构建地图
<node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />
2.将该插件设置在与机器人一起使用的自定义move_base启动文件中。
<param name="base_global_planner" value="Aco_planner/AcoPlanner" />
