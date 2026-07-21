cmake_minimum_required(VERSION 3.0.2)
# 包名必须和package.xml一致
project(multi_uav_strike)

# 设置编译选项（C++11兼容ROS1）
add_compile_options(-std=c++11)

# 查找依赖包：必须包含package.xml里声明的核心依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  tf2_ros
  visualization_msgs
  eigen_conversions
  message_generation
  serial
  roslib
  mavros_msgs
)

# 要生成的消息列表(本项目所有消息归 multi_uav_strike 拥有)
# 注意:ClusterTarget 必须在 ClusterState 之前(后者引用前者)
add_message_files(
  FILES
  WorkMode.msg
  Skill.msg
  TaskFlow.msg
  MissionState.msg
  AttackCmd.msg
  DetectTarget.msg
  DetectTargets.msg
  YoloDetection.msg
  YoloDetections.msg
  SimTarget.msg
  SimTargets.msg
  TrackingState.msg
  WaypointStatus.msg
  InterUavStatus.msg
  AvoidanceCmd.msg
  LidarMeasurement.msg
  UavGatherStatus.msg
  ClusterEvent.msg
  ClusterTarget.msg
  ClusterState.msg
  TargetFilterDebug.msg     # 2026-07-16: 位置/速度滤波前后对比(debug)
)

# 消息生成的依赖项
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

# 声明catkin包（供其他包依赖）
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs std_srvs geometry_msgs sensor_msgs nav_msgs tf tf2_ros visualization_msgs mavros_msgs serial message_runtime
)

# 包含ROS头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ========== 制导策略库 ==========
add_library(guidance_strategies
  src/guidance_strategies.cpp
)
target_link_libraries(guidance_strategies
  ${catkin_LIBRARIES}
)

# ========== 逐个编译所有节点 ==========
# 1. 目标运动模拟节点
add_executable(target_motion_simulator_node
  src/target_motion_simulator_node.cpp
)
target_link_libraries(target_motion_simulator_node
  ${catkin_LIBRARIES}
)
add_dependencies(target_motion_simulator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 2. 云台模拟节点
add_executable(gimbal_simulator_node
  src/gimbal_simulator_node.cpp
)
target_link_libraries(gimbal_simulator_node
  ${catkin_LIBRARIES}
)
add_dependencies(gimbal_simulator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 3. 目标估计节点
add_executable(target_estimator_node
  src/target_estimator_node.cpp
)
target_link_libraries(target_estimator_node
  ${catkin_LIBRARIES}
)
add_dependencies(target_estimator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 4. 制导控制节点
add_executable(guidance_control_node
  src/guidance_control_node.cpp
)
target_link_libraries(guidance_control_node
  guidance_strategies
  ${catkin_LIBRARIES}
)
add_dependencies(guidance_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ========== P0: 核心框架 ==========
# 5. 通信节点（统一机间+地面站通信）
add_executable(comm_node
  src/comm_node.cpp
)
target_link_libraries(comm_node
  ${catkin_LIBRARIES}
)
add_dependencies(comm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 6. 航点执行器节点
add_executable(waypoint_executor_node
  src/waypoint_executor_node.cpp
)
target_link_libraries(waypoint_executor_node
  ${catkin_LIBRARIES}
)
add_dependencies(waypoint_executor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 7. 任务管理器节点（核心状态机）
add_executable(mission_manager_node
  src/mission_manager_node.cpp
)
target_link_libraries(mission_manager_node
  ${catkin_LIBRARIES}
)
add_dependencies(mission_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 8. 航点分发器节点（多机航点分配）
add_executable(waypoint_distributor_node
  src/waypoint_distributor_node.cpp
)
target_link_libraries(waypoint_distributor_node
  ${catkin_LIBRARIES}
)
add_dependencies(waypoint_distributor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ========== Phase 1+: 新增节点 ==========
# 注: detection_simulator_node 已合并到 gimbal_simulator_node (2026-07-07)
# 9. 机间避障决策节点(优先级排序 + 压航线兼容)
add_executable(uav_avoidance_node
  src/uav_avoidance_node.cpp
)
target_link_libraries(uav_avoidance_node
  ${catkin_LIBRARIES}
)
add_dependencies(uav_avoidance_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 10. 电源信息板通信节点
add_executable(power_interact_node 
  src/power_interact_node.cpp
)
target_link_libraries(power_interact_node 
  ${catkin_LIBRARIES}
)
add_dependencies(power_interact_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ========== 安装可执行文件和脚本 ==========
install(TARGETS
  target_motion_simulator_node
  gimbal_simulator_node
  target_estimator_node
  guidance_control_node
  comm_node
  waypoint_executor_node
  mission_manager_node
  waypoint_distributor_node
  uav_avoidance_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 Python 脚本
install(PROGRAMS
  scripts/gs_simulator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
