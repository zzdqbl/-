cmake_minimum_required(VERSION 3.8)
project(ylc_nav)

# C++ 17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译器警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------- 依赖查找 ----------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 核心 ROS2 & Nav2 依赖（补全所有缺失的 tf 相关依赖）
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)          # 路径消息依赖
find_package(tf2 REQUIRED)               # 【新增】tf2 核心依赖
find_package(tf2_ros REQUIRED)           # tf2 ros 封装
find_package(tf2_geometry_msgs REQUIRED) # 【新增】tf2 几何消息转换
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)          # 【新增】基础消息依赖（兜底）
find_package(std_srvs REQUIRED)          # 【新增】基础服务依赖（兜底）

# 【关键】父类 RPP 控制器依赖
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)

# Python 依赖 (若不用可注释)
find_package(rclpy REQUIRED)

# ---------------- 编译逻辑 ----------------

# 1. 创建共享库（库名和 plugins.xml 保持一致）
add_library(ylc_nav_rpp SHARED
  src/ackermann_rpp_controller.cpp
)

add_library(ylc_nav_rpp_3_24 SHARED
  src/ackermann_rpp_controller_3_24.cpp
)

# 2. 头文件目录（确保 include 路径正确）
# 重点：确认你的头文件在 include/ylc_nav_rpp/ 目录下（和代码里的 #include 对应）
target_include_directories(ylc_nav_rpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${tf2_INCLUDE_DIRS}          # 【新增】显式添加 tf2 头文件路径
  ${tf2_ros_INCLUDE_DIRS}      # 【新增】显式添加 tf2_ros 头文件路径
)

target_include_directories(ylc_nav_rpp_3_24 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${tf2_INCLUDE_DIRS}          # 【新增】显式添加 tf2 头文件路径
  ${tf2_ros_INCLUDE_DIRS}      # 【新增】显式添加 tf2_ros 头文件路径
)

# 3. 链接依赖（补全所有 tf 相关依赖）
ament_target_dependencies(ylc_nav_rpp
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  tf2                          # 【新增】
  tf2_ros                      # 【新增】
  tf2_geometry_msgs            # 【新增】
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  std_msgs                     # 【新增】
  std_srvs                     # 【新增】
  nav2_regulated_pure_pursuit_controller  # 父类依赖
)

ament_target_dependencies(ylc_nav_rpp_3_24
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  std_msgs
  std_srvs
  nav2_regulated_pure_pursuit_controller
)

# ========== 新增关键步骤2：安装 plugins.xml 到 share 目录 ==========
install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)
# 4. 导出插件描述文件（确保 plugins.xml 路径正确）
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

# 5. 安装目标
install(TARGETS ylc_nav_rpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ylc_nav_rpp_3_24
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 6. 安装头文件（确保头文件被安装到系统路径）
install(DIRECTORY include/
  DESTINATION include
)

# ---------------- Python & 资源 ----------------!!!!!!!!!!!!!!!!!!!!!!!!
# 若无 Python 代码，建议注释下面这行（避免找不到 Python 包报错）
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/ylc_main.py
  ${PROJECT_NAME}/find_goal.py
  ${PROJECT_NAME}/start_button_node.py
  DESTINATION lib/${PROJECT_NAME}
)
# 安装配置文件 (launch, config, maps, behavior_trees)
install(DIRECTORY
  launch
  config
  maps
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# ---------------- 测试 & 导出 ----------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出依赖供其他包使用（补全 tf 相关）
ament_export_include_directories(include)
ament_export_libraries(ylc_nav_rpp)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  tf2                          # 【新增】
  tf2_ros                      # 【新增】
  tf2_geometry_msgs            # 【新增】
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  std_msgs                     # 【新增】
  std_srvs                     # 【新增】
  nav2_regulated_pure_pursuit_controller
)

ament_package()