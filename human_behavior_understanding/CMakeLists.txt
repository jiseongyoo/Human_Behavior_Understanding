cmake_minimum_required(VERSION 3.5)
project(human_behavior_understanding)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(hbu_interface REQUIRED)

# This package must include directories "include"
include_directories(include)

# add components libraries
add_library(histogram         SHARED src/histogram.cpp)
add_library(histogram_client  SHARED src/histogram_client.cpp)
add_library(RAD               SHARED src/RAD.cpp)
add_library(HJPD              SHARED src/HJPD.cpp)
add_library(HOD               SHARED src/HOD.cpp)
add_library(svm               SHARED src/svm.cpp)
add_library(svm_scale         SHARED src/svm_scale.cpp)
add_library(svm_train         SHARED src/svm_train.cpp)
add_library(svm_predict       SHARED src/svm_predict.cpp)
add_library(SVM_server        SHARED src/SVM_server.cpp)
add_library(SVM_client        SHARED src/SVM_client.cpp)


# add components target dependencies
ament_target_dependencies(histogram rclcpp)
ament_target_dependencies(histogram_client rclcpp hbu_interface)
ament_target_dependencies(RAD rclcpp rclcpp_components hbu_interface)
ament_target_dependencies(HJPD rclcpp rclcpp_components hbu_interface)
ament_target_dependencies(HOD rclcpp rclcpp_components hbu_interface)
ament_target_dependencies(svm rclcpp)
ament_target_dependencies(svm_scale rclcpp)
ament_target_dependencies(svm_train rclcpp)
ament_target_dependencies(svm_predict rclcpp)
ament_target_dependencies(SVM_server rclcpp rclcpp_components hbu_interface)
ament_target_dependencies(SVM_client rclcpp hbu_interface)


# add target link libraries
target_link_libraries(RAD histogram)
target_link_libraries(HJPD histogram)
target_link_libraries(HOD histogram)
target_link_libraries(svm_scale svm)
target_link_libraries(svm_train svm)
target_link_libraries(svm_predict svm)
target_link_libraries(SVM_server svm svm_scale svm_train svm_predict)


# register component node
rclcpp_components_register_nodes(RAD "RAD")
rclcpp_components_register_nodes(HJPD "HJPD")
rclcpp_components_register_nodes(HOD "HOD")
rclcpp_components_register_nodes(SVM_server "SVM_server")


# register libraries to package
install(TARGETS
  histogram
  histogram_client
  RAD
  HJPD
  HOD
  svm
  svm_scale
  svm_train
  svm_predict
  SVM_server
  SVM_client
  LIBRARY DESTINATION lib
)

# add executable
add_executable(SVM_container src/SVM_container.cpp)
target_link_libraries(SVM_container RAD HJPD HOD SVM_server)
ament_target_dependencies(SVM_container rclcpp)

# add executable
add_executable(SVM_controller src/SVM_controller.cpp)
target_link_libraries(SVM_controller histogram_client SVM_client)
ament_target_dependencies(SVM_controller rclcpp)

# register nodes to package
install(TARGETS
  SVM_container
  SVM_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
