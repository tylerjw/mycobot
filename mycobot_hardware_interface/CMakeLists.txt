cmake_minimum_required(VERSION 3.16.3)
project(mycobot_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(fp REQUIRED)
find_package(range-v3 REQUIRED)
find_package(mycobot REQUIRED)
find_package(serial REQUIRED)

add_library(${PROJECT_NAME}
  src/hardware_interface.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)  # C++20
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    fmt::fmt
    fp::fp
    range-v3::range-v3
    mycobot::mycobot
    serial::serial
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
