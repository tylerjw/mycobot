cmake_minimum_required(VERSION 3.16.3)
project(mycobot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cppystruct REQUIRED)
find_package(fmt REQUIRED)
find_package(fp REQUIRED)
find_package(range-v3 REQUIRED)
find_package(serial REQUIRED)

add_library(${PROJECT_NAME}
  src/bytearray.cpp
  src/command.cpp
  src/detect.cpp
  src/mycobot.cpp
  src/process_received.cpp
  src/serialize.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)  # C++20
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    cppystruct::cppystruct
    fmt::fmt
    fp::fp
    range-v3::range-v3
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

if(BUILD_TESTING)
  find_package(catch2 REQUIRED)
  find_package(ament_cmake_catch2 REQUIRED)
  file(GLOB_RECURSE unit_test_srcs "test/*.cpp")
  ament_add_catch2(test_mycobot test/main.cpp ${unit_test_srcs} TIMEOUT 300)
  target_link_libraries(test_mycobot PRIVATE catch2::catch2 mycobot)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
