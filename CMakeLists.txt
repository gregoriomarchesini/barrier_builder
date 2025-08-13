cmake_minimum_required(VERSION 3.8)
project(barrier_builder)

# ========================
# Compiler settings
# ========================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================
# Dependencies
# ========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(barrier_msg REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED) 

# ========================
# Paths
# ========================
set(MOSEK_INCLUDE_DIR
  ${PROJECT_SOURCE_DIR}/lib/mosek/11.0/tools/platform/linux64x86/h
)
set(MOSEK_LIB_DIR
  ${PROJECT_SOURCE_DIR}/lib/mosek/11.0/tools/platform/linux64x86/bin
)
set(PROJECT_INCLUDE_DIR
  ${PROJECT_SOURCE_DIR}/include/barrier_builder
)

# ========================
# barrier_utils library
# ========================
add_library(barrier_utils src/utils.cpp)

target_include_directories(barrier_utils PUBLIC
  ${PROJECT_INCLUDE_DIR}
  ${MOSEK_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/lib
)

target_link_directories(barrier_utils PRIVATE
  ${MOSEK_LIB_DIR}
)

ament_target_dependencies(barrier_utils
  rclcpp
  barrier_msg
  nav_msgs
)

target_link_libraries(barrier_utils
  mosek64
  mosekxx11_0
  tbb
  fusion64
)

# ========================
# barrier_server executable
# ========================
add_executable(barrier_server src/create_barrier_service.cpp)

target_include_directories(barrier_server PRIVATE
  ${PROJECT_INCLUDE_DIR}
  ${MOSEK_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/lib
)

ament_target_dependencies(barrier_server
  rclcpp
  barrier_msg
)

target_link_directories(barrier_server PRIVATE
  ${MOSEK_LIB_DIR}
)

target_link_libraries(barrier_server
  barrier_utils
  mosek64
  mosekxx11_0
  tbb
  fusion64
)

# ========================
# compute_barriers_client executable
# ========================
add_executable(compute_barriers_client src/dummy_client.cpp)

ament_target_dependencies(compute_barriers_client
  rclcpp
  barrier_msg
)

target_link_directories(compute_barriers_client PRIVATE
  ${MOSEK_LIB_DIR}
)

target_link_libraries(compute_barriers_client
  mosek64
  mosekxx11_0
  tbb
  fusion64
)

# ========================
# Installation rules
# ========================

# Install targets
install(TARGETS
  barrier_utils
  barrier_server
  compute_barriers_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install Mosek shared libraries into package lib folder
install(
  DIRECTORY ${MOSEK_LIB_DIR}/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

# ========================
# Optional: Linting and tests
# ========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ========================
# Finalize package
# ========================
ament_package()
