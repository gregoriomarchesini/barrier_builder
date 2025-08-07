cmake_minimum_required(VERSION 3.8)
project(barrier_builder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(barrier_msg REQUIRED)

# Paths for non-ROS includes
set(MOSEK_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/lib/mosek/11.0/tools/platform/linux64x86/h)
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/barrier_builder)




# ==== Utils include ==============

add_library(barrier_utils src/utils.cpp)

target_include_directories(barrier_utils PUBLIC
  ${PROJECT_INCLUDE_DIR}
  ${MOSEK_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/lib
)

target_link_directories(barrier_utils PRIVATE
        ${PROJECT_SOURCE_DIR}/lib/mosek/11.0/tools/platform/linux64x86/bin
    )

ament_target_dependencies(barrier_utils
  rclcpp
  barrier_msg
)

target_link_libraries(barrier_utils
  mosek64
  mosekxx11_0
  tbb
  fusion64
)


# === barrier_server executable ===
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
        ${PROJECT_SOURCE_DIR}/lib/mosek/11.0/tools/platform/linux64x86/bin
    )

# Link against non-ROS external libraries
target_link_libraries(barrier_server
  barrier_utils
  mosek64
  mosekxx11_0
  tbb
  fusion64
)

install(TARGETS barrier_server DESTINATION lib/${PROJECT_NAME})


# === compute_barriers_client executable ===
add_executable(compute_barriers_client src/dummy_client.cpp)

ament_target_dependencies(compute_barriers_client
  rclcpp
  barrier_msg
)

install(TARGETS compute_barriers_client DESTINATION lib/${PROJECT_NAME})


# === Optional: Linting and tests ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
