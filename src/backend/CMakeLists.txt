cmake_minimum_required(VERSION 3.8)
project(backend)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
include(/home/lbw/Apps/libtorch/share/cmake/Torch/TorchConfig.cmake)

ament_auto_find_build_dependencies()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)
ament_auto_add_library(nnTrain SHARED DIRECTORY src/nnTrain)
ament_auto_add_executable(${PROJECT_NAME}_node DIRECTORY src)

target_link_libraries(nnTrain ${TORCH_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node nnTrain)

install(TARGETS
        ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)
ament_auto_package()
