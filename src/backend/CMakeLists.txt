cmake_minimum_required(VERSION 3.8)
project(backend)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(CUDA REQUIRED)
include(/home/lbw/Apps/libtorch/share/cmake/Torch/TorchConfig.cmake)

ament_auto_find_build_dependencies()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${TORCH_INCLUDE_DIRS})

#ament_auto_add_library(Logger SHARED
#        src/logger/logger.cpp)
#
#ament_auto_add_library(Trainer SHARED
#        src/logger/logger.cpp
#        src/nnTrain/nn_MP.cpp
#        src/nnTrain/train_base.cpp
#        src/nnTrain/MPTrainer.cpp)
#target_link_libraries(Trainer ${CUDA_LIBRARIES} ${TORCH_LIBRARIES})
#
#ament_auto_add_library(Memory SHARED
#        src/logger/logger.cpp
#        src/nnTrain/memory.cpp)
#target_link_libraries(Memory ${TORCH_LIBRARIES})

file(GLOB_RECURSE SRCS src/*.cpp)
ament_auto_add_executable(${PROJECT_NAME}_node
        ${SRCS}
)

target_link_libraries(${PROJECT_NAME}_node  ${CUDA_LIBRARIES} ${TORCH_LIBRARIES})
#target_link_libraries(${PROJECT_NAME}_node Trainer Memory)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

install(TARGETS
        ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)
ament_auto_package()
