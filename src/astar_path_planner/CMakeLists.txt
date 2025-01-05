cmake_minimum_required(VERSION 3.0.2)
project(astar_path_planner)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        visualization_msgs
        geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

add_executable(obstacle_generator src/obstacle_generator.cpp)
target_link_libraries(obstacle_generator ${catkin_LIBRARIES})


add_executable(astar_planner src/astar_planner.cpp)
target_link_libraries(astar_planner ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(trajectory_generator src/Trajectory.cpp)
target_link_libraries(trajectory_generator ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
