cmake_minimum_required(VERSION 3.0.2)
project(py_astar_planner)

find_package(catkin REQUIRED
  COMPONENTS
    angles
    base_local_planner
    costmap_2d
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
)

find_package(Python COMPONENTS Interpreter Development)
message("=======================================")
message("Python_FOUND:${Python_FOUND}")
message("Python_VERSION:${Python_VERSION}")
message("Python_Development_FOUND:${Python_Development_FOUND}")
message("Python_LIBRARIES:${Python_LIBRARIES}")
message("Python_INCLUDE_DIRS " ${Python_INCLUDE_DIRS})
message("=======================================")

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${Python_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES py_astar_planner
  CATKIN_DEPENDS
    angles
    base_local_planner
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_ros
    )


add_library(py_astar_planner src/py_astar_planner.cpp)
add_dependencies(py_astar_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(FALSE)
target_include_directories(py_astar_planner 
    ${catkin_INCLUDE_DIRS}
    ${Python_INCLUDE_DIRS}
    )
target_link_directories(py_astar_planner 
    ${catkin_LIBRARY_DIRS}
    ${Python_LIBRARY_DIRS}
    )
endif()

target_link_libraries(py_astar_planner
    ${catkin_LIBRARIES}
    ${Python_LIBRARIES}
    )

install(TARGETS py_astar_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



