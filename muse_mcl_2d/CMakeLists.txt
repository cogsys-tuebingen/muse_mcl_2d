cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d)

#add_definitions("-DMUSE_SMC_USE_DOTTY")
#add_definitions("-DMUSE_SMC_LOG_STATE")
#add_definitions("-DMUSE_SMC_DEBUG")

find_package(catkin REQUIRED COMPONENTS
  cslibs_indexed_storage
  nav_msgs
  sensor_msgs
  pcl_ros
  pluginlib
  roscpp
  rostest
  muse_smc
  cslibs_math_2d
  cslibs_math_ros
  cslibs_plugins
  cslibs_plugins_data
  visualization_msgs
  geometry_msgs
  message_generation
  std_msgs
)

find_package(Eigen3 REQUIRED)

add_service_files(
   FILES
   GlobalInitialization.srv
   PoseInitialization.srv
)

add_message_files(
    FILES
    SampleMsg.msg
    SampleSetMsg.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
    INCLUDE_DIRS    include
    LIBRARIES       ${PROJECT_NAME}_resampling ${PROJECT_NAME}_sampling ${PROJECT_NAME}_scheduling ${PROJECT_NAME}_density ${PROJECT_NAME}_helper
    CATKIN_DEPENDS  muse_smc cslibs_math_ros cslibs_math_2d cslibs_plugins cslibs_plugins_data
)

include_directories(
    include
    ${Eigen3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV 3.4 QUIET)
if(${OpenCV_FOUND})
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}_statistics_plotter
    src/tools/statistics_plotter.cpp
)
target_link_libraries(${PROJECT_NAME}_statistics_plotter
    ${OpenCV_LIBRARIES}
)
endif()

add_library(${PROJECT_NAME}_resampling
    src/resampling/kld_2d.cpp
    src/resampling/local_regeneration_kld_2d.cpp
    src/resampling/systematic_2d.cpp
    src/resampling/residual_2d.cpp
    src/resampling/stratified_2d.cpp
    src/resampling/wheel_2d.cpp
)

target_link_libraries(${PROJECT_NAME}_resampling
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_library(${PROJECT_NAME}_sampling
  src/sampling/normal_sampling_2d.cpp
    src/sampling/uniform_primary_map_2d.cpp
    src/sampling/uniform_all_maps_2d.cpp
)

target_link_libraries(${PROJECT_NAME}_sampling
    ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_density
    src/density/simple_sample_density_2d.cpp
    src/density/mean_centered_sample_density_2d.cpp
)

target_link_libraries(${PROJECT_NAME}_density
    ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_scheduling
    src/scheduling/cfs.cpp
    src/scheduling/rate.cpp
    src/scheduling/cfs_drop_statistic.cpp
    src/scheduling/rate_drop_statistic.cpp
    src/scheduling/dummy.cpp
    src/scheduling/cfs_rate.cpp
)

target_link_libraries(${PROJECT_NAME}_scheduling
    ${catkin_LIBRARIES}
)


add_library(${PROJECT_NAME}_helper
    src/node/state_publisher_2d.cpp
    src/node/sample_set_publisher_2d.cpp
)
target_link_libraries(${PROJECT_NAME}_helper
    ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_helper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node
    src/node/muse_mcl_2d_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_helper
)

add_executable(${PROJECT_NAME}_box_muller
    test/test_box_muller.cpp
)

target_link_libraries(${PROJECT_NAME}_box_muller
    ${catkin_LIBRARIES}
)

set(MUSE_MCL_2D_PERF_TESTS True)
if(${MUSE_MCL_2D_PERF_TESTS})
    message("[muse_mcl_2d]: Building with performance tests!")
    add_executable(${PROJECT_NAME}_perf_runtimes_math_operations
        test/perf_runtimes_math_operations.cpp
    )
    target_link_libraries(${PROJECT_NAME}_perf_runtimes_math_operations
        ${catkin_LIBRARIES}
    )
endif()

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_resampling ${PROJECT_NAME}_sampling
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
