cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d_ndt)

find_package(catkin REQUIRED COMPONENTS
  muse_mcl_2d
  muse_mcl_2d_gridmaps
  cslibs_ndt_2d
  cslibs_math_2d
  cslibs_ndt_3d
  cslibs_math_3d
  cslibs_gridmaps
  cslibs_plugins_data
  pluginlib
  roscpp
  rostest
  tf
)
find_package(PCL REQUIRED COMPONENTS common filters segmentation io)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_providers
            ${PROJECT_NAME}_models
            ${PROJECT_NAME}_maps
  CATKIN_DEPENDS muse_mcl_2d cslibs_math_2d cslibs_ndt_2d cslibs_math_3d
                 cslibs_ndt_3d cslibs_gridmaps cslibs_plugins_data
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}_maps SHARED
  src/maps/gridmap_2d.cpp
  src/maps/occupancy_gridmap_2d.cpp
  src/maps/gridmap_3d.cpp
  src/maps/occupancy_gridmap_3d.cpp
)

target_link_libraries(${PROJECT_NAME}_maps
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_providers SHARED
  src/providers/ndt_gridmap_2d_provider.cpp
  src/providers/ndt_occupancy_gridmap_2d_provider.cpp
  src/providers/ndt_gridmap_2d_service_provider.cpp
  src/providers/ndt_occupancy_gridmap_2d_service_provider.cpp
  src/providers/ndt_gridmap_3d_provider.cpp
  src/providers/ndt_occupancy_gridmap_3d_provider.cpp
  src/providers/ndt_gridmap_3d_service_provider.cpp
  src/providers/ndt_occupancy_gridmap_3d_service_provider.cpp
  src/providers/probability_gridmap_provider.cpp
  src/providers/probability_occupancy_gridmap_provider.cpp
  src/providers/probability_gridmap_service_provider.cpp
  src/providers/probability_occupancy_gridmap_service_provider.cpp
  src/providers/likelihood_field_gridmap_provider.cpp
  src/providers/likelihood_field_occupancy_gridmap_provider.cpp
  src/providers/likelihood_field_gridmap_service_provider.cpp
  src/providers/likelihood_field_occupancy_gridmap_service_provider.cpp
  src/providers/binary_gridmap_provider.cpp
  src/providers/binary_occupancy_gridmap_provider.cpp
  src/providers/binary_gridmap_service_provider.cpp
  src/providers/binary_occupancy_gridmap_service_provider.cpp
  src/providers/distance_gridmap_provider.cpp
  src/providers/distance_occupancy_gridmap_provider.cpp
  src/providers/distance_gridmap_service_provider.cpp
  src/providers/distance_occupancy_gridmap_service_provider.cpp
)

target_link_libraries(${PROJECT_NAME}_providers
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}_maps
  yaml-cpp
)

add_library(${PROJECT_NAME}_models SHARED
  src/models/gridmap_2d_likelihood_field_model.cpp
  src/models/occupancy_gridmap_2d_likelihood_field_model.cpp
  src/models/gridmap_3d_likelihood_field_model.cpp
  src/models/occupancy_gridmap_3d_likelihood_field_model.cpp
  src/models/ndt_2d.cpp
)

target_link_libraries(${PROJECT_NAME}_models
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}_maps
)

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_providers
                ${PROJECT_NAME}_models
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
