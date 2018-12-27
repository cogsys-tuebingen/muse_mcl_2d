cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d_vectormaps)

find_package(catkin REQUIRED COMPONENTS
  muse_mcl_2d
  cslibs_vectormaps
  cslibs_plugins_data
  pluginlib
  roscpp
  rostest
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS muse_mcl_2d cslibs_vectormaps cslibs_plugins_data
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/providers/vectormap_provider.cpp
    src/providers/oriented_grid_vectormap_provider.cpp
    src/models/beam_model_vector.cpp
    src/models/beam_model_amcl_vector.cpp
    src/models/beam_model_amcl_oriented_grid_vector.cpp
    src/models/likelihood_field_model_vector.cpp
    src/models/likelihood_field_model_amcl_vector.cpp
    src/models/likelihood_field_model_amcl_oriented_grid_vector.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
