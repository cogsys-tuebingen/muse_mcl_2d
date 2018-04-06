cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d_odometry)

find_package(catkin REQUIRED COMPONENTS
  muse_mcl_2d
  nav_msgs
  pluginlib
  roscpp
  tf
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}_models
    CATKIN_DEPENDS muse_mcl_2d
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}_models
    src/models/differential_drive.cpp
    src/models/differential_drive_abs.cpp
    src/models/omni_drive.cpp
)
target_link_libraries(${PROJECT_NAME}_models
    ${catkin_LIBRARIES}
)
install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_models
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
