cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d_odometry)

find_package(catkin REQUIRED COMPONENTS
  muse_mcl_2d
  nav_msgs
  pluginlib
  roscpp
)

catkin_package(
    LIBRARIES
        ${PROJECT_NAME}_models
    CATKIN_DEPENDS
        muse_mcl_2d
)


if(NOT ${CMAKE_BUILD_TYPE} STREQUAL Debug)
    set(TARGET_COMPILE_OPTIONS
        -Wall -Werror -Ofast -ffast-math
    )
    # enabling -march=native my result in problems on some machines,
    # my try -mtune=native

    message(STATUS "[${PROJECT_NAME}]: Compiling with optimization!")
endif()

set(TARGET_INCLUDE_DIRS
    include/
    ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}_models SHARED
    src/models/differential_drive.cpp
    src/models/differential_drive_abs.cpp
    src/models/differential_drive_box_muller.cpp
    src/models/differential_drive_uncorrected.cpp
    src/models/omni_drive.cpp
)

target_include_directories(${PROJECT_NAME}_models
    PRIVATE
        ${TARGET_INCLUDE_DIRS}
)

target_compile_options(${PROJECT_NAME}_models
    PRIVATE
        ${TARGET_COMPILE_OPTIONS}
)

target_link_libraries(${PROJECT_NAME}_models
    PRIVATE
        ${catkin_LIBRARIES}
)
install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_models
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
