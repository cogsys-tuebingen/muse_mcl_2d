cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d)

#add_definitions("-DMUSE_SMC_USE_DOTTY")
#add_definitions("-DMUSE_SMC_LOG_STATE")
#add_definitions("-DMUSE_SMC_DEBUG")

find_package(catkin REQUIRED COMPONENTS
  cslibs_indexed_storage
  cslibs_math_2d
  cslibs_math_ros
  cslibs_plugins
  cslibs_plugins_data
  muse_smc
  geometry_msgs
  message_generation
  nav_msgs
  sensor_msgs
  pluginlib
  roscpp
  std_msgs
  visualization_msgs
)

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
    INCLUDE_DIRS
        include/
    LIBRARIES
        ${PROJECT_NAME}_density
        ${PROJECT_NAME}_helper
        ${PROJECT_NAME}_resampling
        ${PROJECT_NAME}_sampling
        ${PROJECT_NAME}_scheduling
    CATKIN_DEPENDS
        muse_smc
        cslibs_math_2d
        cslibs_math_ros
        cslibs_plugins
        cslibs_plugins_data
)

set(TARGET_INCLUDE_DIRS
    include/
    ${catkin_INCLUDE_DIRS}
)

if(NOT ${CMAKE_BUILD_TYPE} STREQUAL Debug)
    set(TARGET_COMPILE_OPTIONS
        -Wall -Werror -Ofast -ffast-math
    )
    # enabling -march=native my result in problems on some machines,
    # my try -mtune=native

    message(STATUS "[${PROJECT_NAME}]: Compiling with optimization!")
endif()

find_package(OpenCV 3.4 QUIET)
if(${OpenCV_FOUND})
    add_executable(${PROJECT_NAME}_statistics_plotter
        src/tools/statistics_plotter.cpp
    )
    target_include_directories(${PROJECT_NAME}_statistics_plotter
        PRIVATE
            ${OpenCV_INCLUDE_DIRS}
            ${TARGET_INCLUDE_DIRS}
    )
    target_link_libraries(${PROJECT_NAME}_statistics_plotter
        PRIVATE
            ${OpenCV_LIBRARIES}
    )
endif()

add_library(${PROJECT_NAME}_resampling
    src/resampling/kld_2d.cpp
    src/resampling/kld_augmented_2d.cpp
    src/resampling/local_regeneration_kld_2d.cpp
    src/resampling/systematic_2d.cpp
    src/resampling/residual_2d.cpp
    src/resampling/stratified_2d.cpp
    src/resampling/wheel_2d.cpp
    src/resampling/multinomial.cpp
)

target_include_directories(${PROJECT_NAME}_resampling
    PRIVATE
        ${TARGET_INCLUDE_DIRS}
)

target_compile_options(${PROJECT_NAME}_resampling
    PRIVATE
        ${TARGET_COMPILE_OPTIONS}
)

target_link_libraries(${PROJECT_NAME}_resampling
    PRIVATE
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

# add_library(${PROJECT_NAME}_normal_sampling
#     src/sampling/normal_sampling_2d.cpp
# )

# target_include_directories(${PROJECT_NAME}_normal_sampling
#     PRIVATE
#         ${TARGET_INCLUDE_DIRS}
# )

# target_compile_options(${PROJECT_NAME}_normal_sampling
#     PRIVATE
#         ${TARGET_COMPILE_OPTIONS}
# )

# target_link_libraries(${PROJECT_NAME}_normal_sampling
#     PRIVATE
#         ${catkin_LIBRARIES}
# )

# add_library(${PROJECT_NAME}_uniform_sampling
#     src/sampling/uniform_primary_map_2d.cpp
#     src/sampling/uniform_all_maps_2d.cpp
# )

# target_include_directories(${PROJECT_NAME}_uniform_sampling
#     PRIVATE
#         ${TARGET_INCLUDE_DIRS}
# )

# target_compile_options(${PROJECT_NAME}_uniform_sampling
#     PRIVATE
#         ${TARGET_COMPILE_OPTIONS}
# )

# target_link_libraries(${PROJECT_NAME}_uniform_sampling
#     PRIVATE
#         ${catkin_LIBRARIES}
# )

# add_library(${PROJECT_NAME}_density
#     src/density/simple_sample_density_2d.cpp
#     src/density/mean_centered_sample_density_2d.cpp
# )

# target_include_directories(${PROJECT_NAME}_density
#     PRIVATE
#         ${TARGET_INCLUDE_DIRS}
# )

# target_compile_options(${PROJECT_NAME}_density
#     PRIVATE
#         ${TARGET_COMPILE_OPTIONS}
# )

# target_link_libraries(${PROJECT_NAME}_density
#     PRIVATE
#         ${catkin_LIBRARIES}
# )

# add_library(${PROJECT_NAME}_scheduling
#     src/scheduling/cfs.cpp
#     src/scheduling/rate.cpp
#     src/scheduling/cfs_drop_statistic.cpp
#     src/scheduling/rate_drop_statistic.cpp
#     src/scheduling/dummy.cpp
#     src/scheduling/cfs_laggy.cpp
# )

# target_include_directories(${PROJECT_NAME}_scheduling
#     PRIVATE
#         ${TARGET_INCLUDE_DIRS}
# )

# target_compile_options(${PROJECT_NAME}_scheduling
#     PRIVATE
#         ${TARGET_COMPILE_OPTIONS}
# )

# target_link_libraries(${PROJECT_NAME}_scheduling
#     PRIVATE
#         ${catkin_LIBRARIES}
# )

# add_library(${PROJECT_NAME}_helper
#     src/instance/state_publisher_2d.cpp
#     src/instance/sample_set_publisher_2d.cpp
# )

# target_include_directories(${PROJECT_NAME}_helper
#     PRIVATE
#         ${TARGET_INCLUDE_DIRS}
# )

# target_compile_options(${PROJECT_NAME}_helper
#     PRIVATE
#         ${TARGET_COMPILE_OPTIONS}
# )

# target_link_libraries(${PROJECT_NAME}_helper
#     PRIVATE
#         ${catkin_LIBRARIES}
# )
# add_dependencies(${PROJECT_NAME}_helper
#     ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
# )

# add_executable(${PROJECT_NAME}_node
#     src/node/muse_mcl_2d_node.cpp
# )

# target_include_directories(${PROJECT_NAME}_node
#     PRIVATE
#         ${TARGET_INCLUDE_DIRS}
# )


# target_compile_options(${PROJECT_NAME}_node
#     PRIVATE
#         ${TARGET_COMPILE_OPTIONS}
# )

# target_link_libraries(${PROJECT_NAME}_node
#     PRIVATE
#         ${catkin_LIBRARIES}
#         ${PROJECT_NAME}_helper
# )


# set(MUSE_MCL_2D_ADDTIONALS True)
# if(${MUSE_MCL_2D_ADDTIONALS})
#     message(STATUS "[muse_mcl_2d]: Building additional executables!")

#     add_executable(${PROJECT_NAME}_box_muller
#         test/test_box_muller.cpp
#     )

#     target_include_directories(${PROJECT_NAME}_box_muller
#         PRIVATE
#             ${TARGET_INCLUDE_DIRS}
#     )

#     target_link_libraries(${PROJECT_NAME}_box_muller
#         PRIVATE
#             ${catkin_LIBRARIES}
#     )

#     add_executable(${PROJECT_NAME}_benchmark_math
#         test/benchmark_math.cpp
#     )

#     target_include_directories(${PROJECT_NAME}_benchmark_math
#         PRIVATE
#             ${TARGET_INCLUDE_DIRS}
#     )

#     target_compile_options(${PROJECT_NAME}_benchmark_math
#     PRIVATE
#         ${TARGET_COMPILE_OPTIONS}
#     )

#     target_link_libraries(${PROJECT_NAME}_benchmark_math
#         ${catkin_LIBRARIES}
#     )
# endif()

# install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# install(TARGETS
#             ${PROJECT_NAME}_resampling
#             ${PROJECT_NAME}_normal_sampling
#             ${PROJECT_NAME}_uniform_sampling
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
