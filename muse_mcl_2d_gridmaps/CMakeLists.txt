cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d_gridmaps)

find_package(catkin REQUIRED COMPONENTS
    cslibs_gridmaps
    cslibs_math_2d
    cslibs_plugins_data
    muse_mcl_2d
    nav_msgs
    pluginlib
    roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_models
    ${PROJECT_NAME}_maps
    ${PROJECT_NAME}_providers
  CATKIN_DEPENDS
    cslibs_gridmaps
    cslibs_math_2d
    cslibs_plugins_data
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

find_package(yaml-cpp REQUIRED)

set(TARGET_INCLUDE_DIRS
    include/
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_maps SHARED
    src/maps/binary_gridmap.cpp
    src/maps/distance_gridmap.cpp
    src/maps/likelihood_field_gridmap.cpp
    src/maps/probability_gridmap.cpp
)

target_include_directories(${PROJECT_NAME}_maps
    PRIVATE
        ${TARGET_INCLUDE_DIRS}
)

target_compile_options(${PROJECT_NAME}_maps
    PRIVATE
        ${TARGET_COMPILE_OPTIONS}
)

target_link_libraries(${PROJECT_NAME}_maps
    PRIVATE
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
)

add_library(${PROJECT_NAME}_providers SHARED
    src/providers/binary_gridmap_provider.cpp
    src/providers/distance_gridmap_provider.cpp
    src/providers/probability_gridmap_provider.cpp
    src/providers/likelihood_field_gridmap_provider.cpp
    src/providers/binary_gridmap_service_provider.cpp
    src/providers/distance_gridmap_service_provider.cpp
    src/providers/probability_gridmap_service_provider.cpp
    src/providers/likelihood_field_gridmap_service_provider.cpp
    src/providers/binary_gridmap_load_provider.cpp
    src/providers/distance_gridmap_load_provider.cpp
    src/providers/probability_gridmap_load_provider.cpp
    src/providers/likelihood_field_gridmap_load_provider.cpp
)

target_include_directories(${PROJECT_NAME}_providers
    PRIVATE
        ${TARGET_INCLUDE_DIRS}
)

target_compile_options(${PROJECT_NAME}_providers
    PRIVATE
        ${TARGET_COMPILE_OPTIONS}
)

target_link_libraries(${PROJECT_NAME}_providers
    PRIVATE
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_maps
        ${YAML_CPP_LIBRARIES}
)

add_library(${PROJECT_NAME}_models SHARED
    src/models/beam_model.cpp
    src/models/likelihood_field_model.cpp
    src/models/beam_model_amcl.cpp
    src/models/likelihood_field_model_amcl.cpp
    src/models/likelihood_field_prob_model_amcl.cpp
    src/models/beam_model_mle.cpp
    src/models/beam_model_parameter_estimator.cpp

    src/models/beam_model_log.cpp
    src/models/likelihood_field_model_log.cpp
    src/models/beam_model_amcl_normalized.cpp
    src/models/likelihood_field_model_amcl_normalized.cpp
    src/models/likelihood_field_model_pc.cpp
    src/models/likelihood_field_model_pc_log.cpp
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
        ${PROJECT_NAME}_maps
)

muse_mcl_2d_gridmaps_add_unit_test_ros(test_load_plugins
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    LAUNCH_FILE
        test/plugins.test
    SOURCE_FILES
        test/plugins.cpp
    LINK_LIBRARIES
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

set(MUSE_MCL_2D_GRIDMAPS_ADDITIONALS True)

if(${MUSE_MCL_2D_GRIDMAPS_ADDITIONALS})
    add_executable(${PROJECT_NAME}_perf_runtimes_exp
        test/perf_runtimes_exp.cpp
    )

    target_include_directories(${PROJECT_NAME}_perf_runtimes_exp
        PRIVATE
            ${TARGET_INCLUDE_DIRS}
    )

    target_compile_options(${PROJECT_NAME}_perf_runtimes_exp
        PRIVATE
            ${TARGET_COMPILE_OPTIONS}
    )
endif()

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS
            ${PROJECT_NAME}_models
            ${PROJECT_NAME}_maps
            ${PROJECT_NAME}_providers
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
