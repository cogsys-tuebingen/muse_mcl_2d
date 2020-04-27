cmake_minimum_required(VERSION 2.8.3)
project(muse_mcl_2d_ndt)

find_package(catkin REQUIRED COMPONENTS
  cslibs_ndt_2d
  cslibs_math_2d
  cslibs_ndt_3d
  cslibs_math_3d
  cslibs_gridmaps
  cslibs_plugins_data
  muse_mcl_2d
  muse_mcl_2d_gridmaps
  pluginlib
  roscpp
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}_providers
    ${PROJECT_NAME}_models
    ${PROJECT_NAME}_maps
  CATKIN_DEPENDS
    cslibs_gridmaps
    cslibs_math_2d
    cslibs_math_3d
    cslibs_ndt_2d
    cslibs_ndt_3d
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

set(TARGET_INCLUDE_DIRS
    include/
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_maps SHARED
  src/maps/gridmap_2d.cpp
  src/maps/occupancy_gridmap_2d.cpp
  src/maps/gridmap_3d.cpp
  src/maps/occupancy_gridmap_3d.cpp
  src/maps/flat_gridmap_2d.cpp
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
)

add_library(${PROJECT_NAME}_providers SHARED
  src/providers/ndt_flat_gridmap_2d_provider.cpp
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
    ${YAML_CPP_LIBRARIES}
    ${PROJECT_NAME}_maps
)

add_library(${PROJECT_NAME}_models SHARED
  src/models/gridmap_2d_likelihood_field_model.cpp
  src/models/occupancy_gridmap_2d_likelihood_field_model.cpp
  src/models/gridmap_3d_likelihood_field_model.cpp
  src/models/occupancy_gridmap_3d_likelihood_field_model.cpp
#  src/models/ndt_2d.cpp
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

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS
          ${PROJECT_NAME}_models
          ${PROJECT_NAME}_providers
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
