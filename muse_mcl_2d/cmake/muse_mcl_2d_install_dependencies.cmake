cmake_minimum_required(VERSION 3.2.3)
include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)
# ${PROJECT_NAME}_install_dependencies([<PREFIX> PREFIX_NAME] [<DEPS> ...]) 
function(${PROJECT_NAME}_install_dependencies)
    cmake_parse_arguments(clone
        ""          # list of names of the boolean arguments (only defined ones will be true)
        "PREFIX"    # list of names of mono-valued arguments
        "DEPS"      # list of names of multi-valued arguments (output variables are lists)
        ${ARGN}     # arguments of the function to parse, here we take the all original ones
    )

if(${clone_PREFIX})
    set(download_PREFIX "${CMAKE_SOURCE_DIR}/${clone_PREFIX}/")
else()
    set(download_PREFIX "${CMAKE_SOURCE_DIR}/")
endif()


foreach(d ${clone_DEPS})
    message("Looking for dependency : '${d}'" )

    find_package(${d} QUIET)
    if(NOT ${${d}_FOUND})
        message("Cloning dependency : '${d}'" )

        ExternalProject_Add(${d}
            GIT_REPOSITORY https://github.com/cogsys-tuebingen/${d}.git
            GIT_TAG "master"
            DOWNLOAD_DIR "${download_PREFIX}${d}"
        )
    else()
        message("Dependeny found : '${d}'")
    endif()
    endforeach()
endfunction()
