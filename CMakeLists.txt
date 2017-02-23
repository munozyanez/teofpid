##functions
function(echo_all_cmake_variable_values)
  message(STATUS "")
  get_cmake_property(vs VARIABLES)
  foreach(v ${vs})
    message(STATUS "${v}='${${v}}'")
  endforeach(v)
  message(STATUS "")
endfunction()

add_definitions(-std=c++0x -DYARP_NO_DEPRECATED_WARNINGS=ON)

##cmake code start
project(teofpid)

cmake_minimum_required(VERSION 2.8)

set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake )

aux_source_directory(./src SRC_LIST)

#cmake lines for libfpid
find_package(fpid QUIET)
if(fpid_FOUND)
  set(fpid_INCLUDE_DIR ${fpid_INCLUDE_DIRS})
else()
  message(STATUS "compiling fpid from sources")
  add_subdirectory(${PROJECT_SOURCE_DIR}/lib/fpid)
  set(fpid_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/lib/fpid/src ${fpid_INCLUDE_DIR})
endif()

INCLUDE_DIRECTORIES(${fpid_INCLUDE_DIR})

#teoMWI library
#add_subdirectory(${PROJECT_SOURCE_DIR}/lib/teoMWI)
#set(teoMWI_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/lib/teoMWI ${teoMWI_INCLUDE_DIR})


#find_package(YARP REQUIRED)

add_subdirectory(./lib)
INCLUDE_DIRECTORIES(${SUBDIR_INCLUDE_DIRECTORIES})


add_executable(${PROJECT_NAME} ${SRC_LIST})

#link with libs
target_link_libraries( ${PROJECT_NAME} LibraryInterface fpid teoMWI ${YARP_LIBRARIES})
echo_all_cmake_variable_values()


