###cmake code start
project(LibraryInterface)

#cmake_minimum_required(VERSION 2.8)
#aux_source_directory(./src SRC_LIST)

##cmake lines for libfpid
#find_package(fpid QUIET)
#if(fpid_FOUND)
#  set(fpid_INCLUDE_DIR ${fpid_INCLUDE_DIRS})
#else()
#  message(STATUS "compiling fpid from sources")
#  add_subdirectory(${PROJECT_SOURCE_DIR}/lib/fpid)
#  set(fpid_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/lib/fpid/src ${fpid_INCLUDE_DIR})
#endif()

#INCLUDE_DIRECTORIES(${fpid_INCLUDE_DIR})
#find_package(MathGL)

#/usr/lib/x86_64-linux-gnu/libplotter.a


# source files
aux_source_directory(. SOURCE_FILES)
FILE(GLOB_RECURSE INCLUDE_LIST *.h *.hpp)
ADD_LIBRARY(${PROJECT_NAME} STATIC ${SOURCE_FILES} ${INCLUDE_LIST})

#teoMWI library
add_subdirectory(./teoMWI)

#fcontrol
add_subdirectory(./fcontrol)

add_subdirectory(./plot)
#add_subdirectory(./mathgl)
#find_library(PLOTUTILS_LIBRARIES libplotter.so
#    PATH /usr/lib/x86_64-linux-gnu/)
#find_package(PLplot)
#set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES} ${PLPLOT_INCLUDE_DIRECTORIES} PARENT_SCOPE)



set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${fcontrol_SOURCE_DIR}/src
    PARENT_SCOPE)

INCLUDE_DIRECTORIES(SUBDIR_INCLUDE_DIRECTORIES)

#set not working here!!!
#set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES} ${fcontrol_SOURCE_DIR}/src/ PARENT_SCOPE)
#LIST(APPEND SUBDIR_INCLUDE_DIRECTORIES ${fcontrol_SOURCE_DIR}/src/ PARENT_SCOPE)


#set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES} ${MathGL_INCLUDE_DIRS} PARENT_SCOPE)


#set(teoMWI_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/lib/teoMWI ${teoMWI_INCLUDE_DIR})


#find_package(YARP REQUIRED)


#add_executable(${PROJECT_NAME} ${SRC_LIST})

##link with libs
target_link_libraries( ${PROJECT_NAME} fcontrol fpid teoMWI ${SUBDIR_LINK_NAMES} ${PLplot_LIBRARY} ${PLplot_cxx_LIBRARY})
##echo_all_cmake_variable_values()


