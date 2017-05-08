cmake_minimum_required(VERSION 2.8)

project(iplot)


# Pick up cmake modules - they all must be in the current subdirectory
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#deps
find_package(PLOTUTILS REQUIRED)

# source files
aux_source_directory(. PLOT_SRCS)
# include files
FILE(GLOB_RECURSE PLOT_INCLUDE_DIR src/*.h src/*.hpp)

set(SUBDIR_LINK_NAMES ${SUBDIR_LINK_NAMES}
    ${PROJECT_NAME}
    PARENT_SCOPE)
set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES}
    ${CMAKE_CURRENT_SOURCE_DIR}
    PARENT_SCOPE)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${PLOT_SRCS} ${PLOT_INCLUDE_DIR})



#Install Library targets if top level project
if("${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")
    #install target
    message(STATUS "Configuring " ${PROJECT_NAME} " as main project")
    install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME})
    install(FILES ${PLOT_INCLUDE_DIR}
        DESTINATION include/${PROJECT_NAME})

    # uninstall target
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)
    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

#Set Library variables if subdirectory (called from add_subdirectory() )
else("${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")
    message(STATUS "Configuring " ${PROJECT_NAME} " as sub-project in " ${CMAKE_CURRENT_SOURCE_DIR})

endif()


target_link_libraries( ${PROJECT_NAME} ${PLOTUTILS_LIBRARIES})



