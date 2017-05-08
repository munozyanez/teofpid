# - Try to find lib
# Once done this will define
#  LIB_FOUND - System has libgait
#  LIB_INCLUDE_DIRS - The libgait include directories
#  LIB_LIBRARIES - The libraries needed to use libgait

#Default location is /usr
set(libName "PLOTUTILS")
set(SEARCH_PATHS /usr)
message(STATUS "Looking for local lib in: [${PLOTUTILS_LOCATION}].")

find_path(PLOTUTILS_INCLUDE_DIR plotter.h
    PATHS "${SEARCH_PATHS}/include/"
    HINTS ${SEARCH_PATHS}
    PATH_SUFFIXES gait )

find_library(PLOTUTILS_LIBRARY NAMES libplotter plotter
    PATHS "${SEARCH_PATHS}/lib/x86_64-linux-gnu/")


if(${PLOTUTILS_LIBRARY} STREQUAL "PLOTUTILS_LIBRARY-NOTFOUND")
    message(STATUS "${libName} not installed.")
else()
    message(STATUS "${libName} library found.")
    set (PLOTUTILS_FOUND TRUE)
    message(STATUS "Local lib files detected: [${PLOTUTILS_LIBRARY}].")
    message(STATUS "Local include directories: [${PLOTUTILS_INCLUDE_DIR}].")
    set(PLOTUTILS_LIBRARIES ${PLOTUTILS_LIBRARY} )
    set(PLOTUTILS_INCLUDE_DIRS ${PLOTUTILS_INCLUDE_DIR} )
endif()
