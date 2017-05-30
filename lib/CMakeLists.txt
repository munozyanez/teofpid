

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



set(SUBDIR_LINK_NAMES ${SUBDIR_LINK_NAMES}  PARENT_SCOPE)
set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES} ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

INCLUDE_DIRECTORIES(SUBDIR_INCLUDE_DIRECTORIES)




