project(**lib**)


# source files for gait
aux_source_directory(src **lib**_SRCS)
#INCLUDE_DIRECTORIES(src)

FILE(GLOB_RECURSE GAIT_INCLUDE_LIST src/*.h src/*.hpp)


ADD_LIBRARY(${PROJECT_NAME} SHARED ${**lib**_SRCS} ${**lib**_INCLUDE_LIST})

