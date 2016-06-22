project(fpid)


# source files
aux_source_directory(src fpid_SRCS)
INCLUDE_DIRECTORIES(src)


FILE(GLOB_RECURSE fpid_INCLUDE_LIST src/*.h src/*.hpp)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${fpid_SRCS} ${fpid_INCLUDE_LIST})

