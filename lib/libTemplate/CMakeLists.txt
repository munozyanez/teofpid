project(**lib**)


# source files
aux_source_directory(src **lib**_SRCS)
#INCLUDE_DIRECTORIES(src)

FILE(GLOB_RECURSE **lib**_INCLUDE_LIST src/*.h src/*.hpp)


ADD_LIBRARY(${PROJECT_NAME} SHARED ${**lib**_SRCS} ${**lib**_INCLUDE_LIST})

#in main CmakeLists.txt add (uncommented):
#add_subdirectory(${PROJECT_SOURCE_DIR}/lib/**lib**)
#set(**lib**INCLUDE_DIR ${PROJECT_SOURCE_DIR}/lib/**lib**/src ${**lib**_INCLUDE_DIR})
#INCLUDE_DIRECTORIES(${**lib**_INCLUDE_DIR})


