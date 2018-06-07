if(WIN32)
    set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/../windows)
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

find_package(SDL REQUIRED)
find_package(PNG REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
find_package(GLEW)
if (NOT GLEW_FOUND)
    message("System installed GLEW not found, falling back to source")
    add_subdirectory(glew)
endif()
