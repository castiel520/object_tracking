cmake_minimum_required( VERSION 2.8 )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

find_package(OpenCV REQUIRED)


#link_directories(${OpenCV_LIBRARY_DIRS})
#message(${OpenCV_INCLUDE_DIRS})
#set(${OpenCV_LIBS_DIR} /home/alejandro/OpenCV/OpenCV_3/lib)
#message(STATUS "HEY" ${OpenCV_LIBRARY_DIRS})
#include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})


find_package( PWP3D REQUIRED)
include_directories( ${PWP3D_INCLUDE_DIRS} /home/lpuig/PWP3D/Alejandro/object_tracking/external/PWP3D/install/include)
add_executable( PWP3DAPP PerseusLibTest.cpp )
target_link_libraries(PWP3DAPP ${PWP3D_LIBRARIES} ${OpenCV_LIBRARY_DIRS}) 


