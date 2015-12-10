cmake_minimum_required(VERSION 2.8.7)
project(obj_recognition)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#find_package(Eigen3 REQUIRED)
find_package(VTK REQUIRED)

#3D renderer
include_directories(/home/alejandro/workspace/renderer/include /home/alejandro/workspace/renderer/src  /home/alejandro/workspace/ork_renderer/include/ /home/alejandro/workspace/ork_renderer/src  ${PCL_INCLUDE_DIRS})
link_directories( ${PCL_LIBRARY_DIRS} /home/alejandro/workspace/ork_renderer/build/devel/lib/)
add_definitions(${PCL_DEFINITIONS})

#include_directories(/usr/include/eigen3/Eigen/  )
link_directories(${PCL_LIBRARY_DIRS} )

find_package( PWP3D REQUIRED)
include_directories( ${PWP3D_INCLUDE_DIRS})

add_executable(rendermodel src/rendermodel.cpp src/image_cropper.cpp src/obj_recognition.cpp)
target_link_libraries(rendermodel ${PCL_LIBRARIES} object_recognition_renderer_3d glut ${OpenCV_LIBRARIES})

add_executable(mi_rendermodel src/mi_rendermodel.cpp src/image_cropper.cpp src/obj_recognition.cpp src/visualizer.cpp src/custom_functions.cpp)
target_link_libraries(mi_rendermodel ${PCL_LIBRARIES} object_recognition_renderer_3d ${OpenCV_LIBRARIES})

add_executable(object_tracking src/object_tracking.cpp src/image_cropper.cpp src/obj_recognition.cpp src/visualizer.cpp src/custom_functions.cpp src/background.cpp)
target_link_libraries(object_tracking ${PCL_LIBRARIES} object_recognition_renderer_3d ${OpenCV_LIBRARIES} ${PWP3D_LIBRARIES})


