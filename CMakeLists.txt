cmake_minimum_required(VERSION 2.8.7)
project(obj_recognition)

find_package(OpenCV REQUIRED)
add_definitions(${OpenCV_DEFINITIONS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

#find_package(Eigen3 REQUIRED)
find_package(VTK REQUIRED)

add_definitions(-std=c++11)
#3D renderer
include_directories(/home/lpuig/PWP3D/Alejandro/object_tracking/external/ork_renderer/include/ /home/lpuig/PWP3D/Alejandro/object_tracking/external/ork_renderer/src include  /home/lpuig/PWP3D/Alejandro/PWP3D/install/include )
link_directories( ${PCL_LIBRARY_DIRS} /home/lpuig/PWP3D/Alejandro/object_tracking/external/ork_renderer/build/devel/lib)

find_package( PWP3D REQUIRED)
include_directories( ${PWP3D_INCLUDE_DIRS} /home/lpuig/PWP3D/Alejandro/PWP3D/installed/include)
MESSAGE( "PWP3D_INCLUDE_DIRS: " ${PWP3D_INCLUDE_DIRS} )
MESSAGE( "PWP3D_LIBRARIES: " ${PWP3D_LIBRARIES} )

add_executable(rendermodel src/rendermodel.cpp src/image_cropper.cpp src/obj_recognition.cpp)
target_link_libraries(rendermodel ${PCL_LIBRARIES} object_recognition_renderer_3d glut ${OpenCV_LIBRARIES})

add_executable(mi_rendermodel 
                src/mi_rendermodel.cpp 
                src/image_cropper.cpp 
                src/obj_recognition.cpp 
                src/visualizer.cpp
                src/custom_functions.cpp
)
target_link_libraries(mi_rendermodel ${PCL_LIBRARIES} object_recognition_renderer_3d ${OpenCV_LIBRARIES})

add_executable(object_tracking 
           src/object_tracking.cpp 
           src/image_cropper.cpp 
           src/obj_recognition.cpp 
           src/visualizer.cpp 
           src/custom_functions.cpp 
           src/background.cpp
)
target_link_libraries(object_tracking ${PCL_LIBRARIES} object_recognition_renderer_3d ${OpenCV_LIBRARIES} ${PWP3D_LIBRARIES})

