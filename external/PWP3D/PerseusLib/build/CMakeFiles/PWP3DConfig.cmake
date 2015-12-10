# Compute paths
get_filename_component( PROJECT_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH )
SET( PWP3D_INCLUDE_DIRS "${PWP3D_CMAKE_DIR}/../../../include;/usr/local/cuda/include" )

# Library dependencies (contains definitions for IMPORTED targets)
if( NOT TARGET pwp3d AND NOT PWP3D_BINARY_DIR )
  include( "${PROJECT_CMAKE_DIR}/PWP3DTargets.cmake" )
endif()

SET( PWP3D_LIBRARIES "pwp3d" )
