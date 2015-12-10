# Generated by CMake 2.8.7

IF("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.5)
   MESSAGE(FATAL_ERROR "CMake >= 2.6.0 required")
ENDIF("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.5)
CMAKE_POLICY(PUSH)
CMAKE_POLICY(VERSION 2.6)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
SET(CMAKE_IMPORT_FILE_VERSION 1)

# Create imported target pwp3d
ADD_LIBRARY(pwp3d STATIC IMPORTED)

# Import target "pwp3d" for configuration ""
SET_PROPERTY(TARGET pwp3d APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
SET_TARGET_PROPERTIES(pwp3d PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "/usr/local/cuda/lib64/libcudart.so;/usr/lib/x86_64-linux-gnu/libcuda.so;/usr/local/cuda/lib64/libnpps.so;/usr/local/cuda/lib64/libnppi.so;/usr/local/cuda/lib64/libnpps.so"
  IMPORTED_LOCATION_NOCONFIG "/home/alejandro/workspace/PWP3D/PerseusLib/build/libpwp3d.a"
  )

# Commands beyond this point should not need to know the version.
SET(CMAKE_IMPORT_FILE_VERSION)
CMAKE_POLICY(POP)
