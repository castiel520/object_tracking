# Install script for directory: /home/alejandro/workspace/PWP3D/PerseusLib

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/PWP3D/config.h")
FILE(INSTALL DESTINATION "/usr/local/include/PWP3D" TYPE FILE FILES "/home/alejandro/workspace/PWP3D/PerseusLib/config.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/PWP3D/CUDAConvolution.h;/usr/local/include/PWP3D/CUDAData.h;/usr/local/include/PWP3D/CUDADT.h;/usr/local/include/PWP3D/CUDAEF.h;/usr/local/include/PWP3D/CUDAEngine.h;/usr/local/include/PWP3D/CUDARenderer.h;/usr/local/include/PWP3D/CUDAScharr.h;/usr/local/include/PWP3D/CUDAUtils.h;/usr/local/include/PWP3D/HistogramVarBin.h;/usr/local/include/PWP3D/ImageRender.h;/usr/local/include/PWP3D/IterationConfiguration.h;/usr/local/include/PWP3D/Object3D.h;/usr/local/include/PWP3D/Object3DParams.h;/usr/local/include/PWP3D/Pose3D.h;/usr/local/include/PWP3D/StepSize3D.h;/usr/local/include/PWP3D/View3D.h;/usr/local/include/PWP3D/View3DParams.h;/usr/local/include/PWP3D/EFStandard.h;/usr/local/include/PWP3D/IEnergyFunction.h;/usr/local/include/PWP3D/OptimisationEngine.h;/usr/local/include/PWP3D/PerseusLibDefines.h;/usr/local/include/PWP3D/PerseusLib.h;/usr/local/include/PWP3D/ImagePerseus.h;/usr/local/include/PWP3D/PixelUCHAR4.h;/usr/local/include/PWP3D/Vector2D.h;/usr/local/include/PWP3D/Vector3D.h;/usr/local/include/PWP3D/Vector4D.h;/usr/local/include/PWP3D/DrawingEngine.h;/usr/local/include/PWP3D/DrawingPrimitives.h;/usr/local/include/PWP3D/Model.h;/usr/local/include/PWP3D/ModelFace.h;/usr/local/include/PWP3D/ModelGroup.h;/usr/local/include/PWP3D/ModelH.h;/usr/local/include/PWP3D/ModelVertex.h;/usr/local/include/PWP3D/Renderer3DObject.h;/usr/local/include/PWP3D/Renderer3DView.h;/usr/local/include/PWP3D/Camera3D.h;/usr/local/include/PWP3D/Quaternion.h;/usr/local/include/PWP3D/CameraCoordinateTransform.h;/usr/local/include/PWP3D/CoordinateTransform.h;/usr/local/include/PWP3D/ObjectCoordinateTransform.h;/usr/local/include/PWP3D/Debug.h;/usr/local/include/PWP3D/FileUtils.h;/usr/local/include/PWP3D/HistogramEngine.h;/usr/local/include/PWP3D/ImageUtils.h;/usr/local/include/PWP3D/MathUtils.h;/usr/local/include/PWP3D/VisualisationEngine.h")
FILE(INSTALL DESTINATION "/usr/local/include/PWP3D" TYPE FILE FILES
    "/home/alejandro/workspace/PWP3D/PerseusLib/CUDA/CUDAConvolution.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/CUDA/CUDAData.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/CUDA/CUDADT.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/CUDA/CUDAEF.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/CUDA/CUDAEngine.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/CUDA/CUDARenderer.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/CUDA/CUDAScharr.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/CUDA/CUDAUtils.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Objects/HistogramVarBin.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Objects/ImageRender.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Objects/IterationConfiguration.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Objects/Object3D.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Objects/Object3DParams.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Objects/Pose3D.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Objects/StepSize3D.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Objects/View3D.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Objects/View3DParams.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Optimiser/EFs/EFStandard.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Optimiser/EFs/IEnergyFunction.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Optimiser/Engine/OptimisationEngine.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Others/PerseusLibDefines.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/PerseusLib.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Primitives/ImagePerseus.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Primitives/PixelUCHAR4.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Primitives/Vector2D.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Primitives/Vector3D.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Primitives/Vector4D.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Renderer/Engine/DrawingEngine.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Renderer/Engine/DrawingPrimitives.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Renderer/Model/Model.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Renderer/Model/ModelFace.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Renderer/Model/ModelGroup.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Renderer/Model/ModelH.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Renderer/Model/ModelVertex.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Renderer/Objects/Renderer3DObject.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Renderer/Objects/Renderer3DView.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Renderer/Primitives/Camera3D.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Renderer/Primitives/Quaternion.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Renderer/Transforms/CameraCoordinateTransform.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Renderer/Transforms/CoordinateTransform.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Renderer/Transforms/ObjectCoordinateTransform.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Utils/Debug.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Utils/FileUtils.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Utils/HistogramEngine.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Utils/ImageUtils.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Utils/MathUtils.h"
    "/home/alejandro/workspace/PWP3D/PerseusLib/Utils/VisualisationEngine.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}/usr/local/lib/libpwp3d.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libpwp3d.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/lib/libpwp3d.so"
         RPATH "")
  ENDIF()
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libpwp3d.so")
FILE(INSTALL DESTINATION "/usr/local/lib" TYPE SHARED_LIBRARY FILES "/home/alejandro/workspace/PWP3D/PerseusLib/libpwp3d.so")
  IF(EXISTS "$ENV{DESTDIR}/usr/local/lib/libpwp3d.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libpwp3d.so")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}/usr/local/lib/libpwp3d.so")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/lib/libpwp3d.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/PWP3D" TYPE FILE FILES
    "/home/alejandro/workspace/PWP3D/PerseusLib/CMakeFiles/PWP3DConfig.cmake"
    "/home/alejandro/workspace/PWP3D/PerseusLib/PWP3DConfigVersion.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/PWP3D/PWP3DTargets.cmake")
    FILE(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/PWP3D/PWP3DTargets.cmake"
         "/home/alejandro/workspace/PWP3D/PerseusLib/CMakeFiles/Export/lib/cmake/PWP3D/PWP3DTargets.cmake")
    IF(EXPORT_FILE_CHANGED)
      FILE(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/PWP3D/PWP3DTargets-*.cmake")
      IF(OLD_CONFIG_FILES)
        MESSAGE(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/PWP3D/PWP3DTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        FILE(REMOVE ${OLD_CONFIG_FILES})
      ENDIF(OLD_CONFIG_FILES)
    ENDIF(EXPORT_FILE_CHANGED)
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/PWP3D" TYPE FILE FILES "/home/alejandro/workspace/PWP3D/PerseusLib/CMakeFiles/Export/lib/cmake/PWP3D/PWP3DTargets.cmake")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/PWP3D" TYPE FILE FILES "/home/alejandro/workspace/PWP3D/PerseusLib/CMakeFiles/Export/lib/cmake/PWP3D/PWP3DTargets-release.cmake")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

