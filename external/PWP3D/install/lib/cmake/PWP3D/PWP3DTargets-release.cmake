#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pwp3d" for configuration "Release"
set_property(TARGET pwp3d APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pwp3d PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "/usr/local/cuda-7.5/lib64/libcudart.so;/usr/local/cuda-7.5/lib64/libnppc.so;/usr/local/cuda-7.5/lib64/libnppi.so;/usr/local/cuda-7.5/lib64/libnpps.so;/usr/lib/libfreeimage.so;/usr/local/lib/libassimp.so;opencv_videostab;opencv_video;opencv_ts;opencv_superres;opencv_stitching;opencv_photo;opencv_ocl;opencv_objdetect;opencv_nonfree;opencv_ml;opencv_legacy;opencv_imgproc;opencv_highgui;opencv_gpu;opencv_flann;opencv_features2d;opencv_core;opencv_contrib;opencv_calib3d"
  IMPORTED_LOCATION_RELEASE "/home/lpuig/PWP3D/Alejandro/object_tracking/external/PWP3D/install/lib/libpwp3d.so"
  IMPORTED_SONAME_RELEASE "libpwp3d.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS pwp3d )
list(APPEND _IMPORT_CHECK_FILES_FOR_pwp3d "/home/lpuig/PWP3D/Alejandro/object_tracking/external/PWP3D/install/lib/libpwp3d.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
