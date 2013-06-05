# ===================================================================================
#  The OpenCV CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(OpenCV REQUIRED )
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME ${OpenCV_LIBS})
#
#    This file will define the following variables:
#      - OpenCV_LIBS                 : The list of libraries to links against.
#      - OpenCV_LIB_DIR              : The directory where lib files are. Calling LINK_DIRECTORIES
#                                      with this path is NOT needed.
#      - OpenCV_INCLUDE_DIRS         : The OpenCV include directories.
#      - OpenCV_COMPUTE_CAPABILITIES : The version of compute capability
#      - OpenCV_VERSION              : The version of this OpenCV build. Example: "1.2.0"
#      - OpenCV_VERSION_MAJOR        : Major version part of OpenCV_VERSION. Example: "1"
#      - OpenCV_VERSION_MINOR        : Minor version part of OpenCV_VERSION. Example: "2"
#      - OpenCV_VERSION_PATCH        : Patch version part of OpenCV_VERSION. Example: "0"
#
# =================================================================================================

# ======================================================
# Version Compute Capability from which library OpenCV
# has been compiled is remembered
# ======================================================
SET(OpenCV_COMPUTE_CAPABILITIES -gencode;arch=compute_11,code=sm_11;-gencode;arch=compute_12,code=sm_12;-gencode;arch=compute_13,code=sm_13;-gencode;arch=compute_20,code=sm_20;-gencode;arch=compute_20,code=sm_21;-gencode;arch=compute_20,code=compute_20)

# Extract the directory where *this* file has been installed (determined at cmake run-time)
#  This variable may or may not be used below, depending on the parsing of OpenCVConfig.cmake
get_filename_component(THIS_OPENCV_CONFIG_PATH "${CMAKE_CURRENT_LIST_FILE}" PATH)

# ======================================================
# Include directories to add to the user project:
# ======================================================

# Provide the include directories to the caller
SET(OpenCV_INCLUDE_DIRS "/usr/include" "/usr/include/opencv")

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# ======================================================
# Link directories to add to the user project:
# ======================================================

# Provide the libs directory anyway, it may be needed in some cases.
SET(OpenCV_LIB_DIR /usr/lib)

LINK_DIRECTORIES(${OpenCV_LIB_DIR})

# ====================================================================
# Link libraries: e.g.   opencv_core220.so, opencv_imgproc220d.lib, etc...
# ====================================================================
if(NOT ANDROID)
    set(OPENCV_LIB_COMPONENTS opencv_core opencv_imgproc opencv_features2d opencv_gpu opencv_calib3d opencv_objdetect opencv_video opencv_highgui opencv_ml opencv_legacy opencv_contrib opencv_flann)
else()
    #libraries order is very important because linker from Android NDK is one-pass linker
    set(OPENCV_LIB_COMPONENTS opencv_contrib opencv_calib3d opencv_objdetect opencv_features2d opencv_imgproc opencv_video  opencv_highgui opencv_ml opencv_legacy  opencv_flann opencv_core )
    IF (NOT ON)
        set(OPENCV_LIB_COMPONENTS ${OPENCV_LIB_COMPONENTS} opencv_androidcamera)
    ENDIF()
endif()

SET(OpenCV_LIBS "")
foreach(__CVLIB ${OPENCV_LIB_COMPONENTS})
    # CMake>=2.6 supports the notation "debug XXd optimized XX"
    if (CMAKE_MAJOR_VERSION GREATER 2  OR  CMAKE_MINOR_VERSION GREATER 4)
        # Modern CMake:
        SET(OpenCV_LIBS ${OpenCV_LIBS} debug ${__CVLIB} optimized ${__CVLIB})
    else(CMAKE_MAJOR_VERSION GREATER 2  OR  CMAKE_MINOR_VERSION GREATER 4)
        # Old CMake:
        SET(OpenCV_LIBS ${OpenCV_LIBS} ${__CVLIB})
    endif(CMAKE_MAJOR_VERSION GREATER 2  OR  CMAKE_MINOR_VERSION GREATER 4)
endforeach(__CVLIB)

# ==============================================================
#  Extra include directories, needed by OpenCV 2 new structure
# ==============================================================
if(NOT "/home/jmbuena/PROYECTOS/OpenCV/OpenCV-2.3.0" STREQUAL  "")
    SET(BASEDIR "/home/jmbuena/PROYECTOS/OpenCV/OpenCV-2.3.0")
    foreach(__CVLIB ${OPENCV_LIB_COMPONENTS})
        # We only need the "core",... part here: "opencv_core" -> "core"
        STRING(REGEX REPLACE "opencv_(.*)" "\\1" MODNAME ${__CVLIB})
        INCLUDE_DIRECTORIES("${BASEDIR}/modules/${MODNAME}/include")
    endforeach(__CVLIB)
endif(NOT "/home/jmbuena/PROYECTOS/OpenCV/OpenCV-2.3.0" STREQUAL  "")

# For OpenCV built as static libs, we need the user to link against
#  many more dependencies:
set(OpenCV_SHARED ON)

IF (NOT OpenCV_SHARED)
    # Under static libs, the user of OpenCV needs access to the 3rdparty libs as well:
    if(WIN32 AND NOT ANDROID)
        LINK_DIRECTORIES("/home/jmbuena/PROYECTOS/OpenCV/OpenCV-2.3.0"/3rdparty/lib)
    else()
        LINK_DIRECTORIES("${THIS_OPENCV_CONFIG_PATH}/3rdparty/lib")
    endif()    

    set(OpenCV_LIBS dl;m;pthread;rt  gtk-x11-2.0;gdk-x11-2.0;atk-1.0;gio-2.0;pangoft2-1.0;pangocairo-1.0;gdk_pixbuf-2.0;cairo;pango-1.0;freetype;fontconfig;gobject-2.0;gmodule-2.0;gthread-2.0;rt;glib-2.0;gthread-2.0;rt;glib-2.0;avcodec;avformat;avutil;swscale;dc1394;v4l1 ${OpenCV_LIBS})

    set(OPENCV_EXTRA_COMPONENTS /usr/lib/x86_64-linux-gnu/libjpeg.so /usr/lib/x86_64-linux-gnu/libpng.so;/usr/lib/x86_64-linux-gnu/libz.so /usr/lib/x86_64-linux-gnu/libtiff.so /usr/lib/x86_64-linux-gnu/libjasper.so;/usr/lib/x86_64-linux-gnu/libjpeg.so zlib)

    if (CMAKE_MAJOR_VERSION GREATER 2  OR  CMAKE_MINOR_VERSION GREATER 4)
        foreach(__EXTRA_LIB ${OPENCV_EXTRA_COMPONENTS})
            set(OpenCV_LIBS ${OpenCV_LIBS}
                debug ${__EXTRA_LIB}
                optimized ${__EXTRA_LIB})
        endforeach(__EXTRA_LIB)
    else(CMAKE_MAJOR_VERSION GREATER 2  OR  CMAKE_MINOR_VERSION GREATER 4)
        set(OpenCV_LIBS ${OpenCV_LIBS} ${OPENCV_EXTRA_COMPONENTS})
    endif(CMAKE_MAJOR_VERSION GREATER 2  OR  CMAKE_MINOR_VERSION GREATER 4)

ENDIF()

# ======================================================
#  Android camera helper macro
# ======================================================
IF (ANDROID)
  macro( COPY_NATIVE_CAMERA_LIBS target )
    IF( ARMEABI_V7A)
      get_target_property(target_location ${target} LOCATION)
      get_filename_component(target_location "${target_location}" PATH)
      file(GLOB camera_wrappers "${OpenCV_LIB_DIR}/libnative_camera_r*.so")
      foreach(wrapper ${camera_wrappers})
          ADD_CUSTOM_COMMAND(
            TARGET ${target}
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy "${wrapper}" "${target_location}"
            )
      endforeach()
    ENDIF( ARMEABI_V7A )
  endmacro()
ENDIF(ANDROID)

# ======================================================
#  Version variables:
# ======================================================
SET(OpenCV_VERSION 2.3.0)
SET(OpenCV_VERSION_MAJOR  2)
SET(OpenCV_VERSION_MINOR  3)
SET(OpenCV_VERSION_PATCH  0)
