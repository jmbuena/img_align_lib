#------------------------------------------------------------------------------
# Tell cmake where to find the project cmake scripts
#------------------------------------------------------------------------------
PROJECT(img_align_lib)
cmake_minimum_required(VERSION 2.8)

#-- Where to look first for cmake modules
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

# ------ VERSION INFORMATION -----------
set(MY_PRODUCT_NUMBER 0)  # Increase when making a revolutionary API change (e.g. change the object model classes interface)
set(MY_PRODUCT_VERSION 0) # Increase when adding a new object model, alignment algorithm, etc. to the framework (without API change). 
set(MY_BUILD_NUMBER 1)    # Increase when fixing a an error.

# configure_file(
#   ${CMAKE_CURRENT_SOURCE_DIR}/version.rc.in
#   ${CMAKE_CURRENT_BINARY_DIR/version.rc
#   @ONLY)
# ------- END VERSION INFORMATION ------

### ----------------- CPP CONFIGURATION -----------------------------------------

# Allows generated files in the current directed to be found by the compiler
#SET(CMAKE_INCLUDE_CURRENT_DIR ON)
# Remove deprecation warnings
set(CMAKE_WARN_DEPRECATED FALSE)
# Full compiler output
set(CMAKE_VERBOSE_MAKEFILE ON)
# C++11 and beyond
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#-- Type of build
#SET(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_BUILD_TYPE Release)
#SET(CMAKE_BUILD_TYPE RelwithDebugInfo)

# REQUIRED: stops processing with an error if the package cannot be found.
FIND_PACKAGE(OpenCV REQUIRED)
MESSAGE("OpenCV information:")
MESSAGE("   > OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
MESSAGE("   > OpenCV_LIB_DIR: ${OpenCV_LIB_DIR}")
MESSAGE("   > OpenCV_LIBS: ${OpenCV_LIBS}")

FIND_PACKAGE(Boost REQUIRED)
MESSAGE("Boost information:")
MESSAGE("   > Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
MESSAGE("   > Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")
MESSAGE("   > Boost_LIBRARIES: ${Boost_LIBRARIES}")

#INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} C:/OpenCV2.1/include E:/Archivos\ de\ programa/boost/boost_1_44 /usr/include .)
#LINK_DIRECTORIES( ${OpenCV_LINKS_DIR} /usr/lib )

ADD_LIBRARY(img_align SHARED 
  image_pca_factorized_problem.hpp
  image_pca_factorized_problem.cpp
  similarity_2d_image_pca_factorized_problem.hpp
  similarity_2d_image_pca_factorized_problem.cpp
  affine_2d_image_pca_factorized_problem.hpp
  affine_2d_image_pca_factorized_problem.cpp
  image_pca_factorized_problem.hpp
  image_pca_factorized_problem.cpp
  utils.hpp
  utils.cpp
  viewer.hpp
  viewer.cpp
  optimizer.hpp
  optimizerGN.hpp
  optimizerGN.cpp
  optimization_problem.hpp
  factorized_jacobian_problem.hpp
  factorized_jacobian_problem.cpp
  similarity_2d_factorized_problem.hpp
  similarity_2d_factorized_problem.cpp
  affine_2d_factorized_problem.hpp
  affine_2d_factorized_problem.cpp
  homography_2d_factorized_problem.hpp
  homography_2d_factorized_problem.cpp
  inverse_compositional_problem.hpp
  inverse_compositional_problem.cpp
  similarity_2d_grad_corr_inv_comp_problem.hpp
  similarity_2d_grad_corr_inv_comp_problem.cpp
  object_model.hpp
  homography_2d.hpp
  homography_2d.cpp
  affine_2d.hpp
  affine_2d.cpp
  similarity_2d.hpp
  similarity_2d.cpp
  motion_model.hpp
  single_image_model.hpp
  single_image_model.cpp
  single_image_gradients_model.hpp
  single_image_gradients_model.cpp
  image_pca_model.hpp
  image_pca_model.cpp
  planar_object_detector.hpp
  planar_object_detector.cpp
  tracker.hpp
  tracker.cpp
  img_align_lib.hpp
  )
TARGET_LINK_LIBRARIES(img_align ${OpenCV_LIBS})

#SET_TARGET_PROPERTIES(img_align PROPERTIES PUBLIC_HEADER "tracklib.hpp")
SET_TARGET_PROPERTIES(img_align
  PROPERTIES
  VERSION ${MY_PRODUCT_NUMBER}.${MY_PRODUCT_VERSION}.${MY_BUILD_NUMBER}
    SOVERSION ${MY_PRODUCT_NUMBER})


ADD_EXECUTABLE(test_img_align
  tracker.hpp
  img_align_lib.hpp
  test_img_align.cpp
  )
TARGET_LINK_LIBRARIES(test_img_align
  ${OpenCV_LIBS}   
  jpeg
  tiff
  boost_program_options
  img_align
)

ADD_EXECUTABLE(test_pca_model_align
  tracker.hpp
  img_align_lib.hpp
  test_pca_model_align.cpp
  )
TARGET_LINK_LIBRARIES(test_pca_model_align
  ${OpenCV_LIBS}   
  jpeg
  tiff
  img_align
)


#-------------------------------------------------------------------
# Process the doxygen (documentation) directory.
#-------------------------------------------------------------------
#INCLUDE(FindDoxygen)
#IF (DOXYGEN)
#  ADD_SUBDIRECTORY(doc)
#ELSE (DOXYGEN)
#  MESSAGE(STATUS "WARNING: Doxygen not found - Reference manual will not be created")
#ENDIF (DOXYGEN)

#-------------------------------------------------------------------
# Process the test directory.
#-------------------------------------------------------------------
#ADD_SUBDIRECTORY(tests)

