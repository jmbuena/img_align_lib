
#------------------------------------------------------------------------------
# Tell cmake where to find the project cmake scripts
#------------------------------------------------------------------------------
PROJECT(img_align_lib)
cmake_minimum_required(VERSION 2.8)

SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)
#
#SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_BUILD_TYPE RelwithDebugInfo)
## If you want the full compiler output, uncomment the following line.
#SET(CMAKE_VERBOSE_MAKEFILE ON)
#
#ENABLE_TESTING()

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Boost REQUIRED)

INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} C:/OpenCV2.1/include E:/Archivos\ de\ programa/boost/boost_1_44 /usr/include .)
LINK_DIRECTORIES( ${OpenCV_LINKS_DIR} /usr/lib )

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
SET_TARGET_PROPERTIES(img_align PROPERTIES PUBLIC_HEADER "tracklib.hpp")

ADD_EXECUTABLE(test_img_align
  tracker.hpp
  img_align_lib.hpp
  test_img_align.cpp
  )
TARGET_LINK_LIBRARIES(test_img_align
  ${OpenCV_LIBS}   
  #jpeg
  #tiff
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
  #jpeg
  #tiff
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

