// -----------------------------------------------------------------------------
/**
 *  @brief Utilities 
 *  @author Jose Miguel Buenaposada
 *  @date 2012/01
 *  @version $Id$
 *
 *  Escuela Técnica Superior de Ingeniería Informática (Computer Science School)
 *  Universidad Rey Juan Carlos (Spain) 
 *  http://www.etsii.urjc.es
 *
 */
// -----------------------------------------------------------------------------

// ------------------ RECURSION PROTECTION -------------------------------------
#ifndef _UTILS_H_
#define _UTILS_H_

// ----------------------- INCLUDES --------------------------------------------
#include <opencv/cv.h> // OpenCV IplImage
#include <opencv/highgui.h> // OpenCV IplImage
#include "img_align_lib.hpp"

namespace upm { namespace pcr {

/**
 *  @brief Computes the grey levels gradients of a an image
 * 
 *  The input image is converted first to gray levels if it is a color one.
 *  The returned gradients matrix G. has the horizontal gradient (X direction) in 
 *  the first column and the vertical direction gradient (Y direction) in the 
 *  second column. The image gradients are stored in row-wise order in the gradients 
 *  matrix results.
 *  
 *  @param image 
 *  @return A matrix with X gradient in column 0 and Y gradient in column 1
 */
cv::Mat 
computeGrayImageGradients
  (
  cv::Mat image
  );

/**
 *  @brief Computes the grey levels second order derivative of a an image
 * 
 *  The input image is converted first to gray levels if it is a color one.
 *  The image gray levels second derivatives are stored in row-wise order in 
 *  each column of the results matrix H.
 * 
 *  It computes the  \frac{\partial^2 I(\vx)}{\partial \vx \partial \vx} (the gradient of the gradients).
 *  
 *  The size of the output matrix is Nx4 being N the number of pixels.
 *  The first column of the results matrix has is I_xx, x derivative of the x gradient.
 *  The second column of the results matrix has is I_xy, x derivative of the y gradient.
 *  The third column of the results matrix has is I_yx, x derivative of the x gradient.
 *  The fourth column of the results matrix has is I_yy, x derivative of the x gradient.
 *  
 *  @param image 
 *  @return An Nx4 matrix with the image second order derivatives of gray levels.
 */
cv::Mat 
computeGrayImageHessians
  (
  cv::Mat image
  );

}}; // namespace

#endif
