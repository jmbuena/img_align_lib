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

// ----------------------- INCLUDES --------------------------------------------
#include <opencv/cv.h>
#include "utils.hpp"
#include "trace.hpp"

namespace upm { namespace pcr {
  
// -----------------------------------------------------------------------------
//
// Purpose and Method: 
// Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
// -----------------------------------------------------------------------------  
cv::Mat 
computeGrayImageGradients
  (
  cv::Mat image
  )
{  
  int num_pixels;
  cv::Mat gradients; 
  cv::Mat grad_x, grad_y;
  cv::Mat gray_image;
  double G[3]       = {0.3078, 0.3844, 0.3078}; // Gaussian
  double DoG[3]     = {-0.5, 0.0, 0.5}; // Derivative of a Gaussian
  int ddepth        = CV_32F;
    
  if (image.channels() > 1)
  {    
    cv::cvtColor(image, gray_image, CV_RGB2GRAY, 1);
  }
  else
  {
    gray_image = image.clone();    
  }
  
  num_pixels = gray_image.rows*gray_image.cols;
  gradients  = cv::Mat::zeros(num_pixels, 2, cv::DataType<MAT_TYPE>::type); 

  // Gradient X
  cv::sepFilter2D( gray_image, grad_x, ddepth, cv::Mat(1, 3, CV_64F, DoG), cv::Mat(3, 1, CV_64F, G));

  // Gradient Y
  cv::sepFilter2D( gray_image, grad_y, ddepth, cv::Mat(1, 3, CV_64F, G), cv::Mat(3, 1, CV_64F, DoG));

  // Copy grad_x as a column vector to the first gradient column
  cv::Mat gradients0  = gradients.col(0);
  cv::Mat grad_x_col  = grad_x.reshape(1, num_pixels); // channels, rows
  // It is contertTo and not copyTo because of the different base types
  grad_x_col.convertTo(gradients0, cv::DataType<MAT_TYPE>::type);  
  
  // Copy grad_y as a column vector to the second gradient column
  cv::Mat gradients1  = gradients.col(1);
  cv::Mat grad_y_col  = grad_y.reshape(1, num_pixels); // channels, rows
  // It is contertTo and not copyTo because of the different base types
  grad_y_col.convertTo(gradients1, cv::DataType<MAT_TYPE>::type);   

#ifdef DEBUG
  // write Mat objects to the file
  cv::FileStorage fs("Utils_image_gradients.xml", cv::FileStorage::WRITE);
  fs << "gray_image" << gray_image;
  fs << "gradients" << gradients;
  fs << "grad_x" << grad_x;
  fs << "grad_y" << grad_y;
  fs.release();
#endif
 
  return gradients;
};

// -----------------------------------------------------------------------------
//
// Purpose and Method: 
// Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
// -----------------------------------------------------------------------------  
cv::Mat 
computeGrayImageHessians
  (
  cv::Mat image
  )
{  
  int num_pixels;
  cv::Mat hessians; 
  cv::Mat grad_x, grad_y;
  cv::Mat grad_xx, grad_xy, grad_yy; //grad_yx
  cv::Mat gray_image;
  double G[3]       = {0.3078, 0.3844, 0.3078}; // Gaussian
  double DoG[3]     = {-0.5, 0.0, 0.5}; // Derivative of a Gaussian
  int ddepth        = CV_32F;
  std::vector<cv::Mat> hessians_vector;
  
  if (image.channels() > 1)
  {    
    cv::cvtColor(image, gray_image, CV_RGB2GRAY, 1);
  }
  else
  {
    gray_image = image.clone();    
  }
  
  num_pixels = gray_image.rows*gray_image.cols;
  hessians   = cv::Mat::zeros(num_pixels, 4, cv::DataType<MAT_TYPE>::type); 

  //-----------------------
  // Gradients on X and Y
  //-----------------------
  cv::sepFilter2D( gray_image, grad_x, ddepth, cv::Mat(1, 3, CV_64F, DoG), cv::Mat(3, 1, CV_64F, G));
  cv::sepFilter2D( gray_image, grad_y, ddepth, cv::Mat(1, 3, CV_64F, G), cv::Mat(3, 1, CV_64F, DoG));

  // Hessian XX
  cv::sepFilter2D( grad_x, grad_xx, ddepth, cv::Mat(1, 3, CV_64F, DoG), cv::Mat(3, 1, CV_64F, G));
  hessians_vector.push_back(grad_xx);

  // Hessian XY
  cv::sepFilter2D( grad_y, grad_xy, ddepth, cv::Mat(1, 3, CV_64F, DoG), cv::Mat(3, 1, CV_64F, G));
  hessians_vector.push_back(grad_xy);

//   // Hessian YX
//   cv::sepFilter2D( grad_x, grad_yx, ddepth, cv::Mat(1, 3, CV_64F, G), cv::Mat(3, 1, CV_64F, DoG));
//   hessians_vector.push_back(grad_xy);

  // Hessian YY
  cv::sepFilter2D( grad_y, grad_yy, ddepth, cv::Mat(1, 3, CV_64F, G), cv::Mat(3, 1, CV_64F, DoG));
  hessians_vector.push_back(grad_yy);
  
  for (int i=0; i<hessians_vector.size(); i++)
  {
    // Copy hessian as a column vector to the first hessian column
    cv::Mat col   = hessians.col(i);
    cv::Mat hess  = hessians_vector[i].reshape(1, num_pixels); // channels, rows
    // It is contertTo and not copyTo because of the different base types
    hess.convertTo(col, cv::DataType<MAT_TYPE>::type);      
  }
  
#ifdef DEBUG
  // write Mat objects to the file
  cv::FileStorage fs("Utils_image_hessians.xml", cv::FileStorage::WRITE);
  fs << "hessians" << hessians;
  fs.release();
#endif
 
  return hessians;
};


}}; // namespace
