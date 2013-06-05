// -----------------------------------------------------------------------------
/**
 *  @brief 2D affine motion model (function) in direct methods tracking.
 *  @author Jose M. Buenaposada
 *  @date 2012/07/25
 *  @version $revision$
 *
 *  $id$
 *
 *  Grupo de investigación en Percepción Computacional y Robótica)
 *  (Perception for Computers & Robots research Group)
 *  Facultad de Informática (Computer Science School)
 *  Universidad Politécnica de Madrid (UPM) (Madrid Technical University)
 *  http://www.dia.fi.upm.es/~pcr
 *
 */
// -----------------------------------------------------------------------------

#include "affine_2d.hpp"
#include "trace.hpp"
#include <limits>

namespace upm { namespace pcr
{
  
// -----------------------------------------------------------------------------
//
// Purpose and Method: 
// Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
// -----------------------------------------------------------------------------  
Affine2D::Affine2D
  () 
{ 
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
Affine2D::~Affine2D
  () 
{
};

// -----------------------------------------------------------------------------
//
// Purpose and Method: 
// Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
// -------------------------------------------------------------------------  
// cv::Mat
// Affine2D::computeMotionJacobian
//   (
//   cv::Mat params  
//   )
// {
// };
  
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
Affine2D::scaleInputImageResolution
  (
  cv::Mat params,
  double scale
  )
{
  cv::Mat new_params; 
  
  params.copyTo(new_params);
    
  new_params *= scale;
  
  return new_params;
}

// -----------------------------------------------------------------------------
//
// Purpose and Method: 
// Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
//    The motion params are always from template to current image and with this
//    method we have to use the inverse motion model.
// -----------------------------------------------------------------------------  
cv::Mat
Affine2D::transformCoordsToTemplate
  (
  cv::Mat coords,
  cv::Mat params   
  )
{
  MAT_TYPE tx = params.at<MAT_TYPE>(0, 0);
  MAT_TYPE ty = params.at<MAT_TYPE>(1, 0);
  MAT_TYPE a  = params.at<MAT_TYPE>(2, 0);
  MAT_TYPE b  = params.at<MAT_TYPE>(3, 0);
  MAT_TYPE c  = params.at<MAT_TYPE>(4, 0);
  MAT_TYPE d  = params.at<MAT_TYPE>(5, 0);
 
  assert(coords.cols == 2); // We need two dimensional coordinates
  
  cv::Mat A = (cv::Mat_<MAT_TYPE>(6,1) << a, b,  
                                          c, d);
  A = A.inv();                                        

  cv::Mat inv_params = (cv::Mat_<MAT_TYPE>(6,1) << -tx, -ty, 
			                           A.at<MAT_TYPE>(0,0), 
						   A.at<MAT_TYPE>(1,0), 
			                           A.at<MAT_TYPE>(0,1), 
			                           A.at<MAT_TYPE>(1,1));
    
  return transformCoordsToImage(coords, inv_params);  
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
Affine2D::transformCoordsToImage
  (
  cv::Mat coords,
  cv::Mat params   
  )
{
  MAT_TYPE tx = params.at<MAT_TYPE>(0, 0);
  MAT_TYPE ty = params.at<MAT_TYPE>(1, 0);
  MAT_TYPE a  = params.at<MAT_TYPE>(2, 0);
  MAT_TYPE b  = params.at<MAT_TYPE>(3, 0);
  MAT_TYPE c  = params.at<MAT_TYPE>(4, 0);
  MAT_TYPE d  = params.at<MAT_TYPE>(5, 0);
  
  assert(coords.cols == 2); // We need two dimensional coordinates
  
  cv::Mat A = (cv::Mat_<MAT_TYPE>(2,2) << a, c, 
		                          b, d);
  
  cv::Mat new_coords    = (coords * A.t()); 
  
  cv::Mat trans         = (cv::Mat_<MAT_TYPE>(1,2) << tx, ty);
  cv::Mat translations  = cv::repeat(trans, new_coords.rows, 1);
  new_coords            = new_coords + translations;
  
#ifdef DEBUG
  // write Mat objects to the file
  cv::FileStorage fs("transformCoordsToImage.xml", cv::FileStorage::WRITE);
  fs << "new_coords" << new_coords;
  fs << "trans" << trans;
  fs << "translations" << translations;
  fs.release();
#endif
  
  return new_coords;  
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
Affine2D::warpImage
  (
  cv::Mat image,
  cv::Mat params,
  cv::Mat template_coords,
  std::vector<int>& template_ctrl_points_indices
  )
{
  MAT_TYPE min_x, max_x, min_y, max_y;
  cv::Mat warped_image;

  MAT_TYPE tx = params.at<MAT_TYPE>(0, 0);
  MAT_TYPE ty = params.at<MAT_TYPE>(1, 0);
  MAT_TYPE a  = params.at<MAT_TYPE>(2, 0);
  MAT_TYPE b  = params.at<MAT_TYPE>(3, 0);
  MAT_TYPE c  = params.at<MAT_TYPE>(4, 0);
  MAT_TYPE d  = params.at<MAT_TYPE>(5, 0);
  
  cv::Mat M;
  cv::Mat A = (cv::Mat_<MAT_TYPE>(3,3) << a,   c, tx,  
		                          b,   d, ty, 
					  0.,  0., 1.);
  
  // Find minimum x and minimum y in template coords
  cv::MatConstIterator_<MAT_TYPE> it;
  max_x = std::numeric_limits<MAT_TYPE>::min();
  max_y = std::numeric_limits<MAT_TYPE>::min();
  min_x = std::numeric_limits<MAT_TYPE>::max();
  min_y = std::numeric_limits<MAT_TYPE>::max();

  for (int i = 0; i < template_coords.rows; i++)
  {  
    MAT_TYPE x = template_coords.at<MAT_TYPE>(i,0);
    MAT_TYPE y = template_coords.at<MAT_TYPE>(i,1);
    
    if (x > max_x) max_x = x;
    if (x < min_x) min_x = x;
    if (y > max_y) max_y = y;
    if (y < min_y) min_y = y;
  }

  cv::Mat TR  = (cv::Mat_<MAT_TYPE>(3,3) << 1.,    0,    min_x, 
		                            0,    1.,    min_y, 
		                            0,     0,    1.);
  warped_image = cv::Mat::zeros(max_y-min_y+1, max_x-min_x+1, cv::DataType<uint8_t>::type);
  
//   if (scale < 0.000000001)
//   {
//     return warped_image;
//   }
  
  // TR is necessary because the Warpers do warping taking
  // the pixel (0,0) as the left and top most pixel of the template.
  // So, we have move the (0,0) to the center of the Template.
  M = A*TR;

#ifdef DEBUG
  // write Mat objects to the file
  cv::FileStorage fs("template_coords_warpImage.xml", cv::FileStorage::WRITE);
  fs << "template_coords" << template_coords;
  fs << "M" << M;
  fs.release();
#endif  

  cv::warpAffine(image, warped_image, M(cv::Range(0, 2), cv::Range::all()), 
		 cv::Size(warped_image.cols,  warped_image.rows), 
		 cv::INTER_AREA | cv::WARP_INVERSE_MAP);
  return warped_image;
};

}; }; // namespace
