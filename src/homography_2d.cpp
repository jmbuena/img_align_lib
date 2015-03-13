// -----------------------------------------------------------------------------
/**
 *  @brief 2D homography motion model (function) in direct methods tracking.
 *  @author Jose M. Buenaposada
 *  @date 2012/10/14
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

#include "homography_2d.hpp"
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
Homography2D::Homography2D
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
Homography2D::~Homography2D
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
// Homography2D::computeMotionJacobian
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
Homography2D::scaleInputImageResolution
  (
  cv::Mat params,
  double scale
  )
{
  cv::Mat newH;
  cv::Mat new_params = cv::Mat::eye(params.rows, 1, cv::DataType<MAT_TYPE>::type);
  cv::Mat H          = params.reshape(1,3).t();
  
  cv::Mat S    = cv::Mat::eye(3,3,cv::DataType<MAT_TYPE>::type);
  S.at<MAT_TYPE>(0,0) = scale;
  S.at<MAT_TYPE>(1,1) = scale;

  newH = S*H;
  newH = newH.t();
  newH.reshape(1,9).copyTo(new_params);
  
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
Homography2D::transformCoordsToTemplate
  (
  cv::Mat coords,
  cv::Mat params   
  )
{
  cv::Mat H = params.reshape(1,3).t();
  cv::Mat invH      = H.inv().t();                                        

  cv::Mat inv_params = invH.reshape(1,9);
  
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
Homography2D::transformCoordsToImage
  (
  cv::Mat coords,
  cv::Mat params   
  )
{
  assert(coords.cols == 2); // We need two dimensional coordinates
  
  cv::Mat H                      = params.reshape(1, 3).t();
  cv::Mat homogeneous_coords     = cv::Mat::ones(coords.rows, 3, cv::DataType<MAT_TYPE>::type);
  cv::Mat homogeneous_coords_ref = homogeneous_coords(cv::Range::all(), cv::Range(0, 2));
  coords.copyTo(homogeneous_coords_ref);
  
  cv::Mat homogeneous_new_coords = (homogeneous_coords * H.t());
  
  // Divide by the third homogeneous coordinates to get the cartersian coordinates.
  for (int j=0; j<3; j++)
  {
    cv::Mat col     = homogeneous_new_coords.col(j).mul(1.0 / homogeneous_new_coords.col(2));
    cv::Mat col_new = homogeneous_new_coords.col(j);
    col.copyTo(col_new);
  }
    
  cv::Mat homogeneous_new_coords_ref = homogeneous_new_coords(cv::Range::all(), cv::Range(0, 2)); 
  cv::Mat new_coords;
  homogeneous_new_coords_ref.copyTo(new_coords);
  
#ifdef DEBUG
  // write Mat objects to the file
  cv::FileStorage fs("transformCoordsToImage.xml", cv::FileStorage::WRITE);
  fs << "H" << H;
  fs << "coords" << coords;
  fs << "new_coords" << new_coords;
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
Homography2D::warpImage
  (
  cv::Mat image,
  cv::Mat params,
  cv::Mat template_coords,
  std::vector<int>& template_ctrl_points_indices
  )
{
  MAT_TYPE min_x, max_x, min_y, max_y;
  cv::Mat warped_image;

  cv::Mat M;
  cv::Mat H = params.reshape(1,3).t();
  
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
  M = H*TR;

#ifdef DEBUG
  // write Mat objects to the file
  cv::FileStorage fs("template_coords_warpImage.xml", cv::FileStorage::WRITE);
  fs << "template_coords" << template_coords;
  fs << "M" << M;
  fs.release();
#endif  

  cv::warpPerspective(image, warped_image, M, 
		      cv::Size(warped_image.cols,  warped_image.rows), 
		      cv::INTER_AREA | cv::WARP_INVERSE_MAP);
  return warped_image;
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
bool
Homography2D::consistentPoints
  (
  cv::Mat points, 
  cv::Mat transformed_points,
  int t1,
  int t2,
  int t3
  )
{
  cv::Mat A = (cv::Mat_<MAT_TYPE>(3, 3) << points.at<MAT_TYPE>(t1, 0), points.at<MAT_TYPE>(t1, 1), 1,
                                           points.at<MAT_TYPE>(t2, 0), points.at<MAT_TYPE>(t2, 1), 1,
                                           points.at<MAT_TYPE>(t3, 0), points.at<MAT_TYPE>(t3, 1), 1);
  cv::Mat B = (cv::Mat_<MAT_TYPE>(3, 3) << transformed_points.at<MAT_TYPE>(t1, 0), transformed_points.at<MAT_TYPE>(t1, 1), 1,
                                           transformed_points.at<MAT_TYPE>(t2, 0), transformed_points.at<MAT_TYPE>(t2, 1), 1,
                                           transformed_points.at<MAT_TYPE>(t3, 0), transformed_points.at<MAT_TYPE>(t3, 1), 1);

  double detA = cv::determinant(A);
  double detB = cv::determinant(B);

  return ((detA*detB) >= 0.0);
}

// -----------------------------------------------------------------------------
//
// Purpose and Method: 
// Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
// -----------------------------------------------------------------------------  
bool
Homography2D::invalidParams
  (
  cv::Mat params
  )
{
  cv::Mat M;
  cv::Mat H = params.reshape(1,3).t();

  cv::SVD svd(H, cv::SVD::NO_UV);
  
  int rank = 0;
  for (int i=0; i<H.rows; i++)
  {
    if (svd.w.at<MAT_TYPE>(i,0)>1.E-6)
    {
      rank++;
    }
  }
  
  SHOW_VALUE(H);
  
  // Stablish the 
  cv::Mat points = (cv::Mat_<MAT_TYPE>(4, 2) <<   0, 0,
                                                100, 0,
		                                100, 100,
		                                  0, 100);
  cv::Mat transformed_points;
  points.copyTo(transformed_points);
//   cv::perspectiveTransform(points, transformed_points, H);
  transformed_points = transformCoordsToImage(points, params);

  double detH = cv::determinant(H);
  
//   return (rank<3) || (!consistentPoints(points, transformed_points, 0, 1, 2)) 
//                   || (!consistentPoints(points, transformed_points, 1, 2, 3)) 
//                   || (!consistentPoints(points, transformed_points, 0, 2, 3)) 
//                   || (!consistentPoints(points, transformed_points, 0, 1, 3)); 
  return (rank<3) ||(detH < (1./10.)) || (detH > 10.) 
                  || (!consistentPoints(points, transformed_points, 0, 1, 2)) 
                  || (!consistentPoints(points, transformed_points, 1, 2, 3)) 
                  || (!consistentPoints(points, transformed_points, 0, 2, 3)) 
                  || (!consistentPoints(points, transformed_points, 0, 1, 3)); 
};


}; }; // namespace
