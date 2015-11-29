// -----------------------------------------------------------------------------
/**
 *  @brief PCA of Images based object model in direct methods tracking.
 *  @author Jose M. Buenaposada
 *  @date 2012/08/30
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

#include "image_pca_model.hpp"
#include "utils.hpp"
#include "trace.hpp"

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
ImagePCAModel::ImagePCAModel
  (
    cv::Mat& mean, 
    size_t img_cols, 
    size_t img_rows,
    cv::Mat& B
  ):
  m_img_cols(img_cols),
  m_img_rows(img_rows)
{
  cv::Mat im_gray;
  cv::Mat gradients;
  cv::Mat mean_image_vector;
  int num_pixels = img_cols * img_rows;
   
  assert(mean.rows > mean.cols);
  assert(mean.rows == img_cols * img_rows);
  assert(B.rows == mean.rows);

  m_B               = B;
  m_invB            = B.inv(cv::DECOMP_SVD);
  m_mean            = mean.clone();

  m_mean_image      = cv::Mat::zeros(img_rows, img_cols, cv::DataType<MAT_TYPE>::type);
  mean_image_vector = m_mean_image.reshape(1, img_rows*img_cols);
  m_mean.convertTo(mean_image_vector, cv::DataType<MAT_TYPE>::type);

#ifdef DEBUG
  cv::imwrite("mean_image.jpg", m_mean_image);
#endif

  // Compute the gradients for each basis vector/image and also for the mean image
  gradients = computeGrayImageGradients(m_mean_image);
  m_gradients_vector.push_back(gradients);
  
#ifdef DEBUG
  // write Mat objects to the file
  cv::FileStorage fs("ImagePCAModel_constructor.xml", cv::FileStorage::WRITE);
  fs << "mean_image_gradients" << gradients;
#endif   

  cv::Mat B_ith_image = cv::Mat::zeros(img_rows, img_cols, cv::DataType<MAT_TYPE>::type);
  for (int i=0; i < B.cols; i++)
  {
    cv::Mat B_ith_image_vector = B_ith_image.reshape(1, img_rows*img_cols);
    cv::Mat B_ith              = B.col(i);
    B_ith.copyTo(B_ith_image_vector);
    
    gradients                  = computeGrayImageGradients(B_ith_image);
    m_gradients_vector.push_back(gradients);
#ifdef DEBUG  
    // write Mat objects to the file
    fs << "B_" << i << "th" << gradients;

    cv::Mat normalized;
    cv::normalize(B_ith_image, normalized, 0, 255, cv::NORM_MINMAX, cv::DataType<uint8_t>::type);
    std::ostringstream out;
    out << "B_" << i << ".bmp";
    cv::imwrite(out.str(), normalized);
#endif
  }
#ifdef DEBUG
  fs.release();
#endif   

  m_template_coordinates = computeTemplateCoordinates(m_mean_image);    
  m_control_points_indices.push_back(0);
  m_control_points_indices.push_back(m_img_cols-1);
  m_control_points_indices.push_back(m_img_cols * m_img_rows -1);
  m_control_points_indices.push_back((m_img_cols * (m_img_rows-1)));  

  for (int i=0; i < 4; i++)
  {
    LineIndices indices; 
    indices.p1_index = m_control_points_indices[i];
    indices.p2_index = m_control_points_indices[(i+1) % 4];
    m_control_points_lines.push_back(indices);    
  }
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
ImagePCAModel::~ImagePCAModel
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
cv::Mat
ImagePCAModel::computeTemplateFeaturesGradient
  (
  cv::Mat object_params
  )
{ 
	assert(0);
	return cv::Mat();
   //return m_gradients;
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
ImagePCAModel::extractFeaturesFromWarpedImage
  (
  cv::Mat warped_image
  )
{
  // We don't do anything special with the warped image (nor DCT, nor Borders, etc).
  assert(warped_image.channels() == 1);
  int num_pixels = warped_image.cols * warped_image.rows;

  cv::Mat warped_image_vector;
  warped_image.reshape(1, num_pixels).convertTo(warped_image_vector, cv::DataType<MAT_TYPE>::type);   
 
  return warped_image_vector;
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
ImagePCAModel::computeTemplateFeatures
  (
  cv::Mat object_params
  )
{
  cv::Mat gray_levels;

  assert(object_params.rows == m_B.cols);
  assert(object_params.cols == 1);
  assert(object_params.rows > object_params.cols);
  
  gray_levels = m_mean + (m_B*object_params);
    
#ifdef DEBUG
  cv::Mat normalized;
  cv::Mat gray_levels_image = gray_levels.reshape(1, m_img_rows);
  cv::normalize(gray_levels_image, normalized, 0, 255, cv::NORM_MINMAX, cv::DataType<uint8_t>::type);
  cv::imwrite("reconstructed_image.png", normalized);
#endif
  
  return gray_levels;
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
std::vector<cv::Mat>
ImagePCAModel::computeModelGradients
  ()
{
  return m_gradients_vector;     
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
void
ImagePCAModel::getCtrlPointsIndices
  (
  std::vector<int>& control_points_indices,
  std::vector<LineIndices>& control_points_lines 
  )
{
  control_points_indices = m_control_points_indices; 
  control_points_lines   = m_control_points_lines; 

#ifdef DEBUG
  // write Mat objects to the file
  cv::FileStorage fs("template_coordinates_getReferenceCoords.xml", cv::FileStorage::WRITE);
  fs << "m_template_coordinates" << m_template_coordinates;
  fs << "reference_coords" << reference_coords;
  fs.release();
#endif  
};

// -----------------------------------------------------------------------------
//
// Purpose and Method: 
// Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
//  We return coordinates centred in the middle of the image template (the center
//  has coordinates (0,0). This makes a similarity motion model tracker to work
//  right.
//
// -----------------------------------------------------------------------------
void
ImagePCAModel::getReferenceCoords
  (
  cv::Mat& reference_coords
  )
{
  m_template_coordinates.convertTo(reference_coords, cv::DataType<MAT_TYPE>::type);
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
ImagePCAModel::computeTemplateCoordinates
  (
  cv::Mat gray_image
  )
{
  assert(gray_image.channels() == 1);

  MAT_TYPE x, y;
  int index;
  int num_pixels   = gray_image.rows*gray_image.cols;
  int width_div_2  = round(static_cast<MAT_TYPE>(gray_image.cols)/2.0);
  int height_div_2 = round(static_cast<MAT_TYPE>(gray_image.rows)/2.0);
      
  cv::Mat coordinates = cv::Mat::zeros(num_pixels, 2, cv::DataType<MAT_TYPE>::type);
 
  for (int i = 0; i < gray_image.rows; i++)
  {
    for (int j = 0; j < gray_image.cols; j++)
    {
      // By rows
      index                             = (i * gray_image.cols) + j;
      coordinates.at<MAT_TYPE>(index,0) = static_cast<MAT_TYPE>(j - width_div_2); 
      coordinates.at<MAT_TYPE>(index,1) = static_cast<MAT_TYPE>(i - height_div_2);
    }
  }

#ifdef DEBUG
  // write Mat objects to the file
  cv::FileStorage fs("template_coordinates.xml", cv::FileStorage::WRITE);
  fs << "coordinates" << coordinates;
  fs.release();
#endif
  
  return coordinates;
}
    

}; }; // namespace
