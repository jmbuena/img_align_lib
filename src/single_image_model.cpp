// -----------------------------------------------------------------------------
/**
 *  @brief Single image object model in direct methods tracking.
 *  @author Jose M. Buenaposada
 *  @date 2012/07/13
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

#include "single_image_model.hpp"
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
SingleImageModel::SingleImageModel
  (
  cv::Mat& template_image,
  bool equalize
  ) 
{
  cv::Mat im_gray;
  int num_pixels = template_image.rows*template_image.cols;
  
  assert(template_image.rows != 0);
  assert(template_image.cols != 0);
    
  if (template_image.channels() > 1)
  {    
    cv::cvtColor(template_image, im_gray, CV_RGB2GRAY, 1);
    m_template_image = im_gray;
  }
  else
  {
    m_template_image = template_image.clone();    
  }

  cv::GaussianBlur(m_template_image, m_template_image, cv::Size(5,5), 1.5, 1.5);

  m_equalize         = equalize;
  if (m_equalize)
  {
    cv::equalizeHist(m_template_image, m_template_image);   
  }
  
//   cv::imwrite("template_image_equalized.bmp", m_template_image);  
  m_gradients            = computeGrayImageGradients(m_template_image);
  m_template_image.reshape(1, num_pixels).convertTo(m_template_gray_levels, cv::DataType<MAT_TYPE>::type);
  m_template_coordinates = computeTemplateCoordinates(m_template_image);    
  
  m_control_points_indices.push_back(0);
  m_control_points_indices.push_back(m_template_image.cols-1);
  m_control_points_indices.push_back(m_template_image.cols * m_template_image.rows -1);
  m_control_points_indices.push_back((m_template_image.cols * (m_template_image.rows-1)));  

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
SingleImageModel::~SingleImageModel
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
SingleImageModel::computeTemplateFeaturesGradient
  (
  cv::Mat object_params
  )
{ 
  return m_gradients;
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
SingleImageModel::extractFeaturesFromWarpedImage
  (
  cv::Mat warped_image
  )
{
  // We don't do anything special with the warped image (nor DCT, nor Borders, etc).
  assert(warped_image.channels() == 1);
  int num_pixels = warped_image.cols * warped_image.rows;

  cv::Mat warped_image_vector;
  if (m_equalize)
  {
    cv::Mat equalized_image;
    cv::equalizeHist(warped_image, equalized_image);   
    equalized_image.reshape(1, num_pixels).convertTo(warped_image_vector, cv::DataType<MAT_TYPE>::type);
  }
  else
  {    
    warped_image.reshape(1, num_pixels).convertTo(warped_image_vector, cv::DataType<MAT_TYPE>::type);   
  }

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
SingleImageModel::computeTemplateFeatures
  (
  cv::Mat object_params
  )
{
  cv::Mat gray_levels;
  m_template_gray_levels.convertTo(gray_levels, cv::DataType<MAT_TYPE>::type);
  
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
void
SingleImageModel::getCtrlPointsIndices
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
SingleImageModel::getReferenceCoords
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
SingleImageModel::computeTemplateCoordinates
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
