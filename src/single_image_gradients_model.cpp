// -----------------------------------------------------------------------------
/**
 *  @brief Single image gradients object model in direct methods tracking.
 *  @author Jose M. Buenaposada
 *  @date 2012/09/13
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

#include "single_image_gradients_model.hpp"
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
SingleImageGradientsModel::SingleImageGradientsModel
  (
  cv::Mat& template_image,
  bool equalize /* false */
  )
  : SingleImageModel(template_image, equalize)
{
  m_hessians   = computeGrayImageHessians(m_template_image);

  m_hessians /= 255.0;
  
//   for (int i=0; i < m_hessians.rows; i++)
//   {
//     MAT_TYPE grad_x = m_gradients.at<MAT_TYPE>(0,0); 
//     MAT_TYPE grad_y = m_gradients.at<MAT_TYPE>(0,1);
//     MAT_TYPE norm   = sqrt(grad_x*grad_x + grad_y*grad_y);
//     
//     m_hessians.at<MAT_TYPE>(0,0) /= norm;
//     m_hessians.at<MAT_TYPE>(0,1) /= norm;
//     m_hessians.at<MAT_TYPE>(0,2) /= norm;
//     m_hessians.at<MAT_TYPE>(0,3) /= norm;
//   }
// 
//   for (int i=0; i < m_gradients.rows; i++)
//   {
//     MAT_TYPE grad_x = m_gradients.at<MAT_TYPE>(0,0); 
//     MAT_TYPE grad_y = m_gradients.at<MAT_TYPE>(0,1);
//     MAT_TYPE norm   = sqrt(grad_x*grad_x + grad_y*grad_y);
//     
//     m_gradients.at<MAT_TYPE>(0,0) /= norm;
//     m_gradients.at<MAT_TYPE>(0,1) /= norm;
//   }

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
SingleImageGradientsModel::~SingleImageGradientsModel
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
SingleImageGradientsModel::computeTemplateFeaturesGradient
  (
  cv::Mat object_params
  )
{ 
  return m_hessians;
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
SingleImageGradientsModel::extractFeaturesFromWarpedImage
  (
  cv::Mat warped_image
  )
{
  assert(warped_image.channels() == 1);
  int num_pixels = warped_image.cols * warped_image.rows;
  cv::Mat gradients;

  if (m_equalize)
  {
    cv::Mat equalized_image;
    cv::equalizeHist(warped_image, equalized_image);   
    gradients = computeGrayImageGradients(equalized_image);
  }
  else
  {    
    gradients = computeGrayImageGradients(warped_image);
  }

//   for (int i=0; i<gradients.rows; i++)
//   {
//     MAT_TYPE grad_x = gradients.at<MAT_TYPE>(0,0); 
//     MAT_TYPE grad_y = gradients.at<MAT_TYPE>(0,1);
//     MAT_TYPE norm   = sqrt(grad_x*grad_x + grad_y*grad_y);
//     
//     gradients.at<MAT_TYPE>(0,0) /= norm;
//     gradients.at<MAT_TYPE>(0,1) /= norm;
//   }

  gradients /= 255.0;

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
SingleImageGradientsModel::computeTemplateFeatures
  (
  cv::Mat object_params
  )
{
  return m_gradients;
};
  

}; }; // namespace
