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

// ------------------ RECURSION PROTECTION -------------------------------------
#ifndef SINGLE_IMAGE_GRADIENTS_MODEL_HPP
#define SINGLE_IMAGE_GRADIENTS_MODEL_HPP

// ----------------------- INCLUDES --------------------------------------------
#include "single_image_model.hpp"

namespace upm { namespace pcr
{
  
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to SingleImageGradientsModel type
 */
// -----------------------------------------------------------------------------
class SingleImageGradientsModel;
typedef std::shared_ptr<SingleImageGradientsModel> SingleImageGradientsModelPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to SingleImageGradientsModels 
 */
// -----------------------------------------------------------------------------
typedef std::vector<SingleImageGradientsModel> SingleImageGradientsModels;

// ----------------------------------------------------------------------------
/**
 * @class SingleImageGradientsModel
 * @brief A class that defines single template image gradients as target model.
 */
// -----------------------------------------------------------------------------
class SingleImageGradientsModel: public SingleImageModel
{
public:

  SingleImageGradientsModel
    (
    cv::Mat& template_image,
    bool equalize = false
    );

  virtual ~SingleImageGradientsModel
    ();
    
  /**
   *  @brief Computes the grey levels second derivative of a template image in the template model.
   *  
   *  It computes the  \frac{\partial^2 I(\vx)}{\partial \vx \partial \vx} (the gradient of the gradients).
   *  The size of the output matrix is Nx4 being N the number of pixels.
   * 
   *  The first column of the results matrix has is I_xx, x derivative of the x gradient.
   *  The second column of the results matrix has is I_xy, x derivative of the y gradient.
   *  The third column of the results matrix has is I_yx, x derivative of the x gradient.
   *  The fourth column of the results matrix has is I_yy, x derivative of the x gradient.
   * 
   *  @return A matrix being Nx4 (number of template pixels x dim(\vx) )
   */
  virtual cv::Mat
  computeTemplateFeaturesGradient
    (
    cv::Mat object_params      
    );
    
  /**
   *  @brief Computes the features vector from the warped image: the gray levels gradient.
   *  
   *  Extracts the gradients from the warped image. The first column of the result is the 
   *  horizontal (x) direction gradient and the second column is the vertical (y) direction 
   *  gradient.
   * 
   *  @param warped_image
   *  @return A matrix that is Nx2 (number of template pixels x 2 ).
   */
  virtual cv::Mat
  extractFeaturesFromWarpedImage
    (
    cv::Mat warped_image
    );

  /**
   *  @brief Computes the template gray levels gradient.
   * 
   *  Extracts the gradients from the template image. The first column of the result is the 
   *  horizontal (x) direction gradient and the second column is the vertical (y) direction 
   *  gradient.
   *
   *  @return A vector that is Nx2 (number of template pixels x 2 ).
   */
  virtual cv::Mat
  computeTemplateFeatures
    (
    cv::Mat object_params
    );
  
protected:
  cv::Mat m_hessians;
};

}; }; // namespace

#endif
