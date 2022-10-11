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

// ------------------ RECURSION PROTECTION -------------------------------------
#ifndef SINGLE_IMAGE_MODEL_HPP
#define SINGLE_IMAGE_MODEL_HPP

// ----------------------- INCLUDES --------------------------------------------
#include "object_model.hpp"

namespace upm { namespace pcr
{
  
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to SingleImageModel type
 */
// -----------------------------------------------------------------------------
class SingleImageModel;
typedef boost::shared_ptr<SingleImageModel> SingleImageModelPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to SingleImageModels 
 */
// -----------------------------------------------------------------------------
typedef std::vector<SingleImageModel> SingleImageModels;

// ----------------------------------------------------------------------------
/**
 * @class SingleImageModel
 * @brief A class that defines single template image (target) model.
 *
 * The gray levels are divided by 255 in order the cost function to use
 * quantities in [0.0, 1.0] avoiding numerical errors (if any).
 */
// -----------------------------------------------------------------------------
class SingleImageModel: public ObjectModel
{
public:

  SingleImageModel
    (
    cv::Mat& template_image, 
    bool equalize = false
    );

  virtual ~SingleImageModel
    ();
    
  /**
   *  @brief Computes the grey levels gradient of a template image or any other feature in the template model.
   *  
   *  It computes the  \frac{\partial I(\vx)}{\partial \vx} (the gradient).
   *  The size of the output matrix is Nxk being N the number of pixels and k the dimensinality of \vx (the 
   *  template coordinates vector).
   * 
   *  @return A matrix being Nxk (number of template pixels x dim(\vx) )
   */
  virtual cv::Mat
  computeTemplateFeaturesGradient
    (
    cv::Mat object_params      
    );
    
  /**
   *  @brief Computes the features vector from the warped image.
   *  
   *  Converts the input image to gray levels and then to a column vector (Nx1 with N the 
   *  number of pixels).
   * 
   *  @param warped_image
   *  @return A vector that is Nx1 (number of template pixels x 1 ).
   */
  virtual cv::Mat
  extractFeaturesFromWarpedImage
    (
    cv::Mat warped_image
    );

  /**
   *  @brief Computes the template gray levels.
   * 
   *  @return A vector that is Nx1 (number of template pixels x 1 ).
   */
  virtual cv::Mat
  computeTemplateFeatures
    (
    cv::Mat object_params
    );
  
  /**
   *  @brief Returns the coordinates of template points (the reference coordinates).
   * 
   *  Returns the coordinates of each pixel from left to right and top to 
   *  bottom (by rows). The coordinates of the top left corner are (-width/2,-height/2),
   *  and the coordinates of the right bottom corner are  (width/2, height/2). That
   *  means that the image center pixel take template (reference) coordinats (0,0).
   *  
   *  The control points are the four corners of a rectangle and there are four lines 
   *  that joins the four control points.
   * 
   *  @param reference_coords A matrix that is Nxk (number of template pixels x dim(\vx)).
   */
  virtual void
  getCtrlPointsIndices
    (
    std::vector<int>& control_points_indices,
    std::vector<LineIndices>& control_points_lines 
    );

    /**
   *  @brief Returns the coordinates of template points (the reference coordinates).
   * 
   *  Returns the coordinates of each pixel from left to right and top to 
   *  bottom (by rows). The coordinates of the top left corner are (-width/2,-height/2),
   *  and the coordinates of the right bottom corner are  (width/2, height/2). That
   *  means that the image center pixel take template (reference) coordinats (0,0).
   * 
   *  @param reference_coords A matrix that is Nxk (number of template pixels x dim(\vx)).
   */
  virtual void
  getReferenceCoords
    (
    cv::Mat& reference_coords
    );

  virtual size_t
  getNumOfReferenceCoords
    () { return m_template_image.rows * m_template_image.cols; };
    
protected:
  
  cv::Mat 
  computeTemplateCoordinates
    (
    cv::Mat gray_image
    );

  cv::Mat m_template_image;
  cv::Mat m_template_gray_levels;
  cv::Mat m_template_coordinates;
  cv::Mat m_gradients;
  bool m_equalize;
  std::vector<int> m_control_points_indices;
  std::vector<LineIndices> m_control_points_lines;
};

}; }; // namespace

#endif
