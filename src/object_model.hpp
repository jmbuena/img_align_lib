// -----------------------------------------------------------------------------
/**
 *  @brief Object model in direct methods tracking.
 *  @author Jose M. Buenaposada
 *  @date 2012/07/10
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
#ifndef OBJECT_MODEL_HPP
#define OBJECT_MODEL_HPP

// ----------------------- INCLUDES --------------------------------------------
#include <boost/shared_ptr.hpp>
#include "viewer.hpp"
#include "img_align_lib.hpp"

namespace upm { namespace pcr
{
  
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to ObjectModel type
 */
// -----------------------------------------------------------------------------
class ObjectModel;
typedef boost::shared_ptr<ObjectModel> ObjectModelPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to ObjectModels 
 */
// -----------------------------------------------------------------------------
typedef std::vector<ObjectModel> ObjectModels;

typedef struct 
{
  int p1_index, p2_index;
} LineIndices; 

// ----------------------------------------------------------------------------
/**
 * @class ObjectModel
 * @brief A class that defines the interface for the object (target) model.
 */
// -----------------------------------------------------------------------------
class ObjectModel
{
public:

  ObjectModel
    () {};

  virtual ~ObjectModel
    () {};
    
  /**
   *  @brief Computes the grey levels gradient of a template image or any other feature in the template model.
   *  
   *  It is the equivalent to the \frac{\partial I(\vx)}{\partial \vx} (the 
   *  gradient) whenever we have an image as a model. The size of the output 
   *  matrix is Nxk being N the number of pixels/features and k the 
   *  dimensionality of \vx (the template coordinates vector). For example, 
   *  in the case of a 2D planar model dim(\vx) = 2 and in the case of a 3D 
   *  object dim(\vx)=3
   * 
   *  @return A matrix being Nxk (number of template points x dim(\vx) )
   */
  virtual cv::Mat
  computeTemplateFeaturesGradient
    (
    cv::Mat object_params
    ) = 0;
    
  /**
   *  @brief Computes the features vector from the warped image.
   *  
   *  This method is intended to compute the image features (DCT, gradients 
   *  orientations, etc) from the warped input image (to the reference 
   *  coordinates of the template). If features are just plain gray levels 
   *  then this method returns the input image grey levels in an Nx1 vector.
   *
   *  Returns a matrix with a feature vector per row. The feature vector 
   *  corresponds to a reference point on the object model.
   * 
   *  @param warped_image
   *  @return A vector that is Nxk (number of template points x f ).
   */
  virtual cv::Mat
  extractFeaturesFromWarpedImage
    (
    cv::Mat warped_image
    ) = 0;

  /**
   *  @brief Computes the features vector from the template (the target object to be tracked)
   * 
   *  Returns a matrix with a feature vector per row. Each features vector corresponds to 
   *  a reference point in the object model.
   * 
   *  @param object_params Actual motion params needed in some object models
   *  @return A vector that is Nxf (number of template pixels/features x f ).+
   */
  virtual cv::Mat
  computeTemplateFeatures
    (
    cv::Mat object_params
    ) = 0;
  
  /**
   *  @brief Returns the coordinates of template points (the reference coordinates).
   * 
   *  reference_coords are the the coordinates of the points used to track. If we
   *  have a template image as object model then those are the coordinates attributed 
   *  to every single pixel used in tracking. If we use a 3D Morphable Model those are
   *  the texture coordinates of every single 3D point used in tracking.  
   *  
   *  On the other hand, only some of the reference points are control points. In the 
   *  case of a template image object model those can be the four corners of a rectangle. 
   *  In the case of the Active Appearance Model only the triangle corners are control 
   *  points. The control_points_indices are the column indices in reference_coords that
   *  are control points. 
   * 
   *  @param control_points_indices Column indices of reference_coords for the control points.
   *  @param control_points_lines 
   */
  virtual void
  getCtrlPointsIndices
    (
    std::vector<int>& control_points_indices,
    std::vector<LineIndices>& control_points_lines 
    ) = 0;

  /**
   *  @brief Returns the coordinates of template points (the reference coordinates).
   * 
   *  reference_coords are the the coordinates of the points used to track. If we
   *  have a template image as object model then those are the coordinates attributed 
   *  to every single pixel used in tracking. If we use a 3D Morphable Model those are
   *  the texture coordinates of every single 3D point used in tracking.  
   *  
   *  @param reference_coords A matrix that is Nxk (number of template pixels/features x dim(\vx)).
   */
  virtual void
  getReferenceCoords
    (
    cv::Mat& reference_coords
    ) = 0;

  virtual size_t
  getNumOfReferenceCoords
    () = 0;
};

}; }; // namespace

#endif


