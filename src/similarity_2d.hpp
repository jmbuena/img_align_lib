// -----------------------------------------------------------------------------
/**
 *  @brief 2D similarity motion model (function) in direct methods tracking.
 *  @author Jose M. Buenaposada
 *  @date 2012/07/16
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
#ifndef SIMILARITY_2D_HPP
#define SIMILARITY_2D_HPP

// ----------------------- INCLUDES --------------------------------------------
#include "motion_model.hpp"

namespace upm { namespace pcr
{
  
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to Similarity2D type
 */
// -----------------------------------------------------------------------------
class Similarity2D;
typedef boost::shared_ptr<Similarity2D> Similarity2DPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to Similarity2Ds 
 */
// -----------------------------------------------------------------------------
typedef std::vector<Similarity2D> Similarity2Ds;

// ----------------------------------------------------------------------------
/**
 * @class Similarity2D
 * @brief A class for the 2D similarity (rotation, traslation, scale) motion model.
 * 
 *  The 2D similarity motion model is: x(k+1) = s*R(alpha)*x(k)+t
 *
 *  Where: - x(k) is the posision vector in the image plane of the image
 *           template center in the instant k.
 *         - s is the scale factor. 
 *         - R(alpha) is a rotation matrix (with the SSD template centered
 *           in the origin).
 *         - alpha is the rotation angle in the image plane of the SSD image
 *           template.
 *         - t is the translation vector of the image template center.
 *
 *
 *  The parameters are arranged in a 4x1 vector:
 *
 *                         ( X traslation              )
 *                         ( Y traslation              )
 *                         ( rotation angle in radians )
 *                         ( scale                     )
 */
// -----------------------------------------------------------------------------
class Similarity2D: public MotionModel
{
public:

  Similarity2D
    (
    bool zero_params_is_identity = false
    );

  virtual ~Similarity2D
    ();
    
  virtual cv::Mat
  scaleInputImageResolution
    (
    cv::Mat params,
    double scale
    ); 

  virtual cv::Mat
  transformCoordsToTemplate
    (
    cv::Mat coords,
    cv::Mat params   
    ); 

  virtual cv::Mat
  transformCoordsToImage
    (
    cv::Mat coords,
    cv::Mat params   
    ); 

  virtual cv::Mat
  warpImage
    (
    cv::Mat image,
    cv::Mat params,
    cv::Mat template_coords,
    std::vector<int>& template_ctrl_points_indices
    ); 

  virtual size_t 
  getNumParams
    () { return 4; }; 

protected:
  bool m_zero_params_is_identity;
    
    
};

}; }; // namespace

#endif
