// -----------------------------------------------------------------------------
/**
 *  @brief Affine 2D motion model (function) in direct methods tracking.
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

// ------------------ RECURSION PROTECTION -------------------------------------
#ifndef AFFINE_2D_HPP
#define AFFINE_2D_HPP

// ----------------------- INCLUDES --------------------------------------------
#include "motion_model.hpp"

namespace upm { namespace pcr
{
  
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to Affine2D type
 */
// -----------------------------------------------------------------------------
class Affine2D;
typedef std::shared_ptr<Affine2D> Affine2DPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to Affine2D 
 */
// -----------------------------------------------------------------------------
typedef std::vector<Affine2D> Affines2D;

// ----------------------------------------------------------------------------
/**
 * @class Affine2D
 * @brief A class for the 2D affine motion model.
 * 
 *  The 2D affin emotion model (6 parameters) is given by:
 *
 *                         (a   c)
 *                x(k+1) = (     )*x(k)+t
 *                         (b   d)
 *
 *  Where: - x(k) is the posision vector in the image plane of the image
 *           template center in the instant k.
 *         - A(a,b,c,d) is the matrix which describes with t a   
 *           linear change of coordinates
 *         - t is the translation vector of the image template center.
 *
 *
 *  The parameters are arranged in a 6x1 vector:
 *
 *                         ( X translation )
 *                         ( Y traslation  )
 *                         ( a             )
 *                         ( b             )
 *                         ( c             )
 *                         ( d             )        
 *
 */
// -----------------------------------------------------------------------------
class Affine2D: public MotionModel
{
public:

  Affine2D
    ();

  virtual ~Affine2D
    ();

//   virtual cv::Mat
//   computeMotionJacobian
//     (
//     cv::Mat params
//     ); 
      
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
    () { return 6; }; 
};

}; }; // namespace

#endif
