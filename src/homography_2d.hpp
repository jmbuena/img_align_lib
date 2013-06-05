// -----------------------------------------------------------------------------
/**
 *  @brief Homography 2D motion model (function) in direct methods tracking.
 *  @author Jose M. Buenaposada
 *  @date 2012/10/13
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
#ifndef HOMOGRAPHY_2D_HPP
#define HOMOGRAPHY_2D_HPP

// ----------------------- INCLUDES --------------------------------------------
#include "motion_model.hpp"

namespace upm { namespace pcr
{
  
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to Homography2D type
 */
// -----------------------------------------------------------------------------
class Homography2D;
typedef boost::shared_ptr<Homography2D> Homography2DPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to Homography2D 
 */
// -----------------------------------------------------------------------------
typedef std::vector<Homography2D> Homographys2D;

// ----------------------------------------------------------------------------
/**
 * @class Homography2D
 * @brief A class for the 2D homography (8 parameters) motion model.
 * 
 *  The model is:
 *
 *                [a b c]
 *       x(k+1) = [d e f] * x(k)
 *                [g h k]
 * 
 *  We use homogeneus coordinates in the model to hand linear equation.
 *
 *  Where: - x(k) is the posision vector in the image plane of the image
 *           template center in the instant k. x(k) is arranged in a 3
 *           component vector (homogeneus coordinates) 
 *         - H(a,b,c,d,e,f,g,h, k) is the matrix which describes a   
 *           linear change of coordinates
 *
 *
 *  The parameters are arranged in a 9x1 vector:
 *
 *                         ( a )
 *                         ( d )
 *                         ( g )
 *                         ( b )
 *                         ( e )
 *                         ( h )
 *                         ( c )
 *                         ( f )
 *                         ( k )
 *
 */
// -----------------------------------------------------------------------------
class Homography2D: public MotionModel
{
public:

  Homography2D
    ();

  virtual ~Homography2D
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
    
  virtual bool
  invalidParams
    (
    cv::Mat params
    );

  virtual size_t 
  getNumParams
    () { return 9; };    
    
protected:
  
  bool  
  consistentPoints
    (
    cv::Mat points, 
    cv::Mat transformed_points,
    int t1,
    int t2,
    int t3
    );
    
};

}; }; // namespace

#endif
