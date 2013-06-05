// -----------------------------------------------------------------------------
/**
 *  @brief motion model (function) in direct methods tracking.
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
#ifndef MOTION_MODEL_HPP
#define MOTION_MODEL_HPP

// ----------------------- INCLUDES --------------------------------------------
#include <boost/shared_ptr.hpp>
#include "viewer.hpp"
#include <vector>
#include "img_align_lib.hpp"

namespace upm { namespace pcr
{
  
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to MotionModel type
 */
// -----------------------------------------------------------------------------
class MotionModel;
typedef boost::shared_ptr<MotionModel> MotionModelPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to MotionModels 
 */
// -----------------------------------------------------------------------------
typedef std::vector<MotionModel> MotionModels;

// ----------------------------------------------------------------------------
/**
 * @class MotionModel
 * @brief A class that defines the interface for the motion model of the target.
 */
// -----------------------------------------------------------------------------
class MotionModel
{
public:

  MotionModel
    () {};

  virtual ~MotionModel
    () {};
    
  virtual cv::Mat
  scaleInputImageResolution
    (
    cv::Mat motion_params,
    double scale
    ) = 0; 
    
  virtual cv::Mat
  transformCoordsToTemplate
    (
    cv::Mat coords,
    cv::Mat motion_params   
    ) = 0; 

  virtual cv::Mat
  transformCoordsToImage
    (
    cv::Mat coords,
    cv::Mat motion_params   
    ) = 0; 

  virtual cv::Mat
  warpImage
    (
    cv::Mat image,
    cv::Mat motion_params,
    cv::Mat template_coords,
    std::vector<int>& template_ctrl_points_indices
    ) = 0; 

  virtual bool
  invalidParams
    (
    cv::Mat params
    ) { return false; };
    
  virtual size_t 
  getNumParams
    () = 0; 
};

}; }; // namespace

#endif
