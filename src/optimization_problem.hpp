// -----------------------------------------------------------------------------
/**
 *  @brief Optimization problem interface
 *  @author Jose M. Buenaposada
 *  @date 2012/07/12
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
#ifndef OPTIMIZATION_PROBLEM_HPP
#define OPTIMIZATION_PROBLEM_HPP

// ----------------------- INCLUDES --------------------------------------------
#include "motion_model.hpp"
#include "object_model.hpp"
#include <boost/shared_ptr.hpp>
#include <vector>
#include <opencv/cv.hpp>
#include "img_align_lib.hpp"

namespace upm { namespace pcr
{
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to the OptimizationProblem type
 */
// -----------------------------------------------------------------------------
class OptimizationProblem;
typedef boost::shared_ptr<OptimizationProblem> OptimizationProblemPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to OptimizationProblem class
 */
// -----------------------------------------------------------------º------------
typedef std::vector<OptimizationProblem> OptimizationProblems;

// ----------------------------------------------------------------------------
/**
 * @class OptimizationProblem
 * @brief The interface for the OptimizationProblem to be used with Optimizer
 */
// -----------------------------------------------------------------------------
class OptimizationProblem
{
public:

  OptimizationProblem
    (
    ObjectModelPtr object_model,
    MotionModelPtr motion_model
    ): 
    m_motion_model(motion_model),
    m_object_model(object_model)
    {};

  virtual ~OptimizationProblem
    () {};

  virtual double 
  computeCostFunction
    (
    cv::Mat& residual_vector,
    cv::Mat& motion_params,
    cv::Mat& delta_params,
    cv::Mat& image
    ) = 0;
    
  virtual cv::Mat
  computeResidual
    (
    cv::Mat& image,
    cv::Mat& motion_params
    ) = 0;

  virtual cv::Mat
  computeJacobian
    (
    cv::Mat& motion_params
    ) = 0;

  virtual cv::Mat
  computeInverseJacobian
    (
    cv::Mat& motion_params
    ) = 0;
    
  virtual cv::Mat
  updateMotionParams
    (
    cv::Mat& motion_params,
    cv::Mat& inc_params
    ) = 0;
    
  ObjectModelPtr
  getObjectModel
    () { return m_object_model; };

  MotionModelPtr
  getMotionModel
    () { return m_motion_model; };

protected:
  MotionModelPtr m_motion_model;
  ObjectModelPtr m_object_model;
}; 

  
}; }; // namespace

#endif
