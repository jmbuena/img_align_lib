// -----------------------------------------------------------------------------
/**
 *  @brief Problem using a template image with affine motion model
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
#ifndef AFFINE_2D_FACTORIZED_PROBLEM_HPP
#define AFFINE_2D_FACTORIZED_PROBLEM_HPP

// ----------------------- INCLUDES --------------------------------------------
#include "factorized_jacobian_problem.hpp"
#include <memory>
#include <vector>
#include <opencv/cv.hpp>

namespace upm { namespace pcr
{
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to the Affine2DFactorizedProblem type
 */
// -----------------------------------------------------------------------------
class Affine2DFactorizedProblem;
typedef std::shared_ptr<Affine2DFactorizedProblem> Affine2DFactorizedProblemPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to Affine2DFactorizedProblem class
 */
// -----------------------------------------------------------------------------
typedef std::vector<Affine2DFactorizedProblem> Affine2DFactorizedProblems;

// ----------------------------------------------------------------------------
/**
 * @class Affine2DFactorizedProblem
 * @brief The interface for the Affine2DFactorizedProblem to be used with Optimizer
 * 
 * The factorization methods assume that the optimization problem jacobian J, 
 * can be factorized into J=M0*Sigma matrix where Sigma matrix depends only
 * on the motion parameters and M0 matrix depends only on the object model 
 * features and coordinates.
 */
// -----------------------------------------------------------------------------
class Affine2DFactorizedProblem: public FactorizedJacobianProblem
{
public:

  Affine2DFactorizedProblem
    (
    ObjectModelPtr object_model,
    MotionModelPtr motion_model
    );

  virtual ~Affine2DFactorizedProblem
    ();

  virtual double 
  computeCostFunction
    (
    cv::Mat& residual_vector,
    cv::Mat& params,
    cv::Mat& delta_params,
    cv::Mat& image     
    );
    
  virtual cv::Mat
  computeResidual
    (
    cv::Mat& image,
    cv::Mat& params
    );

  virtual cv::Mat
  computeInverseJacobian
    (
    cv::Mat& params
    );
    
  virtual cv::Mat
  updateMotionParams
    (
    cv::Mat& params,
    cv::Mat& inc_params
    );
    
protected:    
    
  virtual cv::Mat
  computeSigmaMatrix
    (
    cv::Mat& params
    );
    
  virtual cv::Mat
  computeM0Matrix
    ();
  
  /** In this case the inverse jacobian is constant and we store it */
  cv::Mat m_invM0;
}; 

  
}; }; // namespace

#endif
