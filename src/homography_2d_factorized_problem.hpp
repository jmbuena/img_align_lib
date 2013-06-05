// -----------------------------------------------------------------------------
/**
 *  @brief Problem using a template image with homography motion model
 *  @author Jose M. Buenaposada
 *  @date 2012/10/15
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
#ifndef HOMOGRAPHY_2D_FACTORIZED_PROBLEM_HPP
#define HOMOGRAPHY_2D_FACTORIZED_PROBLEM_HPP

// ----------------------- INCLUDES --------------------------------------------
#include "factorized_jacobian_problem.hpp"
#include <boost/shared_ptr.hpp>
#include <vector>
#include <opencv/cv.hpp>

namespace upm { namespace pcr
{
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to the Homography2DFactorizedProblem type
 */
// -----------------------------------------------------------------------------
class Homography2DFactorizedProblem;
typedef boost::shared_ptr<Homography2DFactorizedProblem> Homography2DFactorizedProblemPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to Homography2DFactorizedProblem class
 */
// -----------------------------------------------------------------------------
typedef std::vector<Homography2DFactorizedProblem> Homography2DFactorizedProblems;

// ----------------------------------------------------------------------------
/**
 * @class Homography2DFactorizedProblem
 * @brief The interface for the Homography2DFactorizedProblem to be used with Optimizer
 * 
 * The factorization methods assume that the optimization problem jacobian J, 
 * can be factorized into J=M0*Sigma matrix where Sigma matrix depends only
 * on the motion parameters and M0 matrix depends only on the object model 
 * features and coordinates.
 */
// -----------------------------------------------------------------------------
class Homography2DFactorizedProblem: public FactorizedJacobianProblem
{
public:

  Homography2DFactorizedProblem
    (
    ObjectModelPtr object_model,
    MotionModelPtr motion_model
    );

  virtual ~Homography2DFactorizedProblem
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
