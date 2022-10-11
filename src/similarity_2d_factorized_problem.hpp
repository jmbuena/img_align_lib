// -----------------------------------------------------------------------------
/**
 *  @brief Problem using a template image with similarity motion model
 *  @author Jose M. Buenaposada
 *  @date 2012/07/15
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
#ifndef SIMILARITY_2D_FACTORIZED_PROBLEM_HPP
#define SIMILARITY_2D_FACTORIZED_PROBLEM_HPP

// ----------------------- INCLUDES --------------------------------------------
#include "factorized_jacobian_problem.hpp"
#include <boost/shared_ptr.hpp>
#include <vector>
#include <opencv2/opencv.hpp>

namespace upm { namespace pcr
{
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to the Similarity2DFactorizedProblem type
 */
// -----------------------------------------------------------------------------
class Similarity2DFactorizedProblem;
typedef boost::shared_ptr<Similarity2DFactorizedProblem> Similarity2DFactorizedProblemPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to Similarity2DFactorizedProblem class
 */
// -----------------------------------------------------------------------------
typedef std::vector<Similarity2DFactorizedProblem> Similarity2DFactorizedProblems;

// ----------------------------------------------------------------------------
/**
 * @class Similarity2DFactorizedProblem
 * @brief The interface for the Similarity2DFactorizedProblem to be used with Optimizer
 * 
 * The factorization methods assume that the optimization problem jacobian J, 
 * can be factorized into J=M0*Sigma matrix where Sigma matrix depends only
 * on the motion parameters and M0 matrix depends only on the object model 
 * features and coordinates.
 */
// -----------------------------------------------------------------------------
class Similarity2DFactorizedProblem: public FactorizedJacobianProblem
{
public:

  Similarity2DFactorizedProblem
    (
    ObjectModelPtr object_model,
    MotionModelPtr motion_model
    );

  virtual ~Similarity2DFactorizedProblem
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
