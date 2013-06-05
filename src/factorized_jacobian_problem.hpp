// -----------------------------------------------------------------------------
/**
 *  @brief Optimization problem interface for 'efficient factorized jacobian'
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
#ifndef FACTORIZED_JACOBIAN_PROBLEM_HPP
#define FACTORIZED_JACOBIAN_PROBLEM_HPP

// ----------------------- INCLUDES --------------------------------------------
#include "optimization_problem.hpp"


namespace upm { namespace pcr
{
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to the FactorizedJacobianProblem type
 */
// -----------------------------------------------------------------------------
class FactorizedJacobianProblem;
typedef boost::shared_ptr<FactorizedJacobianProblem> FactorizedJacobianProblemPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to FactorizedJacobianProblem class
 */
// -----------------------------------------------------------------º------------
typedef std::vector<FactorizedJacobianProblem> FactorizedJacobianProblems;

// ----------------------------------------------------------------------------
/**
 * @class FactorizedJacobianProblem
 * @brief The interface for the FactorizedJacobianProblem to be used with Optimizer
 * 
 * The factorization methods assume that the optimization problem jacobian J, 
 * can be factorized into J=M0*Sigma matrix where Sigma matrix depends only
 * on the motion parameters and M0 matrix depends only on the object model 
 * features and coordinates.
 */
// -----------------------------------------------------------------------------
class FactorizedJacobianProblem: public OptimizationProblem
{
public:

  FactorizedJacobianProblem
    (
    ObjectModelPtr object_model,
    MotionModelPtr motion_model
    );

  virtual ~FactorizedJacobianProblem
    ();

  virtual cv::Mat
  computeJacobian
    (
    cv::Mat& motion_params
    );

  virtual cv::Mat
  computeInverseJacobian
    (
    cv::Mat& motion_params
    );
       
protected:

  virtual cv::Mat
  computeSigmaMatrix
    (
    cv::Mat& motion_params
    ) = 0;
    
  virtual cv::Mat
  computeM0Matrix
    () = 0;

  void
  initialize
    ();

  cv::Mat m_M0;
  cv::Mat m_M0t_M0;
  bool m_initialized;
}; 

  
}; }; // namespace

#endif
