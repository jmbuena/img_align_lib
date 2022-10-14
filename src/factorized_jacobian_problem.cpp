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

#include "factorized_jacobian_problem.hpp"
#include "trace.hpp"

namespace upm { namespace pcr
{

// -----------------------------------------------------------------------------
//
// Purpose and Method: 
// Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
// -----------------------------------------------------------------------------  
FactorizedJacobianProblem::FactorizedJacobianProblem
  (
  ObjectModelPtr object_model,
  MotionModelPtr motion_model
  ): 
  OptimizationProblem(object_model, motion_model)
{
  m_initialized = false;
};

// -----------------------------------------------------------------------------
//
// Purpose and Method: 
// Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
// -----------------------------------------------------------------------------  
void
FactorizedJacobianProblem::initialize
  ()
{  
  m_M0     = computeM0Matrix();
  m_M0t_M0 = m_M0.t() * m_M0;
  
  m_initialized = true;
}


// -----------------------------------------------------------------------------
//
// Purpose and Method: 
// Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
// -----------------------------------------------------------------------------  
FactorizedJacobianProblem::~FactorizedJacobianProblem
  () 
{};
  
// -----------------------------------------------------------------------------
//
// Purpose and Method: 
// Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
// -----------------------------------------------------------------------------  
cv::Mat
FactorizedJacobianProblem::computeJacobian
  (
  cv::Mat& motion_params
  )
{
  if (!m_initialized)
  {
    initialize();
  }
  
  cv::Mat Sigma = computeSigmaMatrix(motion_params);
  cv::Mat J     = m_M0 * Sigma;
  
  return J;
};



// -----------------------------------------------------------------------------
//
// Purpose and Method: 
// Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
// -----------------------------------------------------------------------------
cv::Mat
FactorizedJacobianProblem::computeInverseJacobian
  (
  cv::Mat& motion_params
  )
{
  if (!m_initialized)
  {
    initialize();
  }
  
  cv::Mat Sigma = computeSigmaMatrix(motion_params);

  cv::Mat Hess  = Sigma.t() * m_M0t_M0 * Sigma;
  cv::Mat invJ  = Hess.inv() * (Sigma.t() * m_M0.t());
  
  return invJ;
};



  
}; }; // namespace
