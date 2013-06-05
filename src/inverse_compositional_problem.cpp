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

#include "inverse_compositional_problem.hpp"
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
InverseCompositionalProblem::InverseCompositionalProblem
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
InverseCompositionalProblem::initialize
  ()
{  
  m_J     = computeConstantJacobian();
  m_inv_J = m_J.inv(cv::DECOMP_SVD);
//   m_inv_J = (m_J.t()*m_J).inv()*m_J.t();
  
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
InverseCompositionalProblem::~InverseCompositionalProblem
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
InverseCompositionalProblem::computeJacobian
  (
  cv::Mat& motion_params
  )
{
  if (!m_initialized)
  {
    initialize();
  }
  
  return m_J;
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
InverseCompositionalProblem::computeInverseJacobian
  (
  cv::Mat& motion_params
  )
{
  if (!m_initialized)
  {
    initialize();
  }
  
  return m_inv_J;
};
           
  
}; }; // namespace
