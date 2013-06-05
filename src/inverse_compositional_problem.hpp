// -----------------------------------------------------------------------------
/**
 *  @brief Optimization problem interface for 'inverse compositional solutions'
 *  @author Jose M. Buenaposada
 *  @date 2012/09/10
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
#ifndef INVERSE_COMPOSITIONAL_PROBLEM_HPP
#define INVERSE_COMPOSITIONAL_PROBLEM_HPP

// ----------------------- INCLUDES --------------------------------------------
#include "optimization_problem.hpp"

namespace upm { namespace pcr
{
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to the InverseCompositionalProblem type
 */
// -----------------------------------------------------------------------------
class InverseCompositionalProblem;
typedef boost::shared_ptr<InverseCompositionalProblem> InverseCompositionalProblemPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to InverseCompositionalProblem class
 */
// -----------------------------------------------------------------º------------
typedef std::vector<InverseCompositionalProblem> InverseCompositionalProblems;

// ----------------------------------------------------------------------------
/**
 * @class InverseCompositionalProblem
 * @brief The interface for the InverseCompositionalProblem to be used with Optimizer
 * 
 * The inverse compositional based problems assume that the optimization problem jacobian J, 
 * is constant.
 * 
 */
// -----------------------------------------------------------------------------
class InverseCompositionalProblem: public OptimizationProblem
{
public:

  InverseCompositionalProblem
    (
    ObjectModelPtr object_model,
    MotionModelPtr motion_model
    );

  virtual ~InverseCompositionalProblem
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
  computeConstantJacobian
    () = 0;
    
  void
  initialize
    ();

  cv::Mat m_J;
  cv::Mat m_inv_J;
  bool m_initialized;
}; 

  
}; }; // namespace

#endif
