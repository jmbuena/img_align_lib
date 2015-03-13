// -----------------------------------------------------------------------------
/**
 *  @brief Optimization algorithm interface.
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
#ifndef OPTIMIZER_HPP
#define OPTIMIZER_HPP

// ----------------------- INCLUDES --------------------------------------------
#include <memory>
#include "optimization_problem.hpp"
#include "img_align_lib.hpp"

namespace upm { namespace pcr
{
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to Optimizer type
 */
// -----------------------------------------------------------------------------
class Optimizer;
typedef std::shared_ptr<Optimizer> OptimizerPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to optimizers
 */
// -----------------------------------------------------------------------------
typedef std::vector<Optimizer> Optimizers;

// ----------------------------------------------------------------------------
/**
 * @class Optimizer
 * @brief A class that defines the interface for an optimization algorithms
 */
// -----------------------------------------------------------------------------
class Optimizer
{
public:

  Optimizer
    (
    OptimizationProblemPtr optim_problem
    ):
    m_optim_problem(optim_problem),
    m_max_iterations(20),
    m_tol_gradient(0.001),
    m_tol_params(0.00001),
    m_show_iterations(false)
    {};

  virtual ~Optimizer
    () {};

  virtual cv::Mat 
  solve
    (
    cv::Mat& frame,
    cv::Mat& former_params
    ) = 0;

  OptimizationProblemPtr 
  getOptimizationProblem
    () { return m_optim_problem; };

  void
  setMaxNumIterations
    (
    size_t max_iterations
    )  
  { m_max_iterations = max_iterations; };

  void
  setTolGradient
    (
    float tol
    )
  { m_tol_gradient = tol; }; 

  void
  setTolParams
    (
    float tol
    )
  { m_tol_params = tol; }; 

  std::vector<double>
  getIterationsCosts
    ()
  { return m_iteration_costs; };

  void
  setShowIterations
    (
    bool show
    )
  { m_show_iterations = show; };

  bool
  getShowIterations
    ()
  { return m_show_iterations; };

protected:
  OptimizationProblemPtr m_optim_problem;
  size_t m_max_iterations;
  float m_tol_gradient;
  float m_tol_params;
  bool m_show_iterations;

  // The 0 index is the first iteration costs and
  // the m_iterations_costs.size()-1 is the
  // last iteration cost. Every time
  std::vector<double> m_iteration_costs;
};

}; }; // namespace

#endif
