// -----------------------------------------------------------------------------
/**
 *  @brief Gauss-Newton Optimization algorithm interface.
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
#ifndef OPTIMIZER_GN_HPP
#define OPTIMIZER_GN_HPP

// ----------------------- INCLUDES --------------------------------------------
#include <opencv2/opencv.hpp>
#include "optimizer.hpp"

namespace upm { namespace pcr
{
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to GaussNewtonOptimizer type
 */
// -----------------------------------------------------------------------------
class GaussNewtonOptimizer;
typedef boost::shared_ptr<GaussNewtonOptimizer> GaussNewtonOptimizerPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to Gauss-Newton optimizers
 */
// -----------------------------------------------------------------------------
typedef std::vector<GaussNewtonOptimizer> OptimizersGN;

// ----------------------------------------------------------------------------
/**
 * @class GaussNewtonOptimizer
 * @brief A class that implements Gauss-Newton implementation
 */
// -----------------------------------------------------------------------------
class GaussNewtonOptimizer: public Optimizer
{
public:

  GaussNewtonOptimizer
    (
    OptimizationProblemPtr optim_problem
    ):
    Optimizer(optim_problem)
    {};

  virtual ~GaussNewtonOptimizer
    ();

  cv::Mat 
  solve
    (
    cv::Mat& frame,
    cv::Mat& former_params
    );
};

}; }; // namespace

#endif
