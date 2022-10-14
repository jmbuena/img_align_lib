// -----------------------------------------------------------------------------
/**
 *  @brief Gauss-Newton Optimization algorithm implementation.
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

#include "optimizerGN.hpp"
#include "trace.hpp"

#undef USE_GPU

#define DEBUG

namespace upm { namespace pcr
{
// -----------------------------------------------------------------------------
//
// Purpose and Method: 
// // Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
// -----------------------------------------------------------------------------
GaussNewtonOptimizer::~GaussNewtonOptimizer
  () 
{ 
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
cv::Mat 
GaussNewtonOptimizer::solve
  (
  cv::Mat& frame,
  cv::Mat& former_params
  ) 
{
  int k         = 0 ;
  bool found    = false ;
  double gradient_norm;
  cv::Mat new_params;
  cv::Mat current_params; 
  cv::Mat gradient;
  cv::Mat delta;
  cv::Mat residual;
  cv::Mat invJ;
  cv::Mat J;
  double cost;
  
  if (m_show_iterations)
  {
    // Plot tracking statistics header
    std::cout << std::endl;
    std::cout << "  Iter        F(x)        Gradient        " << std::endl;
    std::cout << "  ----        ----        --------        " << std::endl;
  }

  // Reset stored optimization iterations costs.
  m_iteration_costs.clear();

  former_params.copyTo(current_params);

  while ( (~found) & (k < m_max_iterations) ) 
  {
    k = k + 1;

    // Compute the residual error to minimize for new motion parameters
    residual = m_optim_problem->computeResidual(frame, current_params);
    
    // Compute normal equations
    invJ     = m_optim_problem->computeInverseJacobian(current_params);
    delta    = -invJ * residual; 

    // 2nd stopping criterion: The increment in the parameters vector
    // is under a given threshold. 
//     if (cv::norm(delta) < m_tol_params*(cv::norm(current_params) + m_tol_params)) 
    if (cv::norm(delta) < m_tol_params) //*(cv::norm(current_params) + m_tol_params)) 
    {
      found = true ;
      if (m_show_iterations)
      {
        std::cout << "STOP. Parameters not increasing: norm(delta) = " << cv::norm(delta) << std::endl;
      }
    }

#ifdef DEBUG
    // write Mat objects to the file
    cv::FileStorage fs("GaussNewtonOptimizer_solve.xml", cv::FileStorage::WRITE);
    fs << "current_params" << current_params;
    fs << "delta" << delta;
    fs.release();
#endif  
    
    // Compute parameters update 
    new_params = m_optim_problem->updateMotionParams(current_params, delta);
    
    // Compute the gradient instantiated in current x.
    residual = m_optim_problem->computeResidual(frame, new_params);
    J        = m_optim_problem->computeJacobian(new_params);
    gradient = J.t() * residual;
    gradient_norm = cv::norm(gradient);

    cost = m_optim_problem->computeCostFunction(residual, new_params, delta, frame);
    m_iteration_costs.push_back(cost);

    if (m_show_iterations)
    {
      std::cout << "  " << k << "        " << cost << "        " << gradient_norm << std::endl;
    }
    
    // 1st stopping criterion: the norm of the gradient is under a given
    // threshold.
    if (gradient_norm < m_tol_gradient) 
    {
      found = true ;
      if (m_show_iterations)
      {
        std::cout << "STOP. Norm of the gradient is bellow threshold." << std::endl;
      }
    }
    
    new_params.copyTo(current_params);
  }

  if ((k > m_max_iterations) && (m_show_iterations))
  {
    std::cout << "STOP. Maximun number of iterations exceeded: " << m_max_iterations << std::endl;
  }

  return current_params;
}

} } // namespace
