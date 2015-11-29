// -----------------------------------------------------------------------------
/**
 *  @brief Problem using a template image gradients correlation with similarity motion model
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
#ifndef SIMILARITY_2D_GRAD_CORR_INV_COMP_PROBLEM_HPP
#define SIMILARITY_2D_GRAD_CORR_INV_COMP_PROBLEM_HPP

// ----------------------- INCLUDES --------------------------------------------
#include "inverse_compositional_problem.hpp"
#include "single_image_gradients_model.hpp"
#include <memory>
#include <vector>
#include <opencv/cv.hpp>

namespace upm { namespace pcr
{
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to the Similarity2DGradCorrInvCompProblem type
 */
// -----------------------------------------------------------------------------
class Similarity2DGradCorrInvCompProblem;
typedef std::shared_ptr<Similarity2DGradCorrInvCompProblem> Similarity2DGradCorrInvCompProblemPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to Similarity2DGradCorrInvCompProblem class
 */
// -----------------------------------------------------------------------------
typedef std::vector<Similarity2DGradCorrInvCompProblem> Similarity2DGradCorrInvCompProblems;

// ----------------------------------------------------------------------------
/**
 * @class Similarity2DGradCorrInvCompProblem
 * @brief The interface for the Similarity2DGradCorrInvCompProblem to be used with Optimizer
 * 
 * Optimization problem for Image Template and input image Gradient Correlations maximization.
 * This implementation is the Inverse Compositional version which gives a constsant Jacobian.
 * It is based on the paper:
 * 
 *   "Robust and Efficient Parametric Face Alignment"
 *   G. Tzimiropoulos, S. Zafeiriou, M. Pantic 
 *   Proceedings of IEEE Int’l Conf. on Computer Vision (ICCV 2011) . pp. 1847 - 1854.
 * 
 */
// -----------------------------------------------------------------------------
class Similarity2DGradCorrInvCompProblem: public InverseCompositionalProblem
{
public:

  Similarity2DGradCorrInvCompProblem
    (
//     SingleImageGradientsModelPtr object_model,
    ObjectModelPtr object_model,
    MotionModelPtr motion_model
    );

  virtual ~Similarity2DGradCorrInvCompProblem
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
  computeConstantJacobian
    ();
}; 

  
}; }; // namespace

#endif
