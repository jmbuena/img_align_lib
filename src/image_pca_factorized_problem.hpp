// -----------------------------------------------------------------------------
/**
 *  @brief Problem using a PCA of images 
 *  @author Jose M. Buenaposada
 *  @date 2012/08/11
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
#ifndef IMAGE_PCA_FACTORIZED_PROBLEM_HPP
#define IMAGE_PCA_FACTORIZED_PROBLEM_HPP

// ----------------------- INCLUDES --------------------------------------------
#include "factorized_jacobian_problem.hpp"
#include "image_pca_model.hpp"
#include <boost/shared_ptr.hpp>
#include <vector>
#include <opencv/cv.hpp>

namespace upm { namespace pcr
{
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to the ImagePCAFactorizedProblem type
 */
// -----------------------------------------------------------------------------
class ImagePCAFactorizedProblem;
typedef boost::shared_ptr<ImagePCAFactorizedProblem> ImagePCAFactorizedProblemPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to ImagePCAFactorizedProblem class
 */
// -----------------------------------------------------------------------------
typedef std::vector<ImagePCAFactorizedProblem> ImagePCAFactorizedProblems;

// ----------------------------------------------------------------------------
/**
 * @class ImagePCAFactorizedProblem
 * @brief The interface for the ImagePCAFactorizedProblem to be used with Optimizer
 * 
 * The factorization methods assume that the optimization problem jacobian J, 
 * can be factorized into J=M0*Sigma matrix where Sigma matrix depends only
 * on the motion parameters and M0 matrix depends only on the object model 
 * features and coordinates.
 */
// -----------------------------------------------------------------------------
class ImagePCAFactorizedProblem: public FactorizedJacobianProblem
{
public:

  ImagePCAFactorizedProblem
    (
    ImagePCAModelPtr object_model,
    MotionModelPtr motion_model
    );

  virtual ~ImagePCAFactorizedProblem
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
  updateMotionParams
    (
    cv::Mat& params,
    cv::Mat& inc_params
    );

  cv::Mat
  computeInverseJacobian
    (
    cv::Mat& params
    );
    
protected:    
    
  virtual cv::Mat
  computeSigmaMatrix
    (
    cv::Mat& params
    ) = 0;
    
  virtual cv::Mat
  computeM0Matrix
    () = 0;

  virtual void
  initialize
    ();
    
 ImagePCAModelPtr m_pca_model;
    
  cv::Mat m_M0t_NB;
  cv::Mat m_M0t_NB_M0;

  /** This is the last rectified (warped image) seen as a vector */
  cv::Mat m_features_vector;
}; 

  
}; }; // namespace

#endif
