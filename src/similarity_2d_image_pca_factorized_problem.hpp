// -----------------------------------------------------------------------------
/**
 *  @brief Problem using a PCA of images with similarity motion model
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
#ifndef SIMILARITY_2D_IMAGE_PCA_FACTORIZED_PROBLEM_HPP
#define SIMILARITY_2D_IMAGE_PCA_FACTORIZED_PROBLEM_HPP

// ----------------------- INCLUDES --------------------------------------------
#include "image_pca_factorized_problem.hpp"
#include <boost/shared_ptr.hpp>
#include <vector>
#include <opencv/cv.hpp>

namespace upm { namespace pcr
{
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to the Similarity2DImagePCAFactorizedProblem type
 */
// -----------------------------------------------------------------------------
class Similarity2DImagePCAFactorizedProblem;
typedef boost::shared_ptr<Similarity2DImagePCAFactorizedProblem> Similarity2DImagePCAFactorizedProblemPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to Similarity2DImagePCAFactorizedProblem class
 */
// -----------------------------------------------------------------------------
typedef std::vector<Similarity2DImagePCAFactorizedProblem> Similarity2DImagePCAFactorizedProblems;

// ----------------------------------------------------------------------------
/**
 * @class Similarity2DImagePCAFactorizedProblem
 * @brief The interface for the Similarity2DImagePCAFactorizedProblem to be used with Optimizer
 * 
 * The factorization methods assume that the optimization problem jacobian J, 
 * can be factorized into J=M0*Sigma matrix where Sigma matrix depends only
 * on the motion parameters and M0 matrix depends only on the object model 
 * features and coordinates.
 * 
 * In this case the "motion paramameters" are extended with the "appearance parameters". We model 
 * the appearance of the tracked object with a PCA linear model (e.g. the illumination of the face).
 * The "apperance parameters" are then the PCA parameters of the model.
 * 
 * In this class a column vector (cv::Mat) "params" has first the "motion parameters" 
 * and after them the "appearance parameters". In this sense, if we have a 2D similarity
 * motion model the first 4 elements in "params" are the similarity motion paramters 
 * (tx, ty, orientantion, scale). 
 * 
 */
// -----------------------------------------------------------------------------
class Similarity2DImagePCAFactorizedProblem: public ImagePCAFactorizedProblem
{
public:

  Similarity2DImagePCAFactorizedProblem
    (
    ImagePCAModelPtr object_model,
    MotionModelPtr motion_model
    );

  virtual ~Similarity2DImagePCAFactorizedProblem
    ();
    
protected:    
    
  virtual cv::Mat
  computeSigmaMatrix
    (
    cv::Mat& params
    );
    
  virtual cv::Mat
  computeM0Matrix
    ();
}; 

  
}; }; // namespace

#endif
