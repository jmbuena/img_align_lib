// -----------------------------------------------------------------------------
/**
 *  @brief Problem using a PCA of images object model
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

#include "image_pca_factorized_problem.hpp"
#include "utils.hpp"
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
ImagePCAFactorizedProblem::ImagePCAFactorizedProblem
  (
  ImagePCAModelPtr object_model,
  MotionModelPtr motion_model
  ): 
  FactorizedJacobianProblem(object_model, motion_model)
{
  m_pca_model       = object_model;
  m_features_vector = cv::Mat::zeros(m_object_model->getNumOfReferenceCoords(), 
				     1, 
				     cv::DataType<uchar>::type);

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
ImagePCAFactorizedProblem::initialize
  ()
{  
  cv::Mat B     = m_pca_model->getBasisMatrix();
  cv::Mat invB  = m_pca_model->getInverseBasisMatrix();
  m_M0          = computeM0Matrix();
  
  cv::Mat NB    = cv::Mat::eye(B.rows, B.rows, cv::DataType<MAT_TYPE>::type); 
  NB            = NB - (B * invB);
  
  
  m_M0t_NB    = m_M0.t() * NB;
  m_M0t_NB_M0 = m_M0t_NB * m_M0;
  
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
ImagePCAFactorizedProblem::~ImagePCAFactorizedProblem
  () 
{
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
double 
ImagePCAFactorizedProblem::computeCostFunction
  (
  cv::Mat& residual_vector,
  cv::Mat& params,
  cv::Mat& delta_params,
  cv::Mat& image
  )
{
  float cost           = 0.0;

  cv::Mat J            = this->computeJacobian(params);    
  cv::Mat error_vector = J*delta_params + residual_vector;
  
  cv::Mat cost_        = error_vector.t() * error_vector;
  cost                 = cost_.at<MAT_TYPE>(0,0);
  
  return cost;
};
    
// -----------------------------------------------------------------------------
//
// Purpose and Method: ...
// Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
// -----------------------------------------------------------------------------  
cv::Mat
ImagePCAFactorizedProblem::computeResidual
  (
  cv::Mat& image,
  cv::Mat& params
  )
{
  cv::Mat template_coords;
  std::vector<int> ctrl_coords_indices;
  std::vector<LineIndices> ctrl_coords_lines;
  cv::Mat warped_image, warped_image_gray;
  cv::Mat residual; 
  
//   assert(params.rows == 4);
//   assert(params.cols == 1);
    
  m_object_model->getReferenceCoords(template_coords);
  warped_image      = m_motion_model->warpImage(image, params, template_coords, ctrl_coords_indices);
  warped_image_gray = cv::Mat::zeros(warped_image.rows, warped_image_gray.cols, cv::DataType<uchar>::type);
  
  if (warped_image.channels() == 3)
  {
    cvtColor(warped_image, warped_image_gray, cv::COLOR_RGB2GRAY);
  }
  else
  {
    warped_image_gray = warped_image.clone();
  }

  m_features_vector                = m_object_model->extractFeaturesFromWarpedImage(warped_image_gray);
  
  // get the reconstructed PCA model image using the current appearance params
  cv::Mat object_params            = params(cv::Range(m_motion_model->getNumParams(), params.rows), cv::Range::all());
  cv::Mat template_features_vector = m_object_model->computeTemplateFeatures(object_params);
  
  residual = m_features_vector - template_features_vector;

#ifdef DEBUG
  // Show the warped image
  cv::namedWindow("warped image")
  cv::imshow("warped image", warped_image_gray);

  cv::namedWindow("reconstructed");
  cv::Mat template_uchar;
  template_features_vector.reshape(1, warped_image.rows).convertTo(template_uchar, cv::DataType<uchar>::type);
  cv::imshow("reconstructed", template_uchar);
  
  cv::namedWindow("residual");
  cv::Mat residual_uchar;
  residual.convertTo(residual_uchar, cv::DataType<uchar>::type);
  cv::imshow("residual", residual_uchar.reshape(1, warped_image.rows));

  // write Mat objects to the file
  cv::FileStorage fs("ImagePCAFactorizedProblem_computeResidual.xml", cv::FileStorage::WRITE);
  fs << "template_features_vector" << template_features_vector;
  fs << "features_vector" << features_vector;
  fs << "params" << params;
  fs << "residual" << residual;
  fs.release();
//   cv::imwrite("ImagePCAFactorizedProblem_computeResidual_warped_image.bmp", warped_image_gray);
#endif   
  
  return residual; //residual_float;
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
ImagePCAFactorizedProblem::updateMotionParams
  (
  cv::Mat& params,
  cv::Mat& delta_motion_params
  )
{
  cv::Mat new_params          = params.clone(); 
  
  cv::Mat motion_params       = new_params(cv::Range(0,m_motion_model->getNumParams()), cv::Range::all()); 
  cv::Mat appearance_params   = new_params(cv::Range(m_motion_model->getNumParams(), params.rows), cv::Range::all()); 
  cv::Mat invB                = m_pca_model->getInverseBasisMatrix();
  cv::Mat mean                = m_pca_model->getMeanVector();
  cv::Mat J                   = computeJacobian(params);

  appearance_params = invB * ((J * delta_motion_params) + (m_features_vector - mean));
  motion_params    += delta_motion_params;
  
  return new_params;
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
ImagePCAFactorizedProblem::computeInverseJacobian
  (
  cv::Mat& params
  )
{
  if (!m_initialized)
  {
    initialize();
  }
  
  cv::Mat Sigma = computeSigmaMatrix(params);

  cv::Mat H     = Sigma.t() * m_M0t_NB_M0 * Sigma;
  cv::Mat invJ  = H.inv() * (Sigma.t() * m_M0t_NB);
  
  return invJ;
};
      
  
}; }; // namespace
