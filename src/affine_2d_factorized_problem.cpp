// -----------------------------------------------------------------------------
/**
 *  @brief Problem using a template image with affine motion model
 *  @author Jose M. Buenaposada
 *  @date 2012/07/25
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

#include "affine_2d_factorized_problem.hpp"
//#include "trace.hpp"

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
Affine2DFactorizedProblem::Affine2DFactorizedProblem
  (
  ObjectModelPtr object_model,
  MotionModelPtr motion_model
  ): 
  FactorizedJacobianProblem(object_model, motion_model)
{
  if (!m_initialized)
  {
    initialize();
  }

//   m_invM0 = m_M0t_M0.inv() * m_M0.t();
  m_invM0 = m_M0.inv(cv::DECOMP_SVD);
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
Affine2DFactorizedProblem::~Affine2DFactorizedProblem
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
Affine2DFactorizedProblem::computeCostFunction
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
// Purpose and Method: 
// Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
// -----------------------------------------------------------------------------  
cv::Mat
Affine2DFactorizedProblem::computeResidual
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
  
  assert(params.rows == 6);
  assert(params.cols == 1);
    
  m_object_model->getReferenceCoords(template_coords);
  warped_image      = m_motion_model->warpImage(image, params, template_coords, ctrl_coords_indices);
  warped_image_gray = cv::Mat::zeros(warped_image.rows, warped_image.cols, cv::DataType<uchar>::type);
  
  if (warped_image.channels() == 3)
  {
    cvtColor(warped_image, warped_image_gray, cv::COLOR_RGB2GRAY);
  }
  else
  {
//    warped_image_gray = warped_image.clone();
    warped_image.convertTo(warped_image_gray, cv::DataType<uchar>::type);
  }

  cv::Mat features_vector          = m_object_model->extractFeaturesFromWarpedImage(warped_image_gray);
  cv::Mat template_features_vector = m_object_model->computeTemplateFeatures(params);

  residual = features_vector - template_features_vector;

#ifdef DEBUG_
  // Show the warped image
  cv::namedWindow("warped image");
  cv::imshow("warped image", warped_image_gray);

  cv::namedWindow("template");
  cv::Mat template_uchar;
//  template_features_vector.reshape(1, warped_image.rows).convertTo(template_uchar, cv::DataType<uchar>::type);
//  cv::imshow("template", template_uchar);
  template_features_vector.reshape(1, warped_image.rows).copyTo(template_uchar);
  cv::imshow("template", template_uchar);

  cv::namedWindow("residual");
  cv::Mat residual_uchar;
//  residual.convertTo(residual_uchar, cv::DataType<uchar>::type);
//  cv::imshow("residual", residual_uchar.reshape(1, warped_image.rows));
  cv::imshow("residual", residual.reshape(1, warped_image.rows));

  // write Mat objects to the file
  cv::FileStorage fs("Affine2DFactorizedProblem_computeResidual.xml", cv::FileStorage::WRITE);
  fs << "template_features_vector" << template_features_vector;
  fs << "features_vector" << features_vector;
  fs << "params" << params;
  fs << "residual" << residual;
  fs.release();
//   cv::imwrite("Affine2DFactorizedProblem_computeResidual_warped_image.bmp", warped_image_gray);
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
Affine2DFactorizedProblem::computeInverseJacobian
  (
  cv::Mat& params
  )
{
  assert(params.rows == 6);
  assert(params.cols == 1);
  
  cv::Mat Sigma    = computeSigmaMatrix(params);
  cv::Mat invSigma = Sigma.inv();
  cv::Mat invJ;
  invJ             = invSigma * m_invM0;

#ifdef DEBUG_
  // write Mat objects to the file
  cv::FileStorage fs("Affine2DFactorizedProblem_computeInverseJacobian.xml", cv::FileStorage::WRITE);
  fs << "params" << params;
  fs << "Sigma" << Sigma;
  fs << "invSigma" << invSigma;
  fs << "invJ" << invJ;
  fs << "M0" << m_M0;
  fs << "m_invM0" << m_invM0;
  fs.release();
#endif

  return invJ;
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
Affine2DFactorizedProblem::updateMotionParams
  (
  cv::Mat& params,
  cv::Mat& delta_params
  )
{
  return params + delta_params;
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
Affine2DFactorizedProblem::computeSigmaMatrix
  (
  cv::Mat& params
  )
{ 
  assert(params.rows == 6);
  assert(params.cols == 1);
  
//  MAT_TYPE tx  = params.at<MAT_TYPE>(0, 0);
//  MAT_TYPE ty  = params.at<MAT_TYPE>(1, 0);
  MAT_TYPE a   = params.at<MAT_TYPE>(2, 0);
  MAT_TYPE b   = params.at<MAT_TYPE>(3, 0);
  MAT_TYPE c   = params.at<MAT_TYPE>(4, 0);
  MAT_TYPE d   = params.at<MAT_TYPE>(5, 0);
  
  cv::Mat A    = (cv::Mat_<MAT_TYPE>(2,2) << a, c,  
		                             b, d);
  cv::Mat invA = A.inv();
  
  cv::Mat Sigma     = cv::Mat::zeros(6, 6, cv::DataType<MAT_TYPE>::type);
  cv::Mat Sigma_roi = Sigma(cv::Range(0,2), cv::Range(0,2));
  invA.copyTo(Sigma_roi);
  Sigma_roi = Sigma(cv::Range(2,4), cv::Range(2,4));
  invA.copyTo(Sigma_roi);
  Sigma_roi = Sigma(cv::Range(4,6), cv::Range(4,6));
  invA.copyTo(Sigma_roi);
    
  return Sigma;
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
Affine2DFactorizedProblem::computeM0Matrix
  ()
{
  cv::Mat template_coords; 
  std::vector<int> ctrl_coords_indices;
  cv::Mat M0;
  cv::Mat gradients;
  cv::Mat zero_params;
//  MAT_TYPE norm_grad;
  std::vector<LineIndices> ctrl_coords_lines;
  
  m_object_model->getReferenceCoords(template_coords);
  M0              = cv::Mat::zeros(template_coords.rows, m_motion_model->getNumParams(), cv::DataType<MAT_TYPE>::type);
  zero_params     = cv::Mat::zeros(m_motion_model->getNumParams(), 1, cv::DataType<MAT_TYPE>::type);
  gradients       = m_object_model->computeTemplateFeaturesGradient(zero_params);

  assert(gradients.rows == M0.rows);
  assert(gradients.cols == 2);

  gradients.col(0).copyTo(M0.col(0)); // grad_x
  gradients.col(1).copyTo(M0.col(1)); // grad_y
  M0.col(2) = gradients.col(0).mul(template_coords.col(0)); // grad_x .* x
  M0.col(3) = gradients.col(1).mul(template_coords.col(0)); // grad_y .* x
  M0.col(4) = gradients.col(0).mul(template_coords.col(1)); // grad_x .* y
  M0.col(5) = gradients.col(1).mul(template_coords.col(1)); // grad_y .* y

//  MAT_TYPE x, y;
//  MAT_TYPE grad_x, grad_y;
//  for (int i=0; i < M0.rows; i++)
//  {
//    x      = template_coords.at<MAT_TYPE>(i,0);
//    y      = template_coords.at<MAT_TYPE>(i,1);
//    grad_x = gradients.at<MAT_TYPE>(i,0);
//    grad_y = gradients.at<MAT_TYPE>(i,1);
  
//    M0.at<MAT_TYPE>(i,0) = grad_x;
//    M0.at<MAT_TYPE>(i,1) = grad_y;
//    M0.at<MAT_TYPE>(i,2) = (grad_x * x);
//    M0.at<MAT_TYPE>(i,3) = (grad_y * x);
//    M0.at<MAT_TYPE>(i,4) = (grad_x * y);
//    M0.at<MAT_TYPE>(i,5) = (grad_y * y);
//  }

#ifdef DEBUG
  cv::namedWindow("M0");
  cv::Mat col = template_coords.col(0);
  MAT_TYPE min_x = *std::min_element(col.begin<MAT_TYPE>(), col.end<MAT_TYPE>());
  MAT_TYPE max_x = *std::max_element(col.begin<MAT_TYPE>(), col.end<MAT_TYPE>());
  col = template_coords.col(1);
  MAT_TYPE min_y = *std::min_element(col.begin<MAT_TYPE>(), col.end<MAT_TYPE>());
  MAT_TYPE max_y = *std::max_element(col.begin<MAT_TYPE>(), col.end<MAT_TYPE>());

  int img_width  = round(max_x - min_x+ 1);
  int img_height = round(max_y - min_y + 1);
  
  cv::Mat Jacobian_image = cv::Mat::zeros(img_height, img_width * M0.cols, cv::DataType<uint8_t>::type);

  for (int i=0; i<M0.cols; i++)
  {
    cv::Mat M0_i;
    M0.col(i).copyTo(M0_i);  
    cv::Mat normalized;
    cv::normalize(M0_i.reshape(1, img_height), normalized, 0, 255, cv::NORM_MINMAX,  cv::DataType<uint8_t>::type);
    cv::Mat Jacobian_image_ref = Jacobian_image(cv::Range::all(), cv::Range(i*img_width, (i+1)*img_width));
    normalized.copyTo(Jacobian_image_ref);
  }
  cv::imshow("M0", Jacobian_image);

  // write Mat objects to the file
  cv::FileStorage fs("Affine2DFactorizedProblem_computeM0Matrix.xml", cv::FileStorage::WRITE);
  fs << "M0_0" << M0.col(0);
  fs << "M0_1" << M0.col(1);
  fs << "M0_2" << M0.col(2);
  fs << "M0_3" << M0.col(3);
  fs << "M0_4" << M0.col(3);
  fs << "template_coords" << template_coords;
  fs.release();
#endif  

  return M0;  
};
  
  
}; }; // namespace
