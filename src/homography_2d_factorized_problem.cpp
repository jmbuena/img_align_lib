// -----------------------------------------------------------------------------
/**
 *  @brief Problem using a template image with homography motion model
 *  @author Jose M. Buenaposada
 *  @date 2012/10/15
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

#include "homography_2d_factorized_problem.hpp"
#include "trace.hpp"

//#define DEBUG

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
Homography2DFactorizedProblem::Homography2DFactorizedProblem
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
  
  m_invM0 = m_M0.inv(cv::DECOMP_SVD);
};

// -----------------------------------------------------------------------------
//
// Purpose and Method: 
// // Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
// -----------------------------------------------------------------------------  
Homography2DFactorizedProblem::~Homography2DFactorizedProblem
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
Homography2DFactorizedProblem::computeCostFunction
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
Homography2DFactorizedProblem::computeResidual
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
  
  assert(params.rows == m_motion_model->getNumParams());
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
    warped_image.convertTo(warped_image_gray, cv::DataType<uchar>::type);
  }

  cv::Mat features_vector          = m_object_model->extractFeaturesFromWarpedImage(warped_image_gray);
  cv::Mat template_features_vector = m_object_model->computeTemplateFeatures(params);

  residual = features_vector - template_features_vector;

#ifdef DEBUG
  // Show the warped image
  cv::namedWindow("warped image");
  cv::imshow("warped image", warped_image_gray);

  cv::namedWindow("template");
  cv::Mat template_uchar;
//  template_features_vector.reshape(1, warped_image.rows).convertTo(template_uchar, cv::DataType<uchar>::type);
  template_features_vector.reshape(1, warped_image.rows).copyTo(template_uchar);
  cv::imshow("template", template_uchar);
  
  cv::namedWindow("residual");
//  cv::Mat residual_uchar;
//  residual.convertTo(residual_uchar, cv::DataType<uchar>::type);
//  cv::imshow("residual", residual_uchar.reshape(1, warped_image.rows));
  cv::imshow("residual", residual.reshape(1, warped_image.rows));

  // write Mat objects to the file
  cv::FileStorage fs("Homography2DFactorizedProblem_computeResidual.xml", cv::FileStorage::WRITE);
  fs << "template_features_vector" << template_features_vector;
  fs << "features_vector" << features_vector;
  fs << "params" << params;
  fs << "warped_image" << residual.reshape(1, warped_image.rows);
  fs << "residual" << residual;
  fs.release();
  cv::imwrite("Homography2DFactorizedProblem_computeResidual_warped_image.bmp", warped_image_gray);
#endif   
  
  return residual; 
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
Homography2DFactorizedProblem::computeInverseJacobian
  (
  cv::Mat& params
  )
{
  assert(params.rows == m_motion_model->getNumParams());
  assert(params.cols == 1);

  cv::Mat params2 = cv::Mat::ones(9,1, cv::DataType<MAT_TYPE>::type);
  params.copyTo(params2(cv::Range(0,8), cv::Range::all()));
  cv::Mat H            = params2.reshape(1, 3).t();
  cv::Mat invH         = H.inv();

//  double tic = (double)cv::getTickCount();
#define VERSION1
#ifdef VERSION1
  cv::Mat Sigma = computeSigmaMatrix(params);
  cv::Mat Hess  = Sigma.t() * m_M0t_M0 * Sigma;
  cv::Mat invJ  = Hess.inv() * (Sigma.t() * m_M0.t());
#endif
#ifdef VERSION2
  cv::Mat J = cv::Mat::zeros(m_M0.rows, m_motion_model->getNumParams(), cv::DataType<MAT_TYPE>::type);
  J(cv::Range::all(), cv::Range(0,3)) = m_M0(cv::Range::all(), cv::Range(0,3)) * invH;
  J(cv::Range::all(), cv::Range(3,6)) = m_M0(cv::Range::all(), cv::Range(3,6)) * invH;
  J(cv::Range::all(), cv::Range(6,8)) = m_M0(cv::Range::all(), cv::Range(6,9)) * invH(cv::Range::all(), cv::Range(0,2));
  cv::Mat invJ  = J.inv(cv::DECOMP_SVD);
#endif
//  double toc = ((double)cv::getTickCount() - tic)/cv::getTickFrequency();
//  TRACE_INFO("computeInverseJacobian time = " << toc << std::endl);

#ifdef DEBUG
  // write Mat objects to the file
  cv::FileStorage fs("Homography2DFactorizedProblem_computeInverseJacobian.xml", cv::FileStorage::WRITE);
  fs << "params" << params;
  fs << "Sigma" << Sigma;
//  fs << "Hess" << Hess;
  fs << "invJ" << invJ;
  fs << "m_invM0" << m_invM0;
  fs << "M0" << m_M0;
  fs << "m_M0t_M0" << m_M0t_M0;
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
Homography2DFactorizedProblem::updateMotionParams
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
Homography2DFactorizedProblem::computeSigmaMatrix
  (
  cv::Mat& params
  )
{ 
  assert(params.rows == m_motion_model->getNumParams());
  assert(params.cols == 1);

  cv::Mat params2 = cv::Mat::ones(9,1, cv::DataType<MAT_TYPE>::type);
  params.copyTo(params2(cv::Range(0,8), cv::Range::all()));
  cv::Mat H            = params2.reshape(1, 3).t();
  cv::Mat invH         = H.inv();
  
  cv::Mat Sigma     = cv::Mat::zeros(9, 8, cv::DataType<MAT_TYPE>::type);
  invH.copyTo(Sigma(cv::Range(0,3), cv::Range(0,3)));
  invH.copyTo(Sigma(cv::Range(3,6), cv::Range(3,6)));
//  invH.copyTo(Sigma(cv::Range(6,9), cv::Range(6,9)));
  cv::Mat invH_roi = invH(cv::Range(0,3), cv::Range(0,2));
  invH_roi.copyTo(Sigma(cv::Range(6,9), cv::Range(6,8)));

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
Homography2DFactorizedProblem::computeM0Matrix
  ()
{
  cv::Mat template_coords; 
  cv::Mat M0;
  cv::Mat gradients;
  cv::Mat zero_params;
  std::vector<LineIndices> ctrl_coords_lines;
  
  m_object_model->getReferenceCoords(template_coords);
//  M0              = cv::Mat::zeros(template_coords.rows, m_motion_model->getNumParams(), cv::DataType<MAT_TYPE>::type);
  M0              = cv::Mat::zeros(template_coords.rows, 9, cv::DataType<MAT_TYPE>::type);
  zero_params     = cv::Mat::zeros(m_motion_model->getNumParams(), 1, cv::DataType<MAT_TYPE>::type);
  gradients       = m_object_model->computeTemplateFeaturesGradient(zero_params);

  assert(gradients.rows == M0.rows);
  assert(gradients.cols == 2);

  M0.col(0) = gradients.col(0).mul(template_coords.col(0)); // grad_x * x
  M0.col(1) = gradients.col(1).mul(template_coords.col(0)); // grad_y * x
  cv::Mat grad_xy = -(M0.col(0) + M0.col(1));
  M0.col(2) = grad_xy.mul(template_coords.col(0)); // grad_xy * x
  M0.col(3) = gradients.col(0).mul(template_coords.col(1)); // grad_x * y
  M0.col(4) = gradients.col(1).mul(template_coords.col(1)); // grad_y * y
  M0.col(5) = grad_xy.mul(template_coords.col(1)); // grad_xy * y
  gradients.col(0).copyTo(M0.col(6)); // grad_x
  gradients.col(1).copyTo(M0.col(7)); // grad_y

#ifdef DEBUG  
  cv::namedWindow("M0");
  
  cv::Mat col = template_coords.col(0);
  MAT_TYPE min_x = *std::min_element(col.begin<MAT_TYPE>(), col.end<MAT_TYPE>());
  MAT_TYPE max_x = *std::max_element(col.begin<MAT_TYPE>(), col.end<MAT_TYPE>());
  col = template_coords.col(1);
  MAT_TYPE min_y = *std::min_element(col.begin<MAT_TYPE>(), col.end<MAT_TYPE>());
  MAT_TYPE max_y = *std::max_element(col.begin<MAT_TYPE>(), col.end<MAT_TYPE>());

  int img_width  = round(max_x - min_x + 1);
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
  cv::FileStorage fs("Homography2DFactorizedProblem_computeM0Matrix.xml", cv::FileStorage::WRITE);
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
