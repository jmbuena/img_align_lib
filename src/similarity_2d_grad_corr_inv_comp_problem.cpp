// -----------------------------------------------------------------------------
/**
 *  @brief Problem using a template image gradients correlation with similarity motion model
 *  @author Jose M. Buenaposada
 *  @date 2012/07/16
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

#include "similarity_2d_grad_corr_inv_comp_problem.hpp"
#include "trace.hpp"

const double TINY_NUMBER       = 1.E-30;
const float  TINY_FLOAT_NUMBER = 1.E-30;

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
Similarity2DGradCorrInvCompProblem::Similarity2DGradCorrInvCompProblem
  (
//   SingleImageGradientsModelPtr object_model,
  ObjectModelPtr object_model,
  MotionModelPtr motion_model
  ): 
  InverseCompositionalProblem(object_model, motion_model)
{
  if (!m_initialized)
  {
    initialize();
  }
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
Similarity2DGradCorrInvCompProblem::~Similarity2DGradCorrInvCompProblem
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
computeCorrelationOfGradients
  (
  cv::Mat& features_vector,
  cv::Mat template_features_vector
  )
{
//   cv::Mat q_p_mat;
  double q_p = 0.0;
  
//   cv::Mat col_features; 
//   cv::Mat col_template_features;
// 
//   q_p_mat = (features_vector.col(0).t() * template_features_vector.col(0)) + 
//             (features_vector.col(1).t() * template_features_vector.col(1));
// 	    
//   q_p     = q_p_mat.at<MAT_TYPE>(0,0);	    
	
  for (int i=0; i<features_vector.rows; i++)
  {
    MAT_TYPE grad_x          = features_vector.at<MAT_TYPE>(i,0);
    MAT_TYPE grad_y          = features_vector.at<MAT_TYPE>(i,1);
    MAT_TYPE template_grad_x = template_features_vector.at<MAT_TYPE>(i,0);
    MAT_TYPE template_grad_y = template_features_vector.at<MAT_TYPE>(i,1);

	double norm_grad = std::max<double>(sqrt(grad_x*grad_x + grad_y*grad_y), TINY_NUMBER);
	double norm_template_grad = std::max<double>(sqrt(template_grad_x*template_grad_x + template_grad_y*template_grad_y), TINY_NUMBER);
    
    q_p += (grad_x/norm_grad)*(template_grad_x/norm_template_grad) +
           (grad_y/norm_grad)*(template_grad_y/norm_template_grad);
  }    
  
  return q_p;
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
double
Similarity2DGradCorrInvCompProblem::computeCostFunction
  (
  cv::Mat& residual_vector,
  cv::Mat& params,
  cv::Mat& delta_params,
  cv::Mat& image
  )
{
  cv::Mat warped_image, warped_image_gray;
  cv::Mat template_coords;
  std::vector<int> ctrl_coords_indices;
  double cost          = 0.0;
  cv::Mat J            = this->computeJacobian(params);    
  
  // FIXME: the q_p scalar value depends on rectified input image gradients.
  double q_p = 0.0;
  
  m_object_model->getReferenceCoords(template_coords);
  warped_image      = m_motion_model->warpImage(image, params, template_coords,
                                                ctrl_coords_indices);
  warped_image_gray = cv::Mat::zeros(warped_image.rows, warped_image_gray.cols,
                                     cv::DataType<uchar>::type);
  
  if (warped_image.channels() == 3)
  {
    cv::cvtColor(warped_image, warped_image_gray, CV_RGB2GRAY);
  }
  else
  {
    warped_image_gray = warped_image.clone();
  }

  cv::Mat features_vector = m_object_model->extractFeaturesFromWarpedImage(warped_image_gray);
  cv::Mat template_features_vector = m_object_model->computeTemplateFeatures(params);
  q_p = computeCorrelationOfGradients(features_vector, template_features_vector); 
  
  cv::Mat cost_num_mat = residual_vector.t()*J*delta_params;
  cv::Mat cost_den_mat = delta_params.t() * J.t() * J * delta_params;
  
  cost = (q_p + cost_num_mat.at<MAT_TYPE>(0,0))/sqrt(residual_vector.rows +
                                                     cost_den_mat.at<MAT_TYPE>(0,0));
  cost = -cost;
  
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
Similarity2DGradCorrInvCompProblem::computeResidual
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
  double angle_warped_image;
  double angle_template;
  double q_p;
  
  assert(params.rows == 4);
  assert(params.cols == 1);
    
  m_object_model->getReferenceCoords(template_coords);
  warped_image      = m_motion_model->warpImage(image, params, template_coords,
                                                ctrl_coords_indices);

  warped_image_gray = cv::Mat::zeros(warped_image.rows, warped_image_gray.cols,
                                     cv::DataType<uchar>::type);
  
  if (warped_image.channels() == 3)
  {
    cv::cvtColor(warped_image, warped_image_gray, CV_RGB2GRAY);
  }
  else
  {
    warped_image_gray = warped_image.clone();
  }

  cv::Mat features_vector          = m_object_model->extractFeaturesFromWarpedImage(warped_image_gray);
  cv::Mat template_features_vector = m_object_model->computeTemplateFeatures(params);
  
  // The features vector in this case are, Nx2 matrices: 
  //   - the first column has the x image gradient 
  //   - the second column has the y image gradient
  assert(features_vector.rows == warped_image.rows * warped_image.cols);
  assert(features_vector.cols == 2);
  assert(template_features_vector.rows == features_vector.rows);
  assert(template_features_vector.cols == features_vector.cols);
  
  q_p = computeCorrelationOfGradients(features_vector, template_features_vector); 
  
  residual = cv::Mat::zeros(features_vector.rows, 1, cv::DataType<MAT_TYPE>::type);
  for (int i=0; i<features_vector.rows; i++)
  {
    angle_warped_image = atan2(features_vector.at<MAT_TYPE>(i,1),
                               std::max(features_vector.at<MAT_TYPE>(i,0), TINY_FLOAT_NUMBER));
    angle_template     = atan2(template_features_vector.at<MAT_TYPE>(i,1),
                               std::max(template_features_vector.at<MAT_TYPE>(i,0), TINY_FLOAT_NUMBER));
//     angle_warped_image = atan(features_vector.at<MAT_TYPE>(i,1)/std::max(features_vector.at<MAT_TYPE>(i,0), TINY_FLOAT_NUMBER));
//     angle_template     = atan(template_features_vector.at<MAT_TYPE>(i,1)/std::max(template_features_vector.at<MAT_TYPE>(i,0), TINY_FLOAT_NUMBER));
    residual.at<MAT_TYPE>(i,0) = (static_cast<MAT_TYPE>(features_vector.rows)/q_p) *
                                  sin(angle_template - angle_warped_image);
//     residual.at<MAT_TYPE>(i,0) =  sin(angle_template - angle_warped_image);
  }
  
#ifdef DEBUG
  // Show the warped image
  cv::namedWindow("warped image");
  cv::imshow("warped image", warped_image_gray);

  cv::namedWindow("template Gx");
  cv::Mat col;
  template_features_vector.col(0).convertTo(col, cv::DataType<MAT_TYPE>::type);
  cv::Mat normalized;  
  cv::normalize(col.reshape(1, 50), normalized, 0, 255, cv::NORM_MINMAX,  cv::DataType<uint8_t>::type);
  cv::imshow("template Gx", normalized);

 
  cv::namedWindow("template Gy");
  template_features_vector.col(1).convertTo(col, cv::DataType<MAT_TYPE>::type);
  cv::normalize(col.reshape(1, 50), normalized, 0, 255, cv::NORM_MINMAX,  cv::DataType<uint8_t>::type);
  cv::imshow("template Gy", normalized);

  cv::namedWindow("image Gx");
  features_vector.col(0).convertTo(col, cv::DataType<MAT_TYPE>::type);
  cv::normalize(col.reshape(1, 50), normalized, 0, 255, cv::NORM_MINMAX,  cv::DataType<uint8_t>::type);
  cv::imshow("image Gx", normalized);

  cv::namedWindow("image Gy");
  features_vector.col(1).convertTo(col, cv::DataType<MAT_TYPE>::type);
  cv::normalize(col.reshape(1, 50), normalized, 0, 255, cv::NORM_MINMAX,  cv::DataType<uint8_t>::type);
  cv::imshow("image Gy", normalized);
  
  cv::namedWindow("residual");
  residual.convertTo(col, cv::DataType<MAT_TYPE>::type);
  cv::normalize(col.reshape(1, 50), normalized, 0, 255, cv::NORM_MINMAX,  cv::DataType<uint8_t>::type);
  cv::imshow("residual", normalized);

  // write Mat objects to the file
  cv::FileStorage fs("Similarity2DGradCorrInvCompProblem_computeResidual.xml", cv::FileStorage::WRITE);
  fs << "template_features_vector" << template_features_vector;
  fs << "features_vector" << features_vector;
  fs << "params" << params;
  fs << "residual" << residual;
  fs.release();
//   cv::imwrite("Similarity2DGradCorrInvCompProblem_computeResidual_warped_image.bmp", warped_image_gray);
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
Similarity2DGradCorrInvCompProblem::computeInverseJacobian
  (
  cv::Mat& params
  )
{
  assert(params.rows == 4);
  assert(params.cols == 1);
  
#ifdef DEBUG
  // write Mat objects to the file
  cv::FileStorage fs("Similarity2DGradCorrInvCompProblem_computeInverseJacobian.xml",
                     cv::FileStorage::WRITE);
  fs << "m_inv_J" << m_inv_J;
  fs.release();
#endif  

  return m_inv_J;
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
Similarity2DGradCorrInvCompProblem::updateMotionParams
  (
  cv::Mat& params,
  cv::Mat& delta_params
  )
{
  cv::Mat new_params = cv::Mat::zeros(4, 1, cv::DataType<MAT_TYPE>::type);
  
  // The inverse composition of the f(\vx, delta_params) and f(\vx, params),
  // is f(f(\vx, delta_params), params) = f(\vx, new_params). Therefore
  // we should estimate the new_params from the motion model composition.
  
  assert(params.rows == 4);
  assert(params.cols == 1);
  
  // params vector is params = [tx ty alpha scale]';
  MAT_TYPE alpha       = params.at<MAT_TYPE>(2,0);
  MAT_TYPE scale       = params.at<MAT_TYPE>(3,0);
  MAT_TYPE cosA        = cos(alpha);
  MAT_TYPE sinA        = sin(alpha);

  MAT_TYPE delta_alpha = delta_params.at<MAT_TYPE>(2,0);
  MAT_TYPE delta_scale = delta_params.at<MAT_TYPE>(3,0);
  MAT_TYPE delta_cosA  = cos(delta_alpha);
  MAT_TYPE delta_sinA  = sin(delta_alpha);

  cv::Mat R       = (cv::Mat_<MAT_TYPE>(2,2) << cosA, (-sinA), 
	 	                                sinA, cosA);

  cv::Mat delta_R = (cv::Mat_<MAT_TYPE>(2,2) << delta_cosA, (-delta_sinA), 
	 	                                delta_sinA, delta_cosA);
  cv::Mat new_R                = R*delta_R.t();
  cv::Mat translation          = params(cv::Range(0,2), cv::Range(0,1));
  cv::Mat delta_translation    = delta_params(cv::Range(0,2), cv::Range(0,1));
  double scale_ratio           = (1.0+scale)/(1.0+delta_scale);
  cv::Mat new_translation      = translation - (scale_ratio*(new_R*delta_translation));

  new_params.at<MAT_TYPE>(0,0) = new_translation.at<MAT_TYPE>(0,0);
  new_params.at<MAT_TYPE>(1,0) = new_translation.at<MAT_TYPE>(1,0); 
  cv::Mat new_R_column         = new_R(cv::Range(0,2),cv::Range(0,1));
  new_params.at<MAT_TYPE>(2,0) = asin(new_R.at<MAT_TYPE>(1,0)/norm(new_R_column));
  new_params.at<MAT_TYPE>(3,0) = scale_ratio - 1.0;

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
Similarity2DGradCorrInvCompProblem::computeConstantJacobian
  ()
{
  cv::Mat template_coords; 
  std::vector<int> ctrl_coords_indices;
  cv::Mat J;
  cv::Mat hessians;
  cv::Mat gradients;
  cv::Mat zero_params;
  MAT_TYPE x, y;
  MAT_TYPE grad_xx, grad_xy, grad_yx, grad_yy;
  MAT_TYPE grad_x, grad_y;
  MAT_TYPE norm_grad;
  MAT_TYPE squared_norm_grad;
  std::vector<LineIndices> ctrl_coords_lines;
  
  m_object_model->getReferenceCoords(template_coords);
  J           = cv::Mat::zeros(template_coords.rows, m_motion_model->getNumParams(), cv::DataType<MAT_TYPE>::type);
  zero_params = cv::Mat::zeros(m_motion_model->getNumParams(), 1, cv::DataType<MAT_TYPE>::type);
  hessians    = m_object_model->computeTemplateFeaturesGradient(zero_params);
  gradients   = m_object_model->computeTemplateFeatures(zero_params);

  assert(hessians.rows == J.rows);
  assert(hessians.cols == 4);

  for (int i=0; i < J.rows; i++)
  {     
    x       = template_coords.at<MAT_TYPE>(i,0);
    y       = template_coords.at<MAT_TYPE>(i,1);
    grad_xx = hessians.at<MAT_TYPE>(i,0);
    grad_xy = hessians.at<MAT_TYPE>(i,1);
    grad_yx = hessians.at<MAT_TYPE>(i,2);
    grad_yy = hessians.at<MAT_TYPE>(i,2);
    grad_x  = gradients.at<MAT_TYPE>(i,0);
    grad_y  = gradients.at<MAT_TYPE>(i,1);
    
    norm_grad         = std::max<MAT_TYPE>(sqrt((grad_x*grad_x) + (grad_y*grad_y)), TINY_NUMBER);
    grad_x           /= norm_grad;
    grad_y           /= norm_grad;
    
    cv::Mat f_params  = (cv::Mat_<MAT_TYPE>(2,4) << 1., 0., -y, x,
	 	                                    0., 1.,  x, y);
    cv::Mat H         = (cv::Mat_<MAT_TYPE>(2,2) << grad_xx, grad_xy,
	 	                                    grad_yx, grad_yy);
    cv::Mat deriv     = H * f_params;
    
    for (int j=0; j < 4; j++)
    {
      J.at<MAT_TYPE>(i,j)  = (grad_x * deriv.at<MAT_TYPE>(1,j)) - (grad_y * deriv.at<MAT_TYPE>(0,j));
      J.at<MAT_TYPE>(i,j) /= norm_grad;
    }
  }
  
#ifdef DEBUG
  cv::namedWindow("J_0");
  cv::Mat J_0;
  J.col(0).copyTo(J_0);  
  cv::Mat normalized;
  cv::normalize(J_0.reshape(1, 50), normalized, 0, 255, cv::NORM_MINMAX,  cv::DataType<uint8_t>::type);
  cv::imshow("J_0", normalized);

  cv::namedWindow("J_1");
  cv::Mat J_1;
  J.col(1).copyTo(J_1);  
  cv::normalize(J_1.reshape(1, 50), normalized, 0, 255, cv::NORM_MINMAX, cv::DataType<uint8_t>::type);
  cv::imshow("J_1", normalized);

  cv::namedWindow("J_2");
  cv::Mat J_2;
  J.col(2).copyTo(J_2);  
  cv::normalize(J_2.reshape(1, 50), normalized, 0, 255, cv::NORM_MINMAX, cv::DataType<uint8_t>::type);
  cv::imshow("J_2", normalized);

  cv::namedWindow("J_3");
  cv::Mat J_3;
  J.col(3).copyTo(J_3);  
  cv::normalize(J_3.reshape(1, 50), normalized, 0, 255, cv::NORM_MINMAX, cv::DataType<uint8_t>::type);
  cv::imshow("J_3", normalized);

  cv::namedWindow("Hessian xx");
  cv::Mat col;
  hessians.col(0).copyTo(col);  
  cv::normalize(col.reshape(1, 50), normalized, 0, 255, cv::NORM_MINMAX,  cv::DataType<uint8_t>::type);
  cv::imshow("Hessian xx", normalized);

  cv::namedWindow("Hessian xy");
  hessians.col(1).copyTo(col);  
  cv::normalize(col.reshape(1, 50), normalized, 0, 255, cv::NORM_MINMAX,  cv::DataType<uint8_t>::type);
  cv::imshow("Hessian xy", normalized);
  
//   cv::namedWindow("Hessian yx");
//   hessians.col(2).copyTo(col);  
//   cv::normalize(col.reshape(1, 50), normalized, 0, 255, cv::NORM_MINMAX,  cv::DataType<uint8_t>::type);
//   cv::imshow("Hessian yx", normalized);

  cv::namedWindow("Hessian yy");
  hessians.col(2).copyTo(col);  
  cv::normalize(col.reshape(1, 50), normalized, 0, 255, cv::NORM_MINMAX,  cv::DataType<uint8_t>::type);
  cv::imshow("Hessian yy", normalized);

  // write Mat objects to the file
  cv::FileStorage fs("Similarity2DGradCorrInvCompProblem_computeJMatrix.xml", cv::FileStorage::WRITE);
  fs << "J_0" << J.col(0);
  fs << "J_1" << J.col(1);
  fs << "J_2" << J.col(2);
  fs << "J_3" << J.col(3);
  fs << "template_coords" << template_coords;
  fs.release();
#endif  

  return J;  
};


  
}; }; // namespace
