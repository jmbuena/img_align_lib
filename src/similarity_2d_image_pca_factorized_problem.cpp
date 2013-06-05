// -----------------------------------------------------------------------------
/**
 *  @brief Problem using a PCA of images object model with similarity motion model
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

#include "similarity_2d_image_pca_factorized_problem.hpp"
#include "utils.hpp"
#include "trace.hpp"

#include <strstream>

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
Similarity2DImagePCAFactorizedProblem::Similarity2DImagePCAFactorizedProblem
  (
  ImagePCAModelPtr object_model,
  MotionModelPtr motion_model
  ): 
  ImagePCAFactorizedProblem(object_model, motion_model)
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
Similarity2DImagePCAFactorizedProblem::~Similarity2DImagePCAFactorizedProblem
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
cv::Mat
Similarity2DImagePCAFactorizedProblem::computeSigmaMatrix
  (
  cv::Mat& params
  )
{ 
  assert(params.rows > 4);
  assert(params.cols == 1);
  
  cv::Mat motion_params     = params(cv::Range(0,m_motion_model->getNumParams()), cv::Range::all()); 
  cv::Mat appearance_params = params(cv::Range(m_motion_model->getNumParams(), params.rows), cv::Range::all()); 

  // In this case the inverse jacobian is almost constant,
  // we store it and we use it
  MAT_TYPE alpha     = motion_params.at<MAT_TYPE>(2,0);
  MAT_TYPE scale     = motion_params.at<MAT_TYPE>(3,0);
//   MAT_TYPE inv_scale = 1.0/(scale + 1.0E-10);
  MAT_TYPE inv_scale = 1.0/scale;
  MAT_TYPE cosA      = cos(-alpha);
  MAT_TYPE sinA      = sin(-alpha);

  int num_basis      = 1 + appearance_params.rows;
  cv::Mat C          = cv::Mat::zeros(2 * num_basis, 2, cv::DataType<MAT_TYPE>::type);
  
  C.at<MAT_TYPE>(0, 0)         = 1.0;
  C.at<MAT_TYPE>(num_basis, 1) =  1.0;
  cv::Mat C_range1   = C(cv::Range(1,num_basis), cv::Range(0,1));
  cv::Mat C_range2   = C(cv::Range(num_basis + 1, C.rows), cv::Range(1,2));
  appearance_params.copyTo(C_range1);
  appearance_params.copyTo(C_range2);
  
  
  cv::Mat R = (cv::Mat_<MAT_TYPE>(2,2) << inv_scale * cosA, inv_scale * (-sinA), 
                                          inv_scale * sinA, inv_scale * cosA);
  cv::Mat CR = C * R;
  
  cv::Mat S = (cv::Mat_<MAT_TYPE>(2,2) << 1.0, 0., 
                                          0. , inv_scale);
  cv::Mat CS = C * S;
  
  cv::Mat Sigma = cv::Mat::zeros(2*C.rows, CR.cols + CS.cols, cv::DataType<MAT_TYPE>::type);

  cv::Mat Sigma_CR_range = Sigma(cv::Range(0, CR.rows), cv::Range(0, CR.cols));
  cv::Mat Sigma_CS_range = Sigma(cv::Range(CR.rows, Sigma.rows), cv::Range(CR.cols, Sigma.cols));
  CR.copyTo(Sigma_CR_range);
  CS.copyTo(Sigma_CS_range);
  
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
Similarity2DImagePCAFactorizedProblem::computeM0Matrix
  ()
{
  cv::Mat template_coords; 
  cv::Mat M0;
  cv::Mat gradients;
  std::vector<cv::Mat> gradients_vector;
  MAT_TYPE x, y;
  MAT_TYPE grad_x, grad_y;
  MAT_TYPE norm_grad;
  int num_basis;  
  cv::Mat NB;
  
  m_object_model->getReferenceCoords(template_coords);
  
  // It computes the gradients for each vector basis of the PCA model including 
  // the mean image gradients in gradients_vector[0] position.
  gradients_vector = m_pca_model->computeModelGradients();   
  num_basis  = gradients_vector.size(); // Num basis vectors + mean  

  M0   = cv::Mat::zeros(template_coords.rows, 
		        m_motion_model->getNumParams() * num_basis, 
		        cv::DataType<MAT_TYPE>::type);

  for (int i = 0; i < M0.rows; i++)
  {
    cv::Mat M0_row      = cv::Mat::zeros(1, 4*num_basis, cv::DataType<MAT_TYPE>::type);

    // Set the basis (gradients) matrix for i-th pixel 
    for (int j = 0; j < num_basis; j++)
    {
      gradients         =  gradients_vector[j];
      MAT_TYPE grad_x_j = gradients.at<MAT_TYPE>(i, 0);
      MAT_TYPE grad_y_j = gradients.at<MAT_TYPE>(i, 1);
      int x             = template_coords.at<MAT_TYPE>(i, 0);
      int y             = template_coords.at<MAT_TYPE>(i, 1);
      M0_row.at<MAT_TYPE>(0, j)               = grad_x_j;
      M0_row.at<MAT_TYPE>(0, j + num_basis)   = grad_y_j;
      M0_row.at<MAT_TYPE>(0, j + 2*num_basis) = grad_x_j * (-y) + grad_y_j * x;
      M0_row.at<MAT_TYPE>(0, j + 3*num_basis) = grad_x_j * x    + grad_y_j * y;
    }
    
    cv::Mat M0_row_i = M0.row(i);
    M0_row.copyTo(M0_row_i);
  }
  
#ifdef DEBUG
  int num_rows = m_pca_model->getNumImageRows();
  int num_cols = m_pca_model->getNumImageCols();
  for (int j = 0; j < num_basis; j++)
  {
    gradients =  gradients_vector[j];

    cv::Mat normalized_x;
    cv::Mat normalized_y;
    cv::Mat grad_x;
    gradients.col(0).copyTo(grad_x);
    cv::Mat grad_y;
    gradients.col(1).copyTo(grad_y);
    
    cv::normalize(grad_x.reshape(1,num_rows), normalized_x, 0, 255, cv::NORM_MINMAX, cv::DataType<uint8_t>::type);
    cv::normalize(grad_y.reshape(1,num_rows), normalized_y, 0, 255, cv::NORM_MINMAX, cv::DataType<uint8_t>::type);
    
    std::ostringstream out_x;
    out_x << "Gradient_x_" << j << "_img.bmp";
    cv::imwrite(out_x.str(), normalized_x);

    std::ostringstream out_y;
    out_y << "Gradient_y_" << j << "_img.bmp";
    cv::imwrite(out_y.str(), normalized_y);
  }

  cv::Mat M0_col_img  = cv::Mat::zeros(num_rows, num_cols, cv::DataType<MAT_TYPE>::type);
  for (int i=0; i<M0.cols; i++)
  {
    cv::Mat M0_col      = M0.col(i);
    cv::Mat M0_col_copy; 
    M0_col.copyTo(M0_col_copy);
    M0_col_copy.reshape(1, num_rows).copyTo(M0_col_img);

    cv::Mat normalized;
    cv::normalize(M0_col_img, normalized, 0, 255, cv::NORM_MINMAX, cv::DataType<uint8_t>::type);
    
    std::ostringstream out;
    out << "M0_col_" << i << "_img.bmp";
    cv::imwrite(out.str(), normalized);
  }
  
  // write Mat objects to the file:
  cv::FileStorage fs("Similarity2DImagePCAFactorizedProblem_computeM0Matrix.xml", cv::FileStorage::WRITE);
  fs << "num_rows" << num_rows;
  fs << "num_cols" << num_cols;
  fs << "M0" << M0;
  fs.release();
#endif   
  
//   TRACE_INFO(__FUNCTION__   << " ======>" << std::endl);
  return M0;  
};
  
  
}; }; // namespace
