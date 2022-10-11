// -----------------------------------------------------------------------------
/**
 *  @brief PCA of Images based object model in direct methods tracking.
 *  @author Jose M. Buenaposada
 *  @date 2012/08/10
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
#ifndef IMAGE_PCA_MODEL_HPP
#define IMAGE_PCA_MODEL_HPP

// ----------------------- INCLUDES --------------------------------------------
#include "object_model.hpp"

namespace upm { namespace pcr
{
  
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to ImagePCAModel type
 */
// -----------------------------------------------------------------------------
class ImagePCAModel;
typedef boost::shared_ptr<ImagePCAModel> ImagePCAModelPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to ImagePCAModels 
 */
// -----------------------------------------------------------------------------
typedef std::vector<ImagePCAModel> ImagePCAModels;

// ----------------------------------------------------------------------------
/**
 * @class ImagePCAModel
 * @brief A class that defines an appearance model based on a PCA model of images.
 *
 * The gray levels in the mean image and pca basis are divided by 255 in order
 * the cost function to use quantities in [0.0, 1.0] avoiding numerical errors
 * (if any). Note that the gray levels of the warped image are also divided by
 * 255.
 *
 */
// -----------------------------------------------------------------------------
class ImagePCAModel: public ObjectModel
{
public:

  ImagePCAModel
    (
    cv::Mat& mean, 
    size_t img_cols, 
    size_t img_rows,
    cv::Mat& B
    );

  virtual ~ImagePCAModel
    ();
    
  /**
   *  @brief Computes the grey levels gradient of the PCA model.
   *  
   *  It computes the  \frac{\partial (mean + B*c)[(\vx)]}{\partial \vx} (the gradient).
   *  The size of the output matrix is Nxk being N the number of pixels and k the dimensinality of \vx (the 
   *  template coordinates vector).
   * 
   *  @return A matrix being Nxk (number of template pixels x dim(\vx) )
   */
  virtual cv::Mat
  computeTemplateFeaturesGradient
    (
    cv::Mat object_params
    );
    
  /**
   *  @brief Computes the features vector from the warped image.
   *  
   *  Converts the input image to gray levels and then to a column vector (Nx1 with N the 
   *  number of pixels).
   * 
   *  @param warped_image
   *  @return A vector that is Nx1 ( number of template pixels x 1 ).
   */
  virtual cv::Mat
  extractFeaturesFromWarpedImage
    (
    cv::Mat warped_image
    );

  /**
   *  @brief Computes the reconstructed image gray levels.
   * 
   *  @return A vector that is Nx1 ( number of PCA model template pixels x 1 ).
   */
  virtual cv::Mat
  computeTemplateFeatures
    (
    cv::Mat object_params
    );
    
  /**
   *  @brief Computes the gradients of the PCA of images model
   * 
   *  It computes the gradients (reshaped as images) of the PCA basis vectors and 
   *  the mean (reshaped as a gray levels image). The gradients are returned in an
   *  std::vector. The gradient of the mean image vector is in the std::vector 0 index. 
   *  The rests of the gradients are stored from std::vector position 1 (and in the same
   *  order than the PCA basis matrix columns. The gradients stored in the std::vector
   *  are computed by calling computeGrayImageGradients.
   * 
   *  @return std::vector of cv::Mat (each result of a call to computeGrayImageGradients)
   */
  virtual std::vector<cv::Mat>
  computeModelGradients
    ();

  /**
   *  @brief Returns the coordinates of template points (the reference coordinates).
   * 
   *  Returns the coordinates of each pixel from left to right and top to 
   *  bottom (by rows). The coordinates of the top left corner are (-width/2,-height/2),
   *  and the coordinates of the right bottom corner are  (width/2, height/2). That
   *  means that the image center pixel take template (reference) coordinats (0,0).
   *  
   *  The control points are the four corners of a rectangle and there are four lines 
   *  that joins the four control points.
   * 
   *  @param reference_coords A matrix that is Nxk (number of template pixels x dim(\vx)).
   */
  virtual void
  getCtrlPointsIndices
    (
    std::vector<int>& control_points_indices,
    std::vector<LineIndices>& control_points_lines 
    );

  /**
   *  @brief Returns the coordinates of template points (the reference coordinates).
   * 
   *  Returns the coordinates of each pixel from left to right and top to 
   *  bottom (by rows). The coordinates of the top left corner are (-width/2,-height/2),
   *  and the coordinates of the right bottom corner are  (width/2, height/2). That
   *  means that the image center pixel take template (reference) coordinats (0,0).
   * 
   *  @param reference_coords A matrix that is Nxk (number of template pixels x dim(\vx)).
   */
  virtual void
  getReferenceCoords
    (
    cv::Mat& reference_coords
    );

  virtual size_t
  getNumOfReferenceCoords
    () { return m_B.rows; };    
    
  cv::Mat
  getBasisMatrix
    () { return m_B; };

  cv::Mat
  getInverseBasisMatrix
    () { return m_invB; };

  cv::Mat
  getMeanVector
    () { return m_mean; };

  cv::Mat
  getMeanImage
    () { return m_mean_image; };

  size_t
  getNumImageRows
    () { return m_img_rows; };

  size_t
  getNumImageCols
    () { return m_img_cols; };

  size_t
  getNumBasisVectors
    () { return m_B.cols; };
    
protected:
  
  cv::Mat  
  computeTemplateCoordinates
    (
    cv::Mat gray_image
    );

  size_t m_img_cols;
  size_t m_img_rows;
  cv::Mat m_B;
  cv::Mat m_invB;
  cv::Mat m_mean;
  cv::Mat m_mean_image;
// cv::Mat m_template_gray_levels;
  cv::Mat m_template_coordinates;
  std::vector<cv::Mat> m_gradients_vector;
  std::vector<int> m_control_points_indices;
  std::vector<LineIndices> m_control_points_lines;
};

}; }; // namespace

#endif
