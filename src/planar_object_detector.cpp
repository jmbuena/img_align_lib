// -----------------------------------------------------------------------------
/**
 *  @brief Detector of planar objects implementation
 *  @author Jose M. Buenaposada
 *  @date 2012/10/29
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
#include "planar_object_detector.hpp"
#include <opencv2/features2d/features2d.hpp>
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
PlanarObjectDetector::PlanarObjectDetector
  (
  cv::Mat template_image
  )
{
  if (template_image.channels() == 3)
  {
    cvtColor(template_image, m_template_image, CV_BGR2GRAY);
  }
  else
  {
    template_image.copyTo(m_template_image);
  }
  
   m_detector                 = cv::FeatureDetector::create( "SURF" );
//   m_detector                 = cv::Ptr<cv::FeatureDetector>( new cv::SurfFeatureDetector(100, 3, 4));
//   m_detector                 = cv::FeatureDetector::create( "FAST" );
//   m_detector                 = cv::Ptr<cv::FeatureDetector>( new cv::FastFeatureDetector(100));
  m_descriptorExtractor      = cv::DescriptorExtractor::create( "SURF" );
//   m_descriptorMatcher        = cv::DescriptorMatcher::create( "FlannBased" );
  m_descriptorMatcher        = cv::Ptr<cv::DescriptorMatcher>( new cv::FlannBasedMatcher(new cv::flann::KDTreeIndexParams(4), new cv::flann::SearchParams(64)));

//   m_detector                 = cv::FeatureDetector::create( "ORB" );
//   m_descriptorExtractor      = cv::DescriptorExtractor::create( "ORB" );
//   m_descriptorMatcher        = cv::Ptr<cv::DescriptorMatcher>( new cv::BruteForceMatcher<cv::Hamming>());
 
  m_maxRansacReprojError     = 3;
  m_minNumInliersPercentage  = 0.4;
  
  m_H                        = cv::Mat::eye(3, 3, cv::DataType<MAT_TYPE>::type);
  m_num_inliers              = 0;
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
PlanarObjectDetector::~PlanarObjectDetector
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
bool
PlanarObjectDetector::locateObject 
  (
  cv::Mat& frame,
  cv::Mat& template_corners,
  cv::Mat& template_corners_on_frame
  )
{
  cv::Mat frame_gray;
  std::vector<cv::Point2f> template_points; 
  cv::Mat transformed_template_points; 
  cv::vector<cv::Point2f> frame_points; 
  double maxInlierDist;
  
  m_object_found = false;

  // ----------------------------------------------------------------
  // Detect descriptors on template image.
  // ----------------------------------------------------------------
  if (m_template_keypoints.size() == 0)
  {
    m_detector->detect( m_template_image, m_template_keypoints );
    m_descriptorExtractor->compute( m_template_image, m_template_keypoints, m_template_descriptors );

    cv::vector<cv::Mat> descriptors;
    descriptors.push_back(m_template_descriptors);
    m_descriptorMatcher->add(descriptors);
    m_descriptorMatcher->train();
  }
  
  // ----------------------------------------------------------------
  // Change frame to gray scale.
  // ----------------------------------------------------------------
  if (frame.channels() == 3)
  {
    cvtColor(frame, frame_gray, CV_BGR2GRAY);
  }
  else
  {
    frame.copyTo(frame_gray);
  }

  // ----------------------------------------------------------------
  // Detect keypoints + descriptors in the input frame
  // ----------------------------------------------------------------
  m_frame_keypoints.clear();
  m_detector->detect( frame_gray, m_frame_keypoints );
  m_descriptorExtractor->compute(frame_gray, m_frame_keypoints, m_frame_descriptors );

  // ----------------------------------------------------------------
  // Matching template -> frame descriptors.
  // ----------------------------------------------------------------
  m_filtered_matches.clear();
  std::vector<std::vector<cv::DMatch> > matches12;
  m_descriptorMatcher->knnMatch(m_frame_descriptors, matches12, 2);
  std::vector<int> templateIdxs;
  std::vector<int> frameIdxs;
  for (int i=0; i < matches12.size(); i++) 
  { 
    if (matches12[i].size() < 2)
    {
      break;
    }
    
    cv::DMatch first  = matches12[i][0];
    cv::DMatch second = matches12[i][1]; 
    
    // We declare a match if the most similar descriptor is far more 
    // near than the second most similar descriptor.
    if (first.distance < 0.6*second.distance) 
    {
      m_filtered_matches.push_back(first);
      templateIdxs.push_back(first.trainIdx);
      frameIdxs.push_back(first.queryIdx);
    }
  }

  if (m_filtered_matches.size() < 4)
  {
    return m_object_found;
  }
  
  // ----------------------------------------------------------------
  // Compute Homography template->frame
  // ----------------------------------------------------------------
  cv::KeyPoint::convert(m_template_keypoints, template_points, templateIdxs);
  cv::KeyPoint::convert(m_frame_keypoints, frame_points, frameIdxs);
  m_H = cv::findHomography( cv::Mat(template_points), cv::Mat(frame_points), CV_RANSAC, m_maxRansacReprojError );

  // ----------------------------------------------------------------
  // Compute number of inliers
  // ----------------------------------------------------------------
  cv::perspectiveTransform(cv::Mat(template_points), transformed_template_points, m_H);
  maxInlierDist = m_maxRansacReprojError < 0 ? 3 : m_maxRansacReprojError;
  m_num_inliers = 0;
  m_inliers_indices.clear();
  for( int i1 = 0; i1 < template_points.size(); i1++ )
  {
    if (cv::norm(frame_points[i1] - transformed_template_points.at<cv::Point2f>(i1,0)) <= maxInlierDist ) // inlier
    {
      m_num_inliers++;
      m_inliers_indices.push_back(i1);
    }
  }
  
  // ----------------------------------------------------------------
  // Compute the corresponding corners to template image on the input frame.
  // ----------------------------------------------------------------
  template_corners = (cv::Mat_<MAT_TYPE>(4, 2) << 0.                   , 0.,
                                                  m_template_image.cols, 0.,
                                                  m_template_image.cols, m_template_image.rows,
                                                  0.                   , m_template_image.rows);
  m_H.convertTo(m_H, cv::DataType<MAT_TYPE>::type);
  cv::Mat homogeneous_coords     = cv::Mat::ones(template_corners.rows, 3, cv::DataType<MAT_TYPE>::type);
  cv::Mat homogeneous_coords_ref = homogeneous_coords(cv::Range::all(), cv::Range(0, 2));
  template_corners.copyTo(homogeneous_coords_ref);
  
  cv::Mat homogeneous_new_coords = (homogeneous_coords * m_H.t());
  
  // Divide by the third homogeneous coordinates to get the cartersian coordinates.
  for (int j=0; j<3; j++)
  {
    cv::Mat col     = homogeneous_new_coords.col(j).mul(1.0 / homogeneous_new_coords.col(2));
    cv::Mat col_new = homogeneous_new_coords.col(j);
    col.copyTo(col_new);
  }
    
  cv::Mat homogeneous_new_coords_ref = homogeneous_new_coords(cv::Range::all(), cv::Range(0, 2)); 
  homogeneous_new_coords_ref.copyTo(template_corners_on_frame);
  homogeneous_new_coords_ref.copyTo(m_template_corners_on_frame);

  // ----------------------------------------------------------------
  m_object_found = true;
  
  return m_object_found;
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
void
PlanarObjectDetector::showResults
  (
  Viewer& viewer,
  cv::Mat& frame
  )
{
  float red[3]   = {1.0, 0.0, 0.0};
  float green[3] = {0.0, 1.0, 0.0};
  float yellow[3]= {0.0, 1.0, 1.0};
  
  for (int i=0; i<m_frame_keypoints.size(); i++)
  {
    viewer.filled_ellipse(2, 2, 0.0, m_frame_keypoints[i].pt.x, m_frame_keypoints[i].pt.y, red);
  };

  IplImage iplimage = m_template_image;
  viewer.image(&iplimage, 0, 0, m_template_image.cols, m_template_image.rows);

  for (int i=0; i<m_template_keypoints.size(); i++)
  {
    viewer.filled_ellipse(2, 2, 0.0, m_template_keypoints[i].pt.x, m_template_keypoints[i].pt.y, red);
  };
  
  for (int i=0; i<m_inliers_indices.size(); i++)
  {
    int index1 = m_inliers_indices[i];
    int index2 = m_filtered_matches[index1].queryIdx;
    viewer.line(m_template_keypoints[index1].pt.x, m_template_keypoints[index1].pt.y, 
		m_frame_keypoints[index2].pt.x, m_frame_keypoints[index2].pt.y, 
		1, yellow);
    viewer.filled_ellipse(3, 3, 0.0, m_template_keypoints[index1].pt.x, m_template_keypoints[index1].pt.y, green);
    viewer.filled_ellipse(3, 3, 0.0, m_frame_keypoints[index2].pt.x, m_frame_keypoints[index2].pt.y, green);
  };
  
  if (!m_object_found)
  {
    return;
  }
  
  for (int i=0; i < m_template_corners_on_frame.rows; i++)
  {
    int i2 = (i+1)%m_template_corners_on_frame.rows; 
    viewer.line(m_template_corners_on_frame.at<MAT_TYPE>(i,0), m_template_corners_on_frame.at<MAT_TYPE>(i,1), 
                m_template_corners_on_frame.at<MAT_TYPE>(i2,0), m_template_corners_on_frame.at<MAT_TYPE>(i2,1), 
		2, green);      
  }
}

}; }; // namespace
