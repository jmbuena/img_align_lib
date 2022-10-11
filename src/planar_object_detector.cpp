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
#include "img_align_lib.hpp"
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
    cv::cvtColor(template_image, m_template_image, cv::COLOR_BGR2GRAY);
  }
  else
  {
    template_image.copyTo(m_template_image);
  }
  
//  m_detector                 = cv::SURF::create(100, 4, 3);
////  m_detector->set("hessianThreshold", 100.);
////  m_detector->set("nOctaveLayers", 4);
////  m_detector->set("nOctaves", 3);
//  m_descriptorExtractor      = m_detector;
//  m_descriptorMatcher        = cv::DescriptorMatcher::create("BruteForce-L2");
////  m_descriptorMatcher        = cv::DescriptorMatcher::create("FlannBased");

//  m_detector                 = cv::ORB::create();
//  m_descriptorExtractor      = cv::ORB::create();

  m_detector                 = cv::SIFT::create();
  m_descriptorExtractor      = cv::SIFT::create();
  m_descriptorMatcher        = cv::DescriptorMatcher::create("FlannBased");

  //  m_detector                 = cv::FastFeatureDetector::create(30, true);
//  m_descriptorExtractor      = cv::ORB::create();
//  m_descriptorMatcher        = cv::DescriptorMatcher::create("BruteForce-Hamming");
////  m_descriptorMatcher        = cv::DescriptorMatcher::create("FlannBased");

  m_maxRansacReprojError     = 3;
  m_minNumInliersPercentage  = 0.3;
  
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
  std::vector<cv::Point2f> frame_points;
  double maxInlierDist;
  
  m_object_found = false;

  // ----------------------------------------------------------------
  // Detect descriptors on template image.
  // ----------------------------------------------------------------
  if (m_template_keypoints.size() == 0)
  {
    m_detector->detect( m_template_image, m_template_keypoints );
    m_descriptorExtractor->compute( m_template_image, m_template_keypoints, m_template_descriptors );

    std::vector<cv::Mat> descriptors;
    descriptors.push_back(m_template_descriptors);
    m_descriptorMatcher->add(descriptors);
    m_descriptorMatcher->train(); // Build the lookup structures for descriptors
  }
  
  // ----------------------------------------------------------------
  // Change frame to gray scale.
  // ----------------------------------------------------------------
  if (frame.channels() == 3)
  {
    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
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
  for (auto& dmatches: matches12)
  {
    if (dmatches.size() < 2)
    {
      break;
    }
    
    cv::DMatch first  = dmatches[0];
    cv::DMatch second = dmatches[1];

    // We declare a match if the most similar descriptor is far more 
    // near than the second most similar descriptor.
    if (first.distance < 0.7*second.distance)
    {
      m_filtered_matches.push_back(first);
      templateIdxs.push_back(first.trainIdx);
      frameIdxs.push_back(first.queryIdx);
    }
  }

  if (m_filtered_matches.size() < 10)
  {
    TRACE_INFO(" m_filtered_matches.size() < 10" << std::endl);
    return false;
  }
  
  // ----------------------------------------------------------------
  // Compute Homography template->frame
  // ----------------------------------------------------------------
  cv::KeyPoint::convert(m_template_keypoints, template_points, templateIdxs);
  cv::KeyPoint::convert(m_frame_keypoints, frame_points, frameIdxs);
//  m_H = cv::findHomography( cv::Mat(template_points), cv::Mat(frame_points), cv::RANSAC, m_maxRansacReprojError );
  m_H = cv::findHomography( cv::Mat(template_points), cv::Mat(frame_points), cv::RHO, m_maxRansacReprojError );

  // ----------------------------------------------------------------
  // Compute number of inliers
  // ----------------------------------------------------------------
  cv::perspectiveTransform(cv::Mat(template_points), transformed_template_points, m_H);
  maxInlierDist = m_maxRansacReprojError < 0 ? 3 : m_maxRansacReprojError;
  m_num_inliers = 0;
  m_inliers_indices.clear();
  for( int i1 = 0; i1 < templateIdxs.size(); i1++ )
  {
    int idx_template = templateIdxs[i1];
    int idx_frame = frameIdxs[i1];
    if (cv::norm(frame_points[idx_frame] - transformed_template_points.at<cv::Point2f>(idx_template,0)) <= maxInlierDist ) // inlier
    {
      m_num_inliers++;
      m_inliers_indices.push_back(i1); // i1 is the index of the inlier match in the m_filtered_matches vector
    }
  }

//  for( int i1 = 0; i1 < template_points.size(); i1++ )
//  {
//    if (cv::norm(frame_points[i1] - transformed_template_points.at<cv::Point2f>(i1,0)) <= maxInlierDist ) // inlier
//    {
//      m_num_inliers++;
//      m_inliers_indices.push_back(i1);
//    }
//  }
  
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
  homogeneous_new_coords.col(0) /= homogeneous_new_coords.col(2);
  homogeneous_new_coords.col(1) /= homogeneous_new_coords.col(2);
  homogeneous_new_coords.col(2) /= homogeneous_new_coords.col(2);
    
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
  
  //for (int i=0; i < m_frame_keypoints.size(); i++)
  for (cv::KeyPoint& kp: m_frame_keypoints)
  {
    viewer.filled_ellipse(2, 2, 0.0, kp.pt.x, kp.pt.y, red);
  };

  viewer.image(m_template_image, 0, 0, m_template_image.cols, m_template_image.rows);

  //  for (int i=0; i < m_template_keypoints.size(); i++)
  for (cv::KeyPoint& kp: m_template_keypoints)
  {
    viewer.filled_ellipse(2, 2, 0.0, kp.pt.x, kp.pt.y, red);
  };
  
  TRACE_INFO( " m_inliers_indices.size() = " << m_inliers_indices.size() << std::endl);
  for (size_t i = 0; i < m_inliers_indices.size(); i++)
  {
    int index1 = m_filtered_matches[i].trainIdx;
    int index2 = m_filtered_matches[i].queryIdx;
    viewer.line(m_template_keypoints[index1].pt.x, m_template_keypoints[index1].pt.y,
                m_frame_keypoints[index2].pt.x, m_frame_keypoints[index2].pt.y,
                1, yellow);
    viewer.filled_ellipse(3, 3, 0.0, m_template_keypoints[index1].pt.x, m_template_keypoints[index1].pt.y, green);
    viewer.filled_ellipse(3, 3, 0.0, m_frame_keypoints[index2].pt.x, m_frame_keypoints[index2].pt.y, green);
  };

//  for (auto index1: m_inliers_indices)
//  {
//    int index2 = m_filtered_matches[index1].queryIdx;
//    viewer.line(m_template_keypoints[index1].pt.x, m_template_keypoints[index1].pt.y,
//                m_frame_keypoints[index2].pt.x, m_frame_keypoints[index2].pt.y,
//                1, yellow);
//    viewer.filled_ellipse(3, 3, 0.0, m_template_keypoints[index1].pt.x, m_template_keypoints[index1].pt.y, green);
//    viewer.filled_ellipse(3, 3, 0.0, m_frame_keypoints[index2].pt.x, m_frame_keypoints[index2].pt.y, green);
//  };
  
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

} } // namespace
