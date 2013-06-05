// -----------------------------------------------------------------------------
/**
 *  @brief Detector of planar objects
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

// ------------------ RECURSION PROTECTION -------------------------------------
#ifndef PLANAR_OBJECT_DETECTOR_HPP
#define PLANAR_OBJECT_DETECTOR_HPP

// ----------------------- INCLUDES --------------------------------------------
#include <boost/shared_ptr.hpp>
#include "viewer.hpp"
#include "img_align_lib.hpp"
#include <opencv/cv.h>

namespace upm { namespace pcr
{
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to PlanarObjectDetector type
 */
// -----------------------------------------------------------------------------
class PlanarObjectDetector;
typedef boost::shared_ptr<PlanarObjectDetector> PlanarObjectDetectorPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to detectors
 */
// -----------------------------------------------------------------------------
typedef std::vector<PlanarObjectDetector> PlanarObjectDetectors;

// ----------------------------------------------------------------------------
/**
 * @class PlanarObjectDetector
 * @brief A class that defines the interface for a planar object detector
 */
// -----------------------------------------------------------------------------
class PlanarObjectDetector
{
public:

  PlanarObjectDetector
    (
    cv::Mat template_image
    );

  virtual ~PlanarObjectDetector
    ();

  /**
   * @brief Locate the target object in the current frame.
   *
   * @param frame Image comming from the camera.
   * @return returns false if the object has been found.
   */
  virtual bool
  locateObject 
   (
   cv::Mat& frame,
   cv::Mat& template_corners,
   cv::Mat& template_corners_on_frame
   );
   
  /**
   * @brief Shows the location results on a given Viewer.
   *
   * @param viewer The viewer over to which display results.
   * @param frame Last processed camera frame.
   */
  virtual void
  showResults
   (
   Viewer& viewer,
   cv::Mat& frame
   );
   
protected:
  cv::Mat m_template_image;
  
  cv::Ptr<cv::FeatureDetector> m_detector;
  cv::Ptr<cv::DescriptorExtractor> m_descriptorExtractor;
  cv::Ptr<cv::DescriptorMatcher> m_descriptorMatcher;
  std::vector<cv::KeyPoint> m_template_keypoints;
  cv::Mat m_template_descriptors;
  std::vector<cv::KeyPoint> m_frame_keypoints;
  cv::Mat m_frame_descriptors;
  double m_maxRansacReprojError;
  double m_minNumInliersPercentage;
  int m_num_inliers;
  std::vector<int> m_inliers_indices;
  cv::Mat m_H;
  std::vector<cv::DMatch> m_filtered_matches;
  cv::Mat m_template_corners_on_frame;
  bool m_object_found; 

};

}; }; // namespace

#endif
