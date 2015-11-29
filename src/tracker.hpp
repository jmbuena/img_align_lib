// -----------------------------------------------------------------------------
/**
 *  @brief Visual object tracker interface definition
 *  @author Jose M. Buenaposada
 *  @date 2012/07/10
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
#ifndef TRACKER_HPP
#define TRACKER_HPP

// ----------------------- INCLUDES --------------------------------------------
#include <memory>
#include "viewer.hpp"
#include "optimizer.hpp"
#include "motion_model.hpp"
#include <vector>
#include <exception>
#include "img_align_lib.hpp"

namespace upm { namespace pcr
{
// ----------------------------------------------------------------------------
/**
 *  Definition of exceptions
 */
// -----------------------------------------------------------------------------
class params_incorrect: public std::exception {};
  
// ----------------------------------------------------------------------------
/**
 *  Definition of an smart pointer to Tracker type
 */
// -----------------------------------------------------------------------------
class Tracker;
typedef std::shared_ptr<Tracker> TrackerPtr;

// -----------------------------------------------------------------------------
/**
 *  Definition of a container of smart pointers to trackers
 */
// -----------------------------------------------------------------------------
typedef std::vector<Tracker> Trackers;

// ----------------------------------------------------------------------------
/**
 * @class Tracker
 * @brief A class that defines the interface for a visual tracker.
 */
// -----------------------------------------------------------------------------
class Tracker
{
public:

  Tracker
    (
    OptimizerPtr optim
    );

  virtual ~Tracker
    ();

  /**
   * @brief Track the target object to the current frame.
   *
   * @param frame Image comming from the camera.
   */
  virtual void
  processFrame 
   (
   cv::Mat& frame
   );

  /**
   * @brief Shows the tracking resultis on a given Viewer.
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

  void
  setInitialParams
    (
    cv::Mat initial_params
    );

  void
  setNumPyramidLevels
    (
    unsigned int num_pyramid_levels
    ) { if (num_pyramid_levels > 0) m_num_pyramid_levels = num_pyramid_levels; };

  cv::Mat
  getMotionParams
    ();

  MotionModelPtr
  getMotionModel
    ();

  /**
   * @brief Sets the maximal cost function value for declare sucess in tracking.
   */
  void
  setMaximalCostFunctionValue  
    (
    double max_cost
    ) { m_max_cost = max_cost; }
    
  bool
  isLost
    ();

protected:
  cv::Mat m_motion_params;
  double m_max_cost;
  unsigned int m_num_pyramid_levels;
  OptimizerPtr m_optimizer;
  MotionModelPtr m_motion_model;
  cv::Mat m_params;
  cv::Mat m_template_coords;
  std::vector<int> m_ctrl_points_indices;
  std::vector<LineIndices> m_ctrl_points_lines;
};

}; }; // namespace

#endif
