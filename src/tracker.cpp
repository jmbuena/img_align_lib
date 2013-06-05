// -----------------------------------------------------------------------------
/**
 *  @brief Visual object tracker interface implementation
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

#include "tracker.hpp"
#include "optimizer.hpp"
#include "trace.hpp"

const int MIN_OBJECT_AREA = 100;

namespace upm { namespace pcr
{
  
// -----------------------------------------------------------------------------
//
// Purpose and Method: Constructor
// Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
// -----------------------------------------------------------------------------
Tracker::Tracker
  (
  OptimizerPtr optim
  ) 
{ 
  m_optimizer = optim;
  
  OptimizationProblemPtr optim_problem = m_optimizer->getOptimizationProblem(); 
  ObjectModelPtr object_model = optim_problem->getObjectModel(); 

  m_motion_model   = optim_problem->getMotionModel();
  m_params         = cv::Mat::zeros(m_motion_model->getNumParams(), 1, cv::DataType<MAT_TYPE>::type);
  
  object_model->getReferenceCoords(m_template_coords);
  object_model->getCtrlPointsIndices(m_ctrl_points_indices, m_ctrl_points_lines);
  
  m_num_pyramid_levels = 1;
  
  m_max_cost = std::numeric_limits<double>::max();
};


// -----------------------------------------------------------------------------
//
// Purpose and Method: Destructor
// Inputs: 
// Outputs: 
// Dependencies:
// Restrictions and Caveats:
//
// -----------------------------------------------------------------------------
Tracker::~Tracker
  () 
{};

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
Tracker::setInitialParams
  (
  cv::Mat initial_params
  )
{

//   if ((initial_params.rows != m_motion_model->getNumParams()) || 
//       (initial_params.cols != 1)) 
//   {
//     throw motion_params_incorrect();
//   }
  
  cv::Mat initial_params_MAT_TYPE;
  
  initial_params.convertTo(initial_params_MAT_TYPE, cv::DataType<MAT_TYPE>::type);
  
  m_params = initial_params;
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
Tracker::processFrame 
  (
  cv::Mat& frame
  )
{  
  std::vector<cv::Mat> pyramid;
  cv::Mat dst;
  cv::Mat motion_params_aux;
  int width, height;
  static long int frame_num = 0;

  // Compute the template coordinates bounding box
  cv::Mat x_template_coords = m_template_coords.col(0);
  cv::Mat y_template_coords = m_template_coords.col(1);
  double min_x, max_x, min_y, max_y;
  int minInd_x, maxInd_x, minInd_y, maxInd_y;
  cv::minMaxIdx(x_template_coords, &min_x, &max_x, &minInd_x, &maxInd_x, cv::Mat());
  cv::minMaxIdx(y_template_coords, &min_y, &max_y, &minInd_y, &maxInd_y, cv::Mat());
  int template_width  = max_x - min_x + 1;
  int template_height = max_y - min_y + 1;

  // Put the original frame the first in the multiresolution pyramid
  pyramid.push_back(frame);

  width  = floor(frame.cols/2.0);
  height = floor(frame.rows/2.0);
  dst    = frame;
  for (int i=0; i < (m_num_pyramid_levels - 1); i++)
  {
    cv::pyrDown(dst, dst, cv::Size(width, height));
    pyramid.push_back(dst);
    width  = floor(width/2.0);
    height = floor(height/2.0);

    if ((width < template_width) || (height < template_height))
    {
      break; 
    }
  }
  
  double scale_factor = pow(2.0, pyramid.size()-1);
  
  m_motion_params   = m_params(cv::Range(0, m_motion_model->getNumParams()), cv::Range::all());
  motion_params_aux = m_motion_model->scaleInputImageResolution(m_motion_params, 1.0/scale_factor);
  motion_params_aux.copyTo(m_motion_params);
  
  if (m_optimizer->getShowIterations())
  {
    std::cout << "============= Tracker starts processing frame #" << frame_num << " ..." << std::endl;
  }

  for (int i=pyramid.size()-1; i >= 0; i--)
  {
    if (m_optimizer->getShowIterations())
    {
      std::cout << std::endl;
      std::cout << "== Iterations for " << pyramid[i].cols << "x" << pyramid[i].rows << " pixels: "<< std::endl;
    }

    m_params        = m_optimizer->solve(pyramid[i], m_params);
    m_motion_params = m_params(cv::Range(0, m_motion_model->getNumParams()), cv::Range::all());

    if (i > 0)
    {
      motion_params_aux = m_motion_model->scaleInputImageResolution(m_motion_params, 2.0);
      motion_params_aux.copyTo(m_motion_params);
    }
  }

  if (m_optimizer->getShowIterations())
  {
    std::cout << "============= Tracker ENDS processing frame ... ==========" << std::endl;
    frame_num++;
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
void
Tracker::showResults
  (
  Viewer& viewer,
  cv::Mat& frame
  )
{    
  cv::Mat image_coords;
  float red[]   = {1., 0., 0.}; 
  float blue[]  = {0., 0., 1.}; 
  float white[] = {1., 1., 1.}; 
  int index1, index2;
  int x1, y1, x2, y2;
  cv::Mat motion_params;

  // image_coords are Nx2 : N pixels x 2 coordinates (x,y
  motion_params = m_params(cv::Range(0, m_motion_model->getNumParams()), cv::Range::all());
  image_coords  = m_motion_model->transformCoordsToImage(m_template_coords, motion_params);

  for (int i=0; i < m_ctrl_points_lines.size(); i++)
  {
    index1 = m_ctrl_points_lines[i].p1_index;
    index2 = m_ctrl_points_lines[i].p2_index;
    
    x1 = static_cast<int>(image_coords.at<MAT_TYPE>(index1,0));
    y1 = static_cast<int>(image_coords.at<MAT_TYPE>(index1,1));
    x2 = static_cast<int>(image_coords.at<MAT_TYPE>(index2,0));
    y2 = static_cast<int>(image_coords.at<MAT_TYPE>(index2,1));
    
    viewer.line(x1, y1, x2, y2, 2, white);
  };

  for (int i=0; i < m_ctrl_points_indices.size(); i++)
  {
    index1 = m_ctrl_points_indices[i];
    
    x1 = static_cast<int>(image_coords.at<MAT_TYPE>(index1,0));
    y1 = static_cast<int>(image_coords.at<MAT_TYPE>(index1,1));
    
    viewer.filled_ellipse(3, 3, 0.0, x1, y1, red);
  };
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
bool
Tracker::isLost
  ()
{ 
  cv::Mat image_coords;
  int index1, index2;
  int x, y;
  int min_x, max_x, min_y, max_y;
  int area;
  double last_cost;
  std::vector<double> iteration_costs;

  if (m_motion_params.rows == 0)
  {
    return false; 
  }
  
  image_coords  = m_motion_model->transformCoordsToImage(m_template_coords, m_motion_params);

  // Find the area of the bounding box of the object: 
  // if it is too small we have lost tracking.
  min_x = std::numeric_limits<int>::max();
  max_x = std::numeric_limits<int>::min();
  min_y = std::numeric_limits<int>::max();
  max_y = std::numeric_limits<int>::min();
  for (int i=0; i < m_ctrl_points_indices.size(); i++)
  {
    index1 = m_ctrl_points_indices[i];
    
    x = static_cast<int>(image_coords.at<MAT_TYPE>(index1,0));
    y = static_cast<int>(image_coords.at<MAT_TYPE>(index1,1));    
    
    min_x = std::min(x, min_x);
    max_x = std::max(x, max_x);
    min_y = std::min(y, min_y);
    max_y = std::max(y, max_y);
  };
  
  area = (max_x - min_x) * (max_y - min_y);

  iteration_costs = m_optimizer->getIterationsCosts();
  last_cost = iteration_costs[iteration_costs.size()-1];
  
  return (area < MIN_OBJECT_AREA) ||
         (last_cost > m_max_cost) ||
         (m_motion_model->invalidParams(m_motion_params));
};

}; }; // namespace
