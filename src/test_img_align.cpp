//#include "opencv2/opencv.hpp"
#include <boost/concept_check.hpp>
#include "viewer.hpp"
#include "optimizerGN.hpp"
#include "tracker.hpp"
#include "planar_object_detector.hpp"
#include "trace.hpp"
#include <boost/program_options.hpp>

using namespace cv;
using namespace upm::pcr;
namespace po = boost::program_options;

const int FRAME_HEIGHT              = 480;
const int FRAME_WIDTH               = 640;
const int TEMPLATE_IMG_WIDTH        = 75;
const int TEMPLATE_IMG_WIDTH_DIV2   = static_cast<int>(TEMPLATE_IMG_WIDTH / 2.0);
const int TEMPLATE_IMG_HEIGHT       = 75;
const int TEMPLATE_IMG_HEIGHT_DIV2  = static_cast<int>(TEMPLATE_IMG_HEIGHT / 2.0);
const double TEMPLATE_SCALE         = 3;
const bool TEMPLATE_EQUALIZATION    = true;
const int NUM_MAX_ITERATIONS        = 20;
const bool SHOW_OPTIMIZER_ITERATION_COSTS = true;
const int  NUM_PYRAMID_LEVELS       = 3;
const float MAX_COST_FUNCTION_VALUE = 300;
// const int SURF_HESSIAN_THRESHOLD    = 200;

#undef USE_HOMOGRAPHY_FACTORIZED_PROBLEM
#define USE_AFFINE_FACTORIZED_PROBLEM
#undef USE_SIMILARITY_FACTORIZED_PROBLEM
#undef USE_SIMILARITY_CORR_GRAD_INV_COMP_PROBLEM

#ifdef USE_SIMILARITY_FACTORIZED_PROBLEM 
  #include "single_image_model.hpp"
  #include "similarity_2d.hpp"
  #include "similarity_2d_factorized_problem.hpp"
  #define USE_SIMILARITY_MODEL
#elif defined(USE_AFFINE_FACTORIZED_PROBLEM) 
  #include "single_image_model.hpp"
  #include "affine_2d.hpp"
  #include "affine_2d_factorized_problem.hpp"
  #define USE_AFFINE_MODEL
#elif defined(USE_HOMOGRAPHY_FACTORIZED_PROBLEM)
  #include "single_image_model.hpp"
  #include "homography_2d.hpp"
  #include "homography_2d_factorized_problem.hpp"
  #define USE_HOMOGRAPHY_MODEL
#elif defined(USE_SIMILARITY_CORR_GRAD_INV_COMP_PROBLEM)
  #include "single_image_gradients_model.hpp"
  #include "similarity_2d.hpp"
  #include "similarity_2d_grad_corr_inv_comp_problem.hpp"
  #define USE_SIMILARITY_MODEL
  #define USE_COMPOSITIONAL_MODEL
#else
  #error "Wrong optimization problem configuration"
#endif


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
processFrame
  (
  Mat& frame,
  Mat& template_image,
  upm::pcr::Tracker& tracker,
  upm::pcr::PlanarObjectDetector& detector
  )
{
  double ticks;
//  static bool first_time = true;
  Mat frame_gray;
  Mat template_gray;
  Mat motion_params;

  ticks = static_cast<double>(cv::getTickCount());

//  TRACE_INFO("-> tracker.isLost()"<< std::endl);
  if (tracker.isLost())
  {
//    TRACE_INFO("<- tracker.isLost()"<< std::endl);
    Mat src_corners = (cv::Mat_<MAT_TYPE>(4, 2) << 0,                   0,
                                                   template_image.cols, 0,
                                                   template_image.cols, template_image.rows,
                                                   0                  , template_image.rows);
    Mat dst_corners;
    src_corners.copyTo(dst_corners);
//    TRACE_INFO("-> detector.locateObject()"<< std::endl);
    if (detector.locateObject(frame, src_corners, dst_corners))
    {
//      TRACE_INFO("<- detector.locateObject()"<< std::endl);
      cv::Point2f src_points[4];
      cv::Point2f dst_points[4];

      // We use the src corrdinates of the four template image coorners
      // with the origin on the template image centre
      src_points[0].x = static_cast<MAT_TYPE>(-TEMPLATE_IMG_WIDTH_DIV2);
      src_points[0].y = static_cast<MAT_TYPE>(-TEMPLATE_IMG_HEIGHT_DIV2);
      src_points[1].x = static_cast<MAT_TYPE>(TEMPLATE_IMG_WIDTH_DIV2);
      src_points[1].y = static_cast<MAT_TYPE>(-TEMPLATE_IMG_HEIGHT_DIV2);
      src_points[2].x = static_cast<MAT_TYPE>(TEMPLATE_IMG_WIDTH_DIV2);
      src_points[2].y = static_cast<MAT_TYPE>(TEMPLATE_IMG_HEIGHT_DIV2);
      src_points[3].x = static_cast<MAT_TYPE>(-TEMPLATE_IMG_WIDTH_DIV2);
      src_points[3].y = static_cast<MAT_TYPE>(TEMPLATE_IMG_HEIGHT_DIV2);

      dst_points[0].x = dst_corners.at<MAT_TYPE>(0, 0);
      dst_points[0].y = dst_corners.at<MAT_TYPE>(0, 1);
      dst_points[1].x = dst_corners.at<MAT_TYPE>(1, 0);
      dst_points[1].y = dst_corners.at<MAT_TYPE>(1, 1);
      dst_points[2].x = dst_corners.at<MAT_TYPE>(2, 0);
      dst_points[2].y = dst_corners.at<MAT_TYPE>(2, 1);
      dst_points[3].x = dst_corners.at<MAT_TYPE>(3, 0);
      dst_points[3].y = dst_corners.at<MAT_TYPE>(3, 1);

#ifdef USE_SIMILARITY_MODEL
      Mat A = cv::getAffineTransform(src_points, dst_points);
      
      float dx = (src_points[1].x - src_points[0].x);
      float dy = (src_points[1].y - src_points[0].y);
      float angle = M_PI;
      if (dx > 1.0E-6) // It is big enough
      {
        angle = std::atan(dy/dx);
      }
      motion_params =  (cv::Mat_<MAT_TYPE>(4,1) << A.at<double>(0,2),  // tx
                                                   A.at<double>(1,2),  // ty
                                                   angle,  // angle
                                                   (A.at<double>(0,0) + A.at<double>(1,1))/2.0); // scale
//      TRACE_INFO("-> 2 =========="<< std::endl);

#elif defined(USE_AFFINE_MODEL)
      cv::Mat A = cv::getAffineTransform(src_points, dst_points);
      TRACE_INFO("A = " << A << std::endl);
      motion_params =  (cv::Mat_<MAT_TYPE>(6,1) << A.at<double>(0,2),  // tx
                                                   A.at<double>(1,2),  // ty
                                                   A.at<double>(0,0),  // a
                                                   A.at<double>(1,0),  // b
                                                   A.at<double>(0,1),  // c
                                                   A.at<double>(1,1)); // d
//      TRACE_INFO("-> 2 =========="<< std::endl);

#elif defined(USE_HOMOGRAPHY_MODEL)       
      Mat H = cv::getPerspectiveTransform(src_points, dst_points);

      motion_params =  (cv::Mat_<MAT_TYPE>(8,1) << H.at<double>(0,0),
                                                   H.at<double>(1,0),
                                                   H.at<double>(2,0),
                                                   H.at<double>(0,1),
                                                   H.at<double>(1,1),
                                                   H.at<double>(2,1),
                                                   H.at<double>(0,2),
                                                   H.at<double>(1,2));
#else
  #error "Wrong motion model configuration"
#endif
      tracker.setInitialParams(motion_params);
      tracker.processFrame(frame);
    }
  }
  else
  {
    tracker.processFrame(frame);
//    std::cout << "motion_params_track = " << tracker.getParams() << std::endl;
  }

  return static_cast<double>(cv::getTickCount()) - ticks;
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
showResults
  (
  Mat& frame,
  upm::pcr::Viewer& viewer,
  upm::pcr::Tracker& tracker,
  upm::pcr::PlanarObjectDetector& detector,
  double ticks
  )
{
  float red_color[3] = {1.0, 0.0, 0.0};
  
  if (!viewer.isInitialised())
  {
    viewer.init(frame.cols, frame.rows, "Results");
  }

  std::ostringstream outs;
  outs << "FPS =" << std::setprecision(3) << cv::getTickFrequency() / ticks << std::ends;
  std::string time_info = outs.str();

  // Drawing results
  viewer.beginDrawing(); 
  viewer.image(frame, 0, 0, frame.cols, frame.rows);
  if (!tracker.isLost())
  {
    tracker.showResults(viewer, frame);
  }
  else
  {
    detector.showResults(viewer, frame);
  }
  viewer.text(time_info, 20, frame.rows-20, red_color, 0.5);
  viewer.endDrawing();  
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
Mat
getTemplate
  (
  Mat& frame,
  Viewer& viewer, 
  cv::Mat& initial_params,
  bool& have_template,
  bool is_video_file//,
//  time_t ticks
  )
{
  float red_color[3] = {1.0, 0.0, 0.0};
  int top, left, width, height;
  Mat template_image = cv::Mat(TEMPLATE_IMG_WIDTH, TEMPLATE_IMG_HEIGHT, frame.type());

  have_template = false;
  
  if (!viewer.isInitialised())
  {
    viewer.init(frame.cols, frame.rows, "Results");
  }

  // Drawing results
  viewer.beginDrawing(); 
  viewer.image(frame, 0, 0, frame.cols, frame.rows);
#ifdef USE_SIMILARITY_MODEL
  double scale = initial_params.at<MAT_TYPE>(3,0);
  #ifdef USE_COMPOSITIONAL_MODEL
  scale = scale + 1.0;
  #endif
  top    = initial_params.at<MAT_TYPE>(1,0) - round((TEMPLATE_IMG_HEIGHT/2.0)*scale);
  height = round(TEMPLATE_IMG_HEIGHT * scale);
  left   = initial_params.at<MAT_TYPE>(0,0) - round((TEMPLATE_IMG_WIDTH/2.0)*scale);
  width  = round(TEMPLATE_IMG_WIDTH  * scale);
#elif defined(USE_AFFINE_MODEL) 
  top    = initial_params.at<MAT_TYPE>(1,0) - round((TEMPLATE_IMG_HEIGHT/2.0)*TEMPLATE_SCALE);
  height = round(TEMPLATE_IMG_HEIGHT * TEMPLATE_SCALE);
  left   = initial_params.at<MAT_TYPE>(0,0) - round((TEMPLATE_IMG_WIDTH/2.0)*TEMPLATE_SCALE);
  width  = round(TEMPLATE_IMG_WIDTH  * TEMPLATE_SCALE);  
#elif defined(USE_HOMOGRAPHY_MODEL) 
  top    = initial_params.at<MAT_TYPE>(7,0) - round((TEMPLATE_IMG_HEIGHT/2.0)*TEMPLATE_SCALE);
  height = round(TEMPLATE_IMG_HEIGHT * TEMPLATE_SCALE);
  left   = initial_params.at<MAT_TYPE>(6,0) - round((TEMPLATE_IMG_WIDTH/2.0)*TEMPLATE_SCALE);
  width  = round(TEMPLATE_IMG_WIDTH  * TEMPLATE_SCALE);
#else
  #error "Wrong motion model configuration"
#endif
  viewer.rectangle(left, top, width, height, 2, red_color);
  viewer.endDrawing();  

  char key = waitKey(10);
  if (key == 't') // || (is_video_file))
  {
    have_template        = true;
    cv::Mat template_roi = frame(cv::Rect(left, top, width, height));
    template_roi.copyTo(template_image);
    imwrite("template_image.bmp", template_image);
  }

  return template_image;
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
int 
main
  (
  int argc, 
  char** argv
  )
{
  Viewer viewer;
  Mat frame_cap;
  Mat frame;
  Mat template_image;
  Mat template_image_detection;
  double ticks;
  bool have_template = false;
  cv::Mat initial_params;
  std::string video_file;
  std::string output_name;
  bool use_video_file = false;
  cv::VideoCapture cap;

  try
  {
    // --------------------------------------------------------------
    // Declare the supported program options.
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("output-name", po::value<std::string>(), "set the destination text file.")
      ("video-file", po::value<std::string>(), "set the video file.")
    ;

    po::positional_options_description p;
    p.add("input-name", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return 1;
    }

    if (vm.count("video-file"))
    {
      use_video_file = true;
      video_file     = vm["video-file"].as<std::string>();
    }

    if (vm.count("output-name"))
    {
      output_name = vm["output-name"].as<std::string>();
    }
    else
    {
      output_name = "tracker_evaluation_output.txt";
    }

  }
  catch(std::exception& e)
  {
    std::cout << e.what() << "\n";
    return 1;
  }


  if (use_video_file)
  {
    cap.open(video_file);
  }
  else
  {
    cap.open(0); // open the default camera
    cap.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
  }

  if (!cap.isOpened())  // check if we succeeded
  {      
    return -1;
  }

  while(true)
  {
    if (!cap.read(frame_cap)) // get a new frame from camera
    {
      // The video file finished or the camera stopped working.
      break;
    }
    frame = frame_cap.clone();

#ifdef USE_SIMILARITY_MODEL    
    double scale = TEMPLATE_SCALE;
  #ifdef USE_COMPOSITIONAL_MODEL
    scale = scale - 1.0;
  #endif
    initial_params = (cv::Mat_<MAT_TYPE>(4,1) << (static_cast<MAT_TYPE>(frame.cols)/2.0),
		                                 (static_cast<MAT_TYPE>(frame.rows)/2.0), 
		                                  0., scale);
#elif defined(USE_AFFINE_MODEL)
    initial_params = (cv::Mat_<MAT_TYPE>(6,1) << (static_cast<MAT_TYPE>(frame.cols)/2.0),
		                                 (static_cast<MAT_TYPE>(frame.rows)/2.0), 
		                                 TEMPLATE_SCALE, 0.,
						 0.,             TEMPLATE_SCALE);
#elif defined(USE_HOMOGRAPHY_MODEL)
    initial_params = (cv::Mat_<MAT_TYPE>(8,1) <<  TEMPLATE_SCALE ,             0., 0.,
                                                  0.             , TEMPLATE_SCALE, 0.,
                   (static_cast<MAT_TYPE>(frame.cols)/2.0),  (static_cast<MAT_TYPE>(frame.rows)/2.0));
#else
  #error "Wrong motion model configuration"
#endif
    
    if (!have_template)
    {
      template_image_detection = getTemplate(frame, viewer, initial_params, have_template, use_video_file);
      cv::resize(template_image_detection, template_image, Size(TEMPLATE_IMG_WIDTH, TEMPLATE_IMG_HEIGHT));
    }
    else
    {
      // Setup the optimization problem
#ifdef USE_SIMILARITY_FACTORIZED_PROBLEM      
      static ObjectModelPtr object_model_ptr(dynamic_cast<ObjectModel*>(new SingleImageModel(template_image, TEMPLATE_EQUALIZATION)));
      static MotionModelPtr motion_model_ptr(dynamic_cast<MotionModel*>(new Similarity2D()));
      static OptimizationProblemPtr optim_problem_ptr(dynamic_cast<OptimizationProblem*>(new
      Similarity2DFactorizedProblem(object_model_ptr, motion_model_ptr)));
#elif defined(USE_AFFINE_FACTORIZED_PROBLEM)
      static ObjectModelPtr object_model_ptr(dynamic_cast<ObjectModel*>(new SingleImageModel(template_image, TEMPLATE_EQUALIZATION)));
      static MotionModelPtr motion_model_ptr(dynamic_cast<MotionModel*>(new Affine2D()));
      static OptimizationProblemPtr optim_problem_ptr(dynamic_cast<OptimizationProblem*>(new
      Affine2DFactorizedProblem(object_model_ptr, motion_model_ptr)));      
#elif defined(USE_HOMOGRAPHY_FACTORIZED_PROBLEM)
      static ObjectModelPtr object_model_ptr(dynamic_cast<ObjectModel*>(new SingleImageModel(template_image, TEMPLATE_EQUALIZATION)));
      static MotionModelPtr motion_model_ptr(dynamic_cast<MotionModel*>(new Homography2D()));
      static OptimizationProblemPtr optim_problem_ptr(dynamic_cast<OptimizationProblem*>(new
      Homography2DFactorizedProblem(object_model_ptr, motion_model_ptr)));      
#elif defined(USE_SIMILARITY_CORR_GRAD_INV_COMP_PROBLEM)
      static ObjectModelPtr object_model_ptr(dynamic_cast<ObjectModel*>(new SingleImageGradientsModel(template_image, TEMPLATE_EQUALIZATION)));
      static MotionModelPtr motion_model_ptr(dynamic_cast<MotionModel*>(new Similarity2D(true)));
      static OptimizationProblemPtr optim_problem_ptr(dynamic_cast<OptimizationProblem*>(new
      Similarity2DGradCorrInvCompProblem(object_model_ptr, motion_model_ptr)));            
#else
  #error "Wrong optimization problem configuration"
#endif

      static OptimizerPtr optimizer_ptr(dynamic_cast<Optimizer*>(new GaussNewtonOptimizer(optim_problem_ptr))); 
      optimizer_ptr->setShowIterations(SHOW_OPTIMIZER_ITERATION_COSTS);
      static upm::pcr::Tracker tracker(optimizer_ptr);
      tracker.setNumPyramidLevels(NUM_PYRAMID_LEVELS);
      static upm::pcr::PlanarObjectDetector detector(template_image_detection);
      static bool is_first_time = true;

      if (is_first_time)
      {
        tracker.setInitialParams(initial_params);
        tracker.setMaximalCostFunctionValue(MAX_COST_FUNCTION_VALUE);
        optimizer_ptr->setMaxNumIterations(NUM_MAX_ITERATIONS);
        is_first_time = false;
      }
      
      ticks = processFrame(frame, template_image, tracker, detector);      
      showResults(frame, viewer, tracker, detector, ticks);
    }
    waitKey(10);
  }
 
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}
