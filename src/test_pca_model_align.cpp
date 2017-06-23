#include "opencv2/opencv.hpp"
#include "viewer.hpp"
#include "optimizerGN.hpp"
// #include "single_image_model.hpp"
#include "image_pca_model.hpp"
#include "tracker.hpp"
#include "trace.hpp"
#include <limits>

using namespace cv;
using namespace upm::pcr;

#undef USE_TRACKING_AFTER_DETECTION

const double TICKS_PER_SECOND       = (static_cast<double>(cvGetTickFrequency())*1.0e6);
const double TICKS_PER_MILLISECOND  = (static_cast<double>(cvGetTickFrequency())*1.0e3);
const int FRAME_HEIGHT              = 480;
const int FRAME_WIDTH               = 640;
const int NUM_MAX_ITERATIONS        = 20;
const bool SHOW_OPTIMIZER_ITERATION_COSTS = false;
const int  NUM_PYRAMID_LEVELS       = 1;
const std::string CASCADE_NAME      = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt2.xml";

#define USE_SIMILARITY_PCA_FACTORIZED_PROBLEM
#undef USE_AFFINE_PCA_FACTORIZED_PROBLEM

// #include "pca_illum_9_identity_60_64x64.hpp"
//#include "pca_illum_9_64x64.hpp"

#include "pca_illum_9_identity_40_32x32.hpp"
//#include "pca_illum_9_32x32.hpp"

#ifdef USE_SIMILARITY_PCA_FACTORIZED_PROBLEM 
  #include "image_pca_model.hpp"
  #include "similarity_2d.hpp"
  #include "similarity_2d_image_pca_factorized_problem.hpp"
  #define USE_SIMILARITY_MODEL
#elif defined(USE_AFFINE_PCA_FACTORIZED_PROBLEM) 
  #include "image_pca_model.hpp"
  #include "affine_2d.hpp"
  #include "affine_2d_image_pca_factorized_problem.hpp"
  #define USE_AFFINE_MODEL
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
void
findBiggestFace
  (
  Mat& frame,
  cv::CascadeClassifier& cascade_classifier,
  bool& face_found,
  cv::Rect& face_rect
  )
{
  double ticks;
  std::vector<cv::Rect> faces;
  double max_area;

  cascade_classifier.detectMultiScale( frame, faces,
        1.2, 2, 0
        //|CV_HAAR_FIND_BIGGEST_OBJECT
        //|CV_HAAR_DO_ROUGH_SEARCH
        |CV_HAAR_SCALE_IMAGE,
        Size(80, 80) );

  max_area   = std::numeric_limits<double>::min();
  face_found = false;
  for( std::vector<cv::Rect>::const_iterator r = faces.begin(); r != faces.end(); r++ )
  {
    double area = static_cast<double>(r->width)*static_cast<double>(r->height);
    
    if (max_area < area)
    {     
      max_area         = area;
      face_found       = true;
      face_rect.x      = r->x;
      face_rect.y      = r->y;
      face_rect.width  = r->width;
      face_rect.height = r->height;
    }
  }
}

// -----------------------------------------------------------------------------
//
// Purpose and Method:
// Inputs:
// Outputs:
// // Dependencies:
// Restrictions and Caveats:
//
// -----------------------------------------------------------------------------
double
processFrame
  (
  Mat& frame,
  Tracker& tracker,
  cv::CascadeClassifier& cascade_classifier,
  bool& face_found,
  cv::Rect& face_rect,
  size_t model_pixels_width,
  size_t model_pixels_height,
  size_t num_appearance_params
  )
{
  static bool first_time = true;
  double ticks;
  MAT_TYPE width, height, x_center, y_center, x_scale, y_scale;
  
  face_found = false;
  ticks      = static_cast<double>(cvGetTickCount());
  
#ifndef USE_TRACKING_AFTER_DETECTION
  if (tracker.isLost() || first_time)  
  {
#endif
    findBiggestFace(frame, cascade_classifier, face_found, face_rect);
    
    if (face_found)
    {
      width    = static_cast<MAT_TYPE>(face_rect.width);
      height   = static_cast<MAT_TYPE>(face_rect.height);
      x_center = static_cast<MAT_TYPE>(face_rect.x) + (width/2.0);
      y_center = static_cast<MAT_TYPE>(face_rect.y) + (height/2.0);
      width   *= 0.8;
      height  *= 0.8;
      x_scale  = width/static_cast<MAT_TYPE>(model_pixels_width);
      y_scale  = height/static_cast<MAT_TYPE>(model_pixels_height);
      
#ifdef USE_SIMILARITY_MODEL   
      cv::Mat initial_params            = cv::Mat::zeros(4 + num_appearance_params, 1, cv::DataType<MAT_TYPE>::type);  
      cv::Mat initial_motion_params_ref = initial_params(cv::Range(0,4), cv::Range::all());
      cv::Mat initial_motion_params     = (cv::Mat_<MAT_TYPE>(4,1) <<  x_center, y_center,
		                                                       0., 
			                                               y_scale);
      initial_motion_params.copyTo(initial_motion_params_ref); 
      
#elif defined(USE_AFFINE_MODEL)     
      cv::Mat initial_params = cv::Mat::zeros(6 + num_appearance_params, 1, cv::DataType<MAT_TYPE>::type);  
      cv::Mat initial_motion_params_ref = initial_params(cv::Range(0,6), cv::Range::all());
      cv::Mat initial_motion_params = (cv::Mat_<MAT_TYPE>(6,1) << x_center, y_center, 
 			                                          x_scale, 0.,  0., y_scale);
      initial_motion_params.copyTo(initial_motion_params_ref); 
#else
   #error "Wrong motion model"
#endif
      tracker.setInitialParams(initial_params);
      tracker.processFrame(frame);          
      first_time = false;
    }
#ifndef USE_TRACKING_AFTER_DETECTION
  }
 else
 {
   tracker.processFrame(frame);          
 }
#endif
  
  ticks = static_cast<double>(cvGetTickCount()) - ticks;

  return ticks;
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
showResults
  (
  Mat& frame,
  Viewer& viewer, 
  Tracker& tracker,
  bool& face_found, 
  cv::Rect& face_rect,
  time_t ticks
  )
{
  float red_color[3] = {1.0, 0.0, 0.0};
  float green_color[3] = {0.0, 1.0, 0.0};
  
  if (!viewer.isInitialised())
  {
    viewer.init(frame.cols, frame.rows, "Results");
  }

  std::ostringstream outs;
  outs << "FPS =" << std::setprecision(3); 
  outs << TICKS_PER_SECOND/ticks << std::ends;
  std::string time_info = outs.str();

  // Drawing results
  viewer.beginDrawing(); 
  IplImage ipl_frame = frame;
  viewer.image(&ipl_frame, 0, 0, frame.cols, frame.rows);
  if (face_found)
  {
    viewer.rectangle(face_rect.x, face_rect.y, face_rect.width, face_rect.height, 2, green_color);
  }
#ifdef USE_TRACKING_AFTER_DETECTION
  if (!tracker.isLost() && face_found)
#else
  if (!tracker.isLost())
#endif
  {
    tracker.showResults(viewer, frame);  
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
int 
main
  (
  int argc, 
  char** argv
  )
{
  Viewer viewer;
  bool viewer_initialized = false;
  Mat frame_cap;
  Mat frame;
//   Mat template_image;
  double ticks;
  cv::Mat initial_params;
  cv::Rect face_rect;
  bool face_found = false;
  std::string cascade_name = CASCADE_NAME; 
  double max_cost_function_value = 0.0;
  
  if (argc == 0)
  {
    return 0;
  }

  const std::string cascade_opt = "--cascade=";
  size_t cascade_opt_len = cascade_opt.length();
  string inputName;
  for (int i = 1; i < argc; i++) 
  {
    TRACE_INFO("Processing argument #" << i << ": " <<  argv[i] << std::endl);
    if ( cascade_opt.compare( 0, cascade_opt_len, argv[i], cascade_opt_len ) == 0 )
    {
      cascade_name.assign( argv[i] + cascade_opt_len );
      TRACE_INFO("  from which we have cascadeName= " << cascade_name << std::endl);
    }
    else if( argv[i][0] == '-' )
    {
      TRACE_ERROR("WARNING: Unknown option " << argv[i] << std::endl);
    }
//     else
//     {
//       input_name = argv[i];
//     }
  }

  cv::CascadeClassifier cascade_classifier;
  if ( !cascade_classifier.load( cascade_name ) )
  {
     TRACE_ERROR("ERROR: Could not load cascade classifier \"" << cascade_name << "\"" << std::endl);
     return -1;
  }
    
//   cv::VideoCapture cap(input_name); // open the default camera
  cv::VideoCapture cap(0); // open the default camera
  cap.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
  if (!cap.isOpened())  // check if we succeeded
  {      
    return -1;
  }


//  cv::Mat B_mat(PCA_IMG_WIDTH*PCA_IMG_HEIGHT, PCA_NUM_BASIS, cv::DataType<MAT_TYPE>::type, 
//                &PCA_B, sizeof(MAT_TYPE)*PCA_NUM_BASIS);
  cv::Mat B_mat_orig(PCA_IMG_WIDTH*PCA_IMG_HEIGHT, PCA_NUM_BASIS, cv::DataType<float>::type, &PCA_B, sizeof(float)*PCA_NUM_BASIS);
  cv::Mat B_mat = cv::Mat::ones(PCA_IMG_WIDTH*PCA_IMG_WIDTH, PCA_NUM_BASIS+1, cv::DataType<float>::type);
  cv::Mat B_mat_ref = B_mat(cv::Range::all(), cv::Range(1,B_mat.cols)); 
  B_mat_orig.copyTo(B_mat_ref);
  
  cv::Mat mean_mat(PCA_IMG_WIDTH*PCA_IMG_WIDTH, 1, cv::DataType<MAT_TYPE>::type, &PCA_mean, sizeof(MAT_TYPE));
  
  while(true)
  {
    if (!cap.read(frame_cap)) // get a new frame from camera
    {
      // The video file finished or the camera stopped working.
      break;
    }
    frame = frame_cap.clone();

    // Setup the optimization problem
#ifdef USE_SIMILARITY_PCA_FACTORIZED_PROBLEM     
    static ImagePCAModelPtr object_model_ptr(new ImagePCAModel(mean_mat, PCA_IMG_WIDTH, PCA_IMG_HEIGHT, B_mat));
    static MotionModelPtr motion_model_ptr(dynamic_cast<MotionModel*>(new Similarity2D()));
    static OptimizationProblemPtr optim_problem_ptr(dynamic_cast<OptimizationProblem*>(new
      Similarity2DImagePCAFactorizedProblem(object_model_ptr, motion_model_ptr)));
    max_cost_function_value = 400000.;
#elif defined(USE_AFFINE_PCA_FACTORIZED_PROBLEM)     
    static ImagePCAModelPtr object_model_ptr(new ImagePCAModel(mean_mat, PCA_IMG_WIDTH, PCA_IMG_HEIGHT, B_mat));
    static MotionModelPtr motion_model_ptr(dynamic_cast<MotionModel*>(new Affine2D()));
    static OptimizationProblemPtr optim_problem_ptr(dynamic_cast<OptimizationProblem*>(new
      Affine2DImagePCAFactorizedProblem(object_model_ptr, motion_model_ptr)));
    max_cost_function_value = 400000.;
#else
   #error "Wrong OptimizationProblem selected"
#endif
    static OptimizerPtr optimizer_ptr(dynamic_cast<Optimizer*>(new GaussNewtonOptimizer(optim_problem_ptr))); 
    optimizer_ptr->setShowIterations(SHOW_OPTIMIZER_ITERATION_COSTS);
    static Tracker tracker(optimizer_ptr);      
    tracker.setNumPyramidLevels(NUM_PYRAMID_LEVELS);
    tracker.setMaximalCostFunctionValue(max_cost_function_value);
    static bool is_first_time = true;
      
    if (is_first_time)
    {
      optimizer_ptr->setMaxNumIterations(NUM_MAX_ITERATIONS);
      is_first_time = false;
    }
      
    ticks = processFrame(frame, tracker, cascade_classifier, face_found, face_rect, 
			 PCA_IMG_WIDTH, PCA_IMG_HEIGHT, B_mat.cols);
    showResults(frame, viewer, tracker, face_found, face_rect, ticks);
    waitKey(10);
  }
 
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}
