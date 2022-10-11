// -----------------------------------------------------------------------------
/**
 *  @brief Viewer implementation.
 *  @author Jose Miguel Buenaposada
 *  @date 2009/11/13
 *  @version $Id$
 *
 *  Grupo de investigación en Percepción Computacional y Robótica)
 *  (Perception for Computers & Robots research Group)
 *  Facultad de Informática (Computer Science School)
 *  Universidad Politécnica de Madrid (UPM) (Madrid Technical University)
 *  http://www.dia.fi.upm.es/~pcr
 *
 */
// -----------------------------------------------------------------------------

// ----------------------- INCLUDES --------------------------------------------
#include "viewer.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm> // swap
#include <cmath> // M_PI

#ifndef M_PI
 #define M_PI (2.0*acos(0.0))
#endif

#define ELLIPSE_NUMBER_OF_POINTS 100
#define R_INDEX 2 
#define G_INDEX 1
#define B_INDEX 0

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
Viewer::Viewer
  (): 
  m_drawing(false),
  m_initialised(false),
  m_canvas()
{
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
Viewer::~Viewer
  ()
{
  if (m_initialised)
  {
    cv::destroyWindow(m_window_title);
  }
  m_initialised = false;
  m_drawing     = false;
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
void
Viewer::init
  (
  int width,
  int height,
  std::string window_title
  )
{
  if (m_initialised)
  {
    cv::destroyWindow(m_window_title);
  }

  m_canvas = cv::Mat::zeros(cv::Size(width, height), CV_8UC3);

  m_initialised  = true;
  m_width        = width;
  m_height       = height;
  m_window_title = window_title;

  cv::namedWindow(m_window_title); //, cv::CV_WINDOW_AUTOSIZE);
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
Viewer::beginDrawing
  ()
{
  if (m_initialised && !m_drawing)
  {
    cv::rectangle(m_canvas, cv::Rect(0, 0, m_width, m_height),cv::Scalar(0,0,0));
    m_drawing = true;
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
Viewer::endDrawing
  ()
{
  if (m_initialised && m_drawing)
  {
    cv::imshow(m_window_title, m_canvas);
    cv::waitKey(20);
    m_drawing = false;  
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
Viewer::rectangle
  (
  int x, 
  int y, 
  int width, 
  int height, 
  int line_width,
  float border_color[]
  )
{
  if (!m_initialised || !m_drawing)
  {
    return;
  }

  // For little endian machines RGBA -> BGRA
  cv::rectangle(m_canvas, cv::Rect(x,y,width, height),
                cv::Scalar(border_color[R_INDEX]*255,
	                   border_color[G_INDEX]*255,
			   border_color[B_INDEX]*255),
			   line_width);
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
Viewer::filled_rectangle
  (
  int x, 
  int y, 
  int width, 
  int height, 
  float fill_color[]
  )
{
  if (!m_initialised || !m_drawing)
  {
    return;
  }

  // Fill drawing canvas
  std::vector<cv::Point> pts;
  pts.push_back(cv::Point(x, y));
  pts.push_back(cv::Point(x+width, y));
  pts.push_back(cv::Point(x+width, y+height));
  pts.push_back(cv::Point(x, y+height));

  cv::fillPoly(m_canvas, pts,
               cv::Scalar(fill_color[R_INDEX]*255,
                          fill_color[G_INDEX]*255,
                          fill_color[B_INDEX]*255));
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
Viewer::line
  (
  int x1, 
  int y1, 
  int x2, 
  int y2, 
  int line_width,
  float color[]
  )
{
  if (!m_initialised || !m_drawing)
  {
    return;
  }

  cv::line(m_canvas, cv::Point(x1,y1), cv::Point(x2,y2),
           cv::Scalar(color[R_INDEX]*255, color[G_INDEX]*255, color[B_INDEX]*255),
           line_width);
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
Viewer::ellipse
  (
  int major_axis_length,
  int minor_axis_length,
  float angle, /* with respect to the horizontal axis */
  int x_center,
  int y_center,
  int line_width,
  float color[]
  )
{
  double fi;
  double majorCos;
  double minorSin;
  float sin_angle, cos_angle;

  if (!m_initialised || !m_drawing)
  {
    return;
  }

  std::vector<cv::Point> pts;

  for(int i=0; i<ELLIPSE_NUMBER_OF_POINTS; i++)
  {
    fi                = 2*M_PI*(i/static_cast<double>(ELLIPSE_NUMBER_OF_POINTS));
    majorCos          = major_axis_length*cos(fi);
    minorSin          = minor_axis_length*sin(fi);
    cos_angle         = cos(angle);
    sin_angle         = sin(angle);
    pts.push_back(cv::Point(static_cast<int>(x_center + (cos_angle*majorCos) - (sin_angle*minorSin)),
                            static_cast<int>(y_center + (sin_angle*majorCos) + (cos_angle*minorSin))));
  };
  cv::polylines(m_canvas, pts,
                true,
                cv::Scalar(color[R_INDEX]*255,
                           color[G_INDEX]*255,
                           color[B_INDEX]*255),
                line_width);
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
Viewer::filled_ellipse
  (
  int major_axis_length,
  int minor_axis_length,
  float angle, /* with respect to the horizontal axis */
  int x_center,
  int y_center,
  float fill_color[]
  )
{
  double fi;
  double majorCos;
  double minorSin;
  float sin_angle, cos_angle;

  if (!m_initialised || !m_drawing)
  {
    return;
  }

  std::vector<cv::Point> pts;

  for(int i=0; i<ELLIPSE_NUMBER_OF_POINTS; i++)
  {
    fi                = 2*M_PI*(i/static_cast<double>(ELLIPSE_NUMBER_OF_POINTS));
    majorCos          = major_axis_length*cos(fi);
    minorSin          = minor_axis_length*sin(fi);
    cos_angle         = cos(angle);
    sin_angle         = sin(angle);
    pts.push_back(cv::Point(static_cast<int>(x_center + (cos_angle*majorCos) - (sin_angle*minorSin)),
                            static_cast<int>(y_center + (sin_angle*majorCos) + (cos_angle*minorSin))));
  };

  cv::polylines(m_canvas, pts,
                true,
                cv::Scalar(fill_color[R_INDEX]*255,
                           fill_color[G_INDEX]*255,
                           fill_color[B_INDEX]*255));
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
Viewer::text
  (
  std::string text, 
  int x, 
  int y, 
  float color[],
  float scale, /* = 1.0 */
  int line_width /* = 1 */
  )
{
  if (!m_initialised || !m_drawing)
  {
    return;
  }


  cv::putText(m_canvas, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX,
              scale,
              cv::Scalar(color[R_INDEX]*255,
                         color[G_INDEX]*255,
                         color[B_INDEX]*255,
              line_width));
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
Viewer::image
  (
  cv::Mat img,
  int x, 
  int y, 
  int width, 
  int height
  )
{
  cv::Mat imgAux;
  cv::Mat imgAux2;

  if (!m_initialised || !m_drawing)
  {
    return;
  }

  if (x+width > m_canvas.cols)
  {
    width  = m_canvas.cols - x;
  }

  if (y+height > m_canvas.rows)
  {
    height = m_canvas.rows - y;
  }

  // If the area for the image is very small we do nothing
  if ((width <= 0) || (height <= 0))
  {
    return;
  }

  if ((img.cols != width) || (img.rows != height))
  {
    // create source image with the right size 
    cv::resize(img, imgAux, cv::Size(width, height));
  }
  else
  {
    imgAux = img.clone();
  }

  // Blit the input image over the canvas
  cv::Rect roi(x, y, width, height);
  if ((m_canvas.channels() == 3) && (imgAux.channels() == 1))
  {
    cv::cvtColor(imgAux, imgAux2, cv::COLOR_GRAY2BGR);
    imgAux2.copyTo(m_canvas(roi));
  }
  else
  {
    imgAux.copyTo(m_canvas(roi));
  }
};

void 
Viewer::saveCanvas
  (
  std::string file_name
  )
{
  cv::imwrite(file_name, m_canvas);
}

}; }; // namespaces

