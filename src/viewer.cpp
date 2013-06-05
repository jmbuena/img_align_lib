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
//#include <opencv/cxtypes.h> // IPL_DEPTH_8U
#include <opencv/cv.h>
#include <iostream>
#include <algorithm> // swap
#include <cmath> // M_PI
#include "trace.hpp"

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
  m_pCanvas(NULL)
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
    cvDestroyWindow(m_window_title.c_str());
    cvReleaseImage(&m_pCanvas);
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
  ) throw (viewer_creation_error)
{
  if (m_initialised)
  {
    cvDestroyWindow(m_window_title.c_str());
    cvReleaseImage(&m_pCanvas);
  }
  m_pCanvas = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);

  m_initialised  = true;
  m_width        = width;
  m_height       = height;
  m_window_title = window_title;

  cvNamedWindow(m_window_title.c_str(), CV_WINDOW_AUTOSIZE); 
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
    // Fill drawing canvas on black.
   CvPoint  rectangle[]={0,0,  m_width,0, m_width,m_height, 0,m_height};
   CvPoint* curveArr[G_INDEX]={rectangle};
   int      nCurvePts[G_INDEX]={4};
   int      nCurves=1;
   int      isCurveClosed=1;

   cvFillPoly(m_pCanvas, curveArr, nCurvePts, nCurves,
              cvScalar(0, 0, 0));
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
    cvShowImage(m_window_title.c_str(), m_pCanvas);
    cvWaitKey(20);
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
  cvRectangle(m_pCanvas, cvPoint(x,y), cvPoint(x+width, y+height), 
              cvScalar(border_color[R_INDEX]*255,
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
  
  CvPoint  rectangle[]={x,y,  x+width,y,  x+width,y+height,  x,y+height};
  CvPoint* curveArr[G_INDEX]={rectangle};
  int      nCurvePts[G_INDEX]={4};
  int      nCurves=1;
  int      isCurveClosed=1;

  cvFillPoly(m_pCanvas, curveArr, nCurvePts, nCurves, 
             cvScalar(fill_color[R_INDEX]*255,
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

  cvLine(m_pCanvas, cvPoint(x1,y1), cvPoint(x2,y2), 
         cvScalar(color[R_INDEX]*255, color[G_INDEX]*255, color[B_INDEX]*255), 
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

  CvPoint  ellipse[ELLIPSE_NUMBER_OF_POINTS]; 
  int      nCurvePts[G_INDEX] = {ELLIPSE_NUMBER_OF_POINTS};
  int      nCurves      = 1;
  int      isCurveClosed= 1;

  for(int i=0; i<ELLIPSE_NUMBER_OF_POINTS; i++)
  {
    fi                = 2*M_PI*(i/static_cast<double>(ELLIPSE_NUMBER_OF_POINTS));
    majorCos          = major_axis_length*cos(fi);
    minorSin          = minor_axis_length*sin(fi);
    cos_angle         = cos(angle);
    sin_angle         = sin(angle);
    ellipse[i]        = cvPoint(static_cast<int>(x_center + (cos_angle*majorCos) - (sin_angle*minorSin)),
                                static_cast<int>(y_center + (sin_angle*majorCos) + (cos_angle*minorSin)));
  };
  CvPoint* curveArr[G_INDEX]  = {ellipse};

  cvPolyLine(m_pCanvas, curveArr, nCurvePts, nCurves, 
             true,
             cvScalar(color[R_INDEX]*255,
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

  CvPoint  ellipse[ELLIPSE_NUMBER_OF_POINTS]; 
  int      nCurvePts[G_INDEX] = {ELLIPSE_NUMBER_OF_POINTS};
  int      nCurves      = 1;
  int      isCurveClosed= 1;

  for(int i=0; i<ELLIPSE_NUMBER_OF_POINTS; i++)
  {
    fi                = 2*M_PI*(i/static_cast<double>(ELLIPSE_NUMBER_OF_POINTS));
    majorCos          = major_axis_length*cos(fi);
    minorSin          = minor_axis_length*sin(fi);
    cos_angle         = cos(angle);
    sin_angle         = sin(angle);
    ellipse[i]        = cvPoint(static_cast<int>(x_center + (cos_angle*majorCos) - (sin_angle*minorSin)),
                                static_cast<int>(y_center + (sin_angle*majorCos) + (cos_angle*minorSin)));
  };
  CvPoint* curveArr[G_INDEX]  = {ellipse};

  cvFillPoly(m_pCanvas, curveArr, nCurvePts, nCurves, 
             cvScalar(fill_color[R_INDEX]*255,
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
  CvFont font;
  double hScale=scale;
  double vScale=scale;
  int    lineWidth=line_width;

  if (!m_initialised || !m_drawing)
  {
    return;
  }

//  cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale, vScale,0,lineWidth);
  cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, hScale, vScale,0,lineWidth);
  cvPutText (m_pCanvas, text.c_str(), cvPoint(x,y), &font, 
             cvScalar(color[R_INDEX]*255,
	              color[G_INDEX]*255,
	              color[B_INDEX]*255,
                      0));
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
  IplImage* pImage,
  int x, 
  int y, 
  int width, 
  int height
  )
{
  IplImage* pImageAux;
  IplImage* pImageAux2;
  int depth     = pImage->depth;
  int nchannels = pImage->nChannels;


  if (!m_initialised || !m_drawing)
  {
    return;
  }

  if (x+width > m_pCanvas->width) 
  {
    width  = m_pCanvas->width - x;
  }

  if (y+height > m_pCanvas->height)
  {
    height = m_pCanvas->height - y;
  }

  // If the area for the image is very small we do nothing
  if ((width <= 0) || (height <= 0))
  {
    return;
  }

  if ((pImage->width != width) || (pImage->height != height))
  {
    // create source image with the right size 
    pImageAux = cvCreateImage( cvSize( width, height ), depth, nchannels );
//    cvResize(pImage, pImageAux, CV_INTER_LINEAR);
    cvResize(pImage, pImageAux, CV_INTER_NN);
  }
  else
  {
    pImageAux = pImage;
  }

  // Blit the input image over the canvas
  if ((m_pCanvas->nChannels == 3) && (pImageAux->nChannels == 1))
  {
    pImageAux2 = cvCreateImage( cvGetSize(pImageAux), m_pCanvas->depth, m_pCanvas->nChannels);
    cvCvtColor( pImageAux, pImageAux2, CV_GRAY2BGR );
    cvSetImageROI(m_pCanvas, cvRect(x,y,width,height));
    cvCopy(pImageAux2, m_pCanvas);
    cvResetImageROI(m_pCanvas);
    cvReleaseImage(&pImageAux2);
  }
  else
  {
    cvSetImageROI(m_pCanvas, cvRect(x,y,width,height));
    cvCopy(pImageAux, m_pCanvas);
    cvResetImageROI(m_pCanvas);
  }

  if ((pImage->width != width) || (pImage->height != height))
  {
    cvReleaseImage(&pImageAux);
  }

/*
  IplImage* src1 = cvLoadImage( "MGC.jpg" );
  IplImage* src2 = cvLoadImage( "wheel.jpg" );
  int x = 280;
  int y = 80;
  int width = 60;
  int height = 60;
  double alpha = 0.5;
  double beta = 0.5;
  cvSetImageROI(src1, cvRect(x,y,width,height));
  cvAddWeighted(src1, alpha, src2, beta, 0.0, src1);
  cvResetImageROI(src1);
  cvNamedWindow("Alpha_blend", 1);
  cvShowImage("Alpha_blend", src1);
  cvWaitKey();
*/

/*
  // Make a mask from all non-zero pixels in the decal.
  mask = cvCreateImage(decal_size, IPL_DEPTH_8U, 1);
  cvCvtColor(decal_size, mask, CV_RGB2GRAY);
  cvThreshold(mask, mask, 0.0, 255.0, CV_THRESH_BINARY_INV);

  // Here, the target image ROI needs to be set to the region you wish
  // to be blended. I
  // I assume you've calculated this and put it in target_rect.
  cvSetImageROI(img, target_rect);
  cvCopy(img, decal, mask);

  // Now alpha blend. Note that for img, alpha * img + 1 - alpha *
  // decal gives back 1.0 * img.
  cvAddWeighted(img, 1.0 - alpha, decal, alpha, 0.0, img); 
*/                                                 
};


void 
Viewer::saveCanvas
  (
  std::string file_name
  )
{
  cvSaveImage(file_name.c_str(), m_pCanvas);
}

}; }; // namespaces

