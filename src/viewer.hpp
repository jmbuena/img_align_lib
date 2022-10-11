// -----------------------------------------------------------------------------
/**
 *  @brief Viewer.
 *  @author Pablo Márquez Neila, Jose Miguel Buenaposada
 *  @date 2009/01/05
 *  @date 2009/11/13 Jose M. Buenaposada, Modified to use only OpenCV
 *  @version $revision$
 *
 *  $Id$
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
#ifndef _VIEWER_H_
#define _VIEWER_H_

// ----------------------- INCLUDES --------------------------------------------
#include <string>
#include <exception>
#include <opencv2/opencv.hpp> // OpenCV IplImage
//#include <opencv/highgui.h> // OpenCV IplImage

namespace upm { namespace pcr
{
// ----------------------------------------------------------------------------
/**
 * @class viewer_creation_error
 * @brief Thrown on a viewer implementation class constructor error.
 */
/* ------------------------------------------------------------------------- */
class viewer_creation_error: public std::exception {};

// ----------------------------------------------------------------------------
/**
 * @class Viewer
 * @brief Class interface used by the computer vision algorithms to plot results.
 *
 * The viewer implementation using OpenCV API (HighGUI). This class can be 
 * extended making a Viewer for GTK+, Win32, QT, etc. Just making a subclass
 * will do the work.
 */
// -----------------------------------------------------------------------------
class Viewer
{
public:

  Viewer
    ();

  virtual ~Viewer
    ();

  /** 
   *  @brief Initialisation of the viewer canvas using width and height in pixels 
   *
   *  @param width The given width in pixels of the window canvas.
   *  @param height The given height in pixels of the window canvas.
   *  @param window_title The string with the window title to display.
   *
   */
  bool
  isInitialised
    () { return m_initialised; };


  /**
   *  @brief Initialisation of the viewer canvas using width and height in pixels
   *
   *  @param width The given width in pixels of the window canvas.
   *  @param height The given height in pixels of the window canvas.
   *  @param window_title The string with the window title to display.
   *
   */
  virtual void
  init
    (
    int width,
    int height,
    std::string window_title
    );

  /** 
   *  @brief Drawing interface is double buffered. We need to swap buffers (begin-end). 
   *
   *  All the drawing commands should be between "beginDrawing" and a 
   *  "endDrawing". Only after "endDrawing" the drawing commands result is displayed over
   *  the window canvas.
   */
  virtual void
  beginDrawing
    ();

  /** 
   *  @brief Drawing interface is double buffered. We need to swap buffers (begin-end). 
   *
   *  All the drawing commands should be between "beginDrawing" and a 
   *  "endDrawing". Only after "endDrawing" the drawing commands result is displayed over
   *  the window canvas.
   */
  virtual void
  endDrawing
    ();

  /** 
   *  @brief Draw a rectangle parallel to the viewer axes 
   *
   *  The x axis runs from left to right of the window and the 
   *  y axis is going from top to down of the window.
   *
   *  @param x Horizontal coordinate of the top left corner.
   *  @param y Vertical coordinate of the top left corner.
   *  @param width Size of the window in the horizontal axis.
   *  @param height Size of the window in the vertical axis.
   *  @param line_width Width in pixels of the rectangle border.
   *  @param color Pen color to draw with. It is RGB, with values in the [0.0,1.0] interval.
   *
   */
  virtual void 
  rectangle
    (
    int x, 
    int y, 
    int width, 
    int height, 
    int line_width,
    float color[]
    );

  /** 
   *  @brief Draw a color filled rectangle parallel to the viewer axes 
   *
   *  The x axis runs from left to right of the window and the 
   *  y axis is going from top to down of the window.
   *
   *  @param x Horizontal coordinate of the top left corner.
   *  @param y Vertical coordinate of the top left corner.
   *  @param width Size of the window in the horizontal axis.
   *  @param height Size of the window in the vertical axis.
   *  @param color Fill color. It is RGB, with values in the [0.0,1.0] interval.
   */
  virtual void 
  filled_rectangle
    (
    int x, 
    int y, 
    int width, 
    int height, 
    float color[]
    );

  /** 
   *  @brief Draw a line on the viewer.
   *
   *  @param x1 Horizontal coordinate of the starting pixel.
   *  @param y1 Vertical coordinate of the starting pixel.
   *  @param x2 Horizontal coordinate of the ending pixel.
   *  @param y2 Vertical cordinate of the ending pixel.
   *  @param line_width Width in pixels of the line.
   *  @param color Line color. It is RGB, with values in the [0.0,1.0] interval.
   */ 
  virtual void 
  line
    (
    int x1, 
    int y1, 
    int x2, 
    int y2, 
    int line_width,
    float color[]
    );

  /** 
   *  @brief Draw a general ellipse on the viewer 
   *
   *
   *  @param major_axis_length Major axix lenght in pixels
   *  @param minor_axis_length Minor axix lenght in pixels
   *  @param angle Angle of the major axis with respect to the horizontal axis.
   *  @param x_center Horizontal pixel coordinate of the ellipse center.
   *  @param y_center Vertical pixel coordinate of the ellipse center.
   *  @param line_width Width in pixels of the line.
   *  @param color Border color. It is RGB, with values in the [0.0,1.0] interval.
   */
  virtual void
  ellipse
    (
    int major_axis_length,
    int minor_axis_length,
    float angle, /* with respect to the horizontal axis */
    int x_center, 
    int y_center, 
    int line_width,
    float color[]
    );

  /** 
   *  @brief Draw a general filled ellipse on the viewer 
   *
   *  @param major_axis_length Major axix lenght in pixels
   *  @param minor_axis_length Minor axix lenght in pixels
   *  @param angle Angle of the major axis with respect to the horizontal axis.
   *  @param x_center Horizontal pixel coordinate of the ellipse center.
   *  @param y_center Vertical pixel coordinate of the ellipse center.
   *  @param color Fill color. It is RGB, with values in the [0.0,1.0] interval.
   */
  virtual void
  filled_ellipse
    (
    int major_axis_length,
    int minor_axis_length,
    float angle, 
    int x_center, 
    int y_center, 
    float color[]
    );

  /** Show text string at the viewer's  (x,y) position. */
  virtual void 
  text
    (
    std::string text, 
    int x, 
    int y, 
    float color[],
    float scale = 1.0,
    int line_width = 1
    );

  /** 
   * @brief Draw an image over the background on the viewer.
   *
   * @param pImage to put over the viewer current canvas.
   * @param x The image appear with the top left corner at x horizontal coordinate. 
   * @param y The image will appear with the top left corner at y vertical coordintate. 
   * @param width The image width in pixels over the viewer (it could need to be scaled).
   * @param height The image height in pixels over the viewer (it could need to be scaled).
   *
   */
  virtual void 
  image
    (
    cv::Mat image,
    int x, 
    int y, 
    int width, 
    int height
    );

  virtual void
  saveCanvas
    (
    std::string file_name
    );
    
protected:
  /** Set to true after calling beginDrawing and to false after endDrawing */
  bool m_drawing;

  /** Set to true after calling init */
  bool m_initialised; 

  cv::Mat m_canvas;
  cv::Mat m_buffer;
  int m_width;
  int m_height;
  std::string m_window_title;
};

}; }; // namespace

#endif // _VIEWER_H
