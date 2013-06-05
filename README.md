Incremental Image Alignment Library
===================================

This is a library of incremental image alingment algorithms using OpenCV C++ interface (from 2.0).

Incremental Image alingment is used in tracking applications. In this applications you find the
pose of the object in the scene by any means (i.e. object detection) and subsquently the pose
is reestimated in the current frame using the pose in the former frame. In order to motion
estimation to be possible a motion model of the object is assumed. Therefore, in this context
pose stands for motion parameters for a given motion model.

The elements of any incremental image alignment are:

  * The object model (a single texture image, a planar appearance model, AAM, 3D morphable model, etc).
  * The motion model (2D similarity, 2D affine, 2D homography, 3D pose (rotation and traslation), etc).
  * The optimisation procedure (Gauss-Newton, Levenberg–Marquardt, etc).

Those elements are provided in the library and also a C++ design that allows to easily introduce
efficient technicques for incremental image alignment:

  * Hager and Belhumeur's jacobian factorisation method::

      Efficient Region Tracking with Parametric Models of Geometry and Illumination.
      G. Hager, P. Belhumeur
      IEEE Trans. PAMI, 20(10), pp. 1025-39, October 1998.

  * Baker and Matthews's Inverse Compositional method::

      Lucas-Kanade 20 Years On: A Unifying Framework.
      Simon Baker, Iain Matthews
      International Journal of Computer Vision 56(3), 221–255, 2004

The img_align_lib already provides:

  * Jacobian factorisation for planar tracking (with a single texture object model) with
    2D similarity, affine and homography based motion models.
  * Jacobian factorisation for planar tracking using 2D appearance models
    (i.e. PCA of deforming faces)::

       Efficient illumination independent appearance-based face tracking.
       Jose M. Buenaposada, Enrique Muñoz, Luis Baumela.
       Image and vision computing , 27(5):560-578. 2009

  * Inverse Compositional with image gradients::

        Robust and Efficient Parametric Face Alignment
        Georgios Tzimiropoulos, Stefanos Zafeiriou, Maja Pantic
        ICCV 2011

  * A demo program for planar tracking.
  * A demo program for 2D appearance tracking of faces.

Requirements
------------

You need the following libraries installed on you system in order to
build ImgAlignLib:

* `OpenCV <http://www.opencv.org/>`_
* `CMake <http://www.cmake.org/>`_

Installation
------------

In order to compile the ImgAlignLib package you have to::

  $ cd img_align_lib/src
  $ mkdir build
  $ cd build
  $ cmake ..
  $ make

If everything went OK, you should be able to install the
package with::

  $ sudo make install

And then to test planar alignment you can run::

  $ test_img_align

and then to test the face appearance model based aligment::

  $ test_pca_model_align
