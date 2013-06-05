/* ------------------------------------------------------------------------- */
/*!
 *  \file trace.hpp
 *  \brief Debug macros definition.
 *  \author Jose Miguel Buenaposada.
 *  \date Creation date: 2001/04/04
 *  \date Modified     : 
 *
 */
/*
 * Lab. Visi'on Computacional y Rob'tica   | Computer Vision and Robotics Lab.
 * Facultad de Inform'atica                | Computer Science School
 * Universidad Polit'ecnica de Madrid (UPM)| Madrid Technical University
 * http://www.dia.fi.upm.es
 *
 * ------------------------------------------------------------------------- */

/* ------------------ Recursion Protection --------------------------------- */
#ifndef TRACE_HPP
#define TRACE_HPP

#include <iostream>

#define WRITE_IN(fmt) __FILE__<<"("<<__LINE__<<"):"<<fmt 

#define TRACE_ERROR(fmt) std::cerr << WRITE_IN(fmt)
#define TRACE_INFO(fmt) std::cout << WRITE_IN(fmt)
#define SHOW_VALUE(a) std::cout<<__FILE__<<"("<<__LINE__<<")"<<#a "=[" << a << "]" << std::endl;

#ifdef DEBUG
  #define TRACE_DEBUG(fmt) std::cout << WRITE_IN(fmt)
  #define DEBUG_SHOW_VALUE(a) std::cout<<__FILE__<<"("<<__LINE__<<")"<<#a "=[" << a << "]" << std::endl;
#else
  #define TRACE_DEBUG(fmt)
  #define DEBUG_SHOW_VALUE(a) 
#endif // #ifdef _DEBUG

#endif
