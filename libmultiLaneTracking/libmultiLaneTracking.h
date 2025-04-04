//
// MATLAB Compiler: 24.1 (R2024a)
// Date: Thu Jun 20 10:31:32 2024
// Arguments:
// "-B""macro_default""-W""cpplib:libmultiLaneTracking,all""-T""link:lib""-d""/h
// ome/lzc/multiLaneTracking0530/libmultiLaneTracking/for_testing""-v""/home/lzc
// /multiLaneTracking0530/multiLaneTracking.m"
//

#ifndef libmultiLaneTracking_h
#define libmultiLaneTracking_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" { // sbcheck:ok:extern_c
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_libmultiLaneTracking_C_API 
#define LIB_libmultiLaneTracking_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_libmultiLaneTracking_C_API 
bool MW_CALL_CONV libmultiLaneTrackingInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_libmultiLaneTracking_C_API 
bool MW_CALL_CONV libmultiLaneTrackingInitialize(void);
extern LIB_libmultiLaneTracking_C_API 
void MW_CALL_CONV libmultiLaneTrackingTerminate(void);

extern LIB_libmultiLaneTracking_C_API 
void MW_CALL_CONV libmultiLaneTrackingPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_libmultiLaneTracking_C_API 
bool MW_CALL_CONV mlxMultiLaneTracking(int nlhs, mxArray *plhs[], int nrhs, mxArray 
                                       *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_libmultiLaneTracking
#define PUBLIC_libmultiLaneTracking_CPP_API __declspec(dllexport)
#else
#define PUBLIC_libmultiLaneTracking_CPP_API __declspec(dllimport)
#endif

#define LIB_libmultiLaneTracking_CPP_API PUBLIC_libmultiLaneTracking_CPP_API

#else

#if !defined(LIB_libmultiLaneTracking_CPP_API)
#if defined(LIB_libmultiLaneTracking_C_API)
#define LIB_libmultiLaneTracking_CPP_API LIB_libmultiLaneTracking_C_API
#else
#define LIB_libmultiLaneTracking_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_libmultiLaneTracking_CPP_API void MW_CALL_CONV multiLaneTracking(const mwArray& mat_file,const mwArray& p_start, const mwArray& p_end);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
