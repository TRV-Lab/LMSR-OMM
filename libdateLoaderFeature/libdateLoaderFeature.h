//
// MATLAB Compiler: 24.1 (R2024a)
// Date: Mon Jun 24 10:36:10 2024
// Arguments:
// "-B""macro_default""-W""cpplib:libdateLoaderFeature,all""-T""link:lib""-d""/h
// ome/lzc/multiLaneTracking0530/libdateLoaderFeature/for_testing""-v""/home/lzc
// /multiLaneTracking0530/dateLoaderFeature.m"
//

#ifndef libdateLoaderFeature_h
#define libdateLoaderFeature_h 1

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
#ifndef LIB_libdateLoaderFeature_C_API 
#define LIB_libdateLoaderFeature_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_libdateLoaderFeature_C_API 
bool MW_CALL_CONV libdateLoaderFeatureInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_libdateLoaderFeature_C_API 
bool MW_CALL_CONV libdateLoaderFeatureInitialize(void);
extern LIB_libdateLoaderFeature_C_API 
void MW_CALL_CONV libdateLoaderFeatureTerminate(void);

extern LIB_libdateLoaderFeature_C_API 
void MW_CALL_CONV libdateLoaderFeaturePrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_libdateLoaderFeature_C_API 
bool MW_CALL_CONV mlxDateLoaderFeature(int nlhs, mxArray *plhs[], int nrhs, mxArray 
                                       *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_libdateLoaderFeature
#define PUBLIC_libdateLoaderFeature_CPP_API __declspec(dllexport)
#else
#define PUBLIC_libdateLoaderFeature_CPP_API __declspec(dllimport)
#endif

#define LIB_libdateLoaderFeature_CPP_API PUBLIC_libdateLoaderFeature_CPP_API

#else

#if !defined(LIB_libdateLoaderFeature_CPP_API)
#if defined(LIB_libdateLoaderFeature_C_API)
#define LIB_libdateLoaderFeature_CPP_API LIB_libdateLoaderFeature_C_API
#else
#define LIB_libdateLoaderFeature_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_libdateLoaderFeature_CPP_API void MW_CALL_CONV dateLoaderFeature(const mwArray& oxtsDataFolder, const mwArray& visionDataFileName, const mwArray& datetime_enter, const mwArray& datetime_leave, const mwArray& dataID);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
