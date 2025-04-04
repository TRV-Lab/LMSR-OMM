cmake_minimum_required(VERSION 2.8)
project(main)
add_executable(main main.cpp)
 
set(LIBMWM /home/internal.jadecluster.com/lizhch/MATLAB/MATLAB_Runtime/R2024a/runtime/glnxa64)
 
set(PROJECT_NAME main)
 

include_directories(
    include 
    ${LIBMWM} 
)
 
# matlab
set(MATLAB_ROOT /home/internal.jadecluster.com/lizhch/MATLAB/MATLAB_Runtime/R2024a/)
include_directories(${MATLAB_ROOT}extern/include/)
target_link_libraries(${PROJECT_NAME} /home/internal.jadecluster.com/lizhch/MATLAB/MATLAB_Runtime/R2024a/bin/glnxa64/libmwmclmcr.so)
target_link_libraries(${PROJECT_NAME} /home/internal.jadecluster.com/lizhch/MATLAB/MATLAB_Runtime/R2024a/runtime/glnxa64/libmwmclmcrrt.so)
target_link_libraries(${PROJECT_NAME} /home/internal.jadecluster.com/lizhch/MATLAB/MATLAB_Runtime/R2024a/sys/os/glnxa64/libiomp5.so)
target_link_libraries(${PROJECT_NAME}  /research/tongji_project_ws/src/libmultiLaneTracking/libmultiLaneTracking.so)
target_link_libraries(${PROJECT_NAME}  ${PROJECT_SOURCE_DIR}/lib/libyaml-cpp.so)
