cmake_minimum_required(VERSION 3.0.2)
project(LMSR_OMM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BINARY_DIR ${PROJECT_SOURCE_DIR}/../../build/${PROJECT_NAME})
set(CMAKE_CXX_FLAGS "-Wno-error=deprecated-declarations -Wno-deprecated-declarations ")

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
# set(MADMAP_PACKAGE_PATH ${PROJECT_SOURCE_DIR}/../../../develop_tools/zenseact-madmap-cpp-api-4.0.0-linux-x86_64)
set(MADMAP_PACKAGE_PATH ${PROJECT_SOURCE_DIR}/../../../develop_tools/zenseact-madmap-cpp-api-9.0.0-linux-x86_64)
set(FAKE_MADMAP_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/../VDI_include)

list(APPEND CMAKE_PREFIX_PATH "/research/develop_tools/hdf5-1.14.1-2/build/usr/local/hdf5")
find_package(HDF5 REQUIRED COMPONENTS C CXX)
list(APPEND CMAKE_PREFIX_PATH "/research/develop_tools/eigen-3.4.0/build")
find_package(Eigen3 REQUIRED)
list(APPEND CMAKE_PREFIX_PATH "/research/develop_tools/geographiclib/build")
find_package(GeographicLib REQUIRED)
list(APPEND CMAKE_PREFIX_PATH "/research/develop_tools/json/build")
find_package(nlohmann_json REQUIRED)
find_package(PCL REQUIRED QUIET)
# find_package(Boost REQUIRED COMPONENTS thread)
# add_definitions(-DBOOST_ALL_DYN_LINK)

list(APPEND CMAKE_PREFIX_PATH "~/anaconda3/lib")
# find_package(Python 3.11 REQUIRED COMPONENTS Development)
find_package(Python REQUIRED COMPONENTS Development)

include_directories(
 include
 ${HDF5_INCLUDE_DIRS}
 ${PYTHON_INCLUDE_DIRS}
 "/research/develop_tools/hdf5-1.14.1-2/build/usr/local/hdf5/include"
 "/research/develop_tools/matplotlib-cpp/"
 "/research/develop_tools/json/include"
 "~/anaconda3/include/python3.11/"
 "~/anaconda3/lib/python3.11/site-packages/numpy/core/include/"
 ${EIGEN3_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
#  ${OpenCV_LIBS}
#  ${Boost_INCLUDE_DIR}
 ${GeographicLib_INCLUDE_DIRS}
 ${MADMAP_PACKAGE_PATH}/include
 ${FAKE_MADMAP_INCLUDE_DIRS}
)

add_library(h5parse SHARED
  src/parse/satellite.cpp
  src/parse/positioning_satellite_debug.cpp
  src/parse/holistic_path.cpp
  src/parse/camera_lane_marker_detections.cpp
  # src/parse/motion_data.cpp
  # src/parse/lane_markers.cpp
  # src/parse/vision_traffic_signs.cpp
  # src/parse/ego_vehicle_motion_trail_state.cpp
  # src/parse/visual_odometry_verification.cpp
  # src/parse/nibbler.cpp
  src/parse/oxts.cpp
  # src/parse/localization_data.cpp

)
# add_library(filesearch SHARED
#   src/utils/file_search.cpp
# )

link_directories(
  lib
  ${PCL_LIBRARY_DIRS}
  # ${Boost_LIBRARY_DIRS}

)

set(LIBRARIES 
${HDF5_CXX_LIBRARIES} 
${HDF5_LIBRARIES}
h5parse
# filesearch
yaml-cpp
)
# add_executable(split_nibbler 
#   src/utils/split_nibbler.cpp
# )
# target_link_libraries(split_nibbler
#   ${LIBRARIES}
# )

# add_executable(split_oxts 
#   src/utils/split_oxts.cpp
# )
# target_link_libraries(split_oxts
#   ${LIBRARIES}
# )

# add_executable(sensorfusion_node
#   src/sensorfusion.cpp
#   src/output/evaluation.cpp
#   src/output/evaluation_psd.cpp
#   src/output/evaluation_satellite.cpp
#   src/output/evaluation_mre.cpp
# )
# target_link_libraries(sensorfusion_node
#   ${LIBRARIES}
# )
add_definitions(-DPCL_NO_PRECOMPILE)
if(EXISTS ${MADMAP_PACKAGE_PATH})
  # add_executable(parse 
  #   src/parse.cpp
  #   # src/map_match.cpp
  # )
  # target_link_libraries(parse
  #   ${LIBRARIES}
  #   # 
  #   )
  # add_executable(map_match_node 
  #   src/map_match.cpp
  # )
  # target_link_libraries(map_match_node
  # ${MADMAP_PACKAGE_PATH}/lib/libmadmapstandard.so
  # )
  add_executable(hmm_madmap_node
  src/hmm_madmap.cpp
  src/hmm_geo.cpp
  src/hmm_basis.cpp
  src/hmm_lane.cpp
  src/hmm_io.cpp
  src/hmm_icp.cpp
  src/hmm_holistic_path.cpp
  src/hmm_plot.cpp
  src/hmm_bias_correction.cpp
  src/kalman_filter.cpp
  src/utils/file_search.cpp
  )
  target_link_libraries(hmm_madmap_node
  ${LIBRARIES}
  ${MADMAP_PACKAGE_PATH}/lib/libmadmapstandard.so
  ${GeographicLib_LIBRARIES}
  ${Python_LIBRARIES}
  ${PCL_LIBRARIES}
  # ${OpenCV_LIBS} 
  # ${Boost_LIBRARIES}
  nlohmann_json::nlohmann_json
  )
  target_include_directories(hmm_madmap_node
  PUBLIC
  "~/anaconda3/include/python3.11/"
  )
  target_link_directories(hmm_madmap_node
  PUBLIC
  "~/anaconda3/bin/python3.11-config"
  )
endif()

# add_executable(hmm_node
# src/hmm.cpp)


