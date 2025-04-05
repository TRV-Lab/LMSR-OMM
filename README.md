# README

## I. Project Overview
### Project Name
LMSR-OMM

### Project Introduction
It constructs a novel HMM model with four types of emission probability factors. A lane marking map is established and associated with the SD map using HMM to build the SD+ map. In areas where the SD+ map exists, the vehicle can re-localize itself by the Iterative Closest Point (ICP) registration method for lane markings. Based on the association probability between adjacent lanes and roads, the probability factor of lane marking observation is obtained. The driving scenario recognition model is applied to generate the emission probability factor of scenario recognition, which improves the performance of map matching on elevated roads and the normal roads below them.

### Project Objectives
Current online map matching algorithms are prone to errors in complex road networks, especially at Y-shaped bifurcations and multilevel road areas. This paper proposes an online Standard Definition (SD) map matching method assisted by lane marking mapping and driving scenario recognition based on the Hidden Markov Model (HMM). It effectively improves the accuracy of online map matching, especially on Y-shaped bifurcated roads and multilevel road areas.

## II. Installation and Configuration
### Environment Requirements
Ubuntu (Linux), CMake, C++ 17, HDF5, Eigen3, GeographicLib, nlohmann_json, PCL, Boost

### Additional Requirements
Zenseact's proprietary map software and map dynamic libraries are required (commercial software, not open - source).

### Compilation
Use CMake for compilation.

## III. Usage Guide
Compile to generate the executable file `hmm_madmap_node`. Then, you can achieve map matching under different parameters by modifying the `config.yaml` configuration. Generating the lane marking map depends on `libdateLoaderFeature` and `libmultiLaneTracking`. Then, set the `ASSOCIATE_LANE_MODE` in the `config.yaml` configuration to `true` to generate the lane marking map, and it can be used. For OBDSC, you need to use the code and pre - trained models provided at [https://figshare.com/articles/journal_contribution/Code_and_Data_of_OMM - OBDSC/21782267](https://figshare.com/articles/journal_contribution/Code_and_Data_of_OMM-OBDSC/21782267) in advance.

## IV. Technical Documentation
View our pre-print paper on arXiv: xxx.com

## V. Contact Information
You can contact the main developer of the code via 2636287741@qq.com.
