# README

## I. Project Overview
### Project Name
LMSR-OMM

### Project Introduction
It constructs a novel HMM model with four types of emission probability factors. A lane marking map is established and associated with the SD map using HMM to build the SD+ map. In areas where the SD+ map exists, the vehicle can re-localize itself by the Iterative Closest Point (ICP) registration method for lane markings. Based on the association probability between adjacent lanes and roads, the probability factor of lane marking observation is obtained. The driving scenario recognition model is applied to generate the emission probability factor of scenario recognition, which improves the performance of map matching on elevated roads and the normal roads below them.
![ The overall framework of online map matching in complex road
networks using lane markings and scenario recognition.](doc/12.png) 
To generate the lane marking map, the system processes camera data through multi-lane tracking and associates it with the SD map.
![ Schematic diagram of enriched SD map fusion. The thick polylines with arrows are the roads of SD map, and the thin solid and dashed polylines represent the solid and dashed lane markings, respectively. In this figure, (a) shows the overlay of lane markings and the SD map in the same coordinate system, and note that there is no correlation between the lane markings and the SD map before performing the association. (b) shows the association result of multiple lane markings with multiple roads. The associated roads and lane markings are marked with the same color In this figure. One road can be associated with multiple lane marking instances, and different parts of one lane marking instance can also be associated with multiple roads with different probabilities..](doc/6.png) 

### Project Objectives
Current online map matching algorithms are prone to errors in complex road networks, especially at Y-shaped bifurcations and multilevel road areas. This paper proposes an online Standard Definition (SD) map matching method assisted by lane marking mapping and driving scenario recognition based on the Hidden Markov Model (HMM). It effectively improves the accuracy of online map matching, especially on Y-shaped bifurcated roads and multilevel road areas.
![ case in intersection](doc/case.webm) 
### Key Experimental Results
Ablation studies confirm the critical role of lane marking ($P_L$​) and scenario recognition ($P_S$​) factors.
![ Comparison of map matching results. (b) Baseline with $P_L$​ and $P_S$​ ablated shows errors (red); (c) Proposed method with both factors correctly localizes the vehicle (green).](doc/10.png) 

#### DEMO1：
![ demo1](doc/1.mp4) 
#### DEMO2：
![ demo1](doc/2.mp4) 
#### DEMO3：
![ demo1](doc/3.mp4) 

## II. Installation and Configuration
### Environment Requirements

| Category&#xA;          | Tool/Library Name&#xA; | Recommended Version&#xA; | Description&#xA;                                                                                                                                                   |
| ---------------------- | ---------------------- | ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Operating System&#xA;  | Ubuntu (Linux)&#xA;    | 20.04 LTS&#xA;           | Long-term support version with abundant community resources, ensuring high compatibility and stability.&#xA;                                                       |
| Development Tools&#xA; | CMake&#xA;             | 3.20.0&#xA;              | Supports modern C++ features, offering powerful and stable project building capabilities.&#xA;                                                                     |
| Development Tools&#xA; | C++&#xA;               | C++17 Standard&#xA;      | The core language version, ensuring compatibility with new features like structured bindings and fold expressions.&#xA;                                            |
| Libraries&#xA;         | HDF5&#xA;              | 1.12.2&#xA;              | Efficiently stores and manages large-scale data, with numerous performance and security improvements in this version.&#xA;                                         |
| Libraries&#xA;         | Eigen3&#xA;            | 3.4.0&#xA;               | A linear algebra library, with version 3.4.0 optimizing matrix operation performance.&#xA;                                                                         |
| Libraries&#xA;         | GeographicLib&#xA;     | 2.4&#xA;                 | A geographic coordinate calculation library, enhancing the accuracy of coordinate system transformations in this version.&#xA;                                     |
| Libraries&#xA;         | nlohmann\_json&#xA;    | 3.11.2&#xA;              | A JSON data processing library, providing simple and easy-to-use interfaces, with significant improvements in parsing efficiency in version 3.11.2.&#xA;           |
| Libraries&#xA;         | PCL&#xA;               | 1.12.1&#xA;              | A point cloud processing library, adding multiple point cloud registration algorithms in version 1.12.1, fitting the project's lane positioning requirements.&#xA; |
| Libraries&#xA;         | Boost&#xA;             | 1.78.0&#xA;              | A general C++ library, with version 1.78.0 improving multi-threading and file system modules, enhancing project extensibility.&#xA;                                |


### Additional Requirements
Zenseact's proprietary map software and map dynamic libraries are required (commercial software, not open - source).

### Compilation
Use CMake for compilation.
```
mkdir build
cd build
cmake ..
make
```

## III. Usage Guide
Compile to generate the executable file `hmm_madmap_node`. 
Then, you can achieve map matching under different parameters by modifying the `config.yaml` configuration. 
Generating the lane marking map depends on `libdateLoaderFeature` and `libmultiLaneTracking`. 
Then, set the `ASSOCIATE_LANE_MODE` in the `config.yaml` configuration to `true` to generate the lane marking map, and it can be used. 
For OBDSC, you need to use the code and pre - trained models provided at [https://figshare.com/articles/journal_contribution/Code_and_Data_of_OMM - OBDSC/21782267](https://figshare.com/articles/journal_contribution/Code_and_Data_of_OMM-OBDSC/21782267) in advance.

## IV. Technical Documentation
View our pre-print paper on arXiv: https://arxiv.org/abs/2505.05007

## V. Contact Information
You can contact the main developer of the code via 2636287741@qq.com.
