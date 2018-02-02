# Online Body Schema Adaptation & Markerless eye-hand kinematic calibration

We propose a markerless hand pose estimation software for the iCub humanoid robot. 
The agent can calibrate its eye-hand kinematic chain without any markers which, from a developmental psychology perspective, can be seen as a body schema adaptation.

## Overview:
- Repository Organization
- Dependencies & how to install
- Running the Modules
- References

## Repository Organization:
The code is divided into three logical components \ref modules: 

  i) the hand pose estimation  (see \ref handPoseEstimation-module);
  ii) the Robot’s Internal Model generator (see \ref internalModel);
  iii) the likelihood assessment (see \ref likelihoodAssessment);

 which are implemented, respectively, at the following repository locations:
- modules/handPoseEstimation:
   - include/handPoseEstimationModule.h
   - src/handPoseEstimationMain.cpp
   - src/handPoseEstimationModule.cpp
- modules/internalmodel:
   - icub-internalmodel-rightA-cam-Lisbon.exe
   - icub-internalmodel-leftA-cam-Lisbon.exe
- modules/likelihodAssessment:
   - src/Cuda_Gl.cu
   - src/likelihood.cpp

The software architecture implementing the proposed eye-hand calibration solution can be seen in the following picture:

<p align="center" > <img src="../../../images/UML_robot.png" width=500 > </p>

## How to install

Please refer to the documentation:

\ref installation

## Running the Modules

Please refer to the documentation:

\ref how_to_use

## References

If you use this code please cite the following reference:

    ARTICLE{ 10.3389/frobt.2016.00007,
    AUTHOR={Vicente, Pedro  and  Jamone, Lorenzo  and  Bernardino, Alexandre},
    TITLE={Online body schema adaptation based on internal mental simulation and multisensory feedback},
    JOURNAL={Frontiers in Robotics and AI},
    VOLUME={3},
    YEAR={2016},
    NUMBER={7},
    DOI={10.3389/frobt.2016.00007},
    ISSN={2296-9144}
    }
    
The full open-access article can be found [here](https://doi.org/10.3389/frobt.2016.00007)

 \author  Pedro Vicente pvicente@isr.tecnico.ulisboa.pt
 \copyright  Released under the terms of the GNU GPL v3.0.
