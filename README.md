# Online Body Schema Adaptation

# Dependencies and requirements
* Opencv (tested with 2.4.10)
* CUDA toolkit (tested with 6.5 - 32bit version)
  * Nvidia GPU is needed
* Yarp
* iCub
* Visual Studio (tested with VS 10)

[already embedded in the internal model module build - Emgu CV 2.4.10]

# Installation

* Install CUDA toolkit 6.5 32 bit
  * https://developer.nvidia.com/cuda-toolkit-65
* Install OpenCV with CUDA support (32 bit) (from sources)
  * https://sourceforge.net/projects/opencvlibrary/files/opencv-win/2.4.10/
* Install Yarp and iCub (32 bit)
  * (from binaries) http://wiki.icub.org/wiki/Windows:_installation_from_binaries 
  * (from sources)  http://wiki.icub.org/wiki/Windows:_installation_from_sources
* Yarp Bindings to C# using SWIG
  * http://www.yarp.it/yarp_swig.html#yarp_swig_windows

# Instructions

After the installation of all the dependencies:
  * Define the CUDA_SDK_ROOT_DIR environment variable to point to 'C:\ProgramData\NVIDIA Corporation\CUDA Samples\v6.5\common'
    * to include some headers needed by the CUDACompareEdge2 module compilation
  * Compile the CUDACompareEdge2 (32bit)
   * make sure that Cmake finds the CUDA_SDK_ROOT_DIR
  * Compile the HandPoseEstimation module
  
* Copy the following files to the folder 'modules/internalmodel/icub-internalmodel-rightA-cam-Lisbon_Data/Plugins/'
  * the yarp.dll, generated by the C# bindings
  * the CudaCompareEdge2.dll, generated by the compilation of the HandPoseEstimation module

# Run the online body schema Adaptation
* run the .exe of the internal model in the terminal with the argument '-force-opengl'
  * icub-internalmodel-rightA-cam-Lisbon.exe -force-opengl  
* run the HandPoseEstimation module
* run and connect the modules using the XML file in 'app/scripts/' (with yarpmanager)

* To start the estimation send a START command to the HandPoseEstimation module
  * e.g. yarp write ... /HPE_R/command:i
  * >> START

Options:
* run the forked version of [show-fingers-module] (https://github.com/vicentepedro/icub-contrib/tree/master/show-fingers) in order to see the body schema adaptation (after 45 iterations of the estimation).

# Final notes

Please be aware that this repository is still under development and some errors may occur.
The internal model module can only run in windows machines.

For more details see the following reference:

    @ARTICLE{10.3389/frobt.2016.00007,
    AUTHOR={Vicente, Pedro  and  Jamone, Lorenzo  and  Bernardino, Alexandre},   
    TITLE={Online body schema adaptation based on internal mental simulation and multisensory feedback},      
    JOURNAL={Frontiers in Robotics and AI},      
    VOLUME={3},      
    YEAR={2016},      
    NUMBER={7},     
    DOI={10.3389/frobt.2016.00007},      
    ISSN={2296-9144}
    
The full article can be found [here] (http://journal.frontiersin.org/article/10.3389/frobt.2016.00007/full)
