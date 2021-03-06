// CUDAdll.cpp : Defines the exported functions for the DLL application.
/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 *
 */

// OpenGL
#include <gl\glew.h>
#include <gl\GL.h>
#include <gl\GLU.h>
#include <gl\glext.h>

// CUDA stuff
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include <helper_cuda.h>
#include <helper_cuda_gl.h>

#include <helper_functions.h>
#include <rendercheck_gl.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>        
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/gpu/gpumat.hpp>

#include <stdio.h>

using namespace std;

/**
*
* @ingroup  
* \defgroup likelihoodAssessment likelihoodAssessment
* likelihood Assessment based on edge extraction for the iCub Humanoid Robot
* Version: v2.0 
* \section Description
*
* The likelihood Assessment (likelihood.cpp and Cuda_Gl.cu files) will be used calculate the likelihood metric between the perception and the generated hypotheses inside the internal model exploiting the interoperability between the OpenGL, CUDA and OpenCV libraries.
*
* \section dependencies_sec Dependencies
*
* OpenCV
*
* CUDA ToolKit
*
* Check the installation guide in \ref installation to more details.
*
* \author  Pedro Vicente pvicente@isr.tecnico.ulisboa.pt
* \n
* \copyright  Released under the terms of the GNU GPL v3.0.
*/

/**
*
* Class likelihoodAssessment
*
* Version: v2.0 
* \author  Pedro Vicente pvicente@isr.tecnico.ulisboa.pt
* \file likelihood.cpp
* \n
* \copyright  Released under the terms of the GNU GPL v3.0.
*/
extern "C" __declspec(dllexport) int UploadImToTexture(void* fboColorTex2, int width, int height, void* image, int align, int widthStep, int channels)
{
    if (glGetError())
        return -1;

    GLuint fboColorTex = (GLuint) (size_t) fboColorTex2;

    glBindTexture(GL_TEXTURE_2D, fboColorTex);
    glPixelStorei (GL_UNPACK_ALIGNMENT, align);
    glPixelStorei (GL_UNPACK_ROW_LENGTH, widthStep / channels);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glTexImage2D(GL_TEXTURE_2D, 0,  GL_RGBA16F, 640, 240, 0, GL_BGR, GL_UNSIGNED_BYTE, image);

    if (glGetError())
       return -2;
    return 0;
}

extern "C" __declspec(dllexport) int* CudaEdgeLikelihood(int height,int width,void* ID [200],void* ID_real, void* image, int align, int widthStep, int channels)
{ 
    int * likelihood = new int[200];
    extern void BindToTexture( cudaArray *cuArr );
    extern void BindToTextureGray( cudaArray *cuArr );
    extern void DeviceArrayCopyFromTexture( float3* dst, int dstStep, int width, int height );

    cv::Mat MatDisp;
    cv::Mat MatDispR;
    cv::Mat MatDispM;
    cv::Mat MatDispView;
    float sum, nonZero, zero;

    /************************************************************/

    // Seg
    GLuint gltex;
    struct cudaGraphicsResource *cuda_tex_screen_resource;
    cv::gpu::GpuMat gpuMat(height,width, CV_32FC3 );
    cv::gpu::GpuMat GgpuMat(height,width, CV_32FC1 );
    cudaArray *cuArr;
    // Real
    GLuint gltex_R = (GLuint)(size_t)(ID_real);
    struct cudaGraphicsResource *cuda_tex_screen_resource_R;
    cv::gpu::GpuMat gpuMat_R(height,width, CV_32FC3 );
    cv::gpu::GpuMat GgpuMat_R(height,width, CV_32FC1 );
    cudaArray *cuArr_R;

    // Process
    cv::gpu::GpuMat GpuMatMul(height,width, CV_32FC1);

    /****   Real    ****/

    glBindTexture (GL_TEXTURE_2D, gltex_R);
    glBindTexture(GL_TEXTURE_2D, gltex_R);
    glPixelStorei (GL_UNPACK_ALIGNMENT, align);
    glPixelStorei (GL_UNPACK_ROW_LENGTH, widthStep / channels);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, 640, 240, 0, GL_BGR, GL_UNSIGNED_BYTE, image);

    checkCudaErrors( cudaGraphicsGLRegisterImage( &cuda_tex_screen_resource_R, gltex_R, GL_TEXTURE_2D, cudaGraphicsMapFlagsReadOnly ) );  

    // Copy color buffer
    checkCudaErrors( cudaGraphicsMapResources( 1, &cuda_tex_screen_resource_R, 0 ) );
    checkCudaErrors( cudaGraphicsSubResourceGetMappedArray( &cuArr_R, cuda_tex_screen_resource_R, 0, 0 ) );

    BindToTexture( cuArr_R );

    DeviceArrayCopyFromTexture( (float3*)gpuMat_R.data, gpuMat_R.step, gpuMat_R.cols, gpuMat_R.rows  );

    checkCudaErrors( cudaGraphicsUnmapResources( 1, &cuda_tex_screen_resource_R, 0 ) );
    checkCudaErrors( cudaGraphicsUnregisterResource(cuda_tex_screen_resource_R));    

    cv::gpu::cvtColor(gpuMat_R,GgpuMat_R,CV_RGB2GRAY);  

    float result=0;
    int lambdaEdge = 25; 
    for(int i=0;i<200;i++) 
    {
        gltex = (GLuint)(size_t)(ID[i]); // ID is a vector with pointers to the render textures
        glBindTexture (GL_TEXTURE_2D, gltex);
        GLint width,height,internalFormat;
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_COMPONENTS, &internalFormat); // get internal format type of GL texture
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width); // get width of GL texture
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height); // get height of GL texture
        
        checkCudaErrors( cudaGraphicsGLRegisterImage( &cuda_tex_screen_resource, gltex, GL_TEXTURE_2D, cudaGraphicsMapFlagsReadOnly ) );  
        // Copy color buffer
        checkCudaErrors( cudaGraphicsMapResources( 1, &cuda_tex_screen_resource, 0 ) );
        checkCudaErrors( cudaGraphicsSubResourceGetMappedArray( &cuArr, cuda_tex_screen_resource, 0, 0 ) );
        BindToTexture( cuArr );
        
        DeviceArrayCopyFromTexture( (float3*)gpuMat.data, gpuMat.step, gpuMat.cols, gpuMat.rows  ); // DeviceArrayCopyFromTexture function defined on Cuda_Gl.cu
        
        checkCudaErrors( cudaGraphicsUnmapResources( 1, &cuda_tex_screen_resource, 0 ) );
        checkCudaErrors( cudaGraphicsUnregisterResource(cuda_tex_screen_resource));
        cv::gpu::cvtColor(gpuMat,GgpuMat,CV_RGB2GRAY);

        // Apply the likelihood Assessment 
        // GgpuMat - generated Image
        // GgpuMat_R - Real Distance Transform image
        cv::gpu::multiply(GgpuMat,GgpuMat_R,GpuMatMul);
        cv::Scalar sumS = cv::gpu::sum(GpuMatMul);

        /*
        Check the article:
        Online Body Schema Adaptation Based on Internal Mental Simulation and Multisensory Feedback, Vicente et al.
        In particular, Equation (21)
        */
        sum = sumS[0]*lambdaEdge; // lambdaEdge is a tuning parameter for distance sensitivity 
        nonZero = (float) cv::gpu::countNonZero(GgpuMat); //generated image
        if(nonZero==0) 
        {
            likelihood[i] = 0.000000001; // Almost Zero
        }
        else 
        {
            result = sum/nonZero;
            likelihood[i] = (int)((cv::exp(-result)) *1000);
        }
    }
    return likelihood;
}
