// CUDAdll.cpp : Defines the exported functions for the DLL application.
//

//#include "stdafx.h"

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
//#include <opencv/cvaux.h>
//#include <opencv/cv.h>

#include <stdio.h>

using namespace std;
extern "C" __declspec(dllexport) int UploadImToTexture(void* fboColorTex2, int width, int height, void* image, int align, int widthStep, int channels){
   
	if (glGetError())
       return -1;

	//yarp::sig::ImageOf<yarp::sig::PixelRgb> *image;
//	IplImage *cvImage;
//	cv::Mat ImageMat_Real;
	//cvCvtColor((IplImage*)image, cvImage, CV_RGB2BGR);
	//cvImage = (IplImage*)image;
//	ImageMat_Real=cvImage;
	
//	cv::namedWindow( "Display window 2", cv::WINDOW_AUTOSIZE );// Create a window for display.
//	imshow("Display window 2",ImageMat_Real);
	//cvSetData(cvImage, image,9);
	
	GLuint fboColorTex = (GLuint) (size_t) fboColorTex2;

	glBindTexture(GL_TEXTURE_2D, fboColorTex);
	glPixelStorei (GL_UNPACK_ALIGNMENT, align);
	glPixelStorei (GL_UNPACK_ROW_LENGTH, widthStep / channels);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 240, 0, GL_BGR, GL_UNSIGNED_BYTE, image);
	glTexImage2D(GL_TEXTURE_2D, 0,  GL_RGBA16F, 640, 240, 0, GL_BGR, GL_UNSIGNED_BYTE, image);

    if (glGetError())
       return -2;
    return 0;
}

extern "C" __declspec(dllexport) int* CudaEdgeLikelihood(int height,int width,void* ID [200],void* ID_real, void* image, int align, int widthStep, int channels){ 
	 
	int * likelihood = new int[200];
	//likelihood[0] = 0.02f;
	//return likelihood;

	// void* ID[200]
	extern void BindToTexture( cudaArray *cuArr );
	extern void BindToTextureGray( cudaArray *cuArr );
	extern void DeviceArrayCopyFromTexture( float3* dst, int dstStep, int width, int height );

	cv::Mat MatDisp;
	cv::Mat MatDispR;
	cv::Mat MatDispM;
	cv::Mat MatDispView;
	float sum, nonZero, zero;
	//float* likelihood = new float[200];

//	if (pArray !=NULL){
//		delete[] pArray;
//	}


	/*********Upload texture************/

//	if (glGetError())
 //      return;

	//yarp::sig::ImageOf<yarp::sig::PixelRgb> *image;
//	IplImage *cvImage;
//	cv::Mat ImageMat_Real;
	//cvCvtColor((IplImage*)image, cvImage, CV_RGB2BGR);
	//cvImage = (IplImage*)image;
//	ImageMat_Real=cvImage;
	
//	cv::namedWindow( "Display window 2", cv::WINDOW_AUTOSIZE );// Create a window for display.
//	imshow("Display window 2",ImageMat_Real);
	//cvSetData(cvImage, image,9);



	/*GLuint fboColorTex = (GLuint) (size_t) ID_real;
	glBindTexture(GL_TEXTURE_2D, fboColorTex);
	glPixelStorei (GL_UNPACK_ALIGNMENT, align);
	glPixelStorei (GL_UNPACK_ROW_LENGTH, widthStep / channels);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, 640, 240, 0, GL_RGBA, GL_UNSIGNED_BYTE, image);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, 640, 240, 0, GL_BGR, GL_UNSIGNED_BYTE, image);
	//glTexImage2D(GL_TEXTURE_2D, 0,  GL_FLOAT, 640, 240, 0, GL_R, GL_FLOAT, image); // testing...


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
	//cv::gpu::GpuMat gpuMat_AND(height,width, CV_32FC1);
	//cv::gpu::GpuMat gpuMat_OR (height,width, CV_32FC1);
	//cv::gpu::GpuMat gpuMat_Mult (height,width, CV_32FC1);
	//cv::gpu::GpuMat gpuMat_exp (height,width, CV_32FC1);

	/****   Real    ****/
	
	glBindTexture (GL_TEXTURE_2D, gltex_R);
	glBindTexture(GL_TEXTURE_2D, gltex_R);
	glPixelStorei (GL_UNPACK_ALIGNMENT, align);
	glPixelStorei (GL_UNPACK_ROW_LENGTH, widthStep / channels);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, 640, 240, 0, GL_RGBA, GL_UNSIGNED_BYTE, image);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, 640, 240, 0, GL_BGR, GL_UNSIGNED_BYTE, image);
	//glTexImage2D(GL_TEXTURE_2D, 0,  GL_FLOAT, 640, 240, 0, GL_R, GL_FLOAT, image); // testing...
	
	//cv::Mat MatDisp_R;

	//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	checkCudaErrors( cudaGraphicsGLRegisterImage( &cuda_tex_screen_resource_R, gltex_R, GL_TEXTURE_2D, cudaGraphicsMapFlagsReadOnly ) );  // Rever esta imagem... cudaGraphicsGLRegisterImage n pode ser chamada em loop...
	 
	// Copy color buffer
	checkCudaErrors( cudaGraphicsMapResources( 1, &cuda_tex_screen_resource_R, 0 ) );
	checkCudaErrors( cudaGraphicsSubResourceGetMappedArray( &cuArr_R, cuda_tex_screen_resource_R, 0, 0 ) );
	
	BindToTexture( cuArr_R );
	
	DeviceArrayCopyFromTexture( (float3*)gpuMat_R.data, gpuMat_R.step, gpuMat_R.cols, gpuMat_R.rows  );
	
	
	checkCudaErrors( cudaGraphicsUnmapResources( 1, &cuda_tex_screen_resource_R, 0 ) );
	checkCudaErrors( cudaGraphicsUnregisterResource(cuda_tex_screen_resource_R));    // Acrescentado
	
//	gpuMat_R.download(MatDisp);
//	cv::flip(MatDisp, MatDisp,0);
//	cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
//	imshow("Display window",MatDisp);

	cv::gpu::cvtColor(gpuMat_R,GgpuMat_R,CV_RGB2GRAY);  // Só uma vez... tirar do loop
	//cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
//	cv::namedWindow( "Display window R", cv::WINDOW_AUTOSIZE );
	//cv::namedWindow( "Display window G", cv::WINDOW_AUTOSIZE );
	//cv::namedWindow( "Display window M", cv::WINDOW_AUTOSIZE );

	//GgpuMat_R.download(MatDispR);
	//cv::flip(MatDispR, MatDispR,0);
	//cv::namedWindow( "Display window 2", cv::WINDOW_AUTOSIZE );// Create a window for display.
	//imshow("Display window R",MatDispR);
	
	//ofstream myfile;
	//myfile.open ("example.txt");
	
	//GgpuMat_R.download(MatDispR);

	//imwrite("matR.jpg", MatDispR);
	////myfile << "MATDispR" << MatDispR << endl;





	/*****SEE REAL IMAGES*******/
	/*float alpha, beta;
	alpha = 0.5;
	beta = ( 1.0 - alpha );

	gpuMat_R.download(MatDispR);
	/**/
	float result=0;
	for(int i=0;i<200;i++) {
		gltex = (GLuint)(size_t)(ID[i]);
		//myfile << "Enter loop..." << endl;
		//   Segmentada   
		glBindTexture (GL_TEXTURE_2D, gltex);
		//cv::Mat MatDisp;
		//myfile << "bind..." << endl;

		// DEBUG!
		GLint width,height,internalFormat;
		glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_COMPONENTS, &internalFormat); // get internal format type of GL texture
		glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width); // get width of GL texture
		glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height); // get height of GL texture
		//myfile << "internal format - gerada: "<< internalFormat <<"width: "<< width <<"height: " << height << endl;
		// END DEBUG

		//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGB, GL_UNSIGNED_SHORT_5_6_5, NULL); //GL_UNSIGNED_INT_8_8_8_8_REV
		//glTexSubImage2D(GL_TEXTURE_2D, 0,0,0, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, NULL); //GL_UNSIGNED_INT_8_8_8_8_REV
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		//myfile << glGetError();
		checkCudaErrors( cudaGraphicsGLRegisterImage( &cuda_tex_screen_resource, gltex, GL_TEXTURE_2D, cudaGraphicsMapFlagsReadOnly ) );  // Rever esta imagem... cudaGraphicsGLRegisterImage n pode ser chamada em loop...
		 //myfile << "register..." << endl;
		// Copy color buffer
		checkCudaErrors( cudaGraphicsMapResources( 1, &cuda_tex_screen_resource, 0 ) );
		//myfile << "Enter map..." << endl;
		checkCudaErrors( cudaGraphicsSubResourceGetMappedArray( &cuArr, cuda_tex_screen_resource, 0, 0 ) );
		//myfile << "Enter getmapped..." << endl;
		BindToTexture( cuArr );
		//myfile << "bind2..." << endl;
		DeviceArrayCopyFromTexture( (float3*)gpuMat.data, gpuMat.step, gpuMat.cols, gpuMat.rows  );
		//myfile << "copy..." << endl;
	 
		checkCudaErrors( cudaGraphicsUnmapResources( 1, &cuda_tex_screen_resource, 0 ) );
		//myfile << "Enter unmap..." << endl;
		checkCudaErrors( cudaGraphicsUnregisterResource(cuda_tex_screen_resource));    // Acrescentado
		//myfile << "Enter unregistered..." << endl;
		//GgpuMat.download(MatDisp);
		//cv::flip(MatDisp, MatDisp,0);
		//cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
		//imshow("Display window G",MatDisp);

		//glDeleteTextures(1,&fboColorTex);
		
		cv::gpu::cvtColor(gpuMat,GgpuMat,CV_RGB2GRAY);
		//myfile << "cvtcolor..." << endl;
		//GgpuMat.download(MatDisp);
		//MatDisp = MatDisp*255;
		//imwrite("matG.jpg", MatDisp);
		
		//cv::Scalar sumS3 = cv::gpu::countNonZero(GgpuMat);
		//myfile << "nonzero1..." << endl;
		
		//cv::Scalar sumS2 = cv::gpu::countNonZero(GgpuMat_R);
		//myfile << "nonzero2..." << endl;
		////myfile << "MATDisp" << endl;
		//myfile << "Gray " << sumS3 << sumS2 ;
		
		
		
		cv::gpu::multiply(GgpuMat,GgpuMat_R,GpuMatMul);
		//myfile << "multiply..." << endl;
		//cvtColor(GpuMatMul,
		//GpuMatMul.download(MatDispM);
		//MatDispM = MatDispM*255;
		////myfile << "MATDispM" << endl;
		
		
		//imwrite("matMult.jpg", MatDispM);
		//cv::flip(MatDispM, MatDispM,0);
		// Create a window for display.
		//imshow("Display window M",MatDispM);
		
		// cv::Scalar sumS = cv::gpu::sum(GpuMatMul);
		cv::Scalar sumS = cv::gpu::sum(GpuMatMul);
		
		//sum = sumS[0];
		//sum = sumS[0]*7;
		sum = sumS[0]*25; // Distance between 0 and 15 instead of 0-1 or 0-25
		//myfile << "Sum= "<< sum << endl;
		//cout << sum << endl;
		nonZero = (float) cv::gpu::countNonZero(GgpuMat); //gerada
		zero = GgpuMat_R.cols*GgpuMat_R.rows - (float) cv::gpu::countNonZero(GgpuMat_R); // real
		//myfile << "NonZer: " << sumS3 <<" , "<< nonZero<< " , "<< sumS2 ;
		//myfile << "NonZero= " << nonZero<< endl;
		//likelihood[i] = sum;
		if(nonZero==0) {
			likelihood[i] = 0;
		}
		else {
			result = sum/nonZero;
		//myfile << "result= " << result << endl;
			likelihood[i] = (int)((cv::exp(-result)) *1000);//* cv::exp(-fabs(zero - 5*nonZero)/nonZero));
		}
		//likelihood[i] = (int)((cv::exp(-sum))*10000);
		
		//likelihood[i] = i;


		/**************DISP REAL IMAGES COMPARSION*******************/
		/*gpuMat.download(MatDisp);
		cv::addWeighted(MatDispR, alpha, MatDisp, beta, 0.0, MatDispView);
		imwrite("iter"+i,MatDispView);
		/**/
				
	}
	//for(int i=0;i<200;i++) {
	//	likelihood[i] = i;
	//}
	//myfile.close();
	return likelihood;	
	//likelihood[0] = 0.2;
//	return likelihood;
/*

	// DOWNLOADs

	//glDeleteTextures(1,&fboColorTex);
	gpuMat.download(MatDisp);
	cv::flip(MatDisp, MatDisp,0);
	cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
	imshow("Display window",MatDisp);

	//glDeleteTextures(1,&fboColorTex);
	gpuMat_R.download(MatDisp_R);
	cv::flip(MatDisp_R, MatDisp_R,0);
	cv::namedWindow( "Display window 2", cv::WINDOW_AUTOSIZE );// Create a window for display.
	imshow("Display window 2",MatDisp_R);

	
		
	gpuMat_AND.download(MatDispAnd);
	cv::flip(MatDispAnd, MatDispAnd,0);
	cv::namedWindow( "Display window And", cv::WINDOW_AUTOSIZE );// Create a window for display.
	imshow("Display window And",MatDispAnd);
		
	gpuMat_OR.download(MatDispOr);
	cv::flip(MatDispOr, MatDispOr,0);
	cv::namedWindow( "Display window Or", cv::WINDOW_AUTOSIZE );// Create a window for display.
	imshow("Display window Or",MatDispOr);
	
*/
	//double likelihood = 0.2;
//	return 0.2f;
}
/*
extern "C" __declspec(dllexport) void releasePtr(float* pArray)
{
    delete[] pArray;
}
*/
