/**
* Copyright: (C) 2009 RobotCub Consortium
* Authors: Pedro Vicente
* Adapted from:  Matteo Taiana Particle Filter tracker code

* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include <pf3dTracker.hpp>

#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp>        
#include <opencv2/highgui/highgui.hpp>

#include <opencv/highgui.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <time.h>

#include <gsl/gsl_math.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/os/ResourceFinder.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>

using namespace cv;
using namespace yarp::dev;
using namespace iCub::iKin;
//constructor
PF3DTracker::PF3DTracker()
{
    ;
}

//destructor
PF3DTracker::~PF3DTracker()
{
    cout<<"oh my god! they killed the filter!\n";
}

void PF3DTracker::init() {

	_iter = 0;
	_outputVideoPort.open("/HPE_R/olhosIcub:o"); // Send L&R image to unity
	_HeadPort_out.open("/HPE_R/iCubhead:o"); // encoders head
	_RightArmPort_out.open("/HPE_R/rightArm:o"); // RA encoders
	_likelihood_port.open("/HPE_R/likelihood:i"); // Likelihood of each particle
	_fingers_port.open("/HPE_R/fingerPosition:i"); // Finger tips position in the left image
	_outputDataPort.open("/HPE_R/offsets:o");
	_ini = true;
	Time::delay(1.5);
	Network::connect("/HPE_R/offsets:o","/controller/offsetsRA:i");


	_nParticles=200;
	_accelStDev=3.0;//noise to spread particles

    srand((unsigned int)time(0)); //make sure random numbers are really random.
	//srand(727);
    rngState = cvRNG(rand());
    //allocate memory for the particles;
	_particles=cvCreateMat(8,_nParticles,CV_32FC1); // 0-6 Betas 7- likelihood
	//fill the memory with zeros, so that valgrind won't complain.
	cvSetZero(_particles);
	
	//define ways of accessing the particles:
	// Theta1
	_particles1 = cvCreateMatHeader( 1,_nParticles, CV_32FC1);
	cvInitMatHeader( _particles1, 1, _nParticles, CV_32FC1, _particles->data.ptr, _particles->step );
	// Theta2
	_particles2 = cvCreateMatHeader( 1,_nParticles, CV_32FC1);
	cvInitMatHeader( _particles2, 1, _nParticles, CV_32FC1, _particles->data.ptr + _particles->step*1, _particles->step );
	// Theta3
	_particles3 = cvCreateMatHeader( 1,_nParticles, CV_32FC1);
	cvInitMatHeader( _particles3, 1, _nParticles, CV_32FC1, _particles->data.ptr + _particles->step*2, _particles->step );
	// Theta4
	_particles4 = cvCreateMatHeader( 1,_nParticles, CV_32FC1);
	cvInitMatHeader( _particles4, 1, _nParticles, CV_32FC1, _particles->data.ptr + _particles->step*3, _particles->step );
	// Theta5
	_particles5 = cvCreateMatHeader( 1,_nParticles, CV_32FC1);
	cvInitMatHeader( _particles5, 1, _nParticles, CV_32FC1, _particles->data.ptr + _particles->step*4, _particles->step );
	// Theta6
	_particles6 = cvCreateMatHeader( 1,_nParticles, CV_32FC1);
	cvInitMatHeader( _particles6, 1, _nParticles, CV_32FC1, _particles->data.ptr + _particles->step*5, _particles->step );
	// Theta7
	_particles7 = cvCreateMatHeader( 1,_nParticles, CV_32FC1);
	cvInitMatHeader( _particles7, 1, _nParticles, CV_32FC1, _particles->data.ptr + _particles->step*6, _particles->step );
	// Likelihood
	_particles8 = cvCreateMatHeader( 1,_nParticles, CV_32FC1);
	cvInitMatHeader( _particles8, 1, _nParticles, CV_32FC1, _particles->data.ptr + _particles->step*7, _particles->step );

	//theta1-theta7
	_particles1to7 = cvCreateMatHeader( 7,_nParticles, CV_32FC1);
	cvInitMatHeader( _particles1to7, 7, _nParticles, CV_32FC1, _particles->data.ptr, _particles->step );


	_newParticles=cvCreateMat(8,_nParticles,CV_32FC1);
   _newParticles1to7 = cvCreateMatHeader( 7,_nParticles, CV_32FC1);
   cvInitMatHeader( _newParticles1to7, 7, _nParticles, CV_32FC1, _newParticles->data.ptr, _newParticles->step );

	// Resampling stuff
	_cumWeight =cvCreateMat(1,_nParticles+1,CV_32FC1);
    //allocate memory for "noise"
    _noise=cvCreateMat(7,_nParticles,CV_32FC1);
    cvSetZero(_noise);
	// generate particles

	float mean,StDev;
	StDev=3.5;

    //initialize Theta1
    mean=0;
    cvRandArr( &rngState, _particles1, CV_RAND_NORMAL, cvScalar(mean), cvScalar(StDev));
    //initialize Theta2
    cvRandArr( &rngState, _particles2, CV_RAND_NORMAL, cvScalar(mean), cvScalar(StDev));
    //initialize Theta3
	cvRandArr( &rngState, _particles3, CV_RAND_NORMAL, cvScalar(mean), cvScalar(StDev));
    //initialize Theta4
	cvRandArr( &rngState, _particles4, CV_RAND_NORMAL, cvScalar(mean), cvScalar(StDev));
	//initialize Theta5
	cvRandArr( &rngState, _particles5, CV_RAND_NORMAL, cvScalar(mean), cvScalar(StDev));
	//initialize Theta6
	cvRandArr( &rngState, _particles6, CV_RAND_NORMAL, cvScalar(mean), cvScalar(StDev));
	//initialize Theta7
	cvRandArr( &rngState, _particles7, CV_RAND_NORMAL, cvScalar(mean), cvScalar(StDev));
	

}
int PF3DTracker::run2(yarp::sig::Vector PosInicial, Mat ImageMat_Real_gray,  Mat ImageMat_Real_grayL) {

	//*****************************************
    //calculate the likelihood of each particle
    //*****************************************
	_iter++;
	float likelihood;
	IplImage *cvImageTmp, *cvImageFinal=NULL;
    float sumLikelihood=0.0;
    float maxLikelihood=0.0;
	ImageOf<PixelRgb> *image, *imageL;
	// R
	Mat ImageMat,ImageMat_init,ImageMat_gray,ImageMat_result1,ImageMat_result2;

	Time tempo;
	double timing1, timing2,timing3;
	// L
	Mat ImageMatL,ImageMat_initL, ImageMat_grayL,ImageMat_result1L, ImageMat_result2L;
	yarp::sig::Vector command(16), qarm, xf;
	Matrix MArm;
    iCubArm Arm("right");
	iKinChain *chainArm;
	chainArm=Arm.asChain();

	std::ofstream matrix_file("R2.txt",5);
	std::ofstream endEf_file("EF2.txt",5);
    int   maxIndex=-1;

	ImageOf<PixelBgr> & yarpReturnImage = _outputVideoPort.prepare();
	
	// Merge Right and Left cameras
	Size sz1 = ImageMat_Real_grayL.size();
	Size sz2 = ImageMat_Real_gray.size();
	Mat im4;
	Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC3);
	Mat left(im3, Rect(0, 0, sz1.width, sz1.height));
	ImageMat_Real_grayL.copyTo(left);
	Mat right(im3, Rect(sz1.width, 0, sz2.width, sz2.height));
	ImageMat_Real_gray.copyTo(right);
	
	cv::flip(im3,im4,0);

	cvImageTmp=cvCloneImage(&(IplImage)im4);
	if(cvImageFinal!=NULL)
		cvReleaseImage(&cvImageFinal);		
	cvImageFinal=cvImageTmp;
	cvImageTmp=NULL;
	yarpReturnImage.wrapIplImage(cvImageFinal);
	
	timing1=tempo.now();

	Bottle& sender = _RightArmPort_out.prepare();

	for(int count=0;count< _nParticles;count++) {
		timing1=tempo.now();

		for (unsigned int j=0;j<7;j++) {
			sender.addDouble(PosInicial[j]+cvmGet(_particles,j,count));
		}
		for(unsigned int j=7;j<16;j++) {
			sender.addDouble(PosInicial[j]);
		}
	}
	sender.addInt(_nParticles);  // n_particles

   	_RightArmPort_out.write();
	if(_ini){
		_ini=false;
		Bottle& senderH = _HeadPort_out.prepare();
		for(int k=0;k<6;k++){
			senderH.addDouble(_encodersHead[k]);
		}
		_HeadPort_out.write();  
		
	}
	_outputVideoPort.write();
	
	//Bottle *receive_likelihood;
	//likelihood;

	//std::ofstream likelihoodFile("like.txt",5);
	
	yInfo(" Waiting for Hypotheses generation");
	_fingers = _fingers_port.read();
	yInfo(" Waiting for Hypotheses evaluation");
	_receive_likelihood = _likelihood_port.read();
	yInfo(" DONE");

	timing2=tempo.now();
	yInfo( "Time: %f",timing2-timing1);
	maxLikelihood=0;
	for(int count=0;count<_nParticles;count++) {
		likelihood = _receive_likelihood->pop().asDouble();
		cvmSet(_particles,7,count,likelihood);
		sumLikelihood+=likelihood;
		if(likelihood>maxLikelihood) {
			maxLikelihood=likelihood;
			maxIndex=count;
		}
	}

	double sigma=1.0; // standard deviation guassian centered in J.
	double weight_j=0;
	double maxWeight_j=0;
	int maxWeight_index=-1;

	for(int count=0;count<_nParticles;count++) {
		double sum1=0;
		for(int j=0;j<_nParticles;j++) {
			double sum2=0;

			if( (float) cvmGet(_particles,7,j) > 0 ) {
				for(int junta=0;junta<7;junta++) {
					// || pi-pj||^2 / sigma^2
					sum2+= pow( ((float)cvmGet(_particles,junta,j)-(float)cvmGet(_particles,junta,count)) ,2)/pow(sigma,2); //Multivariate normal distribution
				}
				sum1 += std::exp(-sum2/( 2) )*cvmGet(_particles,7,j);
			}
		}
		sum1 = sum1/(_nParticles*sqrt(pow(2*M_PI,1)*pow(sigma,7)));
		weight_j = 500*sum1 + cvmGet(_particles,7,count);
		if(weight_j>maxWeight_j) {
			maxWeight_j=weight_j;
			maxWeight_index=count;
		}

   }

	// Best Particle
	 Bottle &bestOffset = _outputDataPort.prepare();

	yInfo("likelihood_best: %d", maxWeight_index);
	bestOffset.clear();
	for(int i=0;i<7;i++) {
		command[i]=PosInicial[i] + (float)cvmGet(_particles,i,maxWeight_index);
		out_file7 << (float)cvmGet(_particles,i,maxWeight_index) << " " ;
		if(_iter>45) // START sending offsets
		   bestOffset.addDouble(cvmGet(_particles,i,maxWeight_index));
	}
	bestOffset.addDouble(_iter);
	_outputDataPort.write();
	yInfo("likelihood_best: %f", (float) cvmGet(_particles,7,maxWeight_index));
	out_file7 << endl;
	qarm.resize(chainArm->getDOF());
	for(unsigned int i=0; i<=chainArm->getDOF(); i++) {
		qarm[i]=command[i]*CTRL_DEG2RAD;
	}
	chainArm->setAng(qarm);
	xf=chainArm->EndEffPosition();
	MArm = chainArm->getH();

	float minimum_likelihood=0.55; //do not resample if maximum likelihood is lower than this.
                                      //this is intended to prevent that the particles collapse on the origin when you start the tracker.

  yInfo("Max-likelihood: %f", maxLikelihood);
   if(maxLikelihood>minimum_likelihood) {
		//fflush(stdout);
    
		systematic_resampling(_particles1to7,_particles8,_newParticles,_cumWeight, sumLikelihood);
		//cout<<"T4\n";
		
		_accelStDev=_accelStDev*0.85;//0.85; // -15% every iteration when its doing resampling
    
   }
   
	else { //I can't apply a resampling with all weights equal to 0! 
   
		//fflush(stdout);
    
		cvCopy(_particles,_newParticles);
		_accelStDev=_accelStDev*1.15; //+15%  increase artificial dynamic noise when the particle with the highest likelihood is below the threshold
   }
   if(_accelStDev > 3.5)
		_accelStDev = 3.5;
	//APPLY THE MOTION MODEL:  Artificial Dynamics
   //******************************************

	_A = cvCreateMat(8,8,CV_32FC1);
	cvSetIdentity(_A); //
   cvMatMul(_A,_newParticles,_particles);

	float mean = 0; //NEW
 	CvMat* noiseSingle;
	noiseSingle = cvCreateMat(1,_nParticles,CV_32FC1);
    cvSetZero(noiseSingle);
	if(_accelStDev < 0.04) { // lowerbound of artificial noise
		_accelStDev=0.04;
	}

	cvRandArr( &rngState, _noise, CV_RAND_NORMAL, cvScalar(mean), cvScalar(_accelStDev));
	cvAdd(_particles1to7,_noise,_particles1to7);
	return maxWeight_index;
}

bool PF3DTracker::systematic_resampling(CvMat* oldParticlesState, CvMat* oldParticlesWeights, CvMat* newParticlesState, CvMat* cumWeight, float sum2)
{																								

    double u; //random number [0,1)
    double sum;
    int c1;
    int rIndex;  //index of the randomized array
    int cIndex;  //index of the cumulative weight array. cIndex -1 indicates which particle we think of resampling.
    int npIndex; //%new particle index, tells me how many particles have been created so far.
    int numParticlesToGenerate = _nParticles;

    //%N is the number of particles.
    //[lines, N] = size(oldParticlesWeight);
    //in CPP, _nParticles is the number of particles.


    //%NORMALIZE THE WEIGHTS, so that sum(oldParticles)=1.
    //oldParticlesWeight = oldParticlesWeight / sum(oldParticlesWeight);
    sum=0;

    for(c1=0;c1<_nParticles;c1++)
    {
        ((float*)(oldParticlesWeights->data.ptr + oldParticlesWeights->step*0))[c1] = (((float*)(oldParticlesWeights->data.ptr + oldParticlesWeights->step*0))[c1])/(float)sum2;
    }
	    for(c1=0;c1<_nParticles;c1++)
    {
        sum+=((float*)(oldParticlesWeights->data.ptr + oldParticlesWeights->step*0))[c1];
    }


    //%GENERATE N RANDOM VALUES
    //u = rand(1)/N; %random value [0,1/N)
    u=1/(double)numParticlesToGenerate*((double)rand()/(double)RAND_MAX); //martim

    //%the randomized values are going to be u, u+1/N, u+2/N, etc.
    //%instread of accessing this vector, the elements are computed on the fly:
    //%randomVector(a)= (a-1)/N+u.

    ((float*)(cumWeight->data.ptr))[0]=0;
    for(c1=0;c1<_nParticles;c1++)
    {

        ((float*)(cumWeight->data.ptr))[c1+1]=((float*)(cumWeight->data.ptr))[c1]+((float*)(oldParticlesWeights->data.ptr + oldParticlesWeights->step*0))[c1];

    }

    if(((float*)(cumWeight->data.ptr))[_nParticles]!=1)
    {
        ((float*)(cumWeight->data.ptr))[_nParticles]=1;
        if( ((float*)(cumWeight->data.ptr))[_nParticles]!=1)
        {
            //printf("still different\n");
        }
        else
        {
            //printf("now it-s ok\n");
        }
    }

    //%PERFORM THE ACTUAL RESAMPLING
    rIndex=0; //index of the randomized array
    cIndex=1; //index of the cumulative weight array. cIndex -1 indicates which particle we think of resampling.
    npIndex=0; //new particle index, tells me how many particles have been created so far.

    while(npIndex < numParticlesToGenerate) //martim
    {

        if(((float*)(cumWeight->data.ptr))[cIndex]>=(double)rIndex/(double)numParticlesToGenerate+u) //martim
        {

            ((float*)(newParticlesState->data.ptr + newParticlesState->step*0))[npIndex]=((float*)(oldParticlesState->data.ptr + oldParticlesState->step*0))[cIndex-1];
            ((float*)(newParticlesState->data.ptr + newParticlesState->step*1))[npIndex]=((float*)(oldParticlesState->data.ptr + oldParticlesState->step*1))[cIndex-1];
            ((float*)(newParticlesState->data.ptr + newParticlesState->step*2))[npIndex]=((float*)(oldParticlesState->data.ptr + oldParticlesState->step*2))[cIndex-1];
            ((float*)(newParticlesState->data.ptr + newParticlesState->step*3))[npIndex]=((float*)(oldParticlesState->data.ptr + oldParticlesState->step*3))[cIndex-1];
            ((float*)(newParticlesState->data.ptr + newParticlesState->step*4))[npIndex]=((float*)(oldParticlesState->data.ptr + oldParticlesState->step*4))[cIndex-1];
            ((float*)(newParticlesState->data.ptr + newParticlesState->step*5))[npIndex]=((float*)(oldParticlesState->data.ptr + oldParticlesState->step*5))[cIndex-1];
			((float*)(newParticlesState->data.ptr + newParticlesState->step*6))[npIndex]=((float*)(oldParticlesState->data.ptr + oldParticlesState->step*6))[cIndex-1];
            ((float*)(newParticlesState->data.ptr + newParticlesState->step*7))[npIndex]=0; //initializing weight
            rIndex=rIndex+1;
            npIndex=npIndex+1;
        }
        else
        {
            cIndex=cIndex+1;
        }
    }
    
    return false;        
}
