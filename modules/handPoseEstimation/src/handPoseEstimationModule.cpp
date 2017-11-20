// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 *
 */


#include "handPoseEstimationModule.h"
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace cv;
/******************************************************************************************/
bool handPoseEstimationModule::configure(ResourceFinder &rf)
{
    //parameters' initialization

    moduleName = rf.check("name", Value("hpe")).asString();
    setName(moduleName.c_str());
    arm = rf.check("arm", Value("right")).asString();
    
    initialMean = rf.check("initialMean", Value(0.0)).asDouble();    
    initialStdDev= rf.check("initialStdDev", Value(3.5)).asDouble();
    initialArtificialNoiseStdDev = rf.check("artificialNoiseStdDev", Value(3.0)).asDouble();
    // artifNoiseStdDev = initialArtificialNoiseStdDev; 
    lowerBoundNoise = rf.check("lowerBound", Value(0.04)).asDouble();//
    upperBoundNoise = rf.check("upperBound", Value(3.5)).asDouble();//
    increasedMultiplier = rf.check("increaseMultiplier", Value(1.15)).asDouble();//
    decreasedMultiplier = rf.check("decreaseMultiplier", Value(0.85)).asDouble();//
    minimumLikelihood = rf.check("minimumLikelihood", Value(0.55)).asDouble();//

    KDEStdDev =  rf.check("KDEStdDev", Value(1.0)).asDouble();//
    //Open RPCServer
    handlerPortName = "/" + moduleName + "/rpc:i";
    handlerPort.open(handlerPortName.c_str());
    attach(handlerPort);
    
    /*** Open Input Ports ***/
      /*** From the Robot ***/
    //Right Camera
    imageInputPortRName = "/" + moduleName + "/rightCam:i";
    imageInputPortR.open(imageInputPortRName.c_str());
    //Left Camera
    imageInputPortLName = "/" + moduleName + "/leftCam:i";
    imageInputPortL.open(imageInputPortLName.c_str());
    //Arm
    armPortName = "/" + moduleName + "/" + arm +"Arm:i";
    armPort.open(armPortName.c_str());
    //head
    headPortName = "/" + moduleName + "/head:i";
    headPort.open(headPortName.c_str());
    // Likelihood
    likelihoodPortName = "/" + moduleName + "/likelihood:i";
    likelihoodPort.open(likelihoodPortName.c_str());
    /*** Open Output Ports ***/
    // To InternalModel 

    //Distance Transform concatenation
    LRimageOutputPortName = "/" + moduleName + "/LRimage:o";
    LRimageOutputPort.open(LRimageOutputPortName.c_str());
    //head joints
    headOutPortName = "/" + moduleName + "/head:o";
    headOutPort.open(headOutPortName.c_str()); // output the head encoders
    //arm joints
    particlesOutPortName = "/" + moduleName + "/particles:o";
	particlesOutPort.open(particlesOutPortName.c_str()); // output the arm encoders + offsets to estimate
    offsetsPortName = "/" + moduleName + "/bestOffsets:o";
    offsetsPort.open(offsetsPortName.c_str());
    // Initializing variables
    closing = false;
    stopped = true;
    paused  = false;
	encodersArm.resize(16); // 16 DoF
    encodersHead.resize(6); // 6 DoF
    initializeSMCVariables();
    initSMC();
    

}
/******************************************************************************************/
bool handPoseEstimationModule::initializeSMCVariables()
{
    nParticles=200; // HardCoded since the Internal Model should be recompiled to support a different amount of particles

    //allocate memory for the particles;
	particles=cvCreateMat(8,nParticles,CV_32FC1); // 0-6 Betas 7- likelihood
	//fill the memory with zeros
	cvSetZero(particles);
	
	//define ways of accessing the particles:
	// Beta1
	particles1 = cvCreateMatHeader( 1,nParticles, CV_32FC1);
	cvInitMatHeader( particles1, 1, nParticles, CV_32FC1, particles->data.ptr, particles->step );
	// Beta2
	particles2 = cvCreateMatHeader( 1,nParticles, CV_32FC1);
	cvInitMatHeader( particles2, 1, nParticles, CV_32FC1, particles->data.ptr + particles->step*1, particles->step );
	// Beta3
	particles3 = cvCreateMatHeader( 1,nParticles, CV_32FC1);
	cvInitMatHeader( particles3, 1, nParticles, CV_32FC1, particles->data.ptr + particles->step*2, particles->step );
	// Beta4
	particles4 = cvCreateMatHeader( 1,nParticles, CV_32FC1);
	cvInitMatHeader( particles4, 1, nParticles, CV_32FC1, particles->data.ptr + particles->step*3, particles->step );
	// Beta5
	particles5 = cvCreateMatHeader( 1,nParticles, CV_32FC1);
	cvInitMatHeader( particles5, 1, nParticles, CV_32FC1, particles->data.ptr + particles->step*4, particles->step );
	// Beta6
	particles6 = cvCreateMatHeader( 1,nParticles, CV_32FC1);
	cvInitMatHeader( particles6, 1, nParticles, CV_32FC1, particles->data.ptr + particles->step*5, particles->step );
	// Beta7
	particles7 = cvCreateMatHeader( 1,nParticles, CV_32FC1);
	cvInitMatHeader( particles7, 1, nParticles, CV_32FC1, particles->data.ptr + particles->step*6, particles->step );
	// likelihood
	particles8 = cvCreateMatHeader( 1,nParticles, CV_32FC1);
	cvInitMatHeader( particles8, 1, nParticles, CV_32FC1, particles->data.ptr + particles->step*7, particles->step );

	//theta1-theta7
	particles1to7 = cvCreateMatHeader( 7,nParticles, CV_32FC1);
	cvInitMatHeader( particles1to7, 7, nParticles, CV_32FC1, particles->data.ptr, particles->step );


    newParticles=cvCreateMat(8,nParticles,CV_32FC1);
    newParticles1to7 = cvCreateMatHeader( 7,nParticles, CV_32FC1);
    cvInitMatHeader( newParticles1to7, 7, nParticles, CV_32FC1, newParticles->data.ptr, newParticles->step );
    // Resampling stuff
    cumWeight =cvCreateMat(1,nParticles+1,CV_32FC1);
    noise=cvCreateMat(7,nParticles,CV_32FC1);
    cvSetZero(noise);   
    return true;
}
/******************************************************************************************/
bool handPoseEstimationModule::initSMC()
{
    // Generate random particles

    srand((unsigned int)time(0)); //make sure random numbers are really random.
    rngState = cvRNG(rand());

    cvRandArr( &rngState, particles1, CV_RAND_NORMAL, cvScalar(initialMean), cvScalar(initialStdDev));
    //initialize Theta2
    cvRandArr( &rngState, particles2, CV_RAND_NORMAL, cvScalar(initialMean), cvScalar(initialStdDev));
    //initialize Theta3
	cvRandArr( &rngState, particles3, CV_RAND_NORMAL, cvScalar(initialMean), cvScalar(initialStdDev));
    //initialize Theta4
	cvRandArr( &rngState, particles4, CV_RAND_NORMAL, cvScalar(initialMean), cvScalar(initialStdDev));
	//initialize Theta5
	cvRandArr( &rngState, particles5, CV_RAND_NORMAL, cvScalar(initialMean), cvScalar(initialStdDev));
	//initialize Theta6
	cvRandArr( &rngState, particles6, CV_RAND_NORMAL, cvScalar(initialMean), cvScalar(initialStdDev));
	//initialize Theta7
	cvRandArr( &rngState, particles7, CV_RAND_NORMAL, cvScalar(initialMean), cvScalar(initialStdDev));

    // Artificial Noise Initialization
    artifNoiseStdDev = initialArtificialNoiseStdDev;
    return true;
}
/******************************************************************************************/
bool handPoseEstimationModule::runSMCIteration()
{
    // tmp variables
    double maxLikelihood=0.0;
    double sumLikelihood=0.0;
    double likelihood=0.0;

    mergeAndFlipImages();

    // prepare containers to send data through YARP
    ImageOf<PixelBgr> &yarpReturnImage = LRimageOutputPort.prepare();
    Bottle &outputParticles = particlesOutPort.prepare();
    Bottle &outputHead= headOutPort.prepare();
    // prepare concatenated Image
    IplImage cvImageFinal = concatenatedImage;
    cvCopy( &cvImageFinal, static_cast<IplImage*>(yarpReturnImage.getIplImage()) );

    // Fill Bottle with offsets+encoders to generate
    for(int index=0;index< nParticles;index++) {
        // Arm + offsets
        for (unsigned int joint=0;joint<7;joint++) {
            outputParticles.addDouble(encodersArm[joint]+cvmGet(particles,joint,index));
        }
        // Fingers
        for(unsigned int joint=7;joint<16;joint++) {
            outputParticles.addDouble(encodersArm[joint]);
        }
    }
    outputParticles.addInt(nParticles); // n_particles

    // Fill Bottle with head encoders
    for(int k=0;k<6;k++){
        outputHead.addDouble(encodersHead[k]);
    }

    //Send data to InternalModel
    particlesOutPort.write();
    headOutPort.write();  
    LRimageOutputPort.write();

    // Waitint for results
    yInfo("Waiting for Hypotheses generation and evaluation");
	Bottle *receivedLikelihood = likelihoodPort.read();
	yInfo(" DONE");
    
    // Save the likelihood of each particle
	for(int index=0;index<nParticles;index++) {
		likelihood = receivedLikelihood->pop().asDouble();
		cvmSet(particles,7,index,likelihood);
		sumLikelihood+=likelihood;
        if(likelihood>maxLikelihood) {
			maxLikelihood=likelihood;
		}
	}
    kernelDensityEstimation();

    // Send Best Particle
    Bottle &bestOffset = offsetsPort.prepare();
	bestOffset.clear();
	yInfo("Best likelihood: %f", (float) cvmGet(particles,7,maxWeightIndex));

	for(int i=0;i<7;i++) {
        //if(_iter>45) // START sending offsets
		   bestOffset.addDouble(cvmGet(particles,i,maxWeightIndex));
	}
	bestOffset.addDouble(iteration);
	offsetsPort.write();

    // Resampling or not Resampling. That's the Question

    if(maxLikelihood>minimumLikelihood) {
	    systematic_resampling(particles1to7,particles8,newParticles,cumWeight, sumLikelihood);
	    artifNoiseStdDev=artifNoiseStdDev*decreasedMultiplier;
    }

    else { //I can't apply a resampling with all weights equal to 0! 
	    cvCopy(particles,newParticles);
	    artifNoiseStdDev=artifNoiseStdDev*increasedMultiplier;
    }
    if(artifNoiseStdDev > upperBoundNoise)
	    artifNoiseStdDev = upperBoundNoise;

    // Apply artificial Dynamics

   	CvMat* A = cvCreateMat(8,8,CV_32FC1);
	cvSetIdentity(A); //
    cvMatMul(A,newParticles,particles);

    float mean = 0;
    CvMat* noiseSingle;
    noiseSingle = cvCreateMat(1,nParticles,CV_32FC1);
    cvSetZero(noiseSingle);
    if(artifNoiseStdDev < lowerBoundNoise) { // lowerbound of artificial noise
	    artifNoiseStdDev = lowerBoundNoise;
    }
    cvRandArr( &rngState, noise, CV_RAND_NORMAL, cvScalar(mean), cvScalar(artifNoiseStdDev));
    cvAdd(particles1to7,noise,particles1to7);
    return true;
}
/******************************************************************************************/
void handPoseEstimationModule::kernelDensityEstimation()
{
    // Particle i
    double maxWeight = 0.0;
	for(int iParticle=0;iParticle<nParticles;iParticle++) {
		double sum1=0.0;
        // Particle j
		for(int jParticle=0;jParticle<nParticles;jParticle++) {
			double sum2=0.0;
			if( (float) cvmGet(particles,7,jParticle) > 0 ) {
                // Beta 0..to..6
				for(int joint=0;joint<7;joint++) {
					// || pi-pj||^2 / KDEStdDev^2
					sum2+= pow( ((float)cvmGet(particles,joint,jParticle)-(float)cvmGet(particles,joint,iParticle)) ,2)/pow(KDEStdDev,2); //Multivariate normal distribution
				}
				sum1 += std::exp(-sum2/( 2) )*cvmGet(particles,7,jParticle);
			}
		}
		sum1 = sum1/(nParticles*sqrt(pow(2*M_PI,1)*pow(KDEStdDev,7)));
		double weight = 500*sum1 + cvmGet(particles,7,iParticle);
		if(weight>maxWeight) {
			maxWeightIndex=iParticle;
		}

   }
}
/******************************************************************************************/
bool handPoseEstimationModule::systematic_resampling(CvMat* oldParticlesState, CvMat* oldParticlesWeights, CvMat* newParticlesState, CvMat* cumWeight, float sum2)
{																								

    double u; //random number [0,1)
    double sum;
    int c1;
    int rIndex;  //index of the randomized array
    int cIndex;  //index of the cumulative weight array. cIndex -1 indicates which particle we think of resampling.
    int npIndex; //%new particle index, tells me how many particles have been created so far.

    //%N is the number of particles.
    //[lines, N] = size(oldParticlesWeight);
    //in CPP, _nParticles is the number of particles.


    //%NORMALIZE THE WEIGHTS, so that sum(oldParticles)=1.
    //oldParticlesWeight = oldParticlesWeight / sum(oldParticlesWeight);
    sum=0;

    for(c1=0;c1<nParticles;c1++)
    {
        ((float*)(oldParticlesWeights->data.ptr + oldParticlesWeights->step*0))[c1] = (((float*)(oldParticlesWeights->data.ptr + oldParticlesWeights->step*0))[c1])/(float)sum2;
    }
	    for(c1=0;c1<nParticles;c1++)
    {
        sum+=((float*)(oldParticlesWeights->data.ptr + oldParticlesWeights->step*0))[c1];
    }


    //%GENERATE N RANDOM VALUES
    //u = rand(1)/N; %random value [0,1/N)
    u=1/(double)nParticles*((double)rand()/(double)RAND_MAX);

    //%the randomized values are going to be u, u+1/N, u+2/N, etc.
    //%instead of accessing this vector, the elements are computed on the fly:
    //%randomVector(a)= (a-1)/N+u.

    ((float*)(cumWeight->data.ptr))[0]=0.0;
    for(c1=0;c1<nParticles;c1++)
    {

        ((float*)(cumWeight->data.ptr))[c1+1]=((float*)(cumWeight->data.ptr))[c1]+((float*)(oldParticlesWeights->data.ptr + oldParticlesWeights->step*0))[c1];

    }

    if(((float*)(cumWeight->data.ptr))[nParticles]!=1)
    {
        ((float*)(cumWeight->data.ptr))[nParticles]=1;
        if( ((float*)(cumWeight->data.ptr))[nParticles]!=1)
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

    while(npIndex < nParticles)
    {

        if(((float*)(cumWeight->data.ptr))[cIndex]>=(double)rIndex/(double)nParticles+u) 
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
/******************************************************************************************/
void handPoseEstimationModule::mergeAndFlipImages(){// Merge Right and Left cameras

    Size sz1 = imageProcL.size();
    Size sz2 = imageProcR.size();
    Mat tmp(sz1.height, sz1.width+sz2.width, CV_8UC3);
    Mat left(tmp, Rect(0, 0, sz1.width, sz1.height));
    imageProcL.copyTo(left);
    Mat right(tmp, Rect(sz1.width, 0, sz2.width, sz2.height));
    imageProcR.copyTo(right);

    cv::flip(tmp,concatenatedImage,0);
}
/******************************************************************************************/
bool handPoseEstimationModule::updateModule()
{
   
    if(imageInputPortR.getInputCount()<=0 && imageInputPortL.getInputCount()<=0 && armPort.getInputCount()<=0 && headPort.getInputCount()<=0) {
        yInfo(" Waiting for external connections...");
        Time::delay(0.5);
        return !closing;
    }
    if(stopped)
    {
        yInfo("Module Stopped");
        return !closing;
    }    
    else if(paused)
    {
        yInfo("Module paused");
        return !closing;
    }
    Time time;
    double timing2=time.now();
    yInfo("Iteration: %d", iteration);
    iteration++;
    
    // ReadImages
    IplImage *iR = static_cast<IplImage*> (imageInputPortR.read(false)->getIplImage());  // read an image R
    IplImage *iL = static_cast<IplImage*> (imageInputPortL.read(false)->getIplImage()); // read an image L

    if (iR==NULL || iL ==NULL) { // empty images
        return !closing;    
    }
    // Read Encoders
    readArmJoints();
    readHeadJoints();
	
    imageR=cvarrToMat(iR);
    imageL=cvarrToMat(iL);
    imageProcR = processImages(imageR);
    imageProcL = processImages(imageL);

    //yInfo() << encodersArm.toString().c_str();
    runSMCIteration();
    
    return !closing;
}
/******************************************************************************************/
Mat handPoseEstimationModule::processImages(Mat inputImage)
{
    Mat edges,dtImage;
    cvtColor(inputImage,edges,CV_RGB2GRAY);
    //Blur Image
    blur( edges, edges, Size(3,3) );
    Canny(edges,edges,65,3*65,3);
    threshold(edges,edges,100,255,THRESH_BINARY_INV);
    distanceTransform(edges,dtImage,CV_DIST_L2,CV_DIST_MASK_5);
    return dtImage;
}
/******************************************************************************************/
bool handPoseEstimationModule::readArmJoints()
{
    // read Arm joint angles
    Bottle *receive = armPort.read();

    string s1 = receive->toString();
    char *str = new char[s1.size()+1];
    strcpy(str, s1.c_str());
    char * pch;
    pch = strtok ( (char*) str," ");
    int i=0;
    while (pch != NULL) {
        //printf ("%s\n",pch);
        encodersArm[i]=atof(pch);
        pch = strtok (NULL, " ");
        i++;
    }
}
/******************************************************************************************/
bool handPoseEstimationModule::readHeadJoints()
{
    //Read Head Joint Angles    
    Bottle *receive = headPort.read();
    string s2 = receive->toString();
    char *str = new char[s2.size()+1];
    strcpy(str, s2.c_str());
    char * pch;
    pch = strtok ( (char*) str," ");
    int i=0;
    while (pch != NULL) {
        //printf ("%s\n",pch);
        encodersHead[i]=atof(pch);
        pch = strtok (NULL, " ");
        i++;
    }
}
/******************************************************************************************/
double handPoseEstimationModule::getPeriod()
{
    return 1.0;
}
/******************************************************************************************/
bool handPoseEstimationModule::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}
/******************************************************************************************/
bool handPoseEstimationModule::interruptModule()
{
    //Interrupt Ports
    handlerPort.interrupt();
    armPort.interrupt();
    headPort.interrupt();
    imageInputPortR.interrupt();
    imageInputPortL.interrupt();
    return true;
}
/******************************************************************************************/
bool handPoseEstimationModule::close()
{
    yInfo("starting shutdown procedure");
    //Close Ports
    yInfo("closing ports");
    handlerPort.close();
    armPort.close();
    headPort.close();
    imageInputPortR.close();
    imageInputPortL.close();

    return true;
}

/******************************************************************************************/
/**********************                IDL functions                 **********************/
/******************************************************************************************/

bool handPoseEstimationModule::start()
{
    iteration=0;
    initSMC(); // Generated new particles
    stopped = false;
    yInfo("start command received");
    return true;
}
/******************************************************************************************/
bool handPoseEstimationModule::stop()
{
    stopped = true;
    yInfo("stop command received");
    return true;

}
/******************************************************************************************/
bool handPoseEstimationModule::pause()
{
    paused = true;
    yInfo("pause command received");
    return true;
}
/******************************************************************************************/
bool handPoseEstimationModule::resume()
{
    paused = false;
    yInfo("resume command received");
    return true;
}
/******************************************************************************************/
yarp::os::Bottle handPoseEstimationModule::lastOffsets()
{
    yInfo("lastOffsets command received");
}
/******************************************************************************************/
bool handPoseEstimationModule::quit()
{
    yInfo("quit command received");
    closing = true;
    return true;
}
/******************************************************************************************/
