// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 *
 */

/**
* Class handPoseEstimationModule
* 
*
*
*
*
*
*
*
*
* \note <b>If you're going to use this module for your work, please
*       quote it within any resulting publication</b>
*
*
*
*
* \author Pedro Vicente
*/
#ifndef _HPESTIMATION_
#define _HPESTIMATION_

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

//#include <opencv/cv.h>
#include <opencv2/core/core.hpp>  
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp>

#include "handPoseEstimation_IDL.h"
class handPoseEstimationModule : public yarp::os::RFModule, public handPoseEstimation_IDL
{
    private:
        // module parameters
        std::string moduleName;
        std::string arm;
        //port Names
        std::string handlerPortName;
        std::string imageInputPortRName;
        std::string imageInputPortLName;
        std::string armPortName;
        std::string headPortName;
        std::string likelihoodPortName;
        std::string LRimageOutputPortName;
        std::string	headOutPortName;
	    std::string	particlesOutPortName;
        std::string offsetsPortName;
        bool closing;

        // Variables
        int iteration;
        double initialMean,initialStdDev;
        double initialArtificialNoiseStdDev, artifNoiseStdDev;
        double lowerBoundNoise, upperBoundNoise, increasedMultiplier, decreasedMultiplier;
        double minimumLikelihood;
        double KDEStdDev;
        int maxWeightIndex;
        bool stopped, paused;
        yarp::sig::Vector encodersArm, encodersHead;
        //yarp::sig::ImageOf<yarp::sig::PixelRgb> *imageR, *imageL;
        cv::Mat imageR, imageL;
        cv::Mat imageProcR, imageProcL;
        cv::Mat concatenatedImage;
        // SMC (Sequential Monte Carlo) related variables
        int nParticles;
        CvMat* particles;
        CvMat* newParticles;

        CvMat* particles1; // Beta 1
        CvMat* particles2; // Beta 2
        CvMat* particles3; // Beta 3
        CvMat* particles4; // Beta 4
        CvMat* particles5; // Beta 5
        CvMat* particles6; // Beta 6
        CvMat* particles7; // Beta 7
        CvMat* particles8; // Likelihood
        CvMat* particles1to7; // BETA = [ Beta1, ... , Beta 7]
        CvMat* newParticles1to7;
        CvMat* noise;
        // resampling-related stuff
        CvMat* cumWeight;
        CvRNG rngState;
        /*** Input Ports ***/
        yarp::os::RpcServer handlerPort;
    	yarp::os::BufferedPort< yarp::sig::ImageOf<yarp::sig::PixelRgb> > imageInputPortR;  // Right Image
    	yarp::os::BufferedPort< yarp::sig::ImageOf<yarp::sig::PixelRgb> > imageInputPortL;  // Left Image
        yarp::os::BufferedPort< yarp::os::Bottle >	armPort;
        yarp::os::BufferedPort< yarp::os::Bottle >	headPort;
        yarp::os::BufferedPort< yarp::os::Bottle >	likelihoodPort;
        /*** Output Ports ***/
        yarp::os::BufferedPort< yarp::sig::ImageOf< yarp::sig::PixelBgr> > LRimageOutputPort;
        yarp::os::BufferedPort< yarp::os::Bottle >	headOutPort;
	    yarp::os::BufferedPort< yarp::os::Bottle >	particlesOutPort;
        yarp::os::BufferedPort< yarp::os::Bottle >	offsetsPort;

        
    protected:
        cv::Mat processImages(cv::Mat inputImage);
        bool initializeSMCVariables();
        bool initSMC();
        bool runSMCIteration();
        void mergeAndFlipImages();
        void kernelDensityEstimation();
        bool readArmJoints();
        bool readHeadJoints();
        bool systematic_resampling(CvMat* oldParticlesState, CvMat* oldParticlesWeights, CvMat* newParticlesState, CvMat* cumWeight,float sum2);
    public:
        virtual bool configure(yarp::os::ResourceFinder &rf);
        virtual bool interruptModule();
        virtual bool close();
        virtual bool updateModule();
        virtual double getPeriod();
        
        // IDL functions

        bool attach(yarp::os::RpcServer &source);
        bool start();
        bool stop();
        bool pause();
        bool resume();
        yarp::os::Bottle lastOffsets();
        bool quit();
};

#endif
