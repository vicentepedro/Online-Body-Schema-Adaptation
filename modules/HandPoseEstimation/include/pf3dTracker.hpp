/**
* Copyright: (C) 2009 RobotCub Consortium
* Authors: Matteo Taiana
* Adapted by: Pedro Vicente 2015
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef _PF3DTRACK_
#define _PF3DTRACK_

#include <iostream>
#include <string>
#include <sstream>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/os/all.h>
#include <yarp/os/Stamp.h>

#include <yarp/sig/all.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#ifdef _CH_
#pragma package <opencv>
#endif
#ifndef _EiC
#include "opencv/cv.h"
#include "opencv/highgui.h"
#endif

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;

class PF3DTracker : public Module
{
public:
	yarp::sig::Vector _encodersHead;
	Bottle *_fingers;
private:

int _numParticlesReceived;

IPositionControl *_posArm;
IPositionControl *_posHead;
IEncoders *_encArm;
IEncoders *_encHead;
PolyDriver _robotArm;
PolyDriver _robotHead;

yarp::sig::Vector _encodersArm;
//yarp::sig::Vector _encodersHead;
Property _optionsHead;
Property _optionsArm;

//parameters set during initialization.
ConstString _inputVideoPortName;
ConstString _outputVideoPortName;

// Send L&R Image to Unity
BufferedPort<ImageOf<PixelBgr> > _outputVideoPort;

/** Send Encoders to Unity **/
//Arm
BufferedPort< Bottle >	_RightArmPort_out;
//Head
BufferedPort< Bottle >	_HeadPort_out;

// receive likelihood of each particle
BufferedPort< Bottle >	_likelihood_port;
BufferedPort< Bottle >	_fingers_port;
Bottle *_receive_likelihood;
// BufferedPort<ImageOf<PixelBgr> > _outputVideoPort_RGB;
// BufferedPort<ImageOf<PixelBgr> > _outputVideoPortL_RGB;

ConstString _outputDataPortName;
BufferedPort<Bottle> _outputDataPort;
ConstString _inputParticlePortName;
BufferedPort<Bottle> _inputParticlePort;
ConstString _outputParticlePortName;
BufferedPort<Bottle> _outputParticlePort;

float _likelihoodThreshold;

CvRNG rngState; //something needed by the random number generator
bool _doneInitializing;

CvMat* _A;
int _nParticles;
float _accelStDev;

CvMat* _particles1; //this is used during some operations... it is defined here to avoid repeated instantiations
CvMat* _particles2; //this is used during some operations... it is defined here to avoid repeated instantiations
CvMat* _particles3; //this is used during some operations... it is defined here to avoid repeated instantiations
CvMat* _particles4; //this is used during some operations... it is defined here to avoid repeated instantiations
CvMat* _particles5; //this is used during some operations... it is defined here to avoid repeated instantiations
CvMat* _particles6; //this is used during some operations... it is defined here to avoid repeated instantiations
CvMat* _particles7; //this is used during some operations... it is defined here to avoid repeated instantiations
CvMat* _particles8; //this is used during some operations... it is defined here to avoid repeated instantiations
CvMat* _particles1to7;
CvMat* _newParticles1to7;
CvMat* _uv;

//resampling-related stuff
CvMat* _nChildren;
CvMat* _label;
CvMat* _u;
CvMat* _ramp;

//new resampling-related stuff
CvMat* _cumWeight;

//variables
CvMat* _particles;
CvMat* _newParticles;
CvMat* _noise;
CvMat* _noise1; //lines from 0 to 2
CvMat* _noise2; //lines from 3 to 5

Stamp _yarpTimestamp;
ImageOf<PixelRgb> *_yarpImage;
IplImage *_rawImage;
IplImage* _transformedImage;//_yuvBinsImage[image_width][image_height][3];
double _initialTime;
double _finalTime;

int _frameCounter;
int _framesNotTracking; //number of frames for which the likelihood has been under the threshold. after 20 such frames the tracker is restarted.
int downsampler;
float _lastU;
float _lastV;
bool _firstFrame; //when processing the first frame do not write the fps values, as it's wrong.
bool _ini; // If is the initialization, send the head encoders too.
int _iter;

bool systematic_resampling(CvMat* oldParticlesState, CvMat* oldParticlesWeights, CvMat* newParticlesState, CvMat* cumWeight,float sum2);

public:

PF3DTracker(); //constructor
~PF3DTracker(); //destructor

void PF3DTracker::init();
int PF3DTracker::run2(yarp::sig::Vector PosInicial, cv::Mat ImageMat_Real_gray,  cv::Mat ImageMat_Real_grayL);
//virtual bool open(Searchable& config); //member to set the object up.
//virtual bool close();                  //member to close the object.
//virtual bool interruptModule();        //member to close the object.
//virtual bool updateModule();           //member that is repeatedly called by YARP, to give this object a chance to do something.

};

#endif  // _PF3DTRACK_