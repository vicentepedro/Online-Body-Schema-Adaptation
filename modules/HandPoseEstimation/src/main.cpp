#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>

#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <opencv/cvaux.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Vector.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>

#include <pf3dTracker.hpp>

#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp>        
#include <opencv2/highgui/highgui.hpp>

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;
using namespace yarp::dev;
using namespace yarp::math;

using namespace iCub::ctrl;
using namespace iCub::iKin;

using namespace cv;

 
int main() {

    Network yarp;
    if (!yarp.checkNetwork())
    {
		yError("Yarp Server is running?");
		return -1;
	}
	iCubArm Arm("right");
	iKinChain *chainArm,*chainArm2;
	chainArm=Arm.asChain();
	chainArm2 = Arm.asChain();
	iCubEye Olho("left");
	iKinChain *chainEye;
	chainEye=Olho.asChain();
	
	yarp::sig::Vector commandArm;
	commandArm.resize(16);

    PF3DTracker filtro;

   // Getting Data cycle
	BufferedPort<ImageOf<PixelRgb> > imagePort;  // make a port for reading images
	BufferedPort<ImageOf<PixelRgb> > imagePortL;  // make a port for reading images
	BufferedPort<ImageOf<PixelBgr> > imagePort2;
	BufferedPort<ImageOf<PixelBgr> > imagePort2L;
	BufferedPort<ImageOf<PixelBgr> > imagePort_RGB;
	BufferedPort<ImageOf<PixelBgr> > imagePortL_RGB;
	
	BufferedPort<ImageOf<PixelMono> > edgePort;
	BufferedPort<ImageOf<PixelBgr> > DTPort;
	// Arm and Head
	BufferedPort< Bottle >	RightArmPort;  // make a port for reading arm position
    BufferedPort< Bottle >	HeadPort;
	//BufferedPort< Bottle >	fingertips;

	BufferedPort <Bottle> signalPort;

	signalPort.open("/HPE_R/command:i");
	imagePort.open("/HPE_R/RightEyeimage:i");  // give the port a name
	imagePortL.open("/HPE_R/LeftEyeimage:i");
	imagePort2.open("/HPE_R/RightEyeimage:o");  // give the port a name
	imagePort2L.open("/HPE_R/LeftEyeimage:o");

	edgePort.open("/HPE_R/edgePort");
	DTPort.open("/HPE_R/DTport");
	// open arm and head ports
	RightArmPort.open("/HPE_R/rightArm:i");  // give the port a name
	HeadPort.open("/HPE_R/head:i");
	Time::delay(0.5); // Ports must be open before connect
	filtro.init();
	Time::delay(0.5);
	yInfo(" Waiting for images input...");
	while(imagePort.getInputCount()<=0 && imagePortL.getInputCount()<=0) {
		Time::delay(0.1);
	}

	// Number of images get
	char frase[20];

	// Initialization

	ImageOf<PixelRgb> *image, *imageL;

	IplImage *cvImage,*cvImageGRAY,*cvImageTmp, *cvImageL;
	yarp::sig::Vector ImagePoint;
	yarp::sig::Vector PosInicial, qarm,qarm2, xf,PosHead;
	Matrix MArm,MArm2;
	Bottle *receive, *receive_head; 
	yarp::sig::Vector PointFrameEye;
	// Right Eye
	Mat ImageMat,ImageMat_Real, ImageMat_Real_exp,ImageMat_Real_gray,ImageMat_init,ImageMat_gray,ImageMat_result1,ImageMat_result2, ImageMat_Real_seg,ImageMat_Real_seg2,imgHSV;
	// Left Eye
	Mat ImageMat_RealL,ImageMat_Real_grayL, imgHSVL, ImageMat_Real_segL, ImageMat_Real_segL2,ImageMat_initL;

	ImageOf<PixelBgr> & yarpReturnImage = imagePort2.prepare();
	ImageOf<PixelBgr> & yarpReturnImageL = imagePort2L.prepare();

	ImageOf<PixelMono> & yarpEdge = edgePort.prepare();
	ImageOf<PixelBgr> & yarpDT = DTPort.prepare();
	
	bool close = false;
	yInfo(" Waiting for input/start Signal...");
	/****** wait for motion start  ******/
	Bottle *input = signalPort.read();
	/************************************/
	string cmdS= input->toString();
	if(cmdS.compare("STOP")==0 || cmdS.compare("stop")==0 || cmdS.compare("Stop")==0)
	{
		yInfo("Stopping module");
		close = true;
	}
	else if(cmdS.compare("START")==0 || cmdS.compare("start")==0 || cmdS.compare("Start")==0)
	{
		yInfo("Module will start");
	}
	image = imagePort.read();
	imageL = imagePortL.read();
	receive = RightArmPort.read();
	receive_head = HeadPort.read();
	
	/*** Real Robot input***/
	
	{
		string s1 = receive->toString();
		cout << receive->toString().c_str() << endl;
		char *str = new char[s1.size()+1];
	   strcpy(str, s1.c_str());
		char * pch;
		pch = strtok ( (char*) str," ");
		int i=0;
		PosInicial.resize(16);
		while (pch != NULL) {
			//printf ("%s\n",pch);
			PosInicial[i]=atof(pch);
			cout << PosInicial[i] << endl;
			pch = strtok (NULL, " ");
			i++;
		}
	}
	// HEAD
	
	{
		string s2 = receive_head->toString();
		cout << receive_head->toString().c_str() << endl;
		char *str = new char[s2.size()+1];
		strcpy(str, s2.c_str());
		char * pch;
		pch = strtok ( (char*) str," ");
		int i=0;
		PosHead.resize(6);
		while (pch != NULL) {
			//printf ("%s\n",pch);
			PosHead[i]=atof(pch);
			pch = strtok (NULL, " ");
			i++;
		}
	}
	cout << PosInicial.toString().c_str() << endl;
	cvImageGRAY  = cvCreateImage(cvSize(image->width(), image->height()), IPL_DEPTH_8U, 1 );
	cvImage  = cvCreateImage(cvSize(image->width(), image->height()), IPL_DEPTH_8U, 3 );
	IplImage *cvImageEdge  = cvCreateImage(cvSize(image->width(), image->height()), IPL_DEPTH_8U, 3 );
	IplImage * cvImageDT  = cvCreateImage(cvSize(image->width(), image->height()), IPL_DEPTH_16U, 3 );
	cvImageL = cvCreateImage(cvSize(imageL->width(), imageL->height()), IPL_DEPTH_8U, 3 );

	yarp::sig::Vector zero(4); zero(0)=0; zero(1)=0; zero(2)=0; zero(3)=1;

	Time tempo;
	double timing1, timing2,timing3;
	PosInicial=commandArm;


	ImageMat_Real=cvImage;
	ImageMat_RealL=cvImageL;

	cvtColor( ImageMat_Real, ImageMat_Real_gray, CV_RGB2GRAY );
	cvtColor( ImageMat_RealL, ImageMat_Real_grayL, CV_RGB2GRAY );
	threshold( ImageMat_Real_gray, ImageMat_Real_gray, 3, 255, 1 );  
	threshold( ImageMat_Real_grayL, ImageMat_Real_grayL, 3, 255, 1 );  

	
	filtro._encodersHead = PosHead; // change afterwords

	int iter=0;
	//Time::delay(1.5);

	while(!close) {
		//waiting for encoders Arm
	    timing2=tempo.now();
        yInfo("Iteration:%d", iter);
		iter++;
		image = imagePort.read();  // read an image R
		imageL = imagePortL.read(); // read an image L

		if (image!=NULL && imageL !=NULL) { // check we actually got something
			
			// read joint angles
			receive = RightArmPort.read();
			PosInicial.resize(16);

			/************REAL**************/
		 
			string s1 = receive->toString();
			cout << receive->toString().c_str() << endl;
			char *str = new char[s1.size()+1];
			strcpy(str, s1.c_str());
			char * pch;
			pch = strtok ( (char*) str," ");
			int i=0;
			PosInicial.resize(16);
			while (pch != NULL) {
				//printf ("%s\n",pch);
				PosInicial[i]=atof(pch);
				pch = strtok (NULL, " ");
				i++;
			}
			/**/
			qarm.resize(chainArm->getDOF());
			for(unsigned int i=0; i<=chainArm->getDOF(); i++) {
				qarm[i]=PosInicial[i]*CTRL_DEG2RAD;
				cout << "PosArm: " << PosInicial[i] << endl; 
			}
			chainArm->setAng(qarm);
			xf=chainArm->EndEffPosition();
			MArm = chainArm->getH();

			qarm2.resize(chainArm2->getDOF());
			for(unsigned int i=0; i<=chainArm2->getDOF(); i++) {
				qarm2[i]=PosInicial[i]*CTRL_DEG2RAD;
			}
			chainArm2->setAng(qarm2);
			MArm2 = chainArm2->getH();

			cvCvtColor((IplImage*)imageL->getIplImage(), cvImageL, CV_RGB2BGR);
			cvCvtColor((IplImage*)image->getIplImage(), cvImage , CV_RGB2BGR);

			/******* EDGE BASED ********/
			
			// canny...
			Mat detectedEdges,detectedEdgesL,detectedEdges2,detectedEdges2L, dtImage,dtImageL;
			cvtColor(ImageMat_Real,detectedEdges,CV_RGB2GRAY);
			cvtColor(ImageMat_RealL,detectedEdgesL,CV_RGB2GRAY);
			//int kernel_size = 3;

			/*real...*/
			blur( detectedEdges, detectedEdges, Size(3,3) );
			Canny(detectedEdges,detectedEdges,65,3*65,3); //150,250 - 65,65*3
			blur( detectedEdgesL, detectedEdgesL, Size(3,3) );
			Canny(detectedEdgesL,detectedEdgesL,65,3*65,3); //150,250 - 65,65*3

			//
			threshold(detectedEdges,detectedEdges,100,255,THRESH_BINARY_INV);
			distanceTransform(detectedEdges,dtImage,CV_DIST_L2,CV_DIST_MASK_5);
		
			threshold(detectedEdgesL,detectedEdgesL,100,255,THRESH_BINARY_INV);
			distanceTransform(detectedEdgesL,dtImageL,CV_DIST_L2,CV_DIST_MASK_5); // euclidean distance

			Mat dtImage2,dtImage2L,dtImage2_8,dtImage2L_8;
			cvtColor(dtImage,dtImage2,CV_GRAY2BGR);
			dtImage2.convertTo(dtImage2_8,CV_8UC3);
			cvtColor(dtImageL,dtImage2L,CV_GRAY2BGR);
			dtImage2L.convertTo(dtImage2L_8,CV_8UC3);

			cvImageTmp=cvCloneImage(&(IplImage)detectedEdgesL);
			cvReleaseImage(&cvImageEdge);
			cvImageEdge=cvImageTmp;
			cvImageTmp=NULL;
			cvImageTmp=cvCloneImage(&(IplImage)dtImage2L_8);
			cvReleaseImage(&cvImageDT);
			cvImageDT=cvImageTmp;
			cvImageTmp=NULL;

			edgePort.prepare();
			DTPort.prepare();
			yarpEdge.wrapIplImage(cvImageEdge);
			yarpDT.wrapIplImage(cvImageDT);
			edgePort.write(); 
			DTPort.write(); 
			Point fingerP;

			int max_index = filtro.run2(PosInicial, dtImage2_8, dtImage2L_8);

			Bottle* fingers = filtro._fingers;
			if(fingers->size() ==0)
				cout << "empty fingers" << endl;
			cout << ",max_index" << max_index << endl;
			int init = (max_index)*12;
			
			ImageMat=cvImage;
			ImageMat_RealL=cvImageL;
			/******* END EDGE BASED ********/

			timing3=tempo.now();
			yInfo( "time: %f", timing3 - timing2);

		}
		imageL=NULL;
		image=NULL;
		//return 0;
		Bottle *input = signalPort.read(false);
	    /************************************/
	    if(input!=NULL)
		{
			string cmdS= input->toString();
			if(cmdS.compare("STOP")==0 || cmdS.compare("stop")==0 || cmdS.compare("Stop")==0)
			{
				yInfo("Stopping module");
				close = true;
    		}
		}
	}
	//
	yDebug("closing ports");
	signalPort.close();
	imagePort.close();  
	imagePortL.close();
	imagePort2.close();  
	imagePort2L.close();
	edgePort.close();
	DTPort.close();
	RightArmPort.close();
	HeadPort.close();
	// Close Robot Device
	return 0;
}

