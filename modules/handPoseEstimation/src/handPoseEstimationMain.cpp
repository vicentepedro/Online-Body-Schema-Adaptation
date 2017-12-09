// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 *
 */


#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Log.h>

#include "handPoseEstimationModule.h"

using namespace yarp::os;
using namespace std;

int main(int argc, char *argv[]) {

    Network yarp;

    if(! yarp.checkNetwork() ) {
        yError("yarp server does not seem available");
        return 1; // EXIT_FAILURE
    }

    handPoseEstimationModule module;
    ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultConfigFile("hpe.ini"); // overridden by --from
    rf.configure(argc, argv);

    if(rf.check("help"))
    {
        yInfo("Available options:");
        yInfo("--name prefix (default: hpe)");
        yInfo("--arm arm to estimate (default: right)");
        yInfo("--initialMean spread the particles (default:0.0)");
        yInfo("--initialStdDev spread the particles (default:3.5)");
        yInfo("--artificialNoiseStdDev Initial value after each iteration(default:3.0)");
        yInfo("--lowerBound minimum value of Artificial Noise (default=0.04)");
        yInfo("--upperBound maximum value of Artificial Noise (default=3.5)"); 
        yInfo("--minimumLikelihood to perform resampling (default=0.55)");
        yInfo("--increaseMultiplier for the Artificial Noise (default=1.15)");
        yInfo("--decreaseMultiplier for the Artificial Noise (default=0.85)");
        yInfo("--KDEStdDev StdDev of each kernel (default=1.0)");
        yInfo("--minIteration number of iterations before output estimated Offsets (default=35)");
        return 0; // EXIT_SUCCESS
    }

    module.runModule(rf);

    return 0;
}
