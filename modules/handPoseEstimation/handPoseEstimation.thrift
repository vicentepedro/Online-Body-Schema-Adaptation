# Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
#                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
# Authors: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>
# CopyPolicy: Released under the terms of the GNU GPL v3.0.
#
# handPoseEstimation.thrift

struct Bottle { }
(
yarp.name = "yarp::os::Bottle"
yarp.includefile="yarp/os/Bottle.h"
)

/**
* handPoseEstimation_IDL
*
* IDL Interface to \ref handPoseEstimation-module services.
* This is a fake thrift service, just to show how to structure your repo
* and how to document your code.
*/
service handPoseEstimation_IDL
{
  /**
  * Start (re-start) the hand pose estimation.
  * The filter generates particles and update the particle distribution accordingly
  * @return true/false on success/failure
  */
  bool start();

  /**
  * Stop the hand pose estimation.
  * The filter stop the generation of particles and waits for a start command
  * @return true/false on success/failure
  */
  bool stop();


  /**
  * Pause the hand pose estimation.
  * The filter stop the generation of particles and waits for a resume command
  * @return true/false on success/failure
  */
  bool pause();


  /**
  * Resume the hand pose estimation.
  * The filter resumes the generation of particles after a pause command
  *
  * @return true/false on success/failure
  */
  bool resume();

  /**
  * Ask for the last estimated angular Offsets on the arm (7DoF)
  * 
  * @return Bottle with last angular Offsets
  */
  Bottle lastOffsets();

  /**
  * Quit the module.
  * @return true/false on success/failure
  */
  bool quit();  
}

