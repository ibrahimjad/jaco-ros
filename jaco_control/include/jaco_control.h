#ifndef JACO_CONTROL_H
#define JACO_CONTROL_H

#include <boost/thread/recursive_mutex.hpp>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "kinova_driver/kinova_comm.h"


class Jaco : private kinova::KinovaComm {
public:
  Jaco(ros::NodeHandle &n, boost::recursive_mutex &mutex);
  ~Jaco();

private:
  void initSubPub();
  void publishFeedback();
  void callback(const std_msgs::UInt8& msg);

  void goHome();
  void goToPosition();
  void openFingers(bool isOpen);
  void updateMovementType();

  void parseTrajectoryFile(const std::string& fileName);
  void readPoseFromFile();
  std::string isSleepLine();

  void waitWhileMoving();
  bool isMoving();

  bool isFloat();

private:
  ros::NodeHandle *_pn;
  ros::Subscriber _sub;
  ros::Publisher _feedbackPublisher;

  unsigned char _recievedCommand;
  std_msgs::UInt8 _recievedMessage;

  kinova::FingerAngles _targetFingers;
  kinova::KinovaPose _targetPosition;

  std::ifstream _inputFile;
  std::string _line;
  std::string _movementType;
  bool _isFingersOpen = true;
  bool _isMovementContinous = false;
  const std::string _HOME_DIR_PATH = std::getenv("HOME");

  double _sleepTimeInSecond = 0;
  const float _MOVEMENT_TOLERANCE = 0.02f;
};

#endif
