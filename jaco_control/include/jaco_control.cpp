#include "jaco_control.h"


Jaco::Jaco(ros::NodeHandle &n, boost::recursive_mutex &mut) : _pn(&n), kinova::KinovaComm(n, mut, true, "j2n6s200") {
  initSubPub();
  goHome();
  openFingers(true);
  startForceControl();
}


Jaco::~Jaco() {
  stopAPI();
  stopForceControl();

  if (isStopped())
    ROS_INFO("Disconnected from control API successfully");
}


void Jaco::callback(const std_msgs::UInt8& msg) {
  _recievedMessage = msg;
  _recievedCommand = _recievedMessage.data;

  if (!isMoving()) {
    publishFeedback();

    switch (_recievedCommand) {
      case 0: goHome(); break;
      case 1: openFingers(!_isFingersOpen); break;
      case 2: eraseAllTrajectories(); break;
      case 3: parseTrajectoryFile("door.txt"); break;
      case 4: parseTrajectoryFile("trajectory.txt"); break;
      default: ROS_ERROR("Command not recognized");
    }
  }
  else
    ROS_WARN("Command %d received while robot is moving", _recievedCommand);
}


void Jaco::initSubPub() {
  ROS_INFO("Connected to control API successfully");
  _sub = _pn->subscribe("/jaco/commander", 1, &Jaco::callback, this);
  _feedbackPublisher = _pn->advertise<std_msgs::UInt8>("/jaco/feedback", 1, true);
}


void Jaco::publishFeedback() {
  _feedbackPublisher.publish(_recievedMessage);
}


void Jaco::goHome() {
  waitWhileMoving();

  if (!isHomed()) {
    homeArm();
    waitWhileMoving();
  }
}


bool Jaco::isFloat() {
  try {
    std::stof(_line);
    return true;
  }
  catch (std::exception& e) {
    return false;
  }
}


void Jaco::parseTrajectoryFile(const std::string& fileName) {
  _inputFile.open(_HOME_DIR_PATH + "/.jaco/" + fileName);

  while (std::getline(_inputFile, _line)) {
    if (isFloat())
      goToPosition();
    else if (_line == "OPEN")
      openFingers(true);
    else if (_line == "CLOSE")
      openFingers(false);
    else if (_line == "HOME")
      goHome();
    else if (isSleepLine() == "SLEEP")
      ros::Duration(_sleepTimeInSecond).sleep();
  }
  _inputFile.close();
}


std::string Jaco::isSleepLine() {
  std::string word;
  std::istringstream ss(_line);
  ss >> word;
  ss >> _sleepTimeInSecond;
  return word;
}


void Jaco::goToPosition() {
  readPoseFromFile();
  updateMovementType();
  setCartesianPosition(_targetPosition, 0, _isMovementContinous);
}


void Jaco::readPoseFromFile() {
  std::istringstream ss(_line);
  ss >> _targetPosition.X;
  ss >> _targetPosition.Y;
  ss >> _targetPosition.Z;
  ss >> _targetPosition.ThetaX;
  ss >> _targetPosition.ThetaY;
  ss >> _targetPosition.ThetaZ;
  ss >> _movementType;
}


void Jaco::updateMovementType() {
  if (_movementType == "CONT")
    _isMovementContinous = true;
  else if (_movementType == "NON-CONT")
    _isMovementContinous = false;
}


void Jaco::openFingers(bool isOpen) {
  waitWhileMoving();
  eraseAllTrajectories();
  _isFingersOpen = isOpen;
  _targetFingers.Finger1 = 6400 * (!isOpen); _targetFingers.Finger2 = 6400 * (!isOpen);
  ros::Duration(0.3f).sleep();
  setFingerPositions(_targetFingers, 0, true);
  ros::Duration(1.2f).sleep();
}


void Jaco::waitWhileMoving() {
  ros::Duration(0.3f).sleep();
  while (isMoving())
    continue;
  ros::Duration(0.2f).sleep();
}


bool Jaco::isMoving() {
  kinova::KinovaAngles vel;
  vel.InitStruct();
  getJointVelocities(vel);

  float velocity = abs(vel.Actuator1) + abs(vel.Actuator2) + abs(vel.Actuator3)
                + abs(vel.Actuator4) + abs(vel.Actuator5) + abs(vel.Actuator6);

  return true ? (velocity > _MOVEMENT_TOLERANCE) : false;
}
