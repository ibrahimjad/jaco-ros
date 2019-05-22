#include "jaco_teach.h"


Jaco::Jaco(Gtk::Window &window, ros::NodeHandle &n, boost::recursive_mutex &mut)
      : _pWindow(&window), kinova::KinovaComm(n, mut, true, "j2n6s200") {
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


void Jaco::initTeachMode() {
  createInterface();
  showInterface();
}


void Jaco::createInterface() {
  setWindowSettings();
  setBoxSettings();
  setButtonLabels();
  setButtonCallbacks();
  setButtonSize();
}


void Jaco::setWindowSettings() {
  _pWindow->set_default_size(800, 200);
  _pWindow->set_title("Jaco Teach Mode");
  _pWindow->set_border_width(10);
  _pWindow->add(_box);
}


void Jaco::setBoxSettings() {
  _box.pack_start(_save_button);
  _box.pack_start(_fingers_button);
  _box.pack_start(_saveToFile_button);
  _box.pack_start(_replay_button);
}


void Jaco::setButtonLabels() {
  _save_button.set_label("Save Pose");
  _fingers_button.set_label("Open/Close Fingers");
  _saveToFile_button.set_label("Save to File");
  _replay_button.set_label("Replay");
}


void Jaco::setButtonSize() {
  _save_button.set_size_request(200, 200);
  _fingers_button.set_size_request(200, 200);
  _saveToFile_button.set_size_request(200, 200);
  _replay_button.set_size_request(200, 200);
}


void Jaco::setButtonCallbacks() {
  _save_button.signal_clicked().connect(sigc::mem_fun(*this,&Jaco::savePosition));
  _fingers_button.signal_clicked().connect(sigc::mem_fun(*this,&Jaco::actuateFingers));
  _saveToFile_button.signal_clicked().connect(sigc::mem_fun(*this,&Jaco::saveToFile));
  _replay_button.signal_clicked().connect(sigc::mem_fun(*this,&Jaco::parseFile));
}


void Jaco::showInterface() {
  _pWindow->show_all_children();
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


bool Jaco::isSleepLine() {
  std::string word;
  std::istringstream ss(_line);
  ss >> word;
  if (word == "SLEEP") {
    ss >> _sleepTimeInSecond;
    return true;
  }
  return false;
}


void Jaco::parseFile() {
  waitWhileMoving();

  _inputFile.open(_homeDir + "/.jaco/trajectory.txt");

  while (std::getline(_inputFile, _line)) {
    if (isFloat())
      goToPosition();
    else if (_line == "OPEN")
      openFingers(true);
    else if (_line == "CLOSE")
      openFingers(false);
    else if (_line == "HOME")
      goHome();
    else if (isSleepLine())
      ros::Duration(_sleepTimeInSecond).sleep();
  }
  _inputFile.close();
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


void Jaco::savePosition() {
  openFile();
  getCartesianPosition(_currentPosition);
  savePoseToFile();
  saveFingersToFile();
}


void Jaco::savePoseToFile() {
  _outputFile << _currentPosition.X << " ";
  _outputFile << _currentPosition.Y << " ";
  _outputFile << _currentPosition.Z << " ";
  _outputFile << _currentPosition.ThetaX << " ";
  _outputFile << _currentPosition.ThetaY << " ";
  _outputFile << _currentPosition.ThetaZ << " ";
  _outputFile << "NON-CONT\n";
}


void Jaco::saveFingersToFile() {
  if (_oldFingers != _isFingersOpen)
    if (_isFingersOpen == true) {
      _oldFingers = true;
      _outputFile << "OPEN\n";
    }
    else {
      _oldFingers = false;
      _outputFile << "CLOSE\n";
    }
}


void Jaco::openFingers(bool isOpen) {
  waitWhileMoving();
  eraseAllTrajectories();
  _oldFingers = _isFingersOpen;
  _isFingersOpen = isOpen;
  _fingers.Finger1 = 6400 * (!isOpen); _fingers.Finger2 = 6400 * (!isOpen);
  ros::Duration(0.3f).sleep();
  setFingerPositions(_fingers, 0, true);
  ros::Duration(1.2f).sleep();
}


void Jaco::actuateFingers() {
  if (_isFingersOpen)
    openFingers(false);
  else
    openFingers(true);
}


void Jaco::openFile() {
  if (!_outputFile.is_open()) {
    _outputFile.open(_homeDir + "/.jaco/trajectory.txt", std::ios::trunc);
    _outputFile << "HOME\n";
  }
}


void Jaco::saveToFile() {
  _outputFile.close();
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
