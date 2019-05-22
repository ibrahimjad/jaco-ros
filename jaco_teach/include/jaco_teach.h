#ifndef JACO_CONTROL_H
#define JACO_CONTROL_H


#include "ros/ros.h"
#include "kinova_driver/kinova_comm.h"
#include <boost/thread/recursive_mutex.hpp>
#include <exception>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <string>
#include <gtkmm.h>


class Jaco : private kinova::KinovaComm {
public:
  Jaco(Gtk::Window &window, ros::NodeHandle &n, boost::recursive_mutex &mutex);
  ~Jaco();
  void initTeachMode();

private:
  void goHome();

  void openFingers(bool isOpen);
  void actuateFingers();

  void goToPosition();
  void savePosition();

  void readPoseFromFile();
  void savePoseToFile();
  void saveFingersToFile();
  void parseFile();

  bool isMoving();
  void waitWhileMoving();

  void openFile();
  void saveToFile();

  void updateMovementType();
  bool isSleepLine();
  bool isFloat();


  void createInterface();
  void showInterface();
  void setWindowSettings();
  void setBoxSettings();
  void setButtonLabels();
  void setButtonSize();
  void setButtonCallbacks();

private:
  kinova::KinovaPose _currentPosition;
  kinova::KinovaPose _targetPosition;

  std::string _line;
  std::ofstream _outputFile;
  std::ifstream _inputFile;

  kinova::FingerAngles _fingers;
  bool _isFingersOpen = true;
  bool _oldFingers = true;

  std::string _movementType;
  bool _isMovementContinous = false;

  double _sleepTimeInSecond;

  const std::string _homeDir = std::getenv("HOME");
  const float _MOVEMENT_TOLERANCE = 0.02f;

private:
  Gtk::Window *_pWindow;
  Gtk::Box _box;
  Gtk::Button _save_button;
  Gtk::Button _fingers_button;
  Gtk::Button _saveToFile_button;
  Gtk::Button _replay_button;
};

#endif
