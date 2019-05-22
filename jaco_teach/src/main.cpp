#include "jaco_teach.h"

#define CONNECTION_TYPE "USB" // can also be "ETHERNET"

#define JOINT_123_SPEED_LIMIT 20  // deg/s
#define JOINT_456_SPEED_LIMIT 20  // deg/s

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "Jaco_teach");
  Gtk::Main kit(argc, argv);
  Gtk::Window window;
  boost::recursive_mutex mutex;
  ros::NodeHandle n;

  n.setParam("connection_type", CONNECTION_TYPE);
  n.setParam("jointSpeedLimitParameter1", JOINT_123_SPEED_LIMIT);
  n.setParam("jointSpeedLimitParameter2", JOINT_456_SPEED_LIMIT);

  Jaco robot(window, n, mutex);
  robot.initTeachMode();

  Gtk::Main::run(window);
}
