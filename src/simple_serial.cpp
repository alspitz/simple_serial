#include <simple_serial/SimpleSerial.h>

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_serial");

  ros::NodeHandle n("~");

  if (!SimpleSerial::instance.initialize(n)) {
    return 1;
  }

  SimpleSerial::instance.loop();

  return 0;
}
