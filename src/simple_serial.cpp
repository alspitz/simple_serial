#include <simple_serial/SimpleSerial.h>

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_serial");

  ros::NodeHandle n("~");

  SimpleSerial serial;

  if (!serial.initialize(n)) {
    return 1;
  }

  serial.loop();

  return 0;
}
