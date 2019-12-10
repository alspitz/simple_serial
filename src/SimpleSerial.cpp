#include <simple_serial/SimpleSerial.h>

#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>

#include <quadrotor_msgs/RPMCommand.h>
#include <sensor_msgs/Imu.h>

#include <simple_serial/SimpleMavlink.h>

#include <linux/serial.h>
#include <sys/ioctl.h>

namespace gu = geometry_utils;
namespace gr = geometry_utils::ros;
namespace pu = parameter_utils;

bool SimpleSerial::initialize(const ros::NodeHandle& n) {
  if (!loadParameters(n)) {
    ROS_ERROR("SimpleSerial: failed to load parameters");
    return false;
  }

  if (!registerCallbacks(n)) {
    ROS_ERROR("SimpleSerial: failed to register callbacks");
    return false;
  }

  return true;
}

bool SimpleSerial::loadParameters(const ros::NodeHandle& n) {
  if (!pu::get("serial_device", device_name_)) {
    ROS_ERROR("SimpleSerial: Could not get serial_device parameter");
    return false;
  }

  if (!pu::get("baud", baud_)) {
    ROS_ERROR("SimpleSerial: Could not get baud parameter");
    return false;
  }

  if (!openPort()) {
    ROS_ERROR("SimpleSerial: Could not open port.");
    return false;
  }

  opened_ = true;

  pu::get("full_odom", full_odom_, false);

  return true;
}

bool SimpleSerial::openPort() {
  fd_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY);

  if (fd_ < 0) {
    ROS_ERROR_THROTTLE(0.5, "Could not open serial port %s", device_name_.c_str());
    return false;
  }

  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  if (tcgetattr(fd_, &tty) != 0) {
    ROS_ERROR("Could not read serial port settings on %s", device_name_.c_str());
    return false;
  }

  tty.c_cflag = 7351; // Magic value from Python's setserial ?
  //tty.c_cflag |= CS8; // 8 bits per byte
  //tty.c_cflag |= CREAD | CLOCAL; // read and no modem
  //tty.c_cflag &= ~PARENB; // no parity
  //tty.c_cflag &= ~CSTOPB; // one stop bit
  //tty.c_cflag &= ~CRTSCTS; // no flow control

  tty.c_lflag = 0;
  tty.c_iflag = 0;
  tty.c_oflag = 0;

  tty.c_cc[VTIME] = 1;
  tty.c_cc[VMIN] = 0;

  cfsetispeed(&tty, baud_);
  cfsetospeed(&tty, baud_);

  ROS_INFO("Serial settings are: %d %d %d %d", tty.c_cflag, tty.c_lflag, tty.c_oflag, tty.c_iflag);

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ROS_ERROR("SimpleSerial: Could not save termios settings on %s", device_name_.c_str());
    return false;
  }

  struct serial_struct serinfo;
  if (ioctl(fd_, TIOCGSERIAL, &serinfo)) {
    ROS_ERROR("SimpleSerial: Could not get serial info on %s", device_name_.c_str());
    return false;
  }

  serinfo.flags |= ASYNC_LOW_LATENCY;
  if (ioctl(fd_, TIOCSSERIAL, &serinfo)) {
    ROS_ERROR("SimpleSerial: Could not set serial info on %s", device_name_.c_str());
    return false;
  }

  return true;
}


bool SimpleSerial::registerCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle ln(n);

  if (full_odom_) {
    odom_sub_ = ln.subscribe("odom", 1, &SimpleSerial::odomCallbackFull, this, ros::TransportHints().udp());
  }
  else {
    odom_sub_ = ln.subscribe("odom", 1, &SimpleSerial::odomCallback, this, ros::TransportHints().udp());
  }
  casc_sub_ = ln.subscribe("cascaded_cmd", 1, &SimpleSerial::cascadedCommandCallback, this, ros::TransportHints().tcpNoDelay());
  motors_service_ = ln.advertiseService("motors", &SimpleSerial::motorServiceCallback, this);

  imu_pub_ = ln.advertise<sensor_msgs::Imu>("imu/data", 1, false);
  rpm_pub_ = ln.advertise<quadrotor_msgs::RPMCommand>("rpm", 10, false);

  return true;
}

int compute_checksum(uint8_t *buf, int length) {
  int csum = 0;
  for (int i = 0; i < length; i++) {
    csum += buf[i];
  }
  return csum;
}

bool SimpleSerial::read_n(uint8_t *buf, int to_read) {
  int n_read = 0;
  while (n_read < to_read) {
    int ret = read(fd_, buf, to_read - n_read);

    if (ret < 0) {
      return false;
    }
    else if (ret == 0) {
      opened_ = false;
      close(fd_);
      return false;
    }

    n_read += ret;
    buf += ret;
  }

  return true;
}

void SimpleSerial::loop() {
  // Just needs to be larger than the largest message possible.
  static constexpr int max_msg_length = 50;
  uint8_t buf[max_msg_length];

  while (1) {
    ros::spinOnce();

    if (!ros::ok()) {
      ROS_INFO("Shutting down.");
      break;
    }

    if (!opened_) {
      ROS_WARN_THROTTLE(0.5, "FD is now closed. Attempting to reopen...");
      if (openPort()) {
        opened_ = true;
        ROS_INFO("Port opened");
      }
      else {
        usleep(20000);
        continue;
      }
    }

    // Read first magic
    int ret = read(fd_, buf, 1);
    if (ret < 0) {
      ROS_WARN("Read failure: magic 1 (%d)", ret);
      perror("read failure: ");
      continue;
    }
    else if (ret == 0) {
      opened_ = false;
      close(fd_);
      continue;
    }

    if (buf[0] != MAGIC) {
      ROS_WARN("Unexpected byte");
      continue;
    }

    // Read second magic
    ret = read(fd_, buf + 1, 1);
    if (ret < 0) {
      ROS_WARN("Read failure: magic 2");
      perror("read failure: ");
      continue;
    }
    if (ret == 0) {
      opened_ = false;
      close(fd_);
      continue;
    }

    if (buf[1] != MAGIC) {
      ROS_WARN("Got only one magic!!!");
      continue;
    }

    if (!read_n(buf + 2, META_SIZE - 2)) {
      ROS_WARN("Read failure: meta (length, seq, and msg id)");
      continue;
    }
    uint8_t length = buf[2];
    uint8_t seq = buf[3];
    uint8_t msg_id = buf[4];

    if (length > max_msg_length) {
      ROS_WARN("Invalid length: %d", length);
      continue;
    }

    if (!read_n(buf + META_SIZE, length - META_SIZE)) {
      ROS_WARN("Could not read message with ID %d and length %d", msg_id, length);
      continue;
    }

    uint8_t csum_ours = compute_checksum(buf, length - 1);
    if (csum_ours != buf[length - 1]) {
      ROS_WARN("Checksum mismatch of (%d, %d, %d): %d vs %d", length, seq, msg_id, csum_ours, buf[length]);
      continue;
    }

    if (msg_id == MSG_ID_RPM) {
      if (length != sizeof(struct rpm_msg)) {
        ROS_WARN("Length inconsistent: %d vs %lu!", length, sizeof(struct rpm_msg));
        continue;
      }

      struct rpm_msg* rpm = (struct rpm_msg*)buf;

      quadrotor_msgs::RPMCommand ros_msg;
      ros_msg.header.stamp.fromNSec(rpm->timestamp * 1000);
      for (int i = 0; i < 4; i++) {
        ros_msg.motor_rpm[i] = rpm->rpm[i];
      }
      rpm_pub_.publish(ros_msg);
    }

    else if (msg_id == MSG_ID_IMU) {
      if (length != sizeof(struct imu_msg)) {
        ROS_WARN("Length inconsistent: %d vs %lu!", length, sizeof(struct imu_msg));
      }

      struct imu_msg* imu = (struct imu_msg*)buf;

      sensor_msgs::Imu ros_msg;
      ros_msg.header.stamp.fromNSec(imu->timestamp * 1000);
      ros_msg.linear_acceleration = gr::toVector3(gu::Vector3(imu->accel[0], imu->accel[1], imu->accel[2]));
      ros_msg.angular_velocity = gr::toVector3(gu::Vector3(imu->gyro[0], imu->gyro[1], imu->gyro[2]));
      ros_msg.orientation = gr::ZYXToQuatMsg(gu::Vector3(imu->roll, imu->pitch, yaw_));
      imu_pub_.publish(ros_msg);
    }
    else {
      ROS_WARN("Unsupported msg id: %d", msg_id);
    }
  }
}

void SimpleSerial::odomCallback(const robot_msgs::OdomNoCov::ConstPtr& msg) {
  yaw_ = gu::getYaw(gr::fromROS(msg->pose.orientation));
}

void SimpleSerial::odomCallbackFull(const nav_msgs::Odometry::ConstPtr& msg) {
  yaw_ = gu::getYaw(gr::fromROS(msg->pose.pose.orientation));
}

void SimpleSerial::cascadedCommandCallback(const quadrotor_msgs::CascadedCommand::ConstPtr& ros_msg) {
  if (!opened_) {
    return;
  }

  struct cmd_msg msg;
  uint8_t* buf = (uint8_t*)(&msg);

  msg.magic = MAGICFULL;
  msg.length = sizeof(msg);
  msg.sequence = 0;
  msg.msg_id = MSG_ID_CMD;

  msg.timestamp = ros_msg->header.stamp.nsec / 1000;
  msg.thrust = -ros_msg->thrust;
  msg.q[0] = ros_msg->orientation.w;
  msg.q[1] = ros_msg->orientation.x;
  msg.q[2] = -ros_msg->orientation.y;
  msg.q[3] = -ros_msg->orientation.z;
  msg.yaw = -yaw_;

  msg.csum = compute_checksum(buf, sizeof(msg) - 1);

  int ret = write(fd_, buf, sizeof(msg));
  if (ret != sizeof(msg)) {
    ROS_WARN("Failed to send cmd message: %d", ret);
  }
}

bool SimpleSerial::motorServiceCallback(quadrotor_srvs::Toggle::Request& mreq, quadrotor_srvs::Toggle::Response& mres) {
  if (!opened_) {
    return false;
  }

  mres.status = mreq.enable;

  struct enable_msg msg;
  uint8_t* buf = (uint8_t*)(&msg);
  msg.magic = MAGICFULL;
  msg.length = sizeof(msg);
  msg.sequence = 0;
  msg.msg_id = MSG_ID_ENABLE;

  msg.enable = mreq.enable;

  msg.csum = compute_checksum(buf, sizeof(msg) - 1);

  int ret = write(fd_, buf, sizeof(msg));
  if (ret != sizeof(msg)) {
    ROS_WARN("Failed to send motor state: %d", ret);
    return false;
  }

  return true;
}
