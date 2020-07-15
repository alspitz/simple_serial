#include <simple_serial/SimpleSerial.h>

#include <fcntl.h>
#include <inttypes.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <csignal>

#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>

#include <multirotor_control/FLState.h>
#include <quadrotor_msgs/RPMCommand.h>
#include <simple_serial/IMUDebug.h>

#include <linux/serial.h>
#include <sys/ioctl.h>

namespace gu = geometry_utils;
namespace gr = geometry_utils::ros;
namespace pu = parameter_utils;

SimpleSerial SimpleSerial::instance;

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

void SimpleSerial::static_sighupHandler(int signum) {
  ROS_INFO("Caught hangup.");
  instance.sighupHandler(signum);
}

void SimpleSerial::sighupHandler(int signum) {
  opened_ = false;
  close(fd_);
}

bool SimpleSerial::openPort() {
  fd_ = open(device_name_.c_str(), O_RDWR);

  signal(SIGHUP, SimpleSerial::static_sighupHandler);

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

  ROS_INFO("Serial settings before are: %d %d %d %d", tty.c_cflag, tty.c_lflag, tty.c_oflag, tty.c_iflag);

  // Magic values from Python's pyserial.
  if (baud_ == 921600) {
    tty.c_cflag = 7351;
  }
  else if (baud_ == 230400) {
    tty.c_cflag = 7347;
  }
  else {
    ROS_ERROR("Unsupported baud: FIX ME");
  }
  //tty.c_cflag |= CS8; // 8 bits per byte
  //tty.c_cflag |= CREAD | CLOCAL; // read and no modem
  //tty.c_cflag &= ~PARENB; // no parity
  //tty.c_cflag &= ~CSTOPB; // one stop bit
  //tty.c_cflag &= ~CRTSCTS; // no flow control

  tty.c_lflag = 0;
  tty.c_iflag = 0;
  tty.c_oflag = 0;

  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 0;

  // Does this even do anything?
  cfsetispeed(&tty, baud_);
  cfsetospeed(&tty, baud_);

  ROS_INFO("Serial settings after are: %d %d %d %d", tty.c_cflag, tty.c_lflag, tty.c_oflag, tty.c_iflag);

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
    odom_sub_ = ln.subscribe("odom", 1, &SimpleSerial::odomCallbackFull, this, ros::TransportHints().tcpNoDelay());
  }
  else {
    odom_sub_ = ln.subscribe("odom", 1, &SimpleSerial::odomCallback, this, ros::TransportHints().tcpNoDelay());
  }
  casc_sub_ = ln.subscribe("cascaded_cmd", 1, &SimpleSerial::cascadedCommandCallback, this, ros::TransportHints().tcpNoDelay());
  gains_sub_ = ln.subscribe("cascaded_cmd_gains", 1, &SimpleSerial::cascadedCommandGainsCallback, this, ros::TransportHints().tcpNoDelay());
  fl_cmd_sub_ = ln.subscribe("fl_cmd", 1, &SimpleSerial::flCommandCallback, this, ros::TransportHints().tcpNoDelay());
  fl_gains_sub_ = ln.subscribe("fl_cmd_gains", 1, &SimpleSerial::flCommandGainsCallback, this, ros::TransportHints().tcpNoDelay());
  tv_cmd_sub_ = ln.subscribe("tv_cmd", 1, &SimpleSerial::tvCommandCallback, this, ros::TransportHints().tcpNoDelay());
  rpm_sub_ = ln.subscribe("rpm_cmd", 1, &SimpleSerial::rpmCallback, this, ros::TransportHints().tcpNoDelay());
  fr_sub_ = ln.subscribe("filerequest", 1, &SimpleSerial::fileRequestCallback, this);
  motors_service_ = ln.advertiseService("motors", &SimpleSerial::motorServiceCallback, this);

  imu_pub_ = ln.advertise<simple_serial::IMUDebug>("imu", 1, false);
  rpm_pub_ = ln.advertise<quadrotor_msgs::RPMCommand>("rpm", 1, false);
  fls_pub_ = ln.advertise<multirotor_control::FLState>("flstate", 1, false);

  return true;
}

gu::Vec3 convertframe(const gu::Vec3& in) {
  gu::Vec3 out;
  out(0) =  in(0);
  out(1) = -in(1);
  out(2) = -in(2);
  return out;
}

void SimpleSerial::reset() {
  read_ptr_ = read_buf_;
  read_state_ = MAGIC1;
  to_read_ = 1;
}

int get_length(uint8_t msg_id) {
  if (msg_id == MSG_ID_rpm) {
    return sizeof(struct rpm_msg);
  }
  if (msg_id == MSG_ID_enable) {
    return sizeof(struct enable_msg);
  }
  if (msg_id == MSG_ID_cmd) {
    return sizeof(struct cmd_msg);
  }
  if (msg_id == MSG_ID_imu) {
    return sizeof(struct imu_msg);
  }
  if (msg_id == MSG_ID_gains) {
    return sizeof(struct gains_msg);
  }
  if (msg_id == MSG_ID_flcmd) {
    return sizeof(struct flcmd_msg);
  }
  if (msg_id == MSG_ID_flgains) {
    return sizeof(struct flgains_msg);
  }
  if (msg_id == MSG_ID_flstate) {
    return sizeof(struct flstate_msg);
  }
  if (msg_id == MSG_ID_tvcmd) {
    return sizeof(struct tvcmd_msg);
  }
  if (msg_id == MSG_ID_ack) {
    return sizeof(struct ack_msg);
  }
  if (msg_id == MSG_ID_file_request) {
    return sizeof(struct file_request_msg);
  }
  if (msg_id == MSG_ID_file_chunk) {
    return sizeof(struct file_chunk_msg);
  }


  return -1;
}

void SimpleSerial::loop() {
  ros::Rate rate(2000);

  bool synced = true;

  while (1) {
    ros::spinOnce();

    if (!ros::ok()) {
      ROS_INFO("Shutting down.");
      break;
    }

    rate.sleep();

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

    int ret = read(fd_, read_ptr_, to_read_);
    if (ret < 0 || ret > to_read_) {
      ROS_WARN("Read failure: %d", ret);
      perror("read failure: ");
      reset();
      continue;
    }
    if (ret == 0) {
      continue;
    }

    read_ptr_ += ret;
    to_read_ -= ret;

    if (to_read_) {
      continue;
    }

    if (read_state_ == MAGIC1) {
      if (read_buf_[read_state_] != MAGIC) {
        reset();
        if (synced) {
          ROS_WARN("Expected magic; did not get it.");
          synced = false;
        }
        continue;
      }

      synced = true;

      read_state_ = META;
      to_read_ = META_SIZE - N_MAGIC;
    }
    else if (read_state_ == META) {
      msg_id_ = read_buf_[1];

      length_ = get_length(msg_id_);
      if (length_ < 0) {
        reset();
        ROS_ERROR("Unrecognized msg id: %d", msg_id_);
        synced = false;
        continue;
      }

      read_state_ = MSG;
      to_read_ = length_ - META_SIZE;
    }
    else if (read_state_ == MSG) {
      reset();

      const uint16_t csum_ours = compute_checksum(read_buf_, length_ - 2);
      const uint16_t csum_theirs = (read_buf_[length_ - 2] << 8) | read_buf_[length_ - 1];
      if (csum_ours != csum_theirs) {
        ROS_WARN("Checksum mismatch of (%d, %d): %d vs %d", length_, msg_id_, csum_ours, csum_theirs);
        continue;
      }

      if (msg_id_ == MSG_ID_rpm) {
        struct rpm_msg* rpm = (struct rpm_msg*)read_buf_;
        quadrotor_msgs::RPMCommand ros_msg;
        try {
          ros_msg.header.stamp.fromNSec(rpm->timestamp * 1000);
        }
        catch (const std::runtime_error& e) {
          ROS_ERROR("RPM msg time stamp too big for time! %" PRIu64, rpm->timestamp);
        }

        for (int i = 0; i < 4; i++) {
          ros_msg.motor_rpm[i] = rpm->rpm[i];
        }
        rpm_pub_.publish(ros_msg);
      }

      else if (msg_id_ == MSG_ID_imu) {
        struct imu_msg* imu = (struct imu_msg*)read_buf_;
        simple_serial::IMUDebug ros_msg;
        try {
          ros_msg.header.stamp.fromNSec(imu->timestamp * 1000);
        }
        catch (const std::runtime_error& e) {
          ROS_ERROR("IMU msg time stamp too big for time! %" PRIu64, imu->timestamp);
        }
        ros_msg.accel      = gr::toVector3(convertframe(gu::Vector3(imu->accel     [0], imu->accel     [1], imu->accel     [2])));
        ros_msg.accel_filt = gr::toVector3(convertframe(gu::Vector3(imu->accel_filt[0], imu->accel_filt[1], imu->accel_filt[2])));
        ros_msg.gyro       = gr::toVector3(convertframe(gu::Vector3(imu->gyro      [0], imu->gyro      [1], imu->gyro      [2])));
        ros_msg.gyro_filt  = gr::toVector3(convertframe(gu::Vector3(imu->gyro_filt [0], imu->gyro_filt [1], imu->gyro_filt [2])));
        ros_msg.euler = gr::toVector3(gu::Vector3(imu->roll, -imu->pitch, yaw_));
        imu_pub_.publish(ros_msg);
      }

      else if (msg_id_ == MSG_ID_flstate) {
        struct flstate_msg* fls = (struct flstate_msg*)read_buf_;
        multirotor_control::FLState ros_msg;
        try {
          ros_msg.header.stamp.fromNSec(fls->timestamp * 1000);
        }
        catch (const std::runtime_error& e) {
          ROS_ERROR("flstate msg time stamp too big for time! %" PRIu64, fls->timestamp);
        }
        ros_msg.u = -fls->u;
        ros_msg.udot = -fls->udot;
        fls_pub_.publish(ros_msg);
      }

      else if (msg_id_ == MSG_ID_file_chunk) {
        struct file_chunk_msg* msg = (struct file_chunk_msg*)read_buf_;

        ROS_INFO("Received chunk with id %d and length %d", msg->id, msg->length);

        struct ack_msg ack_msg;
        ack_msg.id = msg->id;
        send_msg(ack_msg, MSG_ID_ack);

        if (msg->length == 0) {
          outfile.close();
        }

        if (msg->id != id_written) {
          if (msg->length > FILECHUNK_SIZE) {
            ROS_WARN("Invalid file chunk length: %d", msg->length);
          }
          else {
            outfile.write((const char *)(msg->data), msg->length);
            id_written = msg->id;
          }
        }
        else {
          ROS_WARN("Duplicate chunk id %d", id_written);
        }
      }

      else {
        ROS_WARN("Unsupported msg id: %d", msg_id_);
      }
    }
  }

  rpm_sub_.shutdown();
  casc_sub_.shutdown();
  fl_cmd_sub_.shutdown();
  gains_sub_.shutdown();
  fl_gains_sub_.shutdown();
  tv_cmd_sub_.shutdown();
  odom_sub_.shutdown();
  fr_sub_.shutdown();
}

void SimpleSerial::odomCallback(const robot_msgs::OdomNoCov::ConstPtr& msg) {
  processOdom(gu::getYaw(gr::fromROS(msg->pose.orientation)), msg->header.stamp, gr::fromROS(msg->twist.linear));
}

void SimpleSerial::odomCallbackFull(const nav_msgs::Odometry::ConstPtr& msg) {
  processOdom(gu::getYaw(gr::fromROS(msg->pose.pose.orientation)), msg->header.stamp, gr::fromROS(msg->twist.twist.linear));
}

void SimpleSerial::processOdom(double yaw, ros::Time time, gu::Vec3 vel) {
  yaw_ = yaw;

  //accel_ = (vel - lastvel_) / (time - lasttime_).toSec();
  lastvel_ = vel;
  lasttime_ = time;
}

void SimpleSerial::rpmCallback(const quadrotor_msgs::RPMCommand::ConstPtr& ros_msg) {
  struct rpm_msg msg;
  msg.timestamp = ros_msg->header.stamp.toNSec() / 1000;
  for (int i = 0; i < 4; i++) {
    msg.rpm[i] = ros_msg->motor_rpm[i];
  }

  send_msg(msg, MSG_ID_rpm);
}

void SimpleSerial::cascadedCommandCallback(const quadrotor_msgs::CascadedCommand::ConstPtr& ros_msg) {
  struct cmd_msg msg;
  msg.timestamp = ros_msg->header.stamp.toNSec() / 1000;
  msg.thrust = -ros_msg->thrust;
  msg.q[0] = ros_msg->orientation.w;
  msg.q[1] = ros_msg->orientation.x;
  msg.q[2] = -ros_msg->orientation.y;
  msg.q[3] = -ros_msg->orientation.z;

  gu::Vec3 angvel = convertframe(gr::fromROS(ros_msg->angular_velocity));
  gu::Vec3 angacc = convertframe(gr::fromROS(ros_msg->angular_acceleration));
  for (int i = 0; i < 3; i++) {
    msg.angvel[i] = angvel(i);
    msg.angacc[i] = angacc(i);
    //msg.accel[i] = accel_(i);
  }

  if (yaw_ == 0.0f) {
    msg.yaw = -ros_msg->current_heading;
  }
  else {
    msg.yaw = -yaw_;
  }

  send_msg(msg, MSG_ID_cmd);
}

void SimpleSerial::flCommandCallback(const multirotor_control::FLCommand::ConstPtr& ros_msg) {
  struct flcmd_msg msg;
  msg.timestamp = ros_msg->header.stamp.toNSec() / 1000;

  gu::Vec3 snap_ff = convertframe(gr::fromROS(ros_msg->snap_ff));
  for (int i = 0; i < 3; i++) {
    msg.snap_ff[i] = snap_ff(i);
  }

  msg.desired_yaw = -ros_msg->desired_yaw;
  msg.desired_yawacc = -ros_msg->desired_yawacc;
  msg.yaw = -yaw_;

  send_msg(msg, MSG_ID_flcmd);
}


void SimpleSerial::cascadedCommandGainsCallback(const quadrotor_msgs::CascadedCommandGains::ConstPtr& ros_msg) {
  struct gains_msg msg;
  gu::Vec3 kR = gr::fromROS(ros_msg->kR);
  gu::Vec3 kOm = gr::fromROS(ros_msg->kOm);
  for (int i = 0; i < 3; i++) {
    msg.kR[i] = kR(i);
    msg.kOm[i] = kOm(i);
  }

  send_msg(msg, MSG_ID_gains);
}

void SimpleSerial::flCommandGainsCallback(const multirotor_control::FLGains::ConstPtr& ros_msg) {
  struct flgains_msg msg;
  gu::Vec3 k3 = gr::fromROS(ros_msg->k3);
  gu::Vec3 k4 = gr::fromROS(ros_msg->k4);
  for (int i = 0; i < 3; i++) {
    msg.k3[i] = k3(i);
    msg.k4[i] = k4(i);
  }
  msg.yaw_kp = ros_msg->yaw_kp;
  msg.yaw_kd = ros_msg->yaw_kd;
  msg.delay_const = ros_msg->delay_const;

  send_msg(msg, MSG_ID_flgains);
}

void SimpleSerial::tvCommandCallback(const multirotor_control::TVCommand::ConstPtr& ros_msg) {
  struct tvcmd_msg msg;
  msg.timestamp = ros_msg->header.stamp.toNSec() / 1000;
  gu::Vec3 accel = convertframe(gr::fromROS(ros_msg->accel));
  gu::Vec3 angacc = convertframe(gr::fromROS(ros_msg->angaccel));
  for (int i = 0; i < 3; i++) {
    msg.accel[i] = accel(i);
    msg.angacc[i] = angacc(i);
  }
  msg.desired_yaw = -ros_msg->desired_yaw;
  msg.yaw = -yaw_;

  send_msg(msg, MSG_ID_tvcmd);
}

void SimpleSerial::fileRequestCallback(const simple_serial::FileRequest::ConstPtr& ros_msg) {
  struct file_request_msg msg;
  ROS_INFO("[SimpleSerial] Received file request for file \"%s\"", ros_msg->filename.data.c_str());
  strcpy(msg.filename, ros_msg->filename.data.c_str());
  send_msg(msg, MSG_ID_file_request);

  outfile.open("/home/alex/from_px4.log", std::ofstream::binary);
  id_written = 0;
}

bool SimpleSerial::motorServiceCallback(quadrotor_srvs::Toggle::Request& mreq, quadrotor_srvs::Toggle::Response& mres) {
  mres.status = mreq.enable;

  struct enable_msg msg;
  msg.enable = mreq.enable;

  return send_msg(msg, MSG_ID_enable);
}
