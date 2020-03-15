#pragma once

#include <string>

#include <ros/ros.h>

#include <geometry_utils/GeometryUtils.h>
#include <multirotor_control/FLCommand.h>
#include <multirotor_control/FLGains.h>
#include <multirotor_control/TVCommand.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/CascadedCommand.h>
#include <quadrotor_msgs/CascadedCommandGains.h>
#include <quadrotor_msgs/RPMCommand.h>
#include <quadrotor_srvs/Toggle.h>
#include <robot_msgs/OdomNoCov.h>
#include <simple_serial/SimpleMavlink.h>

namespace gu = geometry_utils;

class SimpleSerial {
  public:
    bool initialize(const ros::NodeHandle& n);
    void loop();

    static SimpleSerial instance;

  private:
    bool openPort();
    bool loadParameters(const ros::NodeHandle& n);
    bool registerCallbacks(const ros::NodeHandle& n);

    static void static_sighupHandler(int signum);
    void sighupHandler(int signum);

    void odomCallback(const robot_msgs::OdomNoCov::ConstPtr& msg);
    void odomCallbackFull(const nav_msgs::Odometry::ConstPtr& msg);
    void cascadedCommandCallback(const quadrotor_msgs::CascadedCommand::ConstPtr& msg);
    void cascadedCommandGainsCallback(const quadrotor_msgs::CascadedCommandGains::ConstPtr& msg);
    void flCommandCallback(const multirotor_control::FLCommand::ConstPtr& msg);
    void flCommandGainsCallback(const multirotor_control::FLGains::ConstPtr& msg);
    void tvCommandCallback(const multirotor_control::TVCommand::ConstPtr& msg);
    void rpmCallback(const quadrotor_msgs::RPMCommand::ConstPtr& msg);
    bool motorServiceCallback(quadrotor_srvs::Toggle::Request& mreq, quadrotor_srvs::Toggle::Response& mres);

    void processOdom(double yaw, ros::Time time, gu::Vec3 vel);
    void reset();

    std::string device_name_;
    int baud_;

    int fd_;
    bool opened_{false};
    float yaw_{0.0f};
    gu::Vec3 lastvel_;
    ros::Time lasttime_;
    gu::Vec3 accel_;

    static int compute_checksum(uint8_t *buf, int length) {
      int csum = 0;
      for (int i = 0; i < length; i++) {
        csum += buf[i];
      }
      return csum;
    }

    template <class T>
    bool check_length() {
      if (length_ != sizeof(T)) {
        ROS_WARN("Incorrect length: expected %lu, got %d", sizeof(T), length_);
        return false;
      }

      return true;
    }

    template <class T>
    bool send_msg(T msg, uint8_t msg_id) {
      if (!opened_) {
        return false;
      }

      uint8_t* buf = (uint8_t*)(&msg);
      msg.magic = MAGIC;
      msg.msg_id = msg_id;
      msg.csum = compute_checksum(buf, sizeof(T) - 1);

      int ret = write(fd_, buf, sizeof(T));
      if (ret != sizeof(T)) {
        ROS_WARN("Failed to send message: %d", ret);
        return false;
      }

      return true;
    }

    enum parse_state {
      MAGIC1,
      META,
      MSG
    };

    static constexpr int max_msg_length{70};
    parse_state read_state_{MAGIC1};
    int to_read_{1};
    uint8_t* read_ptr_{0};
    int length_{-1};
    uint8_t msg_id_{0};
    uint8_t read_buf_[max_msg_length];

    ros::Subscriber rpm_sub_;
    ros::Subscriber casc_sub_;
    ros::Subscriber gains_sub_;
    ros::Subscriber fl_cmd_sub_;
    ros::Subscriber fl_gains_sub_;
    ros::Subscriber tv_cmd_sub_;
    bool full_odom_{false};
    ros::Subscriber odom_sub_;

    ros::Publisher imu_pub_;
    ros::Publisher rpm_pub_;
    ros::Publisher fls_pub_;

    ros::ServiceServer motors_service_;
};
