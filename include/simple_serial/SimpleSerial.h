#pragma once

#include <string>

#include <ros/ros.h>

#include <multirotor_control/FLCommand.h>
#include <multirotor_control/FLGains.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/CascadedCommand.h>
#include <quadrotor_msgs/CascadedCommandGains.h>
#include <quadrotor_msgs/RPMCommand.h>
#include <quadrotor_srvs/Toggle.h>
#include <robot_msgs/OdomNoCov.h>

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
    void rpmCallback(const quadrotor_msgs::RPMCommand::ConstPtr& msg);
    bool motorServiceCallback(quadrotor_srvs::Toggle::Request& mreq, quadrotor_srvs::Toggle::Response& mres);

    bool read_n(uint8_t *buf, int n_read);

    std::string device_name_;
    int baud_;

    int fd_;
    bool opened_{false};
    float yaw_{0.0f};

    ros::Subscriber rpm_sub_;
    ros::Subscriber casc_sub_;
    ros::Subscriber gains_sub_;
    ros::Subscriber fl_cmd_sub_;
    ros::Subscriber fl_gains_sub_;
    bool full_odom_{false};
    ros::Subscriber odom_sub_;

    ros::Publisher imu_pub_;
    ros::Publisher rpm_pub_;

    ros::ServiceServer motors_service_;
};
