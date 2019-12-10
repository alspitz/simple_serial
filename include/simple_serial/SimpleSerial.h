#pragma once

#include <string>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/CascadedCommand.h>
#include <quadrotor_srvs/Toggle.h>
#include <robot_msgs/OdomNoCov.h>

class SimpleSerial {
  public:
    bool initialize(const ros::NodeHandle& n);
    void loop();

  private:
    bool openPort();
    bool loadParameters(const ros::NodeHandle& n);
    bool registerCallbacks(const ros::NodeHandle& n);

    void odomCallback(const robot_msgs::OdomNoCov::ConstPtr& msg);
    void odomCallbackFull(const nav_msgs::Odometry::ConstPtr& msg);
    void cascadedCommandCallback(const quadrotor_msgs::CascadedCommand::ConstPtr& msg);
    bool motorServiceCallback(quadrotor_srvs::Toggle::Request& mreq, quadrotor_srvs::Toggle::Response& mres);

    bool read_n(uint8_t *buf, int n_read);

    std::string device_name_;
    int baud_;

    int fd_;
    bool opened_{false};
    float yaw_{0.0f};

    ros::Subscriber casc_sub_;
    bool full_odom_{false};
    ros::Subscriber odom_sub_;

    ros::Publisher imu_pub_;
    ros::Publisher rpm_pub_;

    ros::ServiceServer motors_service_;
};
