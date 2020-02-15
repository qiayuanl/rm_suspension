//
// Created by qiayuan on 2/14/20.
//

#ifndef SUSPENSION_SIM_INCLUDE_CONFIG_H_
#define SUSPENSION_SIM_INCLUDE_CONFIG_H_
#include "cTypes.h"
#include "cppTypes.h"
#include <ros/ros.h>

class SimParameters {
 public:
  double dynamics_dt_;
  double control_dt_;
  double floor_kp_;
  double floor_kd_;

  bool use_spring_damper_;
  bool go_home_;

  Vec3<double> home_pos_;
  Vec3<double> home_rpy_;

  double home_kp_lin_;
  double home_kd_lin_;
  double home_kp_ang_;
  double home_kd_ang_;
  void getParam(ros::NodeHandle *nh) {
    XmlRpc::XmlRpcValue param;

    nh->param<double>("dynamics_dt", dynamics_dt_, 0.001);
    nh->param<double>("control_dt", control_dt_, 0.001);
    nh->param<double>("floor_kp", floor_kp_, 5000);
    nh->param<double>("floor_kd", floor_kd_, 500000);
    nh->param<double>("home_kp_lin", home_kp_lin_, 2500);
    nh->param<double>("home_kd_lin", home_kd_lin_, 300);
    nh->param<double>("home_kp_ang", home_kp_ang_, 250);
    nh->param<double>("home_kd_ang", home_kd_ang_, 60);
    nh->param<bool>("use_spring_damper", use_spring_damper_, false);
    nh->param<bool>("go_home", go_home_, false);

    if (nh->getParam("home_pos", param))
      home_pos_ << param[0], param[1], param[2];
    if (nh->getParam("home_rpy", param))
      home_rpy_ << param[0], param[1], param[2];
  };
};

#endif //SUSPENSION_SIM_INCLUDE_CONFIG_H_
