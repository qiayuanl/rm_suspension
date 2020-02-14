//
// Created by qiayuan on 2/14/20.
//

#ifndef SUSPENSION_SIM_INCLUDE_CONFIG_H_
#define SUSPENSION_SIM_INCLUDE_CONFIG_H_
#include "cTypes.h"

class SimParameters {
 public:
  double simulation_speed_;
  bool simulation_paused_;
  double control_dt_ = 0.001;
  double dynamics_dt_ = 0.001;
  double floor_kp_;
  double floor_kd_;

  bool use_spring_damper = false;
  bool go_home_ = true;

  Vec3<double> home_pos_;
  Vec3<double> home_rpy_;

  double home_kp_lin_;
  double home_kd_lin_;
  double home_kp_ang_;
  double home_kd_ang_;
  void getParam() {};
};

#endif //SUSPENSION_SIM_INCLUDE_CONFIG_H_
