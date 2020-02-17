//
// Created by qiayuan on 2/17/20.
//

#ifndef SRC_RM_SUSPENSION_INCLUDE_UTILITIES_CONTROLLER_H_
#define SRC_RM_SUSPENSION_INCLUDE_UTILITIES_CONTROLLER_H_
class Controller {
 public:
  double torque_out_[4];
  Controller() = default;
  ~Controller() = default;
  void setSpeed(double speed);
  void update(double *qd, double cTime);
 private:
  double qd_des_{};
};

#endif //SRC_RM_SUSPENSION_INCLUDE_UTILITIES_CONTROLLER_H_
