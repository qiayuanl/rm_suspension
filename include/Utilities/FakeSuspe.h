//
// Created by qiayuan on 2/16/20.
//

#ifndef SRC_RM_SUSPENSION_INCLUDE_UTILITIES_FAKESUSPE_H_
#define SRC_RM_SUSPENSION_INCLUDE_UTILITIES_FAKESUSPE_H_
#include <config.h>

struct SuspeData {
  double q_[4];
  double qd_[4];
};

class FakeSuspe {
 public:
  FakeSuspe() = default;
  ~FakeSuspe() = default;
  void update(SuspeData &data);
  void setParams(ros::NodeHandle *nh);
  double getSetupAngle(int id);
  double getSpringLength(int id);
  float torque_out_[4]{};
 private:
  SuspeParameters params;
  double signTable[4] = {1., 1., -1., -1.};
  double suspe0ForceAngle_;
  double springLength_[4];
};

#endif //SRC_RM_SUSPENSION_INCLUDE_UTILITIES_FAKESUSPE_H_
