//
// Created by qiayuan on 2/16/20.
//

#ifndef SRC_RM_SUSPENSION_INCLUDE_UTILITIES_FAKESUSPE_H_
#define SRC_RM_SUSPENSION_INCLUDE_UTILITIES_FAKESUSPE_H_
#include <config.h>

struct SuspeData {
  float q_[4];
  float qd_[4];
};

class FakeSuspe {
 public:
  FakeSuspe() = default;
  ~FakeSuspe() = default;
  void update(SuspeData &data);
  float torque_out_[4]{};
 private:

};

#endif //SRC_RM_SUSPENSION_INCLUDE_UTILITIES_FAKESUSPE_H_
