//
// Created by qiayuan on 2/16/20.
//

#include "Utilities/FakeSuspe.h"
void FakeSuspe::update(SuspeData &data) {
  torque_out_[0] = 100. * (0 - data.q_[0]) + 1. * (0 - data.qd_[0]);
  torque_out_[1] = 100. * (0 - data.q_[1]) + 1. * (0 - data.qd_[1]);
  torque_out_[2] = 100. * (0 - data.q_[2]) + 1. * (0 - data.qd_[2]);
  torque_out_[3] = 100. * (0 - data.q_[3]) + 1. * (0 - data.qd_[3]);
}
