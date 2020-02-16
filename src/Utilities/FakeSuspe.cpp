//
// Created by qiayuan on 2/16/20.
//

#include "Utilities/FakeSuspe.h"
void FakeSuspe::update(SuspeData &data) {
  torque_out_[0] = 50 * (0 - data.q_[0]) + 5 * (0 - data.qd_[0]);
  torque_out_[1] = 50 * (0 - data.q_[1]) + 5 * (0 - data.qd_[1]);
  torque_out_[2] = 50 * (0 - data.q_[2]) + 5 * (0 - data.qd_[2]);
  torque_out_[3] = 50 * (0 - data.q_[3]) + 5 * (0 - data.qd_[3]);
}
