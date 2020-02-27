//
// Created by qiayuan on 2/16/20.
//

#include "Utilities/FakeSuspe.h"
void FakeSuspe::setParams(ros::NodeHandle *nh) {
  params.getParam(nh);
  suspe0ForceAngle_ = acos(( //law of cosines
                               params.suspe_length0_ * params.suspe_length0_
                                   + params.suspe_length1_ * params.suspe_length1_
                                   - params.spring_length_ * params.spring_length_)
                               / (2. * params.suspe_length0_ * params.suspe_length1_));

}

double FakeSuspe::getSetupAngle(int id) {
  return signTable[id] * params.suspe_q_offset_;
}

void FakeSuspe::update(SuspeData &data) {
  double q[4], qd[4]{};

  for (int wheelID = 0; wheelID < 4; ++wheelID) {
    //Flip input according  sign table
    q[wheelID] = signTable[wheelID] * data.q_[wheelID];
    qd[wheelID] = signTable[wheelID] * data.qd_[wheelID];
    //Calculate moment of suspension
    double suspeAngle = -q[wheelID] + suspe0ForceAngle_ + params.suspe_q_offset_;
    springLength_[wheelID] = sqrt(params.suspe_length0_ * params.suspe_length0_ + //law of cosines
        params.suspe_length1_ * params.suspe_length1_ -
        2. * params.suspe_length0_ * params.suspe_length1_ * cos(suspeAngle));
    double springVel =
        params.suspe_length0_ * params.suspe_length1_ * sin(suspeAngle) / springLength_[wheelID] * qd[wheelID];
    double springForce;
    //Check if out of range. Use large kp and kd to simulate collision
    //Note that we defines the positive direction of the spring is compress direction
    double delta_length = params.spring_length_ - springLength_[wheelID];

    if (delta_length < 0.) {
      springForce =
          10000. * (delta_length - 0.01) - 50000. * (0 - springVel); //minus 0.001 to avoid stack on critical point
    } else if (delta_length > params.spring_range_) {
      springForce = 50000. * (delta_length - params.spring_range_ + 0.01)
          - 100000. * (0 - springVel); //plus 0.001 to avoid stack on critical point
    } else {
      springForce = params.spring_kp_ * delta_length - params.spring_kd_ * (0 - springVel) + params.spring_preload_;
    }
    double sinTheta = sin(suspeAngle) / springLength_[wheelID] * params.suspe_length0_;//law of sines
    torque_out_[wheelID] = -springForce * params.suspe_length1_ * sinTheta;
    //flip output according  sign table
    torque_out_[wheelID] = signTable[wheelID] * torque_out_[wheelID];
  }
}
double FakeSuspe::getSpringLength(int id) {
  return springLength_[id];
}
