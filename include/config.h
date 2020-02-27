//
// Created by qiayuan on 2/14/20.
//

#ifndef SUSPENSION_SIM_INCLUDE_CONFIG_H_
#define SUSPENSION_SIM_INCLUDE_CONFIG_H_
#include "cTypes.h"
#include "cppTypes.h"
#include "Math/orientation_tools.h"
#include "Dynamics/SpatialInertia.h"

#include <ros/ros.h>

class SimParameters {
 public:
  double dynamics_dt_;
  double control_dt_;
  double vis_fps_;
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

    nh->param<double>("dynamics_dt", dynamics_dt_, 0.00001);
    nh->param<double>("control_dt", control_dt_, 0.001);
    nh->param<double>("vis_fps", vis_fps_, 250);
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

class SuspeParameters {
 public:
  double spring_length_;
  double spring_kp_;
  double spring_kd_;
  double spring_range_;
  double spring_preload_;
  double suspe_length0_;
  double suspe_length1_;
  double suspe_q_offset_;
  void getParam(ros::NodeHandle *nh) {
    XmlRpc::XmlRpcValue param;
    nh->param<double>("spring_length", spring_length_, 0.100);
    nh->param<double>("spring_kp", spring_kp_, 5000.);
    nh->param<double>("spring_kd", spring_kd_, 200.);
    nh->param<double>("spring_range", spring_range_, 0.03);
    nh->param<double>("spring_preload", spring_preload_, 0.);
    nh->param<double>("suspe_length0", suspe_length0_, 0.05168);
    nh->param<double>("suspe_length1", suspe_length1_, 0.10233);
    nh->param<double>("suspe_q_offset", suspe_q_offset_, -0.32);
  };
};

template<typename T>
class ChassisParameters {
 public:

  T _bodyMass;
  T _suspeMass;
  T _wheelMass;
  T _rotorMass;
  T _bodyLength;
  T _bodyWidth;
  T _bodyHeight;
  T _wheelRadius;
  T _wheelGearRatio;

  T _motorTauMax;
  T _batteryV;
  T _motorKT;  // this is flux linkage * pole pairs
  T _motorR;
  T _jointDamping;
  T _jointDryFriction;

  SpatialInertia<T> _bodyInertia, _suspeInertia, _wheelInertia;
  SpatialInertia<T> _wheelRotorInertia, _suspeRotorInertia;
  Vec3<T> _suspeLocation, _wheelLocation, _wheelRotorLocation, _rfidLocation;

  void getParam(ros::NodeHandle *nh) {
    XmlRpc::XmlRpcValue param;
    nh->param<T>("bodyMass", _bodyMass, 12.);
    nh->param<T>("suspeMass", _suspeMass, .900);
    nh->param<T>("wheelMass", _wheelMass, .660);
    nh->param<T>("rotorMass", _rotorMass, .146);

    nh->param<T>("bodyLength", _bodyLength, .58);
    nh->param<T>("bodyWidth", _bodyWidth, .52);
    nh->param<T>("bodyHeight", _bodyHeight, 0.015);
    nh->param<T>("wheelRadius", _wheelRadius, 0.10233);
    nh->param<T>("wheelGearRatio", _wheelGearRatio, 19.);

    nh->param<T>("motorTauMax", _motorTauMax, .15789);
    nh->param<T>("batteryV", _batteryV, 24.);
    nh->param<T>("motorKT", _motorKT, .0059333);
    nh->param<T>("motorR", _motorR, .194);
    nh->param<T>("jointDamping", _jointDamping, .01);
    nh->param<T>("jointDryFriction", _jointDryFriction, .05);

    Mat3<T> bodyRotationalInertia;
    if (nh->getParam("bodyRotationalInertia", param))
      bodyRotationalInertia << param[0], param[1], param[2],
          param[3], param[4], param[5],
          param[6], param[7], param[8];

    Mat3<T> suspeRotationalInertia;
    if (nh->getParam("suspeRotationalInertia", param))
      suspeRotationalInertia << param[0], param[1], param[2],
          param[3], param[4], param[5],
          param[6], param[7], param[8];

    Mat3<T> wheelRotationalInertia;
    if (nh->getParam("wheelRotationalInertia", param))
      wheelRotationalInertia << param[0], param[1], param[2],
          param[3], param[4], param[5],
          param[6], param[7], param[8];

    Mat3<T> rotorRotationalInertiaZ;
    if (nh->getParam("rotorRotationalInertiaZ", param))
      rotorRotationalInertiaZ << param[0], param[1], param[2],
          param[3], param[4], param[5],
          param[6], param[7], param[8];

    Vec3<T> bodyCOM;
    if (nh->getParam("bodyCOM", param))
      bodyCOM << param[0], param[1], param[2];
    Vec3<T> suspeCOM;
    if (nh->getParam("suspeCOM", param))
      suspeCOM << param[0], param[1], param[2];
    Vec3<T> wheelCOM;
    if (nh->getParam("wheelCOM", param))
      wheelCOM << param[0], param[1], param[2];
    Vec3<T> rotorCOM(0., 0., 0.);

    _bodyInertia = SpatialInertia<T>(_bodyMass, bodyCOM, bodyRotationalInertia);
    _suspeInertia = SpatialInertia<T>(_suspeMass, suspeCOM, suspeRotationalInertia);
    _wheelInertia = SpatialInertia<T>(_wheelMass, wheelCOM, wheelRotationalInertia);

    Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
    Mat3<T> rotorRotationalInertiaY =
        RX * rotorRotationalInertiaZ * RX.transpose();
    _suspeRotorInertia = SpatialInertia<T>(_rotorMass, rotorCOM, rotorRotationalInertiaY);
    _wheelRotorInertia = SpatialInertia<T>(_rotorMass, rotorCOM, rotorRotationalInertiaY);

    if (nh->getParam("suspeLocation", param))
      _suspeLocation << param[0], param[1], param[2];
    if (nh->getParam("wheelLocation", param))
      _wheelLocation << param[0], param[1], param[2];
    if (nh->getParam("wheelRotorLocation", param))
      _wheelRotorLocation << param[0], param[1], param[2];
    if (nh->getParam("rfidLocation", param))
      _rfidLocation << param[0], param[1], param[2];
  };
};

#endif //SUSPENSION_SIM_INCLUDE_CONFIG_H_
