//
// Created by qiayuan on 2/14/20.
//

#ifndef SUSPENSION_SIM_INCLUDE_DYNAMICS_STANDARD_CHASSIS_H_
#define SUSPENSION_SIM_INCLUDE_DYNAMICS_STANDARD_CHASSIS_H_

#include "FloatingBaseModel.h"
#include "Chassis.h"
template<typename T>
Chassis<T> buildStandardChassis() {
  Chassis<T> chassis;
  chassis._chassisType = ChassisType::STANDARD;

  chassis._bodyMass = 3.3;
  chassis._bodyLength = 0.2 * 2;
  chassis._bodyWidth = 0.2 * 2;
  chassis._bodyHeight = 0.05 * 2;
  chassis._suspeLinkLength = 0.2;
  chassis._wheelRadius = 0.06;
  chassis._wheelGearRatio = 1;

  chassis._motorTauMax = 3.f;
  chassis._batteryV = 24;
  chassis._motorKT = .05;  // this is flux linkage * pole pairs
  chassis._motorR = 0.173;
  chassis._jointDamping = .01;
  chassis._jointDryFriction = .2;

  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias

  Mat3<T> suspeRotationalInertia;
  suspeRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
  suspeRotationalInertia = suspeRotationalInertia * 1e-6;
  Vec3<T> suspeCOM(0, 0.016, -0.02);
  SpatialInertia<T> suspeInertia(0.634, suspeCOM, suspeRotationalInertia);

  Mat3<T> wheelRotationalInertia, wheelRotationalInertiaRotated;
  wheelRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
  wheelRotationalInertiaRotated = wheelRotationalInertiaRotated * 1e-6;
  wheelRotationalInertia = RY * wheelRotationalInertia * RY.transpose();
  Vec3<T> wheelCOM(0, 0, 0);
  SpatialInertia<T> wheelInertia(0.064, wheelCOM, wheelRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0, 0, 0);
  SpatialInertia<T> bodyInertia(chassis._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  chassis._suspeInertia = suspeInertia;
  chassis._suspeRotorInertia = rotorInertiaY;
  chassis._wheelInertia = wheelInertia;
  chassis._wheelRotorInertia = rotorInertiaY;
  chassis._bodyInertia = bodyInertia;

  // locations
  chassis._suspeLocation = Vec3<T>(chassis._bodyLength, chassis._bodyWidth, 0);
  chassis._suspeRotorLocation = chassis._suspeLocation;
  chassis._wheelLocation = Vec3<T>(-chassis._suspeLinkLength, 0, 0);
  chassis._wheelRotorLocation = chassis._wheelLocation;

  return chassis;
}

#endif //SUSPENSION_SIM_INCLUDE_DYNAMICS_STANDARD_CHASSIS_H_
