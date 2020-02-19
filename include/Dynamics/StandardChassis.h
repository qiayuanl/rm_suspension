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

  chassis._bodyMass = 12.;
  chassis._bodyLength = 0.58;
  chassis._bodyWidth = 0.52;
  chassis._bodyHeight = 0.1;
  chassis._suspeLinkLength = 0.2;
  chassis._wheelRadius = 0.075;
  chassis._wheelGearRatio = 19;

  chassis._motorTauMax = .15789f; //rotor
  chassis._batteryV = 24;
  chassis._motorKT = .005;  // this is flux linkage * pole pairs
  chassis._motorR = 0.194;
  chassis._jointDamping = .01;
  chassis._jointDryFriction = .1;

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

  // spatial inertia

  Mat3<T> suspeRotationalInertia;
  suspeRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
  suspeRotationalInertia = suspeRotationalInertia * 1e-6;
  Vec3<T> suspeCOM(0, 0.016, -0.02);
  SpatialInertia<T> suspeInertia(0.634, suspeCOM, suspeRotationalInertia);

  Mat3<T> wheelRotationalInertia;
  wheelRotationalInertia << 1.6338e-05, 0, 0, 0, 1.2822e-05, 0, 0, 0, 1.6338e-05;
  Vec3<T> wheelCOM(0, 0, 0);
  SpatialInertia<T> wheelInertia(0.146, wheelCOM, wheelRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);


//  Mat3<T> bodyRotationalInertia;
//  bodyRotationalInertia << 0.066958, 0, 0, 0, 0.087692, 0, 0, 0, 0.078993;
  Vec3<T> bodyCOM(0, 0, 0);
  Vec3<T> bodyDims(chassis._bodyLength, chassis._bodyWidth,
                   chassis._bodyHeight);
  SpatialInertia<T> bodyInertia(chassis._bodyMass, bodyCOM,
                                rotInertiaOfBox(chassis._bodyMass, bodyDims));

  chassis._suspeInertia = suspeInertia;
  chassis._suspeRotorInertia = rotorInertiaY;
  chassis._wheelInertia = wheelInertia;
  chassis._wheelRotorInertia = rotorInertiaY;
  chassis._rollerInertia = SpatialInertia<T>(0.005, rotorCOM, rotorRotationalInertiaY * 0.01);
  chassis._rollerRotorInertia = SpatialInertia<T>(0.005, rotorCOM, rotorRotationalInertiaY * 0.01);

  chassis._bodyInertia = bodyInertia;

  // locations
  chassis._suspeLocation = Vec3<T>(0.083, 0.172, -0.025);
  chassis._suspeRotorLocation = chassis._suspeLocation;
  chassis._wheelLocation = Vec3<T>(-0.11135, -0.0324, 0);
  chassis._wheelRotorLocation = chassis._wheelLocation;

  return chassis;
}

#endif //SUSPENSION_SIM_INCLUDE_DYNAMICS_STANDARD_CHASSIS_H_
