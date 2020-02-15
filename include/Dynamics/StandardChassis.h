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
  chassis._bodyLength = 0.19 * 2;
  chassis._bodyWidth = 0.19 * 2;
  chassis._bodyHeight = 0.05 * 2;
  chassis._abadGearRatio = 6;
  chassis._hipGearRatio = 6;
  chassis._kneeGearRatio = 9.33;
  chassis._abadLinkLength = 0.062;
  chassis._hipLinkLength = 0.209;
  chassis._kneeLinkLength = 0.175;
  chassis._maxLegLength = 0.384;

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
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(0, 0.036, 0);  // LEFT
  SpatialInertia<T> abadInertia(0.54, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(0, 0.016, -0.02);
  SpatialInertia<T> hipInertia(0.634, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(0, 0, -0.061);
  SpatialInertia<T> kneeInertia(0.064, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0, 0, 0);
  SpatialInertia<T> bodyInertia(chassis._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  chassis._abadInertia = abadInertia;
  chassis._hipInertia = hipInertia;
  chassis._kneeInertia = kneeInertia;
  chassis._abadRotorInertia = rotorInertiaX;
  chassis._hipRotorInertia = rotorInertiaY;
  chassis._kneeRotorInertia = rotorInertiaY;
  chassis._bodyInertia = bodyInertia;

  // locations
  chassis._abadRotorLocation = Vec3<T>(0.125, 0.049, 0);
  chassis._abadLocation =
      Vec3<T>(chassis._bodyLength, chassis._bodyWidth, 0) * 0.5;
  chassis._hipLocation = Vec3<T>(0, chassis._abadLinkLength, 0);
  chassis._hipRotorLocation = Vec3<T>(0, 0.04, 0);
  chassis._kneeLocation = Vec3<T>(0, 0, -chassis._hipLinkLength);
  chassis._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return chassis;
}

#endif //SUSPENSION_SIM_INCLUDE_DYNAMICS_STANDARD_CHASSIS_H_
