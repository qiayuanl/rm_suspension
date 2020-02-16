//
// Created by qiayuan on 2/13/20.
//

#ifndef SUSPENSION_SIM_INCLUDE_DYNAMICS_CHASSIS_H_
#define SUSPENSION_SIM_INCLUDE_DYNAMICS_CHASSIS_H_
#include "Dynamics/ActuatorModel.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/SpatialInertia.h"

#include <eigen3/Eigen/StdVector>

#include <vector>

using std::vector;

/*!
 * Representation of a chassis robot's physical properties.
 *
 * When viewed from the top, the chassis's wheel are:
 *
 * FRONT
 * 2 1   RIGHT
 * 3 4
 * BACK
 *
 */
template<typename T>
class Chassis {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ChassisType _chassisType;
  T _bodyLength, _bodyWidth, _bodyHeight, _bodyMass;
  T _suspeLinkLength, _wheelRadius;
  T _wheelGearRatio;
  T _motorKT, _motorR, _batteryV;
  T _motorTauMax;
  T _jointDamping, _jointDryFriction;
  SpatialInertia<T> _bodyInertia, _suspeInertia, _wheelInertia, _suspeRotorInertia, _wheelRotorInertia;

  Vec3<T> _suspeLocation, _suspeRotorLocation, _wheelLocation, _wheelRotorLocation;
  FloatingBaseModel<T> buildModel();
  std::vector<ActuatorModel<T>> buildActuatorModels();
};

template<typename T, typename T2>
Vec3<T> withLegSigns(const Eigen::MatrixBase<T2> &v, int legID);

#endif //SUSPENSION_SIM_INCLUDE_DYNAMICS_CHASSIS_H_
