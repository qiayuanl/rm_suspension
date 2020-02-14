//
// Created by qiayuan on 2/13/20.
//
#include "Dynamics/Chassis.h"
#include "Dynamics/spatial.h"
#include "Math/orientation_tools.h"

using namespace ori;
using namespace spatial;

/*!
 * Build a FloatingBaseModel of the chassis
 */
template<typename T>
FloatingBaseModel<T> Chassis<T>::buildModel() {
  FloatingBaseModel<T> model;

  // we assume the Chassis's body can be modeled as a uniformly distributed box.
  Vec3<T> bodyDims(_bodyLength, _bodyWidth, _bodyHeight);
  model.addBase(_bodyInertia);
  // add contact for the cheetah's body
  model.addGroundContactBoxPoints(5, bodyDims);

  Mat3<T> I3 = Mat3<T>::Identity();
  int bodyID = 6;
  //Adab Joint
  Mat6<T> xtreeAbad = createSXform(I3, withLegSigns<T>(_abadLocation, 1));
  Mat6<T> xtreeAbadRotor =
      createSXform(I3, _abadRotorLocation);

  model.addBody(_abadInertia.flipAlongAxis(CoordinateAxis::Y),
                _abadRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                _abadGearRatio, 5, JointType::Revolute,
                CoordinateAxis::X, xtreeAbad, xtreeAbadRotor);

  // Hip Joint
  bodyID++;
  Mat6<T> xtreeHip =
      createSXform(coordinateRotation<T>(CoordinateAxis::Z, T(M_PI)),
                   _hipLocation);
  Mat6<T> xtreeHipRotor =
      createSXform(coordinateRotation<T>(CoordinateAxis::Z, T(M_PI)),
                   _hipRotorLocation);
  model.addBody(_hipInertia.flipAlongAxis(CoordinateAxis::Y),
                _hipRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                _hipGearRatio, bodyID - 1, JointType::Revolute,
                CoordinateAxis::Y, xtreeHip, xtreeHipRotor);
  Vec3<T> g(0, 0, -9.81);
  model.setGravity(g);

  return model;
}

/*!
 * Flip signs of elements of a vector V depending on which leg it belongs to
 */
template<typename T, typename T2>
Vec3<T> withLegSigns(const Eigen::MatrixBase<T2> &v, int legID) {
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 matrix");
  switch (legID) {
    case 0:return Vec3<T>(v[0], -v[1], v[2]);
    case 1:return Vec3<T>(v[0], v[1], v[2]);
    case 2:return Vec3<T>(-v[0], -v[1], v[2]);
    case 3:return Vec3<T>(-v[0], v[1], v[2]);
    default:throw std::runtime_error("Invalid leg id!");
  }
}

template<typename T>
std::vector<ActuatorModel<T>> Chassis<T>::buildActuatorModels() {
  std::vector<ActuatorModel<T>> models;
  models.emplace_back(_abadGearRatio, _motorKT, _motorR, _batteryV,
                      _jointDamping, _jointDryFriction, _motorTauMax);
  models.emplace_back(_hipGearRatio, _motorKT, _motorR, _batteryV,
                      _jointDamping, _jointDryFriction, _motorTauMax);
  models.emplace_back(_kneeGearRatio, _motorKT, _motorR, _batteryV,
                      _jointDamping, _jointDryFriction, _motorTauMax);
  return models;
}

template
class Chassis<double>;
template
class Chassis<float>;
