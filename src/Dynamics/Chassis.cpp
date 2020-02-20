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
  Vec3<T> bodyDims(_params._bodyLength, _params._bodyWidth, _params._bodyHeight);
  model.addBase(_params._bodyInertia);
  // add contact for the chassis's body
  model.addGroundContactBoxPoints(5, bodyDims);
  const int baseID = 5;
  int bodyID = baseID;

  T ySideSign[4] = {1, -1, -1, 1};
  T xSideSign[4] = {1, 1, -1, -1};
  Mat3<T> I3 = Mat3<T>::Identity();

  // loop over 4 wheel
  for (int wheelID = 0; wheelID < 4; wheelID++) {

    // Suspe Joint
    bodyID++;
    Mat6<T> xtreeSuspe =
        createSXform(coordinateRotation<T>(CoordinateAxis::Z, T(M_PI)),
                     withWheelSigns<T>(_params._suspeLocation, wheelID));
    Mat6<T> xtreeSuspeRotor =
        createSXform(coordinateRotation<T>(CoordinateAxis::Z, T(M_PI)),
                     withWheelSigns<T>(_params._suspeLocation, wheelID));
    if (ySideSign[wheelID] < 0.) {
      if (xSideSign[wheelID] < 0.) {
        model.addBody(_params._suspeInertia.flipAlongAxis(CoordinateAxis::Y).flipAlongAxis(CoordinateAxis::X),
                      _params._suspeRotorInertia.flipAlongAxis(CoordinateAxis::Y).flipAlongAxis(CoordinateAxis::X),
                      1., baseID, JointType::Revolute,
                      CoordinateAxis::Y, xtreeSuspe, xtreeSuspeRotor);
      } else {
        model.addBody(_params._suspeInertia.flipAlongAxis(CoordinateAxis::Y),
                      _params._suspeRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                      1., baseID, JointType::Revolute,
                      CoordinateAxis::Y, xtreeSuspe, xtreeSuspeRotor);
      }
    } else {
      if (xSideSign[wheelID] < 0.) {
        model.addBody(_params._suspeInertia.flipAlongAxis(CoordinateAxis::X),
                      _params._suspeRotorInertia.flipAlongAxis(CoordinateAxis::X),
                      1., baseID, JointType::Revolute,
                      CoordinateAxis::Y, xtreeSuspe, xtreeSuspeRotor);
      } else {
        model.addBody(_params._suspeInertia, _params._suspeRotorInertia, 1., baseID,
                      JointType::Revolute, CoordinateAxis::Y, xtreeSuspe,
                      xtreeSuspeRotor);
      }
    }

    // Wheel Joint
    bodyID++;
    Mat6<T> xtreeKnee = createSXform(I3, withWheelSigns<T>(_params._wheelLocation, wheelID));
    Mat6<T> xtreeKneeRotor = createSXform(I3, withWheelSigns<T>(_params._wheelRotorLocation, wheelID));
    if (ySideSign[wheelID] < 0) {
      model.addBody(_params._wheelInertia.flipAlongAxis(CoordinateAxis::Y),
                    _params._wheelRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                    _params._wheelGearRatio, bodyID - 1, JointType::Revolute,
                    CoordinateAxis::Y, xtreeKnee, xtreeKneeRotor);
    } else {
      model.addBody(_params._wheelInertia, _params._wheelRotorInertia, _params._wheelGearRatio, bodyID - 1,
                    JointType::Revolute, CoordinateAxis::Y, xtreeKnee,
                    xtreeKneeRotor);
    }

    //add "foot" of wheel
    for (int i = 0; i < 16; ++i) {
      double angle = 2. * M_PI / 16. * i;
      model.addGroundContactPoint
          (bodyID,
           Vec3<T>(_params._wheelRadius * sin(angle), 0, _params._wheelRadius * cos(angle)),
           true);
    }
    //TODO add rfid contact point
  }

  Vec3<T> g(0, 0, -9.81);
  model.setGravity(g);

  return model;
}

/*!
 * Flip signs of elements of a vector V depending on which leg it belongs to
 */
template<typename T, typename T2>
Vec3<T> withWheelSigns(const Eigen::MatrixBase<T2> &v, int wheelID) {
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 matrix");
  switch (wheelID) {
    case 0:return Vec3<T>(v[0], v[1], v[2]);
    case 1:return Vec3<T>(v[0], -v[1], v[2]);
    case 2:return Vec3<T>(-v[0], -v[1], v[2]);
    case 3:return Vec3<T>(-v[0], v[1], v[2]);
    default:throw std::runtime_error("Invalid wheel id!");
  }
}

template<typename T>
std::vector<ActuatorModel<T>> Chassis<T>::buildActuatorModels() {
  std::vector<ActuatorModel<T>> models;
  models.emplace_back(_params._wheelGearRatio, _params._motorKT, _params._motorR, _params._batteryV,
                      _params._jointDamping, _params._jointDryFriction, _params._motorTauMax);

  return models;
}

template
class Chassis<double>;
template
class Chassis<float>;
