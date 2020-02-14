//
// Created by qiayuan on 2/14/20.
//

#include "Simulation.h"

int main() {
  Simulation simulation(ChassisType::STANDARD);
  simulation.runForTime(3);

  return true;
}

/*!
 * Initialize the simulator here.  It is _not_ okay to block here waiting for
 * the robot to connect. Use firstRun() instead!
 */
Simulation::Simulation(ChassisType type)
    : tau_(2) {
  // init parameters
  printf("[Simulation] Load parameters...\n");

  // init quadruped info
  printf("[Simulation] Build chassis...\n");
  type_ = type;
  chassis_ = buildStandardChassis<double>();
  printf("[Simulation] Build actuator model...\n");
  actuatorModels_ = chassis_.buildActuatorModels();

  // init rigid body dynamics
  printf("[Simulation] Build rigid body model...\n");

  model_ = chassis_.buildModel();
  simulator_ =
      new DynamicsSimulator<double>(model_, simParams_.use_spring_damper);

  DVec<double> zero2(2);
  for (u32 i = 0; i < 1; i++) {
    zero2[i] = 0.;
  }

  // set some sane defaults:
  tau_ = zero2;
  jointState_.q = zero2;
  jointState_.qd = zero2;
  FBModelState<double> x0;
  x0.bodyOrientation = rotationMatrixToQuaternion(
      ori::coordinateRotation(CoordinateAxis::Z, 0.));

  x0.bodyPosition.setZero();
  x0.bodyVelocity.setZero();
  x0.q = zero2;
  x0.qd = zero2;

  x0.bodyPosition[2] = 1.2;
  setRobotState(x0);

  simulator_->addCollisionPlane(0.7, 0, 0);
  printf("[Simulation] Ready!\n");
}

void Simulation::step(double dt, double dtControl) {
  // Low level control (if needed)
  if (currentSimTime_ >= timeOfNextLControl_) {
    timeOfNextLControl_ = timeOfNextLControl_ + dtControl;
  }

  // actuator model:
  if (chassis_._chassisType == ChassisType::STANDARD) {
  } else {
    assert(false);
  }

  // dynamics
  currentSimTime_ += dt;

  // Set Homing Information
  RobotHomingInfo<double> homing;
  homing.active_flag = simParams_.go_home_;
  homing.position = simParams_.home_pos_;
  homing.rpy = simParams_.home_rpy_;
  homing.kp_lin = simParams_.home_kp_lin_;
  homing.kd_lin = simParams_.home_kd_lin_;
  homing.kp_ang = simParams_.home_kp_ang_;
  homing.kd_ang = simParams_.home_kd_ang_;
  simulator_->setHoming(homing);

  simulator_->step(dt, tau_, simParams_.floor_kp_, simParams_.floor_kd_);
}

/*!
 * Add an infinite collision plane to the simulator
 * @param mu          : friction of the plane
 * @param resti       : restitution coefficient
 * @param height      : height of plane
 * @param addToWindow : if true, also adds graphics for the plane
 */
void Simulation::addCollisionPlane(double mu, double resti, double height,
                                   double sizeX, double sizeY, double checkerX,
                                   double checkerY, bool addToWindow) {
  simulator_->addCollisionPlane(mu, resti, height);
}

/*!
 * Add an box collision to the simulator
 * @param mu          : location of the box
 * @param resti       : restitution coefficient
 * @param depth       : depth (x) of box
 * @param width       : width (y) of box
 * @param height      : height (z) of box
 * @param pos         : position of box
 * @param ori         : orientation of box
 * @param addToWindow : if true, also adds graphics for the plane
 */
void Simulation::addCollisionBox(double mu, double resti, double depth,
                                 double width, double height,
                                 const Vec3<double> &pos,
                                 const Mat3<double> &ori, bool addToWindow,
                                 bool transparent) {
  simulator_->addCollisionBox(mu, resti, depth, width, height, pos, ori);
}

void Simulation::addCollisionMesh(double mu, double resti, double grid_size,
                                  const Vec3<double> &left_corner_loc,
                                  const DMat<double> &height_map,
                                  bool addToWindow, bool transparent) {
  simulator_->addCollisionMesh(mu, resti, grid_size, left_corner_loc,
                               height_map);
}

/*!
 * Runs the simulator for time the _running variable is set
 * to false. Updates graphics at 60 fps.
 * @param
 */
void Simulation::runForTime(double time) {
  while (currentSimTime_ < time) {
    step(simParams_.dynamics_dt_, simParams_.control_dt_);
  }
}
