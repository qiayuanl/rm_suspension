//
// Created by qiayuan on 2/14/20.
//

#include "Simulation.h"

/*!
 * Initialize the simulator here.  It is _not_ okay to block here waiting for
 * the robot to connect. Use firstRun() instead!
 */
Simulation::Simulation(ChassisType type)
    : tau_(8) {
  // init ROS
  ROS_INFO("[Simulation] Init ROS...\n");
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  marker_.header.frame_id = "/map";
  marker_.header.stamp = ros::Time::now();
  marker_.ns = "basic_shapes";
//  while (marker_pub_.getNumSubscribers() < 1 && ros::ok()) {
//    ROS_INFO_ONCE("[Simulation]Wait for subscriber to the marker");
//    sleep(1);
//  }
  // init parameters
  ROS_INFO("[Simulation] Load parameters...\n");
  simParams_.getParam(&nh_);
  // init chassis info
  ROS_INFO("[Simulation] Build chassis...\n");
  type_ = type;
  chassis_ = buildStandardChassis<double>();
  printf("[Simulation] Build actuator model...\n");
  actuatorModels_ = chassis_.buildActuatorModels();

  // init rigid body dynamics
  printf("[Simulation] Build rigid body model...\n");

  model_ = chassis_.buildModel();
  simulator_ =
      new DynamicsSimulator<double>(model_, simParams_.use_spring_damper_);

  DVec<double> zero8(8);
  for (u32 i = 0; i < 8; i++) {
    zero8[i] = 0.;
  }

  // set some sane defaults:
  tau_ = zero8;
  jointState_.q = zero8;
  jointState_.qd = zero8;
  FBModelState<double> x0;
  x0.bodyOrientation = rotationMatrixToQuaternion(
      ori::coordinateRotation(CoordinateAxis::Z, 0.0));
  x0.bodyPosition.setZero();
  x0.bodyVelocity.setZero();
  x0.q = zero8;
  x0.qd = zero8;

  x0.bodyPosition[2] = 0.2;
  setRobotState(x0);
  addCollisionPlane(0.5, 0., 0.);
  printf("[Simulation] Ready!\n");

}

void Simulation::step(double dt, double dtControl) {
  //Fake suspension
  for (int wheel = 0; wheel < 4; ++wheel) {
    suspe_data_.q_[wheel] = simulator_->getState().q[wheel * 2];
    suspe_data_.qd_[wheel] = simulator_->getState().qd[wheel * 2];
  }
  fake_suspe_.update(suspe_data_);
  for (int wheel = 0; wheel < 4; ++wheel) {
    tau_[wheel * 2] = fake_suspe_.torque_out[wheel];
  }
//  tau_[1] = 0.3;
//  tau_[3] = 0.3;
//  tau_[5] = 0.3;
//  tau_[7] = 0.3;
//

  // Low level control (if needed)
  if (currentSimTime_ >= timeOfNextLControl_) {
    timeOfNextLControl_ = timeOfNextLControl_ + dtControl;
  }

  // actuator model:
//  if (chassis_._chassisType == ChassisType::STANDARD) {
//    for (int wheel = 0; wheel < 4; wheel++) {
//      tau_[wheel] = 0;
//    }
//  } else {
//    assert(false);
//  }
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
                                   double sizeX, double sizeY) {
  simulator_->addCollisionPlane(mu, resti, height);
  marker_.id = 0;
  marker_.type = visualization_msgs::Marker::CUBE;
  marker_.action = visualization_msgs::Marker::ADD;

  marker_.pose.position.z = -0.001;
  marker_.pose.orientation.x = 0.0;
  marker_.pose.orientation.y = 0.0;
  marker_.pose.orientation.z = 0.0;
  marker_.pose.orientation.w = 1.0;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker_.scale.x = sizeX;
  marker_.scale.y = sizeY;
  marker_.scale.z = 0.001;
  // Set the color -- be sure to set alpha to something non-zero!
  marker_.color.r = 1.0f;
  marker_.color.g = 1.0f;
  marker_.color.b = 1.0f;
  marker_.color.a = 0.5;
  marker_.lifetime = ros::Duration(0);
  marker_pub_.publish(marker_);
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
                                 const Mat3<double> &ori) {
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
 * Runs the simulator for time xxx
 *
 * Updates graphics at 60 fps.
 * @param
 */
void Simulation::runForTime(double time) {
  while (currentSimTime_ < time && ros::ok()) {
    step(simParams_.dynamics_dt_, simParams_.control_dt_);
    if (currentSimTime_ >= timeOfVis_) {
      updateVis();
      timeOfVis_ = currentSimTime_ + 1 / simParams_.vis_fps_;
    }
  }
}

void Simulation::updateVis() {

  Quat<double> quat;
  Vec3<double> pos;
  size_t s = sizeof(pos);
  tf::Quaternion quat_tf;
  tf::Transform tf;
  visTime_ = visTime_ + ros::Duration(1 / simParams_.vis_fps_);
  std::string frame;

  quat = rotationMatrixToQuaternion(model_.getOrientation(5));
  quat_tf.setValue(quat[1], quat[2], quat[3], quat[0]);
  tf.setRotation(quat_tf);
  pos = model_.getPosition(5);
  tf.setOrigin(tf::Vector3(pos.x(), pos.y(), pos.z()));
  br_.sendTransform(tf::StampedTransform(tf, visTime_, "map", "base_link"));

  for (int i = 0; i < 4; ++i) {
    quat = rotationMatrixToQuaternion(model_.getOrientation(i * 2 + 6));
    quat_tf.setValue(quat[1], quat[2], quat[3], quat[0]);
    tf.setRotation(quat_tf);
    pos = model_.getPosition(i * 2 + 6);
    tf.setOrigin(tf::Vector3(pos.x(), pos.y(), pos.z()));
    frame = "suspension_";
    frame += std::to_string(i);
    br_.sendTransform(tf::StampedTransform(tf, visTime_, "map", frame));

    quat = rotationMatrixToQuaternion(model_.getOrientation(i * 2 + 7));
    quat_tf.setValue(quat[1], quat[2], quat[3], quat[0]);
    tf.setRotation(quat_tf);
    pos = model_.getPosition(i * 2 + 7);
    tf.setOrigin(tf::Vector3(pos.x(), pos.y(), pos.z()));
    frame = "wheel_";
    frame += std::to_string(i);
    br_.sendTransform(tf::StampedTransform(tf, visTime_, "map", frame));
  }
}
