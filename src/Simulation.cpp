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
  printf("[Simulation] Init ROS...\n");
  markerPub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  marker_.header.frame_id = "/map";
  marker_.header.stamp = ros::Time::now();
  marker_.ns = "basic_shapes";
  while (markerPub_.getNumSubscribers() < 1 && ros::ok()) {
    ROS_INFO_ONCE("[Simulation] Wait for subscriber to the marker\n");
    sleep(1);
  }
  twistPub_ = nh_.advertise<geometry_msgs::Twist>("base_twist", 100);
  jointPub_ = nh_.advertise<rm_suspension::JointData>("joint_data", 100);

  // init parameters
  printf("[Simulation] Load parameters...\n");
  simParams_.getParam(&nh_);
  fakeSuspe_.setParams(&nh_);
  chassis_._params.getParam(&nh_);
  // init chassis info
  printf("[Simulation] Build chassis...\n");
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

  setupState_.q = zero8;
  setupState_.qd = zero8;
  setupState_.bodyOrientation = rotationMatrixToQuaternion(
      ori::coordinateRotation(CoordinateAxis::Z, 0.0));
  setupState_.q = zero8;
  setupState_.qd = zero8;
  for (int wheelID = 0; wheelID < 4; ++wheelID) {
    setupState_.q[wheelID * 2] = fakeSuspe_.getSetupAngle(wheelID);
  }
  tau_ = zero8;
  printf("[Simulation] Ready!\n");
}

void Simulation::step(double dt, double dtControl) {
  //Fake suspension
  for (int wheel = 0; wheel < 4; ++wheel) {
    suspeData_.q_[wheel] = simulator_->getState().q[wheel * 2];
    suspeData_.qd_[wheel] = simulator_->getState().qd[wheel * 2];
  }
  fakeSuspe_.update(suspeData_);

  for (int wheel = 0; wheel < 4; ++wheel) {
    tau_[wheel * 2] = fakeSuspe_.torque_out_[wheel];
  }

  // Low level control
  if (currentSimTime_ >= timeOfNextLControl_) {
    double qd[4] = {simulator_->getState().qd[1],
                    simulator_->getState().qd[3],
                    simulator_->getState().qd[5],
                    simulator_->getState().qd[7]};
    controller_.update(qd, currentSimTime_);
    timeOfNextLControl_ = timeOfNextLControl_ + dtControl;
  }
  // actuator model: only for wheel acautor
  for (int wheelID = 0; wheelID < 4; wheelID++) {
    tau_[wheelID * 2 + 1] = actuatorModels_[0].getTorque(
        controller_.torque_out_[wheelID],
        simulator_->getState().qd[wheelID * 2 + 1]);
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
  marker_.color.a = .8f;
  marker_.lifetime = ros::Duration(0);
  markerPub_.publish(marker_);
}

/*!
 * Add an box collision to the simulator
 * @param mu          : friction of the mu
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
  marker_.id = ++markerId_;
  marker_.type = visualization_msgs::Marker::CUBE;
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.pose.position.x = pos.x();
  marker_.pose.position.y = pos.y();
  marker_.pose.position.z = pos.z();
  Quat<double> quat = rotationMatrixToQuaternion(ori);
  marker_.pose.orientation.x = quat[1];
  marker_.pose.orientation.y = quat[2];
  marker_.pose.orientation.z = quat[3];
  marker_.pose.orientation.w = quat[0];

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker_.scale.x = depth;
  marker_.scale.y = width;
  marker_.scale.z = height;
  // Set the color -- be sure to set alpha to something non-zero!
  marker_.color.r = 1.0f;
  marker_.color.g = 1.0f;
  marker_.color.b = 1.0f;
  marker_.color.a = 1.0;
  marker_.lifetime = ros::Duration(0);
  markerPub_.publish(marker_);
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
  * @param
 */
void Simulation::runForTime(double time) {
  visData_.clear();
  resetSimTime();
  while (currentSimTime_ < time && ros::ok()) {
    step(simParams_.dynamics_dt_, simParams_.control_dt_);
    if (currentSimTime_ >= timeOfRecord_) {
      record();
      timeOfRecord_ = currentSimTime_ + 1. / simParams_.vis_fps_;
    }
    if (ros::Time::now().toSec() >= timeOfPrint_) {
      printf("\r[Simulation] Computing... %.3lf%%", currentSimTime_ / time * 100.);
      fflush(stdout); //for console in clion
      timeOfPrint_ = ros::Time::now().toSec() + .2;
    }
  }
}

void Simulation::setSpeed(double speed) {
  controller_.setSpeed(speed / chassis_._params._wheelRadius);
}

void Simulation::record() {
  VisData vis_data;
  /////////////////////////////////record TF Data///////////////////////////////////
  vis_data.tfPos[0] = simulator_->getState().bodyPosition;
  vis_data.tfQuat[0] = rotationMatrixToQuaternion(model_.getOrientation(5).transpose());
  for (int wheelID = 0; wheelID < 4; ++wheelID) {
    vis_data.tfQuat[wheelID * 2 + 1] =
        rotationMatrixToQuaternion(model_.getOrientation(wheelID * 2 + 6).transpose()); // note!! need tranpose!!!
    vis_data.tfPos[wheelID * 2 + 1] = model_.getPosition(wheelID * 2 + 6);
    vis_data.tfQuat[wheelID * 2 + 2] =
        rotationMatrixToQuaternion(model_.getOrientation(wheelID * 2 + 7).transpose());// note!! need tranpose!!!
    vis_data.tfPos[wheelID * 2 + 2] = model_.getPosition(wheelID * 2 + 7);
  }
  //////////////////////////record contact force data///////////////////////////////
  int _nTotalGC = simulator_->getTotalNumGC();
  int count = 0;
  for (size_t i(0); i < _nTotalGC; ++i) {
    Vec3<double> f = simulator_->getContactForce(i);
    if (f.norm() > 0.1) {
      vis_data.cpForce.push_back(f);
      vis_data.cpPos.push_back(simulator_->getModel()._pGC[i]);
      count++;
    }
  }
  vis_data.jointData.cp_count = count;
  ///////////////////////////////record base twist and joint data for plot//////////////////////////
  vis_data.baseMsg.linear.x = vis_data.tfPos[0].x();
  vis_data.baseMsg.linear.y = vis_data.tfPos[0].y();
  vis_data.baseMsg.linear.z = vis_data.tfPos[0].z();
  Vec3<double> rpy = quatToRPY(simulator_->getState().bodyOrientation);
  vis_data.baseMsg.angular.x = rpy.x();
  vis_data.baseMsg.angular.y = rpy.y();
  vis_data.baseMsg.angular.z = rpy.z();
  for (int wheelID = 0; wheelID < 4; ++wheelID) {
    vis_data.jointData.q_suspe[wheelID] = fakeSuspe_.getSpringLength(wheelID);
    vis_data.jointData.qd_suspe[wheelID] = simulator_->getState().qd[wheelID * 2];
    vis_data.jointData.tau_suspe[wheelID] = tau_[wheelID * 2];
    vis_data.jointData.qd_wheel[wheelID] = simulator_->getState().qd[wheelID * 2 + 1];
  }

  visData_.push_back(vis_data);
}

void Simulation::play(double scale) {
  printf("\n[Simulation] Start play!\n");
  auto iter = visData_.begin();
  ros::Rate loop_rate(simParams_.vis_fps_ / scale);
  while (ros::ok() && iter != visData_.end()) {
    sendTf(iter);
    sendCP(iter, scale);
    sendMsg(iter);
    ++iter;
    loop_rate.sleep();
  }
}

void Simulation::sendTf(const vector<VisData>::iterator &iter) {
  tf::Quaternion quat_tf;
  tf::Transform tf;
  std::string frame;

  ros::Time now = ros::Time::now();
  quat_tf.setValue(iter->tfQuat[0][1],
                   iter->tfQuat[0][2],
                   iter->tfQuat[0][3],
                   iter->tfQuat[0][0]);
  tf.setRotation(quat_tf);
  tf.setOrigin(tf::Vector3(iter->tfPos[0].x(), iter->tfPos[0].y(), iter->tfPos[0].z()));
  br_.sendTransform(tf::StampedTransform(tf, now, "map", "base_link"));
  for (int wheelID = 0; wheelID < 4; ++wheelID) {
    quat_tf.setValue(iter->tfQuat[wheelID * 2 + 1][1],
                     iter->tfQuat[wheelID * 2 + 1][2],
                     iter->tfQuat[wheelID * 2 + 1][3],
                     iter->tfQuat[wheelID * 2 + 1][0]);
    tf.setRotation(quat_tf);
    tf.setOrigin(tf::Vector3(iter->tfPos[wheelID * 2 + 1].x(),
                             iter->tfPos[wheelID * 2 + 1].y(),
                             iter->tfPos[wheelID * 2 + 1].z()));
    frame = "suspension_";
    frame += std::to_string(wheelID);
    br_.sendTransform(tf::StampedTransform(tf, now, "map", frame));

    quat_tf.setValue(iter->tfQuat[wheelID * 2 + 2][1],
                     iter->tfQuat[wheelID * 2 + 2][2],
                     iter->tfQuat[wheelID * 2 + 2][3],
                     iter->tfQuat[wheelID * 2 + 2][0]);
    tf.setRotation(quat_tf);
    tf.setOrigin(tf::Vector3(iter->tfPos[wheelID * 2 + 2].x(),
                             iter->tfPos[wheelID * 2 + 2].y(),
                             iter->tfPos[wheelID * 2 + 2].z()));
    frame = "wheel_";
    frame += std::to_string(wheelID);
    br_.sendTransform(tf::StampedTransform(tf, now, "map", frame));
  }
}

void Simulation::sendCP(const vector<VisData>::iterator &iter, double scale) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "my_namespace";
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.g = 1.0;
  marker.scale.x = .01;
  marker.pose.orientation.w = 1;
  marker.id = 5; //large enough to avoid cover marker create at set up
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(1. / (simParams_.vis_fps_ / scale));

  auto cpP = iter->cpPos.begin();
  auto cpF = iter->cpForce.begin();

  while (cpP != iter->cpPos.end()) {
    geometry_msgs::Point p;
    p.x = cpP->x();
    p.y = cpP->y();
    p.z = cpP->z();
    marker.points.push_back(p);
    p.x = cpP->x() + 0.005 * cpF->x();
    p.y = cpP->y() + 0.005 * cpF->y();
    p.z = cpP->z() + 0.005 * cpF->z();
    marker.points.push_back(p);
    markerPub_.publish(marker);
    cpP++;
    cpF++;
    marker.points.clear();
    marker.id++;
  }
}

void Simulation::sendMsg(const vector<VisData>::iterator &iter) {
  twistPub_.publish(iter->baseMsg);
  jointPub_.publish(iter->jointData);
}
void Simulation::clearCollision() {
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  markerPub_.publish(marker);

  markerId_ = 0;
  simulator_->deleteAllCollision();
}

