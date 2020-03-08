//
// Created by qiayuan on 2/14/20.
//

#ifndef SUSPENSION_SIM_INCLUDE_SIMULATION_H_
#define SUSPENSION_SIM_INCLUDE_SIMULATION_H_

#include "Dynamics/Chassis.h"
#include "Dynamics/DynamicsSimulator.h"
#include "Utilities/FakeSuspe.h"
#include "Utilities/Controller.h"
#include "config.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <rosbag/bag.h>
#include <geometry_msgs/Twist.h>
#include <rm_suspension/SuspeData.h>

struct VisData {
  Quat<double> tfQuat[9];
  Vec3<double> tfPos[9];
  vector<Vec3<double >> cpPos;
  vector<Vec3<double>> cpForce;
  geometry_msgs::Twist baseMsg;
  rm_suspension::SuspeData suspeData;
};

/*!
 * Top-level control of a simulation.
 * It does not include the graphics window
 */

class Simulation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit Simulation();

  /*!
   * Explicitly set the state of the robot
   */
  void setRobotState(FBModelState<double> &state) {
    simulator_->setState(state);
  }
  void step(double dt, double dtControl);
  void addCollisionPlane(double mu, double resti, double height,
                         double sizeX = 20, double sizeY = 20);
  void addCollisionBox(double mu, double resti, double depth, double width,
                       double height, const Vec3<double> &pos,
                       const Mat3<double> &ori);
  void addCollisionMesh(double mu, double resti, double grid_size,
                        const Vec3<double> &left_corner_loc,
                        const DMat<double> &height_map, bool addToWindow = true,
                        bool transparent = true);
  void clearCollision();

  void setSpeed(double speed);
  void runForTime(double time);
  void play(double scale);
  void resetSimTime() {
    currentSimTime_ = 0.;
    timeOfNextLControl_ = 0.;
    timeOfPrint_ = 0;
    timeOfRecord_ = 0;
  }

  ~Simulation() {
    delete simulator_;
  }
  inline void setFlyRampSpeed(double speed) { flyRampSpeed_ = speed; }
  inline double getFlyRampSpeed() { return flyRampSpeed_; }

  FBModelState<double> setupState_;
 private:
  ros::NodeHandle nh_;
  ros::Publisher markerPub_;
  ros::Publisher twistPub_;
  ros::Publisher suspePub_;
  visualization_msgs::Marker marker_;
  tf::TransformBroadcaster br_;

  Chassis<double> chassis_;
  FloatingBaseModel<double> model_;
  DVec<double> tau_;
  DynamicsSimulator<double> *simulator_ = nullptr;
  std::vector<ActuatorModel<double>> actuatorModels_;
  SimParameters simParams_;

  FakeSuspe fakeSuspe_;
  SuspeData suspeData_{};
  Controller controller_;
  vector<VisData> visData_;

  size_t markerId_{};
  double currentSimTime_{};
  double timeOfNextLControl_{};
  double timeOfRecord_{};
  double timeOfPrint_{};
  double flyRampSpeed_{};

  void record();
  void sendTf(const vector<VisData>::iterator &iter);
  void sendCP(const vector<VisData>::iterator &iter, double scale);
  void sendMsg(const vector<VisData>::iterator &iter);
};

#endif //SUSPENSION_SIM_INCLUDE_SIMULATION_H_
