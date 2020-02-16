//
// Created by qiayuan on 2/14/20.
//

#ifndef SUSPENSION_SIM_INCLUDE_SIMULATION_H_
#define SUSPENSION_SIM_INCLUDE_SIMULATION_H_

#include "Dynamics/Chassis.h"
#include "Dynamics/DynamicsSimulator.h"
#include "Dynamics/StandardChassis.h"
#include "config.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <rosbag/bag.h>
#include <Utilities/FakeSuspe.h>

/*!
 * Top-level control of a simulation.
 * It does not include the graphics window
 */

class Simulation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit Simulation(ChassisType tpye);

  /*!
   * Explicitly set the state of the robot
   */
  void setRobotState(FBModelState<double> &state) {
    simulator_->setState(state);
  }

  void step(double dt, double dtControl);
  void addCollisionPlane(double mu, double resti, double height,
                         double sizeX = 10, double sizeY = 10);
  void addCollisionBox(double mu, double resti, double depth, double width,
                       double height, const Vec3<double> &pos,
                       const Mat3<double> &ori);
  void addCollisionMesh(double mu, double resti, double grid_size,
                        const Vec3<double> &left_corner_loc,
                        const DMat<double> &height_map, bool addToWindow = true,
                        bool transparent = true);

  void runForTime(double time);
  void updateVis();
  void resetSimTime() {
    currentSimTime_ = 0.;
    timeOfNextLControl_ = 0.;
  }

  ~Simulation() {
    delete simulator_;
  }

  const FBModelState<double> &getRobotState() { return simulator_->getState(); }

 private:
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  visualization_msgs::Marker marker_;
  tf::TransformBroadcaster br_;
  ros::Time visTime_{};

  Chassis<double> chassis_;
  FBModelState<double> jointState_;
  FloatingBaseModel<double> model_;
  DVec<double> tau_;
  DynamicsSimulator<double> *simulator_ = nullptr;
  std::vector<ActuatorModel<double>> actuatorModels_;
  SimParameters simParams_;
  ChassisType type_;

  FakeSuspe fake_suspe_;
  SuspeData suspe_data_;

  double timeOfNextLControl_{};
  double timeOfVis_{};
  double currentSimTime_{};
};

#endif //SUSPENSION_SIM_INCLUDE_SIMULATION_H_
