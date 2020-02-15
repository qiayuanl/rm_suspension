//
// Created by qiayuan on 2/15/20.
//
#include "Simulation.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sim");

  Simulation simulation(ChassisType::STANDARD);
  simulation.runForTime(99);
}