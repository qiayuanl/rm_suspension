//
// Created by qiayuan on 2/15/20.
//
#include "Simulation.h"
void selectSimType(Simulation *sim);
bool selectSimTime(Simulation *sim);
bool selectPlaySpeed(Simulation *sim);

int main(int argc, char **argv) {
  ros::init(argc, argv, "sim");
  Simulation simulation(ChassisType::STANDARD);
  printf("\n\r\n\r RM Suspension Simulation\n\r");
  printf("\n\r Author: Qiayuan Liao(liaoqiayuan@gmail.com)\n\r\n\r");

  while (ros::ok()) {
    simulation.clearCollision();
    simulation.resetSimTime();
    selectSimType(&simulation);
    if (selectSimTime(&simulation) && ros::ok()) {
      while (selectPlaySpeed(&simulation) && ros::ok()) {
      }
    }
  }
}

void selectSimType(Simulation *sim) {
  //perpare init robot state
  DVec<double> zero8(8);
  for (u32 i = 0; i < 8; i++) {
    zero8[i] = 0.;
  }
  FBModelState<double> robotState;
  robotState.q = zero8;
  robotState.qd = zero8;
  robotState.bodyOrientation = rotationMatrixToQuaternion(
      ori::coordinateRotation(CoordinateAxis::Z, 0.0));
  robotState.bodyPosition.setZero();
  robotState.bodyVelocity.setZero();
  robotState.q = zero8;
  robotState.qd = zero8;
  robotState.bodyPosition[2] = 0.1;
  sim->addCollisionPlane(0.7, 0, 0.);
  char input;
  while (ros::ok()) { //loop until legal input
    printf(" Simulation Type:\n\r");
    printf(" a - Drop from 1.2 meter height\n\r");
    printf(" b - Run down the stairs\n\r");
    printf(" c - Fly the ramp (17 degree)\n\r");
    printf(" d - Brake in maximum speed\n\r");
    printf(" e - Diagonally across the ramp\n\r");
    std::cin >> input;
    double stairs_height = 0.2;
    switch (input) {
      case 'a': {
        robotState.bodyPosition[2] = 1.2; //Reconfigure z to 1.2 meter
        robotState.bodyOrientation = rpyToQuat(Vec3<double>(0, 0, M_PI_2));
        sim->setSpeed(0.);
        break;
      }
      case 'b': {
        robotState.bodyPosition[2] = stairs_height + 0.1; //Reconfigure z
        sim->addCollisionBox(0.7, 0., 3., 1., stairs_height, Vec3<double>(0., 0., stairs_height / 2.),
                             coordinateRotation<double>(CoordinateAxis::Z, 0.));//turn 0 degree
        sim->setSpeed(3.);
        break;
      }
      case 'c': {
        robotState.bodyPosition[2] = stairs_height + 0.1;
        sim->addCollisionBox(0.7, 0., (1. + 1.145) * 2, 1., stairs_height, Vec3<double>(0., 0., stairs_height / 2.),
                             coordinateRotation<double>(CoordinateAxis::Z, 0.));
        sim->addCollisionBox(0.7, 0,
                             1.197323, 1., 0.01,
                             Vec3<double>(1. + 1.145 / 2. + 0.005,
                                          0.,
                                          1.145 * tan(0.094444 * M_PI) / 2. + stairs_height - 0.005),
                             coordinateRotation<double>(CoordinateAxis::Y, -0.094444 * M_PI)); //turn 17 degree
        sim->addCollisionBox(0.7, 0., 2., 1., stairs_height, Vec3<double>(3.795, 0., stairs_height / 2.),
                             coordinateRotation<double>(CoordinateAxis::Z, 0.));
        sim->setSpeed(3.);
        break;
      }
      case 'd': {
        robotState.bodyVelocity[3] = 1.;
        sim->setSpeed(0.);
      }
      case 'e': {
        sim->addCollisionBox(0.7, 0.,
                             2., 3., 0.1,
                             Vec3<double>(1.6, .5, 1. * sin(0.072222 * M_PI) - 0.05),
                             rpyToRotMat(Vec3<double>(0., -0.072222 * M_PI, M_PI_4))//turn 13 degree
        );
        sim->setSpeed(1.);
        break;
      }
      default: {
        printf("\n\r Illegal Input! Check and try again.\n\r");
        continue;
      }
    }
    break; //get out on loop
  }
  sim->setRobotState(robotState);
}

bool selectSimTime(Simulation *sim) {

  char input;
  while (ros::ok()) {
    printf("\n\r Simulation Time:\n\r");
    printf(" a - 0.5 s\n\r");
    printf(" b - 1.0 s\n\r");
    printf(" c - 2.0 s\n\r");
    printf(" d - 3.0 s\n\r");
    printf(" q - quit \n\r");
    std::cin >> input;
    switch (input) {
      case 'a':sim->runForTime(.5);
        break;
      case 'b':sim->runForTime(1.);
        break;
      case 'c':sim->runForTime(2.);
        break;
      case 'd':sim->runForTime(3.);
        break;
      case 'q':return false;
      default: {
        printf("\n\r Illegal Input! Check and try again.\n\r");
        continue;
      }
    }
    break;
  }
  return true;
}

bool selectPlaySpeed(Simulation *sim) {
  char input;
  printf("\n\r Play Speed:\n\r");
  printf(" a - x1\n\r");
  printf(" b - x3\n\r");
  printf(" c - x5\n\r");
  printf(" d - x10\n\r");
  printf(" q - quit \n\r");
  std::cin >> input;
  switch (input) {
    case 'a':sim->play(1);
      break;
    case 'b':sim->play(3);
      break;
    case 'c':sim->play(5);
      break;
    case 'd':sim->play(10);
      break;
    case 'q':return false;
    default:printf("\n\r Illegal Input! Check and try again.\n\r");
      break;
  }
  return true;
}