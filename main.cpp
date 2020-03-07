//
// Created by qiayuan on 2/15/20.
//
#include "Simulation.h"
void selectSimType(Simulation *sim);
bool selectSimTime(Simulation *sim);
bool selectPlaySpeed(Simulation *sim);
bool selectFlyRampSpeed(Simulation *sim);

int main(int argc, char **argv) {
  ros::init(argc, argv, "sim");
  Simulation simulation;
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
  //prepare init robot state
  sim->addCollisionPlane(0.7, 0, 0.);
  char input;
  while (ros::ok()) { //loop until legal input
    sim->setupState_.bodyPosition.setZero();
    sim->setupState_.bodyVelocity.setZero();
    sim->setupState_.bodyPosition[2] = 0.15;
    printf(" Simulation Type:\n\r");
    printf(" a - Drop from 0.5 meter height\n\r");
    printf(" b - Run down the stairs\n\r");
    printf(" c - Brake in maximum X speed\n\r");
    printf(" d - Brake in maximum Y speed\n\r");
    printf(" e - Diagonally across the ramp\n\r");
    printf(" f - Fly the ramp\n\r");

    std::cin >> input;
    double stairs_height = 0.2;
    switch (input) {
      case 'a': {
        sim->setupState_.bodyPosition[2] = 0.5; //Reconfigure z to 0.5 meter
        sim->setSpeed(0.);
        break;
      }
      case 'b': {
        sim->setupState_.bodyPosition[2] = stairs_height + 0.15; //Reconfigure z
        sim->addCollisionBox(0.7, 0., 3., 1., stairs_height, Vec3<double>(0., 0., stairs_height / 2.),
                             coordinateRotation<double>(CoordinateAxis::Z, 0.));//turn 0 degree
        sim->setSpeed(3.);
        sim->setupState_.bodyVelocity[3] = 3.; //[3] mean x axis
        break;
      }
      case 'c': {
        sim->setSpeed(0.);
        sim->setupState_.bodyVelocity[3] = 3.;
        break;
      }
      case 'd': {
        sim->setSpeed(0.);
        sim->setupState_.bodyVelocity[4] = 3.;
        break;
      }
      case 'e': {
        sim->addCollisionBox(0.7, 0.,
                             2., 3., 0.1,
                             Vec3<double>(1.6, .5, 1. * sin(0.072222 * M_PI) - 0.05),
                             rpyToRotMat(Vec3<double>(0., -0.072222 * M_PI, M_PI_4))//turn 13 degree
        );
        sim->setSpeed(1.);
        sim->setupState_.bodyVelocity[3] = 1.;
        break;
      }
      case 'f': {
        sim->setupState_.bodyPosition[2] = stairs_height + 0.15;
        sim->addCollisionBox(0.7, 0., (1. + 1.145) * 2, 1., stairs_height, Vec3<double>(0., 0., stairs_height / 2.),
                             coordinateRotation<double>(CoordinateAxis::Z, 0.));
        sim->addCollisionBox(0.7, 0,
                             1.197323, 1., 0.1,
                             Vec3<double>(1. + 1.145 / 2. + 0.05,
                                          0.,
                                          1.145 * tan(0.094444 * M_PI) / 2. + stairs_height - 0.05),
                             coordinateRotation<double>(CoordinateAxis::Y, -0.094444 * M_PI)); //turn 17 degree
        sim->addCollisionBox(0.7, 0., 2., 1., stairs_height, Vec3<double>(3.795, 0., stairs_height / 2.),
                             coordinateRotation<double>(CoordinateAxis::Z, 0.));
        if (selectFlyRampSpeed(sim) && ros::ok()) {
          sim->setSpeed(sim->getFlyRampSpeed());
          sim->setupState_.bodyVelocity[3] = sim->getFlyRampSpeed();
        }
        break;
      }

      default: {
        printf("\n\r Illegal Input! Check and try again.\n\r");
        continue;
      }
    }
    break; //get out on loop
  }
  sim->setRobotState(sim->setupState_);
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

bool selectFlyRampSpeed(Simulation *sim) {
  char input;
  printf("\n\rFly speed: \n\r");
  printf(" a - 2.5\n\r");
  printf(" b - 2.8\n\r");
  printf(" c - 3.0\n\r");
  printf(" d - 3.4\n\r");
  printf(" e - 3.8\n\r");

  std::cin >> input;
  while (ros::ok()) {
    switch (input) {
      case 'a' : sim->setFlyRampSpeed(2.5);
        break;
      case 'b' : sim->setFlyRampSpeed(2.8);
        break;
      case 'c' : sim->setFlyRampSpeed(3.0);
        break;
      case 'd' : sim->setFlyRampSpeed(3.4);
        break;
      case 'e' : sim->setFlyRampSpeed(3.8);
        break;
      default: {
        printf("\n\rIllegal Input! Check and try again.\n\r");
        continue;
      }
    }
    break;
  }
  return true;
}