// File:          NMP_Controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <limits>
// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
   // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  Lidar *lidar = robot->getLidar("lidar");
  Motor *lmotor = robot->getMotor("left wheel motor");
  Motor *rmotor = robot->getMotor("right wheel motor");
  DistanceSensor *frontLeftDs = robot->getDistanceSensor("ds2");
  DistanceSensor *frontRightDs = robot->getDistanceSensor("ds1");
  DistanceSensor *leftDs = robot->getDistanceSensor("ds3");
  DistanceSensor *rightDs = robot->getDistanceSensor("ds0");
  DistanceSensor *leftFrontLeftDs = robot->getDistanceSensor("ds5");
  DistanceSensor *rightFrontRightDs= robot->getDistanceSensor("ds4");
  Camera * cam = robot->getCamera("camera");
  Accelerometer * accelerometer = robot->getAccelerometer("accelerometer");
  Gyro *gyro = robot->getGyro("gyro");

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();


  // Enable the sensors, feel free to change the sampling rate
  lidar->enable(50);
  frontLeftDs->enable(100);
  frontRightDs->enable(100);
  leftDs->enable(100);
  rightDs->enable(100);
  leftFrontLeftDs->enable(100);
  rightFrontRightDs->enable(100);
  accelerometer->enable(100);
  gyro->enable(100);
  cam->enable(50);
  
  lmotor->setPosition(std::numeric_limits<double>::infinity());
  rmotor->setPosition(std::numeric_limits<double>::infinity());
  lmotor->setVelocity(0);
  rmotor->setVelocity(0);
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    lmotor->setVelocity(10);
    rmotor->setVelocity(10);
     // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
