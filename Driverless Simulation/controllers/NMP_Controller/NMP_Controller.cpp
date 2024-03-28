// File:          NMP_Controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes

#include <cstring>
#include <string>
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/LED.hpp>
#include <limits>
// All the webots classes are defined in the "webots" namespace
using namespace webots;

float front_distance_threshold = 450; // 450
float left_right_distance_threshold = 400; // 400
float left_right_wall_threshold = 450; // 450
float left_right_wall_crit_threshold = 550; // 550
float front_wall_threshold = 500; // 500
float wall_collide_threshold = 950; // 950


float max_velocity = 10;
float turn_coefficient = 0.4;
float soft_turn_coefficient = 0.75;
float sharp_turn_coefficient = 0.3;

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
DistanceSensor *frontDs = robot->getDistanceSensor("ds6");
DistanceSensor *frontLeftDs = robot->getDistanceSensor("ds2");
DistanceSensor *frontRightDs = robot->getDistanceSensor("ds1");
DistanceSensor *leftDs = robot->getDistanceSensor("ds3");
DistanceSensor *rightDs = robot->getDistanceSensor("ds0");
DistanceSensor *leftFrontLeftDs = robot->getDistanceSensor("ds5");
DistanceSensor *rightFrontRightDs= robot->getDistanceSensor("ds4");
// Camera * cam = robot->getCamera("camera");
Accelerometer * accelerometer = robot->getAccelerometer("accelerometer");
Gyro *gyro = robot->getGyro("gyro");
// LED *led = robot->getLED("led");

void robot_move(const char* direction) {
  // code to be executed
  if (strcmp(direction, "forward") == 0) {
      lmotor->setVelocity(max_velocity);
      rmotor->setVelocity(max_velocity);
  }
  else if (strcmp(direction, "back") == 0)
  {
      lmotor->setVelocity(-max_velocity);
      rmotor->setVelocity(-max_velocity);
  }
  else if (strcmp(direction, "left") == 0) {
      lmotor->setVelocity(turn_coefficient * max_velocity);
      rmotor->setVelocity(1 * max_velocity);
  }
  else if (strcmp(direction, "right") == 0) {
      lmotor->setVelocity(1 * max_velocity);
      rmotor->setVelocity(turn_coefficient * max_velocity);
  }
  else if (strcmp(direction, "soft left") == 0) {
      lmotor->setVelocity(soft_turn_coefficient * max_velocity);
      rmotor->setVelocity(1 * max_velocity);
  }
  else if (strcmp(direction, "soft right") == 0) {
      lmotor->setVelocity(1 * max_velocity);
      rmotor->setVelocity(soft_turn_coefficient * max_velocity);
  }
  else if (strcmp(direction, "sharp left") == 0) {
      lmotor->setVelocity(sharp_turn_coefficient * max_velocity);
      rmotor->setVelocity(1 * max_velocity);
  }
  else if (strcmp(direction, "sharp right") == 0) {
      lmotor->setVelocity(1 * max_velocity);
      rmotor->setVelocity(sharp_turn_coefficient * max_velocity);
  }
  else if (strcmp(direction, "stop") == 0) {
      lmotor->setVelocity(0);
      rmotor->setVelocity(0);
  };
};


// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {

// get the time step of the current world.
int timeStep = (int)robot->getBasicTimeStep();

// Enable the sensors, feel free to change the sampling rate
lidar->enable(timeStep);
frontDs->enable(timeStep);
frontLeftDs->enable(timeStep);
frontRightDs->enable(timeStep);
leftDs->enable(timeStep);
rightDs->enable(timeStep);
leftFrontLeftDs->enable(timeStep);
rightFrontRightDs->enable(timeStep);
accelerometer->enable(timeStep);
gyro->enable(timeStep);
// cam->enable(50);
// led->enable(50);

lmotor->setPosition(std::numeric_limits<double>::infinity());
rmotor->setPosition(std::numeric_limits<double>::infinity());
lmotor->setVelocity(0);
rmotor->setVelocity(0);


  // - perform simulation steps until Webots is stopping the controller
    while (robot->step(timeStep) != -1) {
        // Read the sensors:
        // Enter here functions to read sensor data, like:
        //  double val = ds->getValue();

        float frontDsVal = frontDs->getValue();
        float frontLeftDsVal = frontLeftDs->getValue();
        float frontRightDsVal = frontRightDs->getValue();
        float leftDsVal = leftDs->getValue();
        float rightDsVal = rightDs->getValue();
        float leftFrontLeftDsVal = leftFrontLeftDs->getValue();
        float rightFrontRightDsVal = rightFrontRightDs->getValue();

    // Process sensor data here.

    if (frontDsVal < front_distance_threshold) {
    robot_move("forward");

    if ( std::min(leftDsVal, std::min(frontLeftDsVal, leftFrontLeftDsVal)) > left_right_wall_crit_threshold ) {
        robot_move("sharp right");
        // if (frontDsVal < left_right_wall_threshold) { robot_move("forward"); }
        }
    else if ( std::min(leftDsVal, leftFrontLeftDsVal) > left_right_wall_threshold ) {
        robot_move("soft right");
        if (leftFrontLeftDsVal < left_right_wall_threshold) {robot_move("forward");}
        };
    if ( std::min(rightDsVal, std::min(frontRightDsVal, rightFrontRightDsVal)) > left_right_wall_crit_threshold) {
        robot_move("sharp left");
        // if (frontDsVal < left_right_wall_threshold) {robot_move("forward");}
        }
    else if ( std::min(rightDsVal, rightFrontRightDsVal) > left_right_wall_threshold ) {
        robot_move("soft left");
        if (rightFrontRightDsVal < left_right_wall_threshold) {robot_move("forward"); }
        };
    };
    if (((std::min(rightFrontRightDsVal, rightDsVal) < left_right_distance_threshold)) )
    {
      robot_move("right");
    }
    else if (((std::min(leftFrontLeftDsVal, leftDsVal) < left_right_distance_threshold)))
    {
      robot_move("left");
    }
    if (std::max(frontDsVal,std::max(leftDsVal,std::max(frontLeftDsVal,std::max(leftFrontLeftDsVal,std::max(rightDsVal,std::max(frontRightDsVal,rightFrontRightDsVal)))))) > wall_collide_threshold  ) {
      robot_move("back");
    };

    // DEBUG Print the values
    printf("\n \n");
    printf("Front %f \n", frontDsVal);
    printf("Front Left %f Front Right %f \n", frontLeftDsVal, frontRightDsVal);
    printf(
        "Left Front Left %f Right Front Right %f \n", leftFrontLeftDsVal, rightFrontRightDsVal
    );
    printf("Left %f Right %f \n\n", leftDsVal, rightDsVal);
    printf("\n\n");


    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
