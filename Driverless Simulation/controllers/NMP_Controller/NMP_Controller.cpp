// File:          NMP_Controller.cpp
// Date:          31/March/2024 (Easter and Transgender Visibility Day !!!)
// Description:   A Controller for a Webots Driverless Simulation Robot
// Author:        Natalie
// Modifications: Almost completely rewritten, built on sample code and ported from Python one

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes

#include <cstring>
#include <string>
#include <webots/Robot.hpp>
// #include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
// #include <webots/LED.hpp>
#include <limits>

#include <webots/Keyboard.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

float front_distance_threshold = 400; // 400
float left_right_distance_threshold = 400; // 400
float left_right_wall_threshold = 450; // 450
float left_right_wall_crit_threshold = 550; // 550
float wall_collide_threshold = 950; // 950

float max_velocity = 10;
float slow_coefficient = 0.5;
float turn_coefficient = 0.5;
float soft_turn_coefficient = 0.7;
float sharp_turn_coefficient = 0.1;

bool val_debug = false;
bool dir_debug = false;
int collide_counter = 0;

// create the Robot instance.
Robot *robot = new Robot();

// You should insert a getDevice-like function in order to get the
// instance of a device of the robot. Something like:
//  Motor *motor = robot->getMotor("motorname");
//  DistanceSensor *ds = robot->getDistanceSensor("dsname");
//  ds->enable(timeStep);

// Lidar *lidar = robot->getLidar("lidar");
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
GPS * gps = robot->getGPS("gps");
Compass * compass = robot->getCompass("compass");
Keyboard keyboard = Keyboard();
// LED *led = robot->getLED("led");


void robot_move(const char* direction) {
  // code to be executed
  if (strcmp(direction, "forward") == 0) {
      lmotor->setVelocity(max_velocity);
      rmotor->setVelocity(max_velocity);
  }
  else if (strcmp(direction, "slow forward") == 0) {
      lmotor->setVelocity(slow_coefficient * max_velocity);
      rmotor->setVelocity(slow_coefficient * max_velocity);
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

    // lidar->enable(timeStep);
    frontDs->enable(timeStep);
    frontLeftDs->enable(timeStep);
    frontRightDs->enable(timeStep);
    leftDs->enable(timeStep);
    rightDs->enable(timeStep);
    leftFrontLeftDs->enable(timeStep);
    rightFrontRightDs->enable(timeStep);
    accelerometer->enable(timeStep);
    gyro->enable(timeStep);
    gps->enable(timeStep);
    compass->enable(timeStep);
    // cam->enable(50);
    // led->enable(50);

    keyboard.enable(timeStep);

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

        const double * acceleration = accelerometer->getValues();
        const double * gyroscope = gyro->getValues();
        const double * gpsval = gps->getValues();
        const double * compassval = compass->getValues();

        // Process sensor data here.

        if (std::max(frontDsVal,std::max(leftDsVal,std::max(frontLeftDsVal,std::max(leftFrontLeftDsVal,std::max(rightDsVal,std::max(frontRightDsVal,rightFrontRightDsVal)))))) > wall_collide_threshold  ) {
        robot_move("back");
        collide_counter++;
        if (dir_debug == true) {
            printf("Back\n");
        }
        else {
            printf("Collide Counter: %i \n", collide_counter);
        }
        }
        else if (frontDsVal < front_distance_threshold) {
            if (dir_debug == true) {
                printf("Forward\n");
            };
            robot_move("forward");

            if (gpsval[2] > 0.2) {  // gps z value is on upper platform
                if (dir_debug == true) {
                    printf("Slow Forward\n");
                }
                robot_move("slow forward");
            };

            if ( std::min(leftDsVal, std::min(frontLeftDsVal, leftFrontLeftDsVal)) > left_right_wall_crit_threshold ) {
                if (dir_debug == true) {
                    printf("Sharp Right\n");
                };
                robot_move("sharp right");
                // if (frontDsVal < left_right_wall_threshold) { robot_move("forward"); }
                }
            else if ( std::min(leftDsVal, leftFrontLeftDsVal) > left_right_wall_threshold ) {
                if (dir_debug == true) {
                    printf("Soft Right\n");
                };
                robot_move("soft right");
                if (leftFrontLeftDsVal < left_right_wall_threshold) {
                    if (dir_debug == true) {
                        printf("Forward\n");
                    };
                    robot_move("forward");
                    }
                };
            if ( std::min(rightDsVal, std::min(frontRightDsVal, rightFrontRightDsVal)) > left_right_wall_crit_threshold) {
                if (dir_debug == true) {
                    printf("Sharp Left\n");
                };
                robot_move("sharp left");
                // if (frontDsVal < left_right_wall_threshold) {robot_move("forward");}
                }
            else if ( std::min(rightDsVal, rightFrontRightDsVal) > left_right_wall_threshold ) {
                if (dir_debug == true) {
                    printf("Soft Left\n");
                };
                robot_move("soft left");
                if (rightFrontRightDsVal < left_right_wall_threshold) {
                    if (dir_debug == true) {
                        printf("Forward\n");
                    };
                    robot_move("forward");
                    }
                };
        }
        else if (((std::min(rightFrontRightDsVal, rightDsVal) < left_right_distance_threshold)) )
        {
        if (dir_debug == true) {
            printf("Right\n");
        };
        robot_move("right");
        }
        else if (((std::min(leftFrontLeftDsVal, leftDsVal) < left_right_distance_threshold)))
        {
        if (dir_debug == true) {
            printf("Left\n");
        };
        robot_move("left");
        };

        switch (keyboard.getKey()) {
            case 'A': {
                printf("Acceleration\nX: %.3f Y: %.3f Z: %.3f \n", acceleration[0], acceleration[1], acceleration[2]);
                break;
            }
            case 'C': {
                printf("Compass\nX: %.3f Y: %.3f Z: %.3f \n", compassval[0], compassval[1], compassval[2]);
                break;
            }
            case 'D': {
                printf("Distance Sensors\nFront: %f \n", frontDsVal);
                printf("Front Left: %f Front Right: %f \n", frontLeftDsVal, frontRightDsVal);
                printf(
                    "Left Front Left: %f Right Front Right: %f \n", leftFrontLeftDsVal, rightFrontRightDsVal
                );
                printf("Left: %f Right: %f", leftDsVal, rightDsVal);
                printf("\n \n");
                break;
            }
            case 'E': {
                printf("Collide Counter: %i \n", collide_counter);
                break;
            }
            case 'G': {
                printf("GPS\nX: %.3f Y: %.3f Z: %.3f \n", gpsval[0], gpsval[1], gpsval[2]);
                break;
            }
            case 'V': {
                while (robot->step(timeStep) != -1) {
                    printf("Value Debug Toggled\n\n");
                    val_debug = !val_debug;
                    break;
                }
                break;
            }
            case 'X': {
                printf("Gyroscope\nX: %.3f Y: %.3f Z: %.3f \n", gyroscope[0], gyroscope[1], gyroscope[2]);
                break;
            }
            case 'Z': {
                while (robot->step(timeStep) != -1) {
                    printf("Direction Debug Toggled\n\n");
                    dir_debug = !dir_debug;
                    break;
                }
                break;
            }
        }

        // DEBUG Print the values

        if (val_debug == true) {
            printf("Acceleration\nX: %.3f Y: %.3f Z: %.3f \n", acceleration[0], acceleration[1], acceleration[2]);
            printf("Gyroscope\nX: %.3f Y: %.3f Z: %.3f \n", gyroscope[0], gyroscope[1], gyroscope[2]);
            printf("GPS\nX: %.3f Y: %.3f Z: %.3f \n", gpsval[0], gpsval[1], gpsval[2]);
            printf("Compass\nX: %.3f Y: %.3f Z: %.3f \n", compassval[0], compassval[1], compassval[2]);
            printf("Distance Sensors\nFront: %f \n", frontDsVal);
            printf("Front Left: %f Front Right: %f \n", frontLeftDsVal, frontRightDsVal);
            printf(
                "Left Front Left: %f Right Front Right: %f \n", leftFrontLeftDsVal, rightFrontRightDsVal
            );
            printf("Left: %f Right: %f", leftDsVal, rightDsVal);
            printf("\n \n");
        };

        // Enter here functions to send actuator commands, like:
        //  motor->setPosition(10.0);
}

    // Enter here exit cleanup code.

    delete robot;
    return 0;

};