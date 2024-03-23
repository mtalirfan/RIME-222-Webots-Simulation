"""nmp_controller_python controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, Device
from controller import (
    Robot,
    Lidar,
    Motor,
    DistanceSensor,
    Camera,
    Accelerometer,
    Gyro,
    LED,
)

# create the Robot instance.
robot = Robot()

front_distance_threshold = 450  # 450,
left_right_distance_threshold = 400  # 400,
left_right_wall_threshold = 450  # 450,
left_right_wall_crit_threshold = 550  # 550,
front_wall_threshold = 500  # 500,
wall_collide_threshold = 950  # 950,

max_velocity = 10
turn_coefficient = 0.65
soft_turn_coefficient = 0.75
sharp_turn_coefficient = 0.1

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Initialise motors
lmotor = robot.getDevice("left wheel motor")
rmotor = robot.getDevice("right wheel motor")

lmotor.setPosition(float("inf"))
rmotor.setPosition(float("inf"))
lmotor.setVelocity(0.0)
rmotor.setVelocity(0.0)


def robot_move(direction="forward"):
    match direction:
        case "forward":
            lmotor.setVelocity(max_velocity)
            rmotor.setVelocity(max_velocity)
        case "back":
            lmotor.setVelocity(-max_velocity)
            rmotor.setVelocity(-max_velocity)
        case "left":
            lmotor.setVelocity(turn_coefficient * max_velocity)
            rmotor.setVelocity(1 * max_velocity)
        case "right":
            lmotor.setVelocity(1 * max_velocity)
            rmotor.setVelocity(turn_coefficient * max_velocity)
        case "soft left":
            lmotor.setVelocity(soft_turn_coefficient * max_velocity)
            rmotor.setVelocity(1 * max_velocity)
        case "soft right":
            lmotor.setVelocity(1 * max_velocity)
            rmotor.setVelocity(soft_turn_coefficient * max_velocity)
        case "sharp left":
            lmotor.setVelocity(sharp_turn_coefficient * max_velocity)
            rmotor.setVelocity(1 * max_velocity)
        case "sharp right":
            lmotor.setVelocity(1 * max_velocity)
            rmotor.setVelocity(sharp_turn_coefficient * max_velocity)
        case "stop":
            lmotor.setVelocity(0)
            rmotor.setVelocity(0)


lidar = robot.getDevice("lidar")
frontDs = robot.getDevice("ds6")
frontLeftDs = robot.getDevice("ds2")
frontRightDs = robot.getDevice("ds1")
leftDs = robot.getDevice("ds3")
rightDs = robot.getDevice("ds0")
leftFrontLeftDs = robot.getDevice("ds5")
rightFrontRightDs = robot.getDevice("ds4")
# cam = robot.getDevice("camera")
accelerometer = robot.getDevice("accelerometer")
gyro = robot.getDevice("gyro")
led = robot.getDevice("led")

lidar.enable(timestep)
frontDs.enable(timestep)
frontLeftDs.enable(timestep)
frontRightDs.enable(timestep)
leftDs.enable(timestep)
rightDs.enable(timestep)
leftFrontLeftDs.enable(timestep)
rightFrontRightDs.enable(timestep)
accelerometer.enable(timestep)
gyro.enable(timestep)
# cam.enable(timestep)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    frontDsVal = frontDs.getValue()
    frontLeftDsVal = frontLeftDs.getValue()
    frontRightDsVal = frontRightDs.getValue()
    leftDsVal = leftDs.getValue()
    rightDsVal = rightDs.getValue()
    leftFrontLeftDsVal = leftFrontLeftDs.getValue()
    rightFrontRightDsVal = rightFrontRightDs.getValue()
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    # motor.setPosition(10.0)
    if (
        max(
            frontDsVal,
            leftDsVal,
            frontLeftDsVal,
            leftFrontLeftDsVal,
            rightDsVal,
            frontRightDsVal,
            rightFrontRightDsVal,
        )
        > wall_collide_threshold
    ):  # tooooooooo close to wall
        robot_move("back")
    elif (
        # abs(rightDsVal - leftDsVal) < left_right_distance_threshold and
        frontDsVal
        < front_distance_threshold
    ):  # left and right walls are equidistant, and there is enough way forward
        robot_move("forward")
        if (
            min(leftDsVal, frontLeftDsVal, leftFrontLeftDsVal)
            > left_right_wall_crit_threshold
        ):  # too close to left wall
            robot_move("sharp right")
            # if frontDsVal < left_right_wall_threshold:
            #     robot_move("forward")
        elif (
            min(leftDsVal, leftFrontLeftDsVal) > left_right_wall_threshold
        ):  # close to left wall
            robot_move("soft right")
            if leftFrontLeftDsVal < left_right_wall_threshold:
                robot_move("forward")
        if (
            min(rightDsVal, frontRightDsVal, rightFrontRightDsVal)
            > left_right_wall_crit_threshold
        ):  # too close to right wall
            robot_move("sharp left")
            # if frontDsVal < left_right_wall_threshold:
            #     robot_move("forward")
        elif (
            min(rightDsVal, rightFrontRightDsVal)
            > left_right_wall_threshold
            # or rightFrontRightDsVal > left_right_wall_threshold
        ):  # close to right wall
            robot_move("soft left")
            if rightFrontRightDsVal < left_right_wall_threshold:
                robot_move("forward")

    elif (
        min(rightFrontRightDsVal, rightDsVal) < left_right_distance_threshold
        or leftFrontLeftDsVal >= left_right_distance_threshold
        and abs(rightFrontRightDsVal - leftFrontLeftDsVal)
        > left_right_distance_threshold
    ):  # there is way on right, or no way on left, and right way is more than left
        robot_move("sharp right")

    elif (
        min(leftFrontLeftDsVal, leftDsVal) <= left_right_distance_threshold
        or rightFrontRightDsVal > left_right_distance_threshold
        and abs(rightFrontRightDsVal - leftFrontLeftDsVal)
        > left_right_distance_threshold
    ):  # there is way on left, or no way on right, and left way is more than right
        robot_move("sharp left")

    # DEBUG Print the values
    print(f" \n ")
    print(f"Front {frontDsVal}")
    print(f"Front Left {frontLeftDsVal} Front Right {frontRightDsVal}")
    print(
        f"Left Front Left {leftFrontLeftDsVal} Right Front Right {rightFrontRightDsVal}"
    )
    print(f"Left {leftDsVal} Right {rightDsVal}")
    print(f" \n ")

    pass

# Enter here exit cleanup code.
