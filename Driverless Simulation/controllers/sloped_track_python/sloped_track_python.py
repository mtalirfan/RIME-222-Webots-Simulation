"""nmp_controller_python controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, Device
from controller import (
    Robot,
    # Lidar,
    Motor,
    DistanceSensor,
    # Camera,
    Accelerometer,
    Gyro,
    GPS,
    Compass,
    # LED,
    Keyboard,
)

# create the Robot instance.
robot = Robot()

front_distance_threshold = 500  # 400, general front distance
left_right_distance_threshold = 350  # 350, general left right distance
left_right_wall_threshold = (
    450  # 450, while moving forward, if robot gets close to wall do soft turn
)
left_right_wall_crit_threshold = 700  # 600, if robot gets closer to wall do sharp turn
wall_collide_threshold = 950  # 950, absolute limit, prevent wall collide and hence unexpected movement by moving back

max_velocity = 10
turn_coefficient = 0.5
soft_turn_coefficient = 0.75
sharp_turn_coefficient = 0.3

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
        # case "descend":
        #     lmotor.setVelocity(0.5 * max_velocity)
        #     rmotor.setVelocity(0.5 * max_velocity)
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


def key_press_check():
    if key == ord("A"):
        print(
            f"Acceleration\nX: {acceleration[0]} Y: {acceleration[1]} Z: {acceleration[2]}"
        )
        print(f"\n ")
    elif key == ord("C"):
        print(f"Compass\nX: {compassval[0]} Y: {compassval[1]} Z: {compassval[2]}")
        print(f"\n ")
    elif key == ord("D"):
        print(f"Distance Sensors\nFront: {frontDsVal}")
        print(f"Front Left: {frontLeftDsVal} Front Right: {frontRightDsVal}")
        print(
            f"Left Front Left: {leftFrontLeftDsVal} Right Front Right: {rightFrontRightDsVal}"
        )
        print(f"Left: {leftDsVal} Right: {rightDsVal}")
        print(f"\n ")
    elif key == ord("G"):
        print(f"GPS\nX: {gpsval[0]} Y: {gpsval[1]} Z: {gpsval[2]}")
        print(f"\n ")
    elif key == ord("X"):
        print(f"Gyroscope\nX: {gyroscope[0]} Y: {gyroscope[1]} Z: {gyroscope[2]}")
        print(f"\n ")
    # elif key == Keyboard.CONTROL + ord("B"):
    #     print("Ctrl+B is pressed")


# lidar = robot.getDevice("lidar")
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
gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
# led = robot.getDevice("led")
keyboard = Keyboard()

# lidar.enable(timestep)
frontDs.enable(timestep)
frontLeftDs.enable(timestep)
frontRightDs.enable(timestep)
leftDs.enable(timestep)
rightDs.enable(timestep)
leftFrontLeftDs.enable(timestep)
rightFrontRightDs.enable(timestep)
accelerometer.enable(timestep)
gyro.enable(timestep)
gps.enable(timestep)
compass.enable(timestep)
keyboard.enable(timestep)
# cam.enable(timestep)

dir_debug = False
val_debug = False

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
    acceleration = accelerometer.getValues()
    gyroscope = gyro.getValues()
    gpsval = gps.getValues()
    compassval = compass.getValues()
    # Process sensor data here.

    key = keyboard.getKey()
    key_press_check()
    if key == ord("V"):
        while robot.step(timestep) != -1:
            print("Value Debug Toggled")
            val_debug = not val_debug
            break
    elif key == ord("Z"):
        while robot.step(timestep) != -1:
            print("Direction Debug Toggled")
            dir_debug = not dir_debug
            break

    # Enter here functions to send actuator commands, like:
    # motor.setPosition(10.0)
    if (
        max(
            frontDsVal,
            # leftDsVal,
            frontLeftDsVal,
            # leftFrontLeftDsVal,
            # rightDsVal,
            frontRightDsVal,
            # rightFrontRightDsVal,
        )
        > wall_collide_threshold
    ):  # tooooooooo close to wall
        robot_move("back")
        if dir_debug:
            print("Back")
    elif frontDsVal < front_distance_threshold:  # there is enough way forward
        if dir_debug:
            print("Forward")
        robot_move("forward")

        # if gpsval[2] > 0.2:  # gps z value is on upper platform
        #     if dir_debug:
        #         print("Slow forward on upper platform")
        #     robot_move("descend")

        if (
            min(leftDsVal, frontLeftDsVal, leftFrontLeftDsVal)
            > left_right_wall_crit_threshold
        ):  # too close to left wall
            if dir_debug:
                print("Sharp Right")
            robot_move("sharp right")
            # if frontDsVal < left_right_wall_threshold:
            #     if dir_debug:
            #         print("Forward")
            #     robot_move("forward")
        elif (
            min(leftDsVal, leftFrontLeftDsVal) > left_right_wall_threshold
        ):  # close to left wall
            if dir_debug:
                print("Soft Right")
            robot_move("soft right")
            if leftFrontLeftDsVal < left_right_wall_threshold:
                if dir_debug:
                    print(" Soft Right Forward")
                robot_move("forward")
        if (
            min(rightDsVal, frontRightDsVal, rightFrontRightDsVal)
            > left_right_wall_crit_threshold
        ):  # too close to right wall
            if dir_debug:
                print("Sharp Left")
            robot_move("sharp left")
            # if frontDsVal < left_right_wall_threshold:
            # if dir_debug:
            #     print("Forward")
            # robot_move("forward")
        elif (
            min(rightDsVal, rightFrontRightDsVal) > left_right_wall_threshold
        ):  # close to right wall
            if dir_debug:
                print("Soft Left")
            robot_move("soft left")
            if rightFrontRightDsVal < left_right_wall_threshold:
                if dir_debug:
                    print("Soft Left Forward")
                robot_move("forward")

    elif (
        min(rightFrontRightDsVal, rightDsVal) < left_right_distance_threshold
        or leftFrontLeftDsVal >= left_right_distance_threshold
        and abs(rightFrontRightDsVal - leftFrontLeftDsVal)
        > left_right_distance_threshold
    ):  # there is way on right, or no way on left, and right way is more than left
        if dir_debug:
            print("Right")
        robot_move("right")

    elif (
        min(leftFrontLeftDsVal, leftDsVal) <= left_right_distance_threshold
        or rightFrontRightDsVal > left_right_distance_threshold
        and abs(rightFrontRightDsVal - leftFrontLeftDsVal)
        > left_right_distance_threshold,
    ):  # there is way on left, or no way on right, and left way is more than right
        if dir_debug:
            print("Left")
        robot_move("left")

    # DEBUG Print the values
    if val_debug:
        print(
            f"Acceleration\nX: {acceleration[0]} Y: {acceleration[1]} Z: {acceleration[2]}"
        )
        print(f"Gyroscope\nX: {gyroscope[0]} Y: {gyroscope[1]} Z: {gyroscope[2]}")
        print(f"GPS\nX: {gpsval[0]} Y: {gpsval[1]} Z: {gpsval[2]}")
        print(f"Compass\nX: {compassval[0]} Y: {compassval[1]} Z: {compassval[2]}")
        print(f"Distance Sensors\nFront: {frontDsVal}")
        print(f"Front Left: {frontLeftDsVal} Front Right: {frontRightDsVal}")
        print(
            f"Left Front Left: {leftFrontLeftDsVal} Right Front Right: {rightFrontRightDsVal}"
        )
        print(f"Left: {leftDsVal} Right: {rightDsVal}")
        print(f"\n ")

# Enter here exit cleanup code.
