"""nmp_controller_python controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, Device
from controller import Robot, Lidar, Motor, DistanceSensor, Camera, Accelerometer, Gyro, LED

# create the Robot instance.
robot = Robot()


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
max_velocity = 10

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

lmotor = robot.getDevice("left wheel motor")
rmotor = robot.getDevice("right wheel motor")

lmotor.setPosition(float("inf"))
rmotor.setPosition(float("inf"))
lmotor.setVelocity(0.0)
rmotor.setVelocity(0.0)

lidar = robot.getDevice("lidar")
frontLeftDs = robot.getDevice("ds2")
frontRightDs = robot.getDevice("ds1")
leftDs = robot.getDevice("ds3")
rightDs = robot.getDevice("ds0")
leftFrontLeftDs = robot.getDevice("ds5")
rightFrontRightDs = robot.getDevice("ds4")
cam = robot.getDevice("camera")
accelerometer = robot.getDevice("accelerometer")
gyro = robot.getDevice("gyro")
led = robot.getDevice("led")

lidar.enable(50)
frontLeftDs.enable(100)
frontRightDs.enable(100)
leftDs.enable(100)
rightDs.enable(100)
leftFrontLeftDs.enable(100)
rightFrontRightDs.enable(100)
accelerometer.enable(100)
gyro.enable(100)
cam.enable(50)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

    lmotor.setVelocity(10)
    rmotor.setVelocity(10)
    pass

# Enter here exit cleanup code.
