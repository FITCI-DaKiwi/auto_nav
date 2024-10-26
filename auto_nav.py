from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import umath

hub = PrimeHub()

from pybricks.parameters import Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase

wheel_diameter = 80
robot_base = 114

# Set up all devices.
right_motor = Motor(Port.A, Direction.CLOCKWISE)
left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
motor_3 = Motor(Port.F, Direction.CLOCKWISE)
drive_base = DriveBase(left_motor, right_motor, wheel_diameter, robot_base)

left_motor.reset_angle(0)
right_motor.reset_angle(0)

x = 0
y = 0
loc_x = []
loc_y = []

# Update position of the robot
def update_position():
    global x, y
    left_deg = left_motor.angle()
    right_deg = right_motor.angle()
    
    # Calculate how far each wheel has moved
    left_dist = (left_deg / 360) * (umath.pi * wheel_diameter)
    right_dist = (right_deg / 360) * (umath.pi * wheel_diameter)
    
    # Average distance moved
    dist = (left_dist + right_dist) / 2
    heading = hub.imu.heading()  # Use the hub's built-in gyro/IMU for heading
    print(f'{left_deg} {right_deg} {heading} {dist}')
    # Update x, y using trigonometry based on the current heading
    x += dist * umath.sin(umath.radians(heading))
    y += dist * umath.cos(umath.radians(heading))
    
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    return x, y


# Function to control frequency of position updates
def track_position(update_interval_ms):
    global loc_x, loc_y
    while True:
        # Call update_position to get the current x, y
        tem_x, tem_y = update_position()
        loc_x.append(tem_x)
        loc_y.append(tem_y)
        print(f"location: {tem_x} {tem_y}")
        # Control the output frequency with a wait (in milliseconds)
        wait(update_interval_ms)


# Example usage to update position every 500 milliseconds (0.5 seconds)
track_position(500)

