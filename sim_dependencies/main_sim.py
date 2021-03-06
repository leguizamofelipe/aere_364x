# Put the various CircuitPython import statements here
import board
import busio
import adafruit_bno055
import pwmio
import pulseio

from digitalio import Direction
from digitalio import DigitalInOut

from motor_control import *
from setpoints import *

from time import sleep
import time

# Configuration of the simulator (will have to remove these lines
# when running on real hardware) 
import dynamic_model
dynamic_model.enable_wind=False   # Set this to True to add a moment from the wind

# ***NOTE*** Do not get the various files which are part of the simulator
# (adafruit_bno055.py, busio.py, board.py, etc.) confused with similarly
# named files used for actual CircuitPython hardware. Do NOT copy the
# simulator versions onto your Feather microcontroller! 


# Now create objects for the I2C bus, for the BNO055,
# PulseIn, PWMOut, and/or DigitalInOut
#
# Will only work with pins used as follows (per pin
# definitions in board.py)
#    SCL: SCL
#    SDA: SDA
#    D5: Rotational command pulse input from RC receiver
#    D6: Forward command pulse input from RC receiver (fixed at 1500us)
#    D9: PWM pulse out to left-side fan.
#        Running this fan forwards helps turn right,
#        Running backwards helps turn left 
#    D10: PWM pulse out to right-side fan
#        Running this fan forwards helps turn left
#        Running backwards helps turn right
#    D13: LED output. Use to indicate calibrationstate of BNO055
#
# You may also want variables for other values such as your chosen PWM frequency,

rotation_command = pulseio.PulseIn(board.D5, maxlen=10, idle_state=False)
forward_command = pulseio.PulseIn(board.D6)

right_fan = pwmio.PWMOut(board.D10, frequency=40, duty_cycle=3932.1)
left_fan = pwmio.PWMOut(board.D9, frequency=40, duty_cycle=3932.1)

led_out = pwmio.PWMOut(board.D13, frequency=5000, duty_cycle=65000)

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# When adding integration capability
# You will need a variable to accumulate the integral term, etc. 
# You will also need to keep track of the time (from time.monotonic())
# of the previous iteration from the loop so you can accumulate the
# right amount

kp = 0.0001
ki = 0
kd = 0.001

first_loop = True

# Start an infinite loop here:
while True:
    start_time = time.time()

    angle_setpoint = get_desired_angle(rotation_command)
    angle_measurement = sensor.euler[0]

    if first_loop:
        prev_angle = angle_setpoint
        direction_PID = PIDController(angle_setpoint, angle_measurement, kp, ki, kd)
        first_loop = False

    if prev_angle != angle_setpoint:
        direction_PID.set_setpoint(angle_setpoint)

    sleep(0.12)
    prev_angle = angle_setpoint
    end_time = time.time()

    dt = end_time-start_time
    pid_signal = direction_PID.timestep(angle_measurement, dt = dt)

    des_motor_spd = min(0.01, pid_signal)

    print(f'[PID SIGNAL]         -     {pid_signal}')
    print(f'[ANGLE SETPOINT]     -     {angle_setpoint}')
    print(f'[ANGLE ACTUAL]       -     {angle_measurement}')

    set_motor_speed(right_fan, pid_signal)
    set_motor_speed(left_fan, -pid_signal)

    # print(get_rotation_signal(rotation_command))

    # print(f'[BNO055 CALIBRATION STATUS] - {sensor.calibrated}')
    # print(f'[ROTATION SIGNAL] - {rotation_signal}')
    # print(f'[GYRO] - {sensor.euler}')

    # Check whether the BNO055 is calibrated
    # and turn on the LED on D13 as appopriate

    # Extract the euler angles (Heading, Roll, Pitch)
    # in degrees from the IMU
    
    # Determine the commanded orientation based on from your pulse input from
    # pin D5
    
    # Select a coefficient for the proportional term
    # It will probably have units similar to output_command_ms/degree
    # Determine the proportional term by obtaining the error (subtracting
    # the commanded and actual headings), then multiplying by the proportional
    # coefficient
    #
    # TIP: If you just do simple subtraction you have a problem if the
    # commanded heading is (for example) +179 deg and the actual heading is
    # -179 deg. as it will try to turn the long way around.
    # The simplest solution is to add 360+180 deg. to the error, then use the
    # Python modulus operator (%) to get the remainder when dividing by
    # 360, then subtract 180 deg., e.g. replace simple subtraction with
    #  ((Heading_command - Heading + 360 + 180) % 360  - 180)

    # To start with use just the proportional term to determine the output rotation
    # command, which is an offset in ms from the nominal 1.5 ms that commands
    # the fully reversing motors to not move.

    # Bound the output rotation command so that it cannot exceed 0.5 or be less than
    # -0.5

    # Apply the output rotation command in opposite senses to determine the duty
    # cycle for the PWM outputs to the left- and right- side fans (pins D7, D8)

    pass # Done with loop


# Once you have the above working, you can test it out (making sure it is saved
# as "main.py" by running it with the simulator "python simulator.py"

# You can control the commanded heading by clicking in the lower-right quadrant.
# The x-y arrow in the upper-left quadrant should show you the orientation of
# your craft. You can add debugging prints as needed. Start out with a small
# proportional coefficient and increase it so that it makes the craft rotate
# at a reasonable rate. 

# You will find that the craft is extremely under-damped. This is because there is
# little air resistance to rotation.

# To get it to rapidly stabilize at the desired orientation you will need to add
# a derivative term. You can get the z axis rotation rate from the .gyro[2]
# property of the BNO055.
#
# Then you will need to define a derivative coefficient (which may have to be
# negative) represented as command_ms / (rad/sec). You can create the derivative
# by multiplying your derivative coefficient by the gyro rate and include it in the
# output rotation command.

# Once you have that working consider the case when some breeze tends to
# make the craft spontaneously rotate (set dynamic_model.enable_wind=True,
# above). To still equilibriate to the correct orientation you will need
# to add an integral term.
#
# Each time through the loop the integral gets added to it
# the integral coefficient * error * dt
# where dt is the difference in time (as measured with time.monotonic())
# from the previous loop iteration to this loop iteration. 
#  * Remember to use the TIP above when calculating the error.
#
# Every time you update the integral term you also have to bound it,
# lest it get too big and cause instability. This limit would probably
# have units of command_ms and would be a reasonably small fraction of
# the maximum command of 0.5 ms. Remember to bound it on both
# positive and negative sides. 
