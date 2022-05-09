# Put the various CircuitPython import statements here
import board
import busio

import adafruit_bno055
import adafruit_pca9685
import measure_servofreq_adafruit_pca9685
import pwmio
import pulseio

from digitalio import Direction
from digitalio import DigitalInOut

from motor_control import *
from setpoints import *

from time import sleep
import time

'''
import dynamic_model
dynamic_model.enable_wind=False   # Set this to True to add a moment from the wind
'''

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
#    D11: Lift command for bottom motor
#    D12: Lift command for top motor
#    D13: LED output. Use to indicate calibrationstate of BNO055
#
# You may also want variables for other values such as your chosen PWM frequency,

i2c = busio.I2C(board.SCL, board.SDA)
import pulse_commands
import adafruit_pca9685
import measure_servofreq_adafruit_pca9685

sensor = adafruit_bno055.BNO055_I2C(i2c)

servo_breakout = adafruit_pca9685.PCA9685(i2c)
servo_breakout.frequency = int(40) # This is the commanded frequency, but it is no
#Perform measurement:
servo_breakout.channels[15].duty_cycle=int(1.5e-3*servo_breakout.frequency*65536.0)
pwm_freq = measure_servofreq_adafruit_pca9685.measure_servofreq(servo_breakout,15,board.D26)# Then let’s suppose you are commanding channel #0 (LEFT OUT):

#rotation_command = pulseio.PulseIn(board.D5,maxlen=8,idle_state=False)
#forward_command = pulseio.PulseIn(board.D6,maxlen=8,idle_state=False)

right_fan = servo_breakout.channels[10]
left_fan = servo_breakout.channels[9]

bottom = servo_breakout.channels[11]
top = servo_breakout.channels[12]

def initialize_motors(motors, pwm_frequency):
    pulse_width = 1/pwm_frequency # Should be 0.025 s, but get the measurement

    low = int(0.001/pulse_width * 65536)
    high = int(0.002/pulse_width * 65536)

    print('Initializing single direction motors')
    for motor in motors:
        motor.duty_cycle = low
    sleep(3)
    
    for motor in motors:
        motor.duty_cycle = high
    sleep(3)
    
    for motor in motors:
        motor.duty_cycle = low
    sleep(3)

def initialize_reversing_motors(motors, pwm_frequency):
    pulse_width = 1/pwm_frequency # Should be 0.025 s, but get the measurement
    print('Initializing reversing motors')
    med = int(0.0015/pulse_width * 65536)
    for motor in motors:
        motor.duty_cycle = med
    sleep(3)
    
def initialize_reversing_motor(motor, pwm_frequency):
    pulse_width = 1/pwm_frequency # Should be 0.025 s, but get the measurement
    print('Initializing reversing motor')
    med = int(0.0015/pulse_width * 65536)
    motor.duty_cycle = med
    sleep(3)

initialize_motors([top, bottom], pwm_freq)
initialize_reversing_motors([right_fan, left_fan], pwm_freq)

'''
kp = 0.005
ki = 0
kd = 0.2
'''

kp = 0.005
ki = 0
kd = 0.3

first_loop = True

# Start an infinite loop here:
while True:
    start_time = time.time()
    
    #(angle_setpoint,thrust_setpoint) = pulse_commands.get_pulse_commands([rotation_command,forward_command])

    angle_setpoint = 20#get_desired_angle(rotat)
    angle_measurement = sensor.euler[0]

    if first_loop:
        prev_angle = angle_setpoint
        direction_PID = PIDController(angle_setpoint, angle_measurement, kp, ki, kd)
        first_loop = False

    if prev_angle != angle_setpoint:
        direction_PID.set_setpoint(angle_setpoint)

    prev_angle = angle_setpoint
    end_time = time.time()

    dt = end_time-start_time
    pid_signal = direction_PID.timestep(angle_measurement, dt = dt)

    des_motor_spd = np.sign(pid_signal) * min(0.15, abs(pid_signal))

    print(f'[PID SIGNAL]         -     {pid_signal}')
    print(f'[ANGLE SETPOINT]     -     {angle_setpoint}')
    print(f'[ANGLE ACTUAL]       -     {angle_measurement}')
    '''
    for i in [0]:
        # -0.1 is zero lol
        set_motor_speed(bottom, i)
        print(f'Right fan: {round(get_motor_speed(bottom),2)}, {bottom.duty_cycle}')
        sleep(1)
    '''
    
    set_motor_speed(right_fan, des_motor_spd, pwm_freq, single_direction=False)
    set_motor_speed(left_fan, des_motor_spd, pwm_freq, single_direction=False)
    
    #set_motor_speed(bottom, -0.1, single_direction = True)
    #set_motor_speed(top, 0.1, single_direction = True)
    #sleep(0.12)

    #print(get_rotation_signal(rotation_command))

    # print(f'[BNO055 CALIBRATION STATUS] - {sensor.calibrated}')
    #print(f'[ROTATION SIGNAL] - {rotation_signal}')
    print(f'[GYRO] - {sensor.euler}')
    print(f'[ROT CMD] - {angle_setpoint}')
    #right_fan.duty_cycle = 4000
    #left_fan.duty_cycle = 4000
    
    print(f'Right fan: {round(get_motor_speed(right_fan, pwm_freq),4)}, {right_fan.duty_cycle}')
    print(f'Left fan: {round(get_motor_speed(left_fan, pwm_freq),4)}, {left_fan.duty_cycle}')


    #print(f'Right fan: {round(des_motor_spd,4)}')
    #print(f'Left fan: {round(des_motor_spd,4)}')
    #sleep(0.12)

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

