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


#import dynamic_model

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

rotation_command = pulseio.PulseIn(board.D5,maxlen=8,idle_state=False)
forward_command = pulseio.PulseIn(board.D12,maxlen=8,idle_state=False)

right_fan = servo_breakout.channels[10]
left_fan = servo_breakout.channels[9]

bottom = servo_breakout.channels[11]
top = servo_breakout.channels[12]

initialize = True

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

if initialize:
    #initialize_motors([top, bottom], pwm_freq)
    initialize_reversing_motors([right_fan, left_fan], pwm_freq)

kp = 0.005
ki = 0
kd = .3

first_loop = True

try:
    # Start an infinite loop here:
    while True:
        start_time = time.time()
        delta_error = -sensor.gyro[2]
        
        angle_setpoint = get_desired_angle(rotation_command, pwm_freq)
        thrust = get_desired_thrust(forward_command, pwm_freq, max_thrust = 0.4)
        
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
        pid_signal = direction_PID.timestep(angle_measurement, dt = dt, delta_error = delta_error)

        des_motor_spd = np.sign(pid_signal) * min(0.15, abs(pid_signal))
        
        if thrust != 0:
            set_motor_speed(right_fan, des_motor_spd + thrust, pwm_freq, single_direction=False)
            set_motor_speed(left_fan, des_motor_spd - thrust, pwm_freq, single_direction=False)
        else:
            set_motor_speed(right_fan, des_motor_spd, pwm_freq, single_direction=False)
            set_motor_speed(left_fan, des_motor_spd, pwm_freq, single_direction=False)
         
        
        #print(f'[PID SIGNAL]         -     {pid_signal}')
        print(f'[SETPOINT]     -     {angle_setpoint}\n')
        print(f'[ACTUAL]       -     {angle_measurement}')
        #print(f'[GYRO] - {sensor.euler}')
        
        #print(f'Right fan: {round(get_motor_speed(right_fan, pwm_freq),4)}, {right_fan.duty_cycle}')
        #print(f'Left fan: {round(get_motor_speed(left_fan, pwm_freq),4)}, {left_fan.duty_cycle}')
        
        #print(f'Top fan: {round(get_motor_speed(top, pwm_freq, single_direction=True),4)}, {top.duty_cycle}')
        #print(f'Bottom fan: {round(get_motor_speed(bottom, pwm_freq, single_direction=True),4)}, {bottom.duty_cycle}')
        

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
    
except KeyboardInterrupt:
    set_motor_speed(right_fan, 0, pwm_freq, single_direction=False)
    set_motor_speed(left_fan, 0, pwm_freq, single_direction=False)
    
    set_motor_speed(bottom, 0, pwm_frequency = pwm_freq, single_direction = True)
    set_motor_speed(top, 0, pwm_frequency = pwm_freq,  single_direction = True)
        #sleep(0.12)


