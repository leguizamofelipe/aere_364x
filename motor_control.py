import numpy as np

class PIDController():
    def __init__(self, setpoint, measurement, kp = 0, ki = 0, kd = 0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.measurement = measurement
        self.integrated_error = 0
        self.prev_error = 0
        self.first_loop = True

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
    
    def timestep(self, measurement, dt):
        error = measurement - self.setpoint
        self.integrated_error += error * dt
        if self.first_loop == True:
            delta_error = 0
        else:
            delta_error = error - self.prev_error

        self.prev_error = error

        return self.kp * error + self.ki * self.integrated_error + self.kd * delta_error

def set_motor_speed(motor_pwm, value):
    '''
    Assuming a pulse width of 25 ms
    Assuming 1ms (full reverse)
    Assuming 1.5ms (neutral)
    Assuming 2ms (full forward)
    
    Min Duty Cycle = 2621
    Neutral Duty Cycle = 3932.1
    Max Duty Cycle = 5292.8
    '''

    duty_cycle = 3932.1 + value * 1311.1
    motor_pwm.duty_cycle = duty_cycle

def get_motor_speed(motor_pwm):
    value = (motor_pwm.duty_cycle - 3932.1)/1311.1

    return value
