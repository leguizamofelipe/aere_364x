import numpy as np

class PIDController():
    def __init__(self, setpoint, measurement, kp = 0, ki = 0, kd = 0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integrated_error = 0
        self.prev_error = 0
        self.first_loop = True

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.integrated_error = 0
        self.prev_error = 0
        self.first_loop = True
    
    def timestep(self, measurement, dt):
        try:
            error = ((measurement - self.setpoint + 360 + 180) % 360  - 180) #measurement - self.setpoint
        except:
            error = self.prev_error
        self.integrated_error += error * dt
        if self.first_loop == True:
            delta_error = 0
        else:
            delta_error = error - self.prev_error

        self.prev_error = error
        self.first_loop=False

        return self.kp * error + self.ki * self.integrated_error + self.kd * delta_error

def set_motor_speed(motor_pwm, value, pwm_frequency, single_direction = False):
    '''
    Assuming a pulse width of 25 ms
    Assuming 1ms (full reverse)
    Assuming 1.5ms (neutral)
    Assuming 2ms (full forward)
    '''
    pulse_width = 1/pwm_frequency # Should be 0.025 s, but get the measurement

    low = 0.001/pulse_width * 65536
    high = 0.002/pulse_width * 65536

    if single_direction:
        duty_cycle = int(low + value*(high-low))
    else:
        slope = (high-low)/2
        duty_cycle = int(low + (value+1) * slope)
        
    motor_pwm.duty_cycle = duty_cycle

def get_motor_speed(motor_pwm, pwm_freq, single_direction = False):
    pulse_width = 1/pwm_freq
    
    low = 0.001/pulse_width * 65536
    high = 0.002/pulse_width * 65536

    if single_direction:
        value = (motor_pwm.duty_cycle - low)/(high-low)
    else:
        slope = (high-low)/2
        value = (motor_pwm.duty_cycle-low)/slope -1

    return value
