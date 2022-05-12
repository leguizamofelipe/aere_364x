import pulse_commands
import numpy as np

def get_signal_ms(command):
    
    '''
    while(len(rotation_command) == 0):
        pass

    rotation_command.pause()
    rotation_signal = rotation_command.popleft()
    rotation_command.clear()
    rotation_command.resume()
    '''
    signal = pulse_commands.get_pulse_commands([command])

    return signal[0]

def get_desired_angle(rotation_command, pwm_freq):
    '''
    Assuming A 1.5 ms pulse is considered 0 deg (N)
    A 1.75 ms pulse is considered 90 deg (E)
    A 1.25 ms pulse is considered -90 deg (W) 
    Either a 1.0 or a 2.0 ms pulse would map to +/- 180 deg (S)
    '''
    pulse_width_micros = get_signal_ms(rotation_command)
    '''
    pulse_width = 1/pwm_frequency
    
    min =
    '''
    full_range_angle = 360
    full_range_pw = 1000
    target_angle = (1500-pulse_width_micros)*full_range_angle/full_range_pw

    return 1.25*target_angle


def get_desired_thrust(thrust_command, pwm_freq, max_thrust = 0.3):
    '''
    Assuming A 1.5 ms pulse is considered 0 deg (N)
    A 1.75 ms pulse is considered 90 deg (E)
    A 1.25 ms pulse is considered -90 deg (W) 
    Either a 1.0 or a 2.0 ms pulse would map to +/- 180 deg (S)
    '''
    forward = get_signal_ms(thrust_command)
    #print(f'Forward \n\n\n {forward}')
    
    lift_power = (forward-1500)/500 * max_thrust
    
    lift_power = np.sign(lift_power) * min(max_thrust, abs(lift_power))
    lift_power = max(0, lift_power)
    
    if lift_power < 0.05:
        lift_power = 0

    return lift_power
