def get_rotation_signal(rotation_command):
    while(len(rotation_command) == 0):
        pass

    rotation_command.pause()
    rotation_signal = rotation_command.popleft()
    rotation_command.clear()
    rotation_command.resume()

    return rotation_signal

def get_desired_angle(rotation_command):
    '''
    Assuming A 1.5 ms pulse is considered 0 deg (N)
    A 1.75 ms pulse is considered 90 deg (E)
    A 1.25 ms pulse is considered -90 deg (W) 
    Either a 1.0 or a 2.0 ms pulse would map to +/- 180 deg (S)
    '''
    signal = get_rotation_signal(rotation_command)
    target_angle = -540 + 360 * signal/1000

    return target_angle
