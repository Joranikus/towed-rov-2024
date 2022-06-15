import pigpio


class GPIOWriter:
    """
    Writes PWM signal to the GPIO output for controlling servo and lights.
    """
    def __init__(self):
        self.pin_led_lights = 12
        self.pin_camera_tilt = 18
        self.camera_tilt_offset = 0
        #self.lights =  pigpio.pi()
        #self.lights.set_mode(self.pin_led_lights, pigpio.OUTPUT)
        #self.camera_tilt = pigpio.pi()
        #self.camera_tilt.set_mode(self.pin_camera_tilt, pigpio.OUTPUT)
        self.adjust_camera(0)
        self.pitch = 0

    def set_lights(self, brightness):
        """
        Sets the given brightness to the led lights
        :param brightness: the desired brightness
        """
        out_min = 1100
        out_max = 1900
        in_min = 0
        in_max = 100
        PWM = self.__map(brightness, in_max, in_min, out_max, out_min)
       # print(PWM)
        # self.lights.set_PWM_dutycycle(self.pin_led_lights, 255)
        return True

    def adjust_camera(self, pitch):
        """
        adjust the pitch of the camera according to pitch of rov, so camera is always horizontal.
        :param pitch: the pitch of ROV
        """
        self.pitch = pitch
        out_min = 1100
        out_max = 1900
        in_min = -45
        in_max = 45
        adjusted_pitch = self.pitch + self.camera_tilt_offset
        pwm = self.__map(adjusted_pitch, in_max, in_min, out_max, out_min)
        #self.camera_tilt.set_servo_pulsewidth(self.pin_camera_tilt, pwm)
        
#         print(self.camera_tilt.get_servo_pulsewidth(self.pin_camera_tilt),'dfs')

    def __map(self, in_value, in_max, in_min, out_max, out_min):
        """
        maps an in value between a max-in and min-in to max-out and min-out
        :param in_value: the value to be mapped
        :param in_max: max in-value
        :param in_min: min in-value
        :param out_max: max out-value
        :param out_min: min out-value
        :return: the mapped value
        """
        x = min(in_max, max(in_min, in_value))
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def set_manual_offset_camera_tilt(self, offset):
        """
        sets a manual offset of the camera tilt
        :param offset: the given offset
        """
        self.camera_tilt_offset = offset
        self.adjust_camera(self.pitch)
        return True


if __name__ == '__main__':
    gpi = GPIOWriter()
    while True:
        pass
