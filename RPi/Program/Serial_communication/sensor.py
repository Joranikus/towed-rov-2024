class Sensor:
    """
    Reprecent sensor object storing the name and the value of a sensor
    """
    def __init__(self, name, value):
        self.name = name
        self.value = value

    def set_sensor_value(self, value):
        self.value = value

    def get_sensor_value(self):
        return self.value

    def set_sensor_name(self, name):
        self.name = name

    def get_sensor_name(self):
        return self.name
