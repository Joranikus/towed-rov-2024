import queue
from threading import Thread
from Serial_communication.sensor import Sensor


class SerialMessageRecivedHandler(Thread):
    """
    Handling messages from the arduino. Sensor data are added to a list of sensor object which are updated when a
    message is received.
    """
    def __init__(self, gui_command_queue, sensor_list, alarm_list, valid_sensor_list, valid_alarm_list, message_queue):
        Thread.__init__(self)
        self.message_received_queue = gui_command_queue
        self.sensor_list = sensor_list
        self.alarm_list = alarm_list
        self.valid_sensor_list = valid_sensor_list
        self.valid_alarm_list = valid_alarm_list
        self.valid_commands = ['reset', 'IMU', 'SensorArduino', 'StepperArduino',
                               'depth_beneath_rov_offset', 'depth_rov_offset', 'pid_depth_p',
                               'pid_depth_i', 'pid_depth_d', 'pid_roll_p', 'pid_roll_i', 'pid_roll_d',
                               'auto_mode', 'manual_wing_pos',
                               'emergency_surface'
                               ]
        self.message_queue = message_queue
        self.count = 0

    def run(self):
        """
        Gets message from a queue with data from the Arduinos, forward the responce to payload_writer, or if sensor data
        the list of sensor object will be updated.
        """
        counter_sent = 0
        counter_skip = 0
        while True:
            try:
                received_message = self.message_queue.get(timeout=0.01)
                counter_sent = counter_sent + 1
                # print(received_message)
                message = received_message.split(':', 1)
                if message[0] in self.valid_commands:
                    print("----------------------")
                    print(received_message)
                    print("----------------------")
                    self.message_received_queue.put(received_message)
                elif message[0] in self.valid_alarm_list:
                    self.__add_alarm(message)
                else:
                    self.__add_sensor(message)
            except queue.Empty:
                counter_skip = counter_skip + 1
            except ValueError:
                pass

    def __add_sensor(self, message):
        """
        checks of message is an expected on, and if, it adds the message/sensor to the list of sensors.
        If the sensor is already in the list it updates it's value. Filtering out startup message from Arduinos.
        :param message: message read from the Arduino
        """
        if ('IMU' or 'sensorArduino' or 'stepperArduino') in message[0]:
            pass
        else:
            try:
                name = message[0]
                value = float(message[1])
                if name in self.valid_sensor_list:
                    for sensor in self.sensor_list:
                        if sensor.get_sensor_name() == name:
                            sensor.set_sensor_value(value)
                            break
                    else:
                        sensor = Sensor(name, value)
                        self.sensor_list.append(sensor)
            except IndexError:
                pass
    
    def __add_alarm(self, message):
        """
        checks of message is an expected on, and if, it adds the message/alarm to the list of alarm.
        :param message: message read from the Arduino
        """
        
        try:
            name = message[0]
            value = message[1]
            if name in self.valid_alarm_list:
                for alarm in self.alarm_list:
                    if alarm.get_sensor_name() == name:
                        alarm.set_alarm_value(value)
                        break                
        except IndexError:
            pass
   

if __name__ == '__main__':
    q1 = queue.Queue()
    q2 = queue.Queue()
    q2.put("sdtig:2")
    test = SerialMessageRecivedHandler(q1, [], {}, q2)
    test.start()
    while True:
        pass
