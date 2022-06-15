import queue
from send_and_receive.message_receiver import MessageReceiver
from send_and_receive.command_receiver import CommandReceiver
from GPIO_writer import GPIOWriter
from threading import Thread
from multiprocessing import Queue


class PayloadHandler(Thread):
    def __init__(self, sensor_list, command_queue, gui_command_queue, seafloor_sonar_queue, new_set_point_event,
                 start_event, stop_event):
        """
        Handles every sensor data or request from the onshore suitcase RPi or laptop
        :param sensor_list:
        :param command_queue:
        """
        Thread.__init__(self)
        self.sensor_list = sensor_list
        self.received_data = ''
        self.message_queue = Queue()
        self.message_receiver = MessageReceiver(self.message_queue)
        self.message_receiver.daemon = True
        self.message_receiver.start()
        self.command_receiver = CommandReceiver(self.message_queue)
        self.command_receiver.daemon = True
        self.command_receiver.start()
        self.gui_command_queue = gui_command_queue
        self.command_queue = command_queue
        self.gpio_writer = GPIOWriter()
        self.start_event = start_event
        self.stop_event = stop_event
        self.seafloor_sonar_queue = seafloor_sonar_queue
        self.new_set_point_event = new_set_point_event
        self.depth_or_seafloor = ""  # dont know where to send yet, due to the seafloor tracking not implemented.
        self.commands_to_serial = ['com_port_search', 'reset', 'pid_depth_p', 'pid_depth_i',
                                   'pid_depth_d', 'pid_roll_p', 'pid_roll_i', 'pid_roll_d',
                                   'manual_wing_pos', 'emergency_surface',
                                   'depth_beneath_rov_offset', 'depth_rov_offset', 'set_point_depth', 'auto_mode']

    def run(self):
        while True:
            # self.__update_pitch()
            self.__sort_payload()

    def __sort_payload(self):
        """
        takes the received payload and reads it and either handles it or forwards it to the serial output queue
        """
        try:

            payload_type, payload_names, payload_data = self.payload_reader.read_payload(
                self.message_queue.get(timeout=0.01))
            if payload_type == 'commands':
                if payload_data[0] in self.commands_to_serial:
                    self.command_queue.put(str(payload_data[0]) + ':' + str(payload_data[1]))
                elif payload_data[0] == 'start_system':
                    if payload_data[1] == True:
                        self.start_event.set()
                    elif payload_data[1] == False:
                        self.stop_event.set()
                    self.gui_command_queue.put(str(payload_data[0]) + ':' + str(payload_data[1]))
                elif payload_data[0] == 'brightness_light':
                    # print("light")
                    if self.gpio_writer.set_lights(payload_data[1]):
                        self.gui_command_queue.put(payload_data[0] + ':' + str(True))
                elif payload_data[0] == 'camera_offset_angle':
                    if self.gpio_writer.set_manual_offset_camera_tilt(payload_data[1]):
                        self.gui_command_queue.put(payload_data[0] + ':' + str(True))
                elif payload_data[0] == 'depth_or_seafloor':
                    self.depth_or_seafloor = payload_data[1]

                    self.gui_command_queue.put(payload_data[0] + ':' + str(True))

                elif payload_data[0] == "has_traveled_set_distance":
                    self.new_set_point_event.set()

            elif payload_type == 'settings':
                if payload_data[0] == 'arduino_sensor':
                    self.command_queue.put('arduino_sensor:' + payload_data[1] + ':' + payload_data[2])
                if payload_data[0] == 'arduino_stepper':
                    self.command_queue.put('arduino_stepper:' + payload_data[1] + ':' + payload_data[2])
            elif payload_type == 'sensor_data':
                if 'depth_beneath_boat' in payload_data:
                    i = payload_data.index('depth_beneath_boat')+1
                    self.seafloor_sonar_queue.put(payload_data[i])

        except queue.Empty:
            pass

    def __update_pitch(self):
        """
        reads the value of the pitch sensor and send this to the camera servo,
        so it can adjust its angle according to pitch
        """
        try:
            for sensor in self.sensor_list:
                if sensor.name.strip() == 'pitch':
                    # self.gpio_writer.adjust_camera(sensor.value.strip())
                    pass
        except RuntimeError:
            pass


if __name__ == '__main__':
    pass
