import json
import time
import queue
from send_and_receive.message_dispatcher import MessageDispatcher
from threading import Thread
from multiprocessing import Queue


class PayloadWriter(Thread):
    """
    Creates payload from received data from the Arduinos
    """
    def __init__(self, sensor_list, alarm_list, gui_command_queue, thread_running_event):
        Thread.__init__(self)
        self.sensor_list = sensor_list
        self.alarm_list = alarm_list
        self.message_queue = Queue()
        self.gui_command_queue = gui_command_queue
        self.message_dispatcher = MessageDispatcher(self.message_queue)
        self.interval_sensor = 0.1
        self.interval_alarm = 3
        self.thread_running_event = thread_running_event

    def run(self):
        """
        Create responce command instant and a sensor command every 100ms
        """
        previous_sensor = 0
        previous_alarm = 0
        while self.thread_running_event.is_set():
            self.__add_respons_to_queue()
            self.message_dispatcher.publish()
            #self.__add_alarm_to_queue()
            now  = time.monotonic()
            if now - previous_sensor >= self.interval_sensor:
                self.__merge_sensor_payload()
                self.message_dispatcher.publish()
                previous_sensor = now
            if now - previous_alarm >= self.interval_alarm:
                self.__merge_alarm_payload()
                self.message_dispatcher.publish()
                previous_alarm = now       




    def __merge_sensor_payload(self):
        """
        Takes all sensor and creates a json with sensor data, adds the json to a queue.
        """
        sensors = []
        if self.sensor_list:
            for sensor in self.sensor_list:
                sensor_name = sensor.name
                sensor_value = sensor.value
                sensors.append({"name" : sensor_name,
                                "value" : sensor_value})
            time.sleep(0.001)
            sensor_structure = {
                "payload_name": "sensor_data",
                "payload_data": sensors
            }
            self.message_queue.put(sensor_structure)


    def __add_respons_to_queue(self):
        """
        Checks if there are a responce to be made, and if true, it's made
        """
        try:
            message = self.gui_command_queue.get(timeout=0.001)
            message = message.split(":",1)
            if message[1] == "True":
                message[1] = True
            elif message[1] == "False":
                message[1] = False
            json_command = [{"name" : message[0],
                        "success" : message[1]}]
            command_structure = {
                "payload_name": "response",
                "payload_data": json_command
            }
            print(f"json_command: {json_command}")
            self.message_queue.put(command_structure)
            print(command_structure, "responce")
        except queue.Empty:
            pass
        except IndexError:
            pass

    def __merge_alarm_payload(self):
        """
        Takes all alarms and creates a json with sensor data, adds the json to a queue.
        """
        alarms = []
        if self.alarm_list:
            for alarm in self.alarm_list:
                alarm_name = alarm.name
                alarm_value = alarm.value
                alarms.append({"name" : alarm_name,
                                "value" : alarm_value})
            time.sleep(0.001)
            alarm_structure = {
                "payload_name": "alarms",
                "payload_data": alarms
            }
            self.message_queue.put(alarm_structure)
