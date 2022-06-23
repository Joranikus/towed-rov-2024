import threading
import queue
from time import sleep
from sea_floor_tracker import SeafloorTracker
from payloads.payload_writer import PayloadWriter
from payloads.payload_handler import PayloadHandler
from Serial_communication.serial_handler import SerialHandler
from multiprocessing import Event, Queue
from alarm import Alarm

arduino_command_queue = Queue()
sensor_list = []
alarm_list = []
gui_command_queue = Queue()
seafloor_sonar_queue = queue.Queue()
flag_queue = queue.Queue()
set_point_queue = Queue()
rov_depth_queue = Queue()
new_set_point_event = Event()
start_event = Event()
stop_event = Event()
thread_running_event = Event()

#append alarms
alarm1 = Alarm('water_leakage', 'False')
alarm2 = Alarm('no_legal_sp', 'False')
alarm3 = Alarm('incline_too_steep', 'False')
alarm_list.append(alarm1)
alarm_list.append(alarm2)
alarm_list.append(alarm3)

#Creating threads
payload_handler = PayloadHandler(sensor_list, arduino_command_queue, gui_command_queue,
                                 seafloor_sonar_queue, new_set_point_event,
                                 start_event, stop_event)

payload_writer = PayloadWriter(sensor_list, alarm_list, gui_command_queue, thread_running_event)
serial_handler = SerialHandler(sensor_list, alarm_list, arduino_command_queue, gui_command_queue,
                                       set_point_queue, rov_depth_queue, thread_running_event)
sea_floor_tracker = SeafloorTracker(150, 20, 20, 6, 10, seafloor_sonar_queue, new_set_point_event, set_point_queue, alarm_list)

if __name__ == "__main__":
    arduino_command_queue = Queue()
    sensor_list = []
    gui_command_queue = Queue()
    seafloor_sonar_queue = queue.Queue()
    flag_queue = queue.Queue()
    set_point_queue = Queue()
    rov_depth_queue = Queue()
    new_set_point_event = Event()
    start_event = Event()
    start_event.set()
    stop_event = Event()
    thread_running_event = Event()
    # Creating threads
    payload_handler = PayloadHandler(sensor_list, arduino_command_queue, gui_command_queue,
                                     seafloor_sonar_queue, new_set_point_event,
                                     start_event, stop_event)
    payload_writer = PayloadWriter(sensor_list, alarm_list, gui_command_queue, thread_running_event)
    serial_handler = SerialHandler(sensor_list, arduino_command_queue, gui_command_queue,
                                   set_point_queue, rov_depth_queue, thread_running_event)
    sea_floor_tracker = SeafloorTracker(length_rope=200, desired_distance=15, min_dist=10, dist_to_skip=5,
                                        depth_of_rov=0, depth_beneath_boat=seafloor_sonar_queue,
                                        new_set_point_event=new_set_point_event, set_point_queue=set_point_queue)

    payload_handler.daemon = True
    payload_handler.start()
    sea_floor_tracker.daemon = True
    #print(sea_floor_tracker.daemon)
    sea_floor_tracker.start()

    print(sea_floor_tracker.is_alive())


    def __start_communication_threads():
        try:
            thread_running_event.set()
            payload_writer.daemon = True
            payload_writer.start()

            serial_handler = SerialHandler(sensor_list, arduino_command_queue, gui_command_queue,
            set_point_queue, rov_depth_queue, thread_running_event)
            serial_handler.daemon = True
            serial_handler.start()
        except (Exception) as e:
            print(e, ' main')


    def __stop_communication_threads():
        thread_running_event.clear()


    try:
        if start_event.is_set() and not (payload_writer.is_alive() or serial_handler.is_alive()):
            print('starting threads')
            __start_communication_threads()
            start_event.clear()
        if stop_event.is_set() and (payload_writer.is_alive() or serial_handler.is_alive()):
            __stop_communication_threads()
            stop_event.clear()
    except (Exception) as e:
        print(e, 'main')

def __stop_communication_threads():
    thread_running_event.clear()

try:
    if start_event.is_set() and not(payload_writer.is_alive() or serial_handler.is_alive()):
        print('starting threads')
        __start_communication_threads()
        start_event.clear()
    if stop_event.is_set() and (payload_writer.is_alive() or serial_handler.is_alive()):
        __stop_communication_threads()
        stop_event.clear()
except (Exception) as e:
    print(e, 'main')


print ('Press Ctrl+C to exit')
try:
    while True:
        sleep(5)
except KeyboardInterrupt:
    print("exit prg")
