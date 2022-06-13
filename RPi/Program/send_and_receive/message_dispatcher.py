import zmq
import queue
from time import time


class MessageDispatcher():
    """
    ZMQ publisher for sending sensor data and responces to commands back to the onshore laptop
    """
    def __init__(self, data_queue):
        self.ip = 'tcp://127.0.0.1:8050'
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.connect()
        self.data_queue = data_queue
        self.counter = 0

    def publish(self):
        try:
            self.socket.send_json(self.data_queue.get(timeout=0.01))
        except queue.Empty:
            self.counter_skip += 1
            test = self.data_queue.get(timeout=0.01)
            self.counter_sent += 1
            # print("send",test)
            self.socket.send_json(test)
            # self.counter = self.counter +1
            # print(self.counter)
        except queue.Empty:
            self.counter_skip += 1
        # DEBUGG HELP

    #         if (time() - self.start) > 5:
    #             print("TIME________: ", str(time() - self.start))
    #             print("Times sent  : ", str(self.counter_sent))
    #             print("Times skips : ", str(self.counter_skip))
    #             self.counter_sent = 0
    #             self.counter_skip = 0
    #             self.start = time()

    def disconnect(self):
        self.socket.disconnect()

    def connect(self):
        self.socket.bind(self.ip)
