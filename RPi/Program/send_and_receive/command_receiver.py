import zmq
from threading import Thread


class CommandReceiver(Thread):
    """
    ZMQ reply socket to receive request from the onshore computer
    """

    def __init__(self, cmd_queue):
        Thread.__init__(self)
        self.ctx = zmq.Context()
        self.connection = self.ctx.socket(zmq.REP)
        self.cmd_queue = cmd_queue
        self.ip = 'tcp://127.0.0.1:9004'

    def bind(self):
        self.connection.bind(self.ip)
        print("[STARTED] CommandReceiver")

    def send(self, data):
        self.connection.send_json(data)

    def recv(self):
        command_received = self.connection.recv_json()
        return command_received

    def run(self):
        print("started")
        self.bind()
        while True:
            print("run")
            try:
                cmd = self.recv()
                print("inc: ",cmd)
                self.cmd_queue.put(cmd)
                self.send({"success": True})
            except (Exception) as e:
                print(e, 'CommandReceiver')
