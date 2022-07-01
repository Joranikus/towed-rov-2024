import zmq
from threading import Thread


class CommandReceiver(Thread):
    """
    ZMQ reply socket to receive commands from the onshore computer (API)
    
    CommandReceiver will wait for a cmd, store the cmd in cmd_queue, and reply back with 'success: True'
    """

    def __init__(self, cmd_queue):
        Thread.__init__(self)
        self.ctx = zmq.Context()
        self.connection = self.ctx.socket(zmq.REP)
        self.cmd_queue = cmd_queue
        self.ip = 'tcp://192.168.0.102:9500'

    def bind(self):
        self.connection.bind(self.ip)
        print("[CommandReceiver]: Started")

    def send(self, data):
        self.connection.send_json(data)

    def recv(self):
        command_received = self.connection.recv_json()
        return command_received

    def run(self):
        self.bind()
        while True:
            try:
                print("[CommandReciever]: Waiting for command from API") 
                cmd = self.recv()
                print(f"[CommandReciever]: Commmand recieved from API: {cmd}")
                self.cmd_queue.put(cmd)
                self.send({"success": True})
            except Exception as e:
                print(e, 'CommandReceiver')
