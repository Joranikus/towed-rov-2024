import serial
import queue
from threading import Thread
from time import time

ENCODING = 'utf-8'
TERMINATOR = ':'
START_CHAR = '<'
END_CHAR = '>'
NEW_LINE = '\n'
START = b'<'
STOP = b'>'


class SerialWriterReader(Thread):
    def __init__(self, output_queue, input_queue, com_port, baud_rate, from_arduino_to_arduino_queue):
        Thread.__init__(self)
        self.from_arduino_to_arduino_queue = from_arduino_to_arduino_queue
        self.output_queue = output_queue
        self.input_queue = input_queue
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.serial_port = serial.Serial(
            port=self.com_port,
            baudrate=self.baud_rate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0)
        self.stop = False
        self.in_packet = bytearray()
        self.packet = bytearray()
        self.last_output = ''
        self.FROM_ARDUINO_TO_ARDUINO = ['depth', 'roll', 'pitch']
        self.counter = 0

    def run(self):
        while not self.stop:
            try:
                test = self.output_queue.get(timeout=0.001)

                self.__write_serial_data(test)
                #print("test",test)
            except queue.Empty:
                pass
            except TypeError:
                pass
            incoming_message = self.__read_incoming_data()
            for message in incoming_message:
                if message:
                    try:
                        if not "Arduino" in message:
                            try:
                                self.input_queue.put_nowait(message)
                                msg = message.split(TERMINATOR, 1)[0]
                                if msg in self.FROM_ARDUINO_TO_ARDUINO:
                                    try:
                                        self.from_arduino_to_arduino_queue.put_nowait(message)
                                    except queue.Full:
                                        pass
                            except queue.Full:
                                pass
                    except TypeError:
                        pass

    def __write_serial_data(self, message):
        """
        write message to serial port
        :param message: message to send to serial
        """
        #print(self.serial_port.isOpen(),self.com_port)
        if self.serial_port.isOpen():
            output = START_CHAR + message + END_CHAR + NEW_LINE
            if output != self.last_output:
                try:
                    output = output.encode(ENCODING)

                    self.serial_port.write(output)
                    #print(output, '    ', str(self.counter), '    ', self.baud_rate)
                    self.counter = self.counter + 1
                    self.last_output = output
                except (Exception) as e:
                    print(e, 'serial writer')
        else:
            self.serial_port.open()
            print('Serial port not open : ' + str(self.com_port))
            self.output_queue.append(message)

    def handle_packet(self, data):
        message_received = data.decode(ENCODING). \
            replace(START_CHAR, ""). \
            replace(END_CHAR, ""). \
            replace(" ", "")
        #print(message_received)
        return message_received

    def __read_incoming_data(self):
        """
        reads from serial port
        :return: message read from serial port
        """
        data_received = []
        message_received = ""
        try:
            data = self.serial_port.read(self.serial_port.in_waiting or 1)
            for byte in serial.iterbytes(data):
                if byte == START:
                    self.in_packet = True
                elif byte == STOP:
                    self.in_packet = False
                    data_received.append(self.handle_packet(bytes(self.packet)))  # make read-only copy
                    del self.packet[:]
                elif self.in_packet:
                    self.packet.extend(byte)
                else:
                    pass

        except (Exception) as e:
            print(e, "serial1")
        return data_received

    def stop_thread(self):
        self.stop = True
        self.serial_port.close()
        print('closed port', self.serial_port.isOpen())


if __name__ == '__main__':
    q1 = queue.Queue()
    q2 = queue.Queue()
    q3 = queue.Queue()
    ser = SerialWriterReader(q1, q2, '/dev/ttyUSB0', 115200, q3)
    ser.daemon = True
    ser.start()
    while True:
        pass
