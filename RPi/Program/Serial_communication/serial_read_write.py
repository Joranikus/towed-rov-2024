import serial
import queue
from threading import Thread
from time import sleep

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
            # Send serial data from output_queue
            try:
                output_message = self.output_queue.get(timeout=0.001)
                self.__write_serial_data(output_message)
            except queue.Empty:
                pass
            except TypeError:
                pass

            # Get input from serial and put into input queue
            incoming_message = self.__read_incoming_data()
            for message in incoming_message:
                if message:
                    try:
                        if not "Arduino" in message:
                            self.input_queue.put_nowait(message)
                            msg = message.split(TERMINATOR, 1)[0]
                            if msg in self.FROM_ARDUINO_TO_ARDUINO:
                                self.from_arduino_to_arduino_queue.put_nowait(message)
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
        :return: message read from serial port as a string "name:number"
        """
        msg_received = ''
        try:
        
            if (self.serial_port.in_waiting > 0):
                msg_received = self.serial_port.readline()
                # format message: remove newline, decode from utf-8, remove outer symbols, 
                msg_received = msg_received.strip().decode('utf-8').replace('<', '').replace('>', '')

        except Exception as e:
            print(f"{e} - could not read data from {self.com_port} - serial_read_write")
          
            print(f"Is port open: {self.serial_port.is_open}")
            self.serial_port.close()
            print(f"closed port {self.com_port}")
            print(f"Is port open: {self.serial_port.is_open}")
            sleep(1)
            
            try:
                self.serial_port.open()
            except serial.SerialException as ErrSer:
                print(ErrSer.__class__.__name__)
                print(f"{ErrSer}: could not reconnect! (loose wires?)")
                
            print(f"open port {self.com_port}")
            print(f"Is port open: {self.serial_port.is_open}")
        return msg_received

    def stop_thread(self):
        self.stop = True
        self.serial_port.close()
        print('closed port', self.serial_port.isOpen())


if __name__ == '__main__':
    q1 = queue.Queue()
    q2 = queue.Queue()
    q3 = queue.Queue()
    ser = SerialWriterReader(q1, q2, '/dev/ttyACM1', 115200, q3)  # tip: run the serial_finder to find a port
    ser.daemon = True
    ser.start()
    while True:
        pass
