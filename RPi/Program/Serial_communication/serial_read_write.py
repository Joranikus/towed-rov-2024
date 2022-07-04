import serial
import queue
from threading import Thread
from time import sleep


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
        self.running = True
        self.in_packet = bytearray()
        self.packet = bytearray()
        self.FROM_ARDUINO_TO_ARDUINO = ['depth', 'roll', 'pitch']

    def run(self):
        while self.running:
            # write data to serial from output_queue
            try:
                output_message = self.output_queue.get(timeout=0.001)
                self.__write_serial_data(output_message)
            except queue.Empty:
                pass
            except TypeError:
                pass

            # Read from serial and put into input_queue or arduinio_to_arduino_queqe
            message = self.__read_incoming_data()
            if message:
                try:
                    if not "Arduino" in message:
                        self.input_queue.put_nowait(message)
                        msg = message.split(':', 1)[0]
                        if msg in self.FROM_ARDUINO_TO_ARDUINO:
                            self.from_arduino_to_arduino_queue.put_nowait(message)
                            
                except queue.Full:
                            pass
                except TypeError:
                    pass

    def __write_serial_data(self, message):
        """
        write a string to serial port
        :param message: message to send to serial
        """
        if self.serial_port.isOpen():
            output = '<' + message + '>'
            # print(f"[serial_read_write]: Write message to serial: {output}")
            try:
                output = output.encode('utf-8')
                self.serial_port.write(output)
            except Exception as Err:
                print(f"[serial_read_write]: {Err} - Could not write data to serial port {self.serial_port}")
        else:
            self.serial_port.open()
            self.output_queue.append(message)  # take message back to queue if message was not sent

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
                msg_received = msg_received.strip().decode('utf-8')
                # print(f"[serial_read_write]: Read message from serial: {msg_received}")
                msg_received = msg_received.replace('<', '').replace('>', '')

        except Exception as Err:
            print(f"[serial_read_write]: {Err} - could not read data from {self.com_port}")
            self.serial_port.close()
            print(f"closed port {self.com_port}")
            sleep(1)
            
            try:
                self.serial_port.open()
            except serial.SerialException as ErrSer:
                print(f"{ErrSer}: could not reconnect! (loose wires?)")
            print(f"Is port open: {self.serial_port.is_open}")
        return msg_received

    def stop_thread(self):
        self.running = False
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
