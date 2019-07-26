import serial


class PyRPlidarSerial:
    
    def __init__(self):
        self._serial = None

    def open(self, port, baudrate, timeout):
        if self._serial is not None:
            self.disconnect()
        try:
            self._serial = serial.Serial(port, baudrate, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=timeout, dsrdtr=True)
        except serial.SerialException as err:
            print("Failed to connect to the rplidar")
    
    def close(self):
        if self._serial is None:
            return
        self._serial.close()
    
    def wait_data(self):
        pass
    
    def send_data(self, data):
        self._serial.write(data)
    
    def receive_data(self, size):
        return self._serial.read(size)

    def set_dtr(self, value):
        self._serial.dtr = value
