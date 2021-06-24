import math
import struct

from can import CANDevice

class CANSensors(CANDevice):
    """ 
        This class provide interface to the ... over CAN socket.
        Format:
            
            Frame type specifier: standard frame
            Frame format: DATA
            DLC: 8 bytes
            Identifier: 0x01 (by default)
    """

    def __init__(self, can_socket=None, id=0x01):
        
        super().__init__(can_socket, id)

        self.protocol = {
            "get_state":        b"\x9C",
            "reset":            b"\x05",
        }

        self.pos_scale = 25.4 / (360 * 4)

        self.g = 9.8067
        self.accel_offset = self.calibrate_imu()

        self.force_scale = 1
        self.force_offset = 0

    def __get_state(self):
        """
            uint16_t all
        """
        code = self.protocol["get_state"]
        self.command = code + 7 * b"\x00"
        self.execute()
        # no check here, since command and reply has different codes

        pos = self.from_bytes(self.reply[0:2], signed=False)
        force = self.from_bytes(self.reply[2:4], signed=False)
        
        accel_bytes = self.reply[4:]
        accel = struct.unpack('f', accel_bytes)[0]

        return pos, force, accel

    def get_state(self):
        _pos, _force, _accel = self.__get_state()
        pos = self.pos_scale * _pos
        force = self.force_scale * (_force - self.force_offset)
        # accel = _accel
        # accel = self.g * (_accel - 1)
        accel = _accel - self.g - self.accel_offset
        return pos, force, accel

    def calibrate_imu(self, n=10000):
        accel_offset = 0
        for _ in range(n):
            _, _, _accel = self.__get_state()
            accel = _accel - self.g
            accel_offset += accel
        accel_offset /= n
        return accel_offset


    def calibrate_force(self, mass=1.25, n=10000):
        '''
        mass - mass used for calibration
        n - number of measurments
        '''
        weight = mass * self.g

        force_offset = 0
        for _ in range(n):
            _, force, _ = self.__get_state()
            force_offset += force
        force_offset /= n

        force_scale = 0
        for _ in range(n):
            _, force, _ = self.__get_state()
            force_scale += weight / (force - force_offset)
        force_scale /= n

        self.force_scale = force_scale
        self.force_offset = force_offset
        print(self.force_scale, self.force_offset)

    def reset(self):
        """
            Description
        """
        code = self.protocol["reset"]
        self.command = code + 7 * b"\x00"
        self.execute()
        self.check()
