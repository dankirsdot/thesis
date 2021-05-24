import math

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
            "get_imu_settings": b"\x9B",
            "get_state":        b"\x9C",
            "reset":            b"\x05",
        }

        self.pos_scale = 25.4 / (360 * 4)
        self.pos_offset = 0
        
        self.force_scale = 1
        self.force_offset = 0

        self.accel_scale = 0.000061
        self.accel_offset = 0.602295

    def get_imu_settings(self):
        """
            Description
        """
        code = self.protocol["get_imu_setting"]
        self.command = code + 7 * b"\x00"
        self.execute()
        # no check here, since command and reply has different codes
        # TODO: check reply 

        # TODO: convert bytes to float
        self.accel_scale = self.from_bytes(self.reply[0:4], signed=True)
        self.accel_offset = self.from_bytes(self.reply[4:], signed=True)

    def __get_state(self):
        """
            Description
        """
        code = self.protocol["get_state"]
        self.command = code + 7 * b"\x00"
        self.execute()
        self.check()

        pos = self.from_bytes(self.reply[2:4], signed=True)
        force = self.from_bytes(self.reply[4:6], signed=True)
        accel = self.from_bytes(self.reply[6:], signed=False)
        return pos, force, accel

    def get_state(self):
        _pos, _force, _accel = self.__get_state()
        pos = self.pos_scale * _pos
        force = self.force_scale * _force
        accel = self.accel_scale * _accel
        return pos, force, accel

    def reset(self):
        """
            Description
        """
        code = self.protocol["reset"]
        self.command = code + 7 * b"\x00"
        self.execute()
        self.check()