import socket
import struct

class CANSocket:
    def __init__(self, interface):
        self.interface = interface
        self.frame_format = "=IB3x8s"

        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.socket.bind((self.interface, ))

    def build_frame(self, id, data):
        dlc = len(data)
        data = data.ljust(8, b'\x00')
        return struct.pack(self.frame_format, id, dlc, data)

    def send(self, id, data):
        frame = self.build_frame(id, data)
        self.socket.send(frame)

    def parse_frame(self, frame):
        id, dlc, data = struct.unpack(self.frame_format, frame)
        return id, dlc, data[:dlc]

    def recv(self):
        frame, _ = self.socket.recvfrom(16)
        id, dlc, data = self.parse_frame(frame)
        return id, dlc, data

class CANDevice:
    def __init__(self, can_socket: CANSocket, id=0x01):
        self.can_socket = can_socket
        self.id = id

        self.command = 8 * b"\x00"
        self.reply = 8 * b"\x00"

    def to_bytes(self, n, value, signed=True):
        return int(value).to_bytes(n, byteorder="little", signed=signed)

    def from_bytes(self, byte_string, signed=True):
        return int.from_bytes(byte_string, byteorder="little", signed=signed)
    
    def send(self):
        self.can_socket.send(self.id, self.command)

    def recv(self):
        id, _, reply = self.can_socket.recv()

        # FIXME: if you have two devices connected to the same CAN
        # the first device can steal frame of the second one.
        if id == self.id:
            self.reply = reply
        else:
            print("FIXME!")

    def execute(self):
        """
            Send the command from self.command variable to the device
            and store reply in self.reply.
        """
        self.send()
        self.recv()
    
    def check(self):
        """
            Check if the device responded in a correct way by comparing
            command code in self.command and self.reply strings.
        """
        if self.reply[0] is not self.command[0]:
            print("Motor responded in a wrong way!")
            raise Exception