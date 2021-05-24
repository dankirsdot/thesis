import math

from can import CANDevice

class GyemsRMD(CANDevice):
    """ 
        This class provide interface to the Gyems RMD servo motor over CAN
        socket using GYEMS motor control protocol V1.61. The format of the
        message used to send control commands and receive motor replies:
            
            Frame type specifier: standard frame
            Frame format: DATA
            DLC: 8 bytes
            Identifier: 0x140 + ID(1-32)

        Read more here:
        http://dowload.gyems.cn/RMD%20servo%20motor%20control%20protocol%20%28CAN%20BUS%20%29V1.61.pdf
    """

    def __init__(self, can_socket, id=0x141):

        super().__init__(can_socket, id)

        self.protocol = {
            "get_PID_gains":                      b"\x30",
            "set_PID_gains_RAM":                  b"\x31",
            "set_PID_gains_ROM":                  b"\x32",
            "get_acceleration_limit":             b"\x33",
            "set_acceleration_limit_RAM":         b"\x34",
            "get_encoder_offset":                 b"\x90",
            "set_encoder_offset_ROM":             b"\x91",
            "set_position_as_encoder_offset_ROM": b"\x19",
            "get_multi_turns_angle":              b"\x92",
            "get_single_circle_angle":            b"\x94",
            "get_error_state":                    b"\x9A",
            "clean_error_state":                  b"\x9B",
            "get_state":                          b"\x9C",
            "get_phase_current":                  b"\x9D",
            "turn_off":                           b"\x80",
            "stop":                               b"\x81",
            "resume":                             b"\x88",
            "control_current":                    b"\xA1",
            "control_speed":                      b"\xA2",
            "control_position_1":                 b"\xA3",
            "control_position_2":                 b"\xA4",
            "control_position_3":                 b"\xA5",
            "control_position_4":                 b"\xA6",
        }

        # useful limits
        self.uint8_min = 0
        self.uint8_max = 2**8 - 1

        self.uint16_min = 0
        self.uint16_max = 2**16 - 1

        self.int32_min = -2**31
        self.int32_max =  2**31 - 1

        # 14 bit encoder (from 0 to 16383)
        self.encoder_min = 0
        self.encoder_max = 2**14 - 1
        self.encoder_scale = (2 * math.pi) / self.encoder_max

        # for send, range -2000~2000, corresponding to -32~32 A
        self.current_min = -2000
        self.current_max =  2000
        self.current_scale_set = 2000 / 32
        # for receive, range: -2048~2048, corresponding to -33~33 A
        self.current_scale_get = 33 / 2048

        self.speed_scale_set = 100

        self.sc_angle_min = 0
        self.sc_angle_max = 35999
        self.angle_set = 100
        self.angle_get = 0.01
        
        self.voltage_scale = 0.1
        self.phase_current_scale = 1 / 64

    def clip(self, n, n_min, n_max):
        return min(max(n, n_min), n_max) 

    def get_gains(self):
        """
            Send the command to the motor to get current PID gains.
            
            Returns
            -------
            gains : tuple of `uint8_t`
                Format: (pos_Kp, pos_Ki, vel_Kp, vel_Ki, cur_Kp, cur_Ki).
        """
        code = self.protocol["get_PID_gains"]
        self.command = code + 7 * b"\x00"
        self.execute()
        self.check()

        pos_Kp = self.reply[2]
        pos_Ki = self.reply[3]
        vel_Kp = self.reply[4]
        vel_Ki = self.reply[5]
        cur_Kp = self.reply[6]
        cur_Ki = self.reply[7]
        return pos_Kp, pos_Ki, vel_Kp, vel_Ki, cur_Kp, cur_Ki

    def set_gains(self, gains, memory="RAM"):
        """
            Send the command to the motor to set new PID gains.
            
            NOTE: Multiple writes to `ROM` will affect the chip life!
            Do not recommended to use frequently!

            Parameters
            ----------
            gains : tuple of `uint8_t`
                Tulpe containing PID gains to be set.
                Format: (pos_Kp, pos_Ki, vel_Kp, vel_Ki, cur_Kp, cur_Ki).
            memory : string, optional
                Type of the memory, where new gains are going to be written.
                Can be `RAM` or `ROM`.
        """
        if memory is "RAM":
            code = self.protocol["set_PID_gains_RAM"]
        elif memory is "ROM":
            code = self.protocol["set_PID_gains_ROM"]
        else:
            print(f"Wrong memory type: {memory}!")
            raise Exception

        self.command = code + b"\x00"
        for k in gains:
            k = self.clip(k, self.uint8_min, self.uint8_max)
            self.command += self.to_bytes(1, k, signed=False)
        self.execute()
        self.check()

    def __get_accel_limit(self):
        """
            Send the command to the motor to get current acceleration limit.
            
            Returns
            -------
            accel_limit : `int32_t`
                Unit: `(1 degree / s**2) / LSB`.
                `0` means unlimited acceleration.
        """
        code = self.protocol["get_acceleration_limit"]
        self.command = code + 7 * b"\x00"
        self.execute()
        self.check()

        accel_limit = self.from_bytes(self.reply[4:], signed=True)
        return accel_limit
    
    def get_accel_limit(self):
        _accel_limit = self.__get_accel_limit()
        accel_limit = math.radians(_accel_limit)
        return accel_limit

    def __set_accel_limit(self, accel_limit):
        """
            Send the command to the motor to set new acceleration limit.
            
            NOTE: writes the data to `RAM`, resets after reboot.
            
            Parameters
            ----------
            accel_limit : `int32_t`
                Unit: `1 (degree / s**2) / LSB`.
                Set `0` to get unlimited acceleration.
        """
        accel_limit = self.clip(accel_limit, self.int32_min, self.int32_max)
        code = self.protocol["set_acceleration_limit_RAM"]
        self.command = code + 3 * b"\x00"
        self.command += self.to_bytes(4, accel_limit, signed=True)
        self.execute()
        self.check()

    def set_accel_limit(self, accel_limit):
        _accel_limit = math.degrees(accel_limit)
        self.__set_accel_limit(_accel_limit)

    def __get_encoder_offset(self):
        """
            Send the command to the motor to get encoder state and offset.
            The motor has 14 bit encoder (range from 0 to 16383).
            
            Returns
            -------
            pos : `uint16_t`
                Current position of the motor, which is an original position
                minus encoder offset.
            pos_orig : `uint16_t`
                Encoder original position.
            offset : `uint16_t`
                Encoder offset.
        """
        code = self.protocol["get_encoder_offset"]
        self.command = code + 7 * b"\x00"
        self.execute()
        self.check()

        pos = self.from_bytes(self.reply[2:4], signed=False)
        pos_orig = self.from_bytes(self.reply[4:6], signed=False)
        offset = self.from_bytes(self.reply[6:], signed=False)
        return pos, pos_orig, offset

    def get_encoder_offset(self):
        _pos, _pos_orig, _offset = self.__get_encoder_offset()
        pos = self.encoder_scale * _pos
        pos_orig = self.encoder_scale * _pos_orig
        offset = self.encoder_scale * _offset
        return pos, pos_orig, offset

    def __set_encoder_offset(self, offset=None):
        """
            Send the command to the motor to set new encoder offset.
            The motor has 14 bit encoder.

            NOTE: Multiple writes to `ROM` will affect the chip life!
            Do not recommended to use frequently!
            
            Parameters
            ----------
            offset : `uint16_t`, optional
                Encoder offset to be written. Range from 0 to 16383, with
                the actual range from 0 to 360 degrees. If `None` current
                encoder position would be written as the encoder offset.
        """
        if offset is not None:
            offset = self.clip(offset, self.encoder_min, self.encoder_max)
            code = self.protocol["set_encoder_offset_ROM"]
            self.command = code + 5 * b"\x00"
            self.command += self.to_bytes(2, offset, signed=False)
        else:
            code = self.protocol["set_position_as_encoder_offset_ROM"]
            self.command = code + 7 * b"\x00"
        self.execute()
        self.check()

    def set_encoder_offset(self, offset=None):
        if offset is not None:
            _offset = (1 / self.encoder_scale) * offset
        else:
            _offset = None
        self.__set_encoder_offset(_offset)

    def __get_multi_turns_angle(self):
        """
            Send the command to the motor to get multi-turns angle.
            
            Returns
            -------
            mt_angle : `int64_t`
                Unit: `0.01 degree / LSB`.
                Positive value indicates clockwise angle, negative value
                indicates counter-clockwise angle.
        """
        code = self.protocol["get_multi_turns_angle"]
        self.command = code + 7 * b"\x00"
        self.execute()
        self.check()

        mt_angle = self.from_bytes(self.reply[1:], signed=True)
        return mt_angle

    def get_multi_turns_angle(self):
        _mt_angle = self.__get_multi_turns_angle()
        mt_angle = math.radians(self.angle_get * _mt_angle)
        return mt_angle

    def __get_single_circle_angle(self):
        """
            Send the command to the motor to get single-circle angle.
            
            Returns
            -------
            sc_angle : `uint16_t`
                Unit: `0.01 degree / LSB`.
                Range from 0 to 35999, with the actual range from 0
                to 359,99 degrees. Increased by clockwise rotation.
        """
        code = self.protocol["get_single_circle_angle"]
        self.command = code + 7 * b"\x00"
        self.execute()
        self.check()
        
        sc_angle = self.from_bytes(self.reply[6:], signed=False)
        return sc_angle

    def get_single_circle_angle(self):
        _sc_angle = self.__get_single_circle_angle()
        sc_angle = math.radians(self.angle_get * _sc_angle)
        return sc_angle

    def __get_error_state(self):
        """
            Send the command to the motor to get motor's error state,
            voltage and temperature.
            
            Returns
            -------
            temperature : `int8_t`
                Unit: `1 C / LSB`.
            voltage : `uint16_t`
                Unit: `0.1 V / LSB`.
            error : `uint8_t`
                Each bit represents a different motor state. You can get table
                with error states and their descriptions in the datasheet.
        """
        code = self.protocol["get_error_state"]
        self.command = code + 7 * b"\x00"
        self.execute()
        self.check()

        temperature = self.reply[1] # BUG: inadequate temperature numbers
        voltage = self.from_bytes(self.reply[3:5], signed=False)
        error_state = self.reply[7]
        return temperature, voltage, error_state

    def get_error_state(self):
        temperature, _voltage, error_state = self.__get_error_state()
        voltage = self.voltage_scale * _voltage
        return temperature, voltage, error_state

    def __clean_error_state(self):
        """
            Send the command to the motor to clear its error state and get
            get back new error state, voltage and temperature.
            
            Returns
            -------
            temperature : `int8_t`
                Unit: `1 C / LSB`.
            voltage : `uint16_t`
                Unit: `0.1 V / LSB`.
            error : `uint8_t`
                Each bit represents a different motor state. You can get table
                with error states and their descriptions in the datasheet.
        """
        code = self.protocol["clean_error_state"]
        self.command = code + 7 * b"\x00"
        self.execute()
        # no check here, since command and reply has different codes
        # TODO: check reply

        temperature = self.reply[1] # BUG: inadequate temperature numbers
        voltage = self.from_bytes(self.reply[3:5], signed=False)
        error_state = self.reply[7]
        return temperature, voltage, error_state

    def clean_error_state(self):
        temperature, _voltage, error_state = self.__clean_error_state()
        voltage = self.voltage_scale * _voltage
        return temperature, voltage, error_state

    def __get_state(self):
        """
            Send the command to the motor to get its temperature, torque
            current, speed and encoder position.
            
            Returns
            -------
            temperature : `int8_t`
                Unit: `1 C / LSB`.
            current : `int16_t`
                Range from -2048 to 2048, with the actual torque current range
                from -33 A to 33 A.
            speed : `int16_t`
                Unit: `1 (degree / s) / LSB`
            pos : `uint16_t`
                Current position of the motor. Range from 0 to 16383, with the
                actual range from 0 to 360 degrees.
        """
        code = self.protocol["get_state"]
        self.command = code + 7 * b"\x00"
        self.execute()
        self.check()

        temperature = self.reply[1] # BUG: inadequate temperature numbers
        current = self.from_bytes(self.reply[2:4], signed=True)
        speed = self.from_bytes(self.reply[4:6], signed=True)
        pos = self.from_bytes(self.reply[6:], signed=False)
        return temperature, current, speed, pos

    def __convert_state(self, state):
        temperature, _current, _speed, _pos = state
        current = self.current_scale_get * _current
        speed = math.radians(_speed)
        pos = self.encoder_scale * _pos
        return temperature, current, speed, pos

    def get_state(self):
        _state = self.__get_state()
        state = self.__convert_state(_state)
        return state

    def __get_phase_current(self):
        """
            Send the command to the motor to get its phase current.
            
            Returns
            -------
            phase_current : tuple of `int16_t`
                Unit: `1 A / 64 LSB`.
                Tuple containing phase current of the motor.
                Format: (iA, iB, iC).
        """
        code = self.protocol["get_phase_current"]
        self.command = code + 7 * b"\x00"
        self.execute()
        self.check()

        iA = self.from_bytes(self.reply[2:4], signed=True)
        iB = self.from_bytes(self.reply[4:6], signed=True)
        iC = self.from_bytes(self.reply[6:], signed=True)
        return iA, iB, iC

    def get_phase_current(self):
        _iA, _iB, _iC = self.__get_phase_current()
        iA = self.phase_current_scale * _iA
        iB = self.phase_current_scale * _iB
        iC = self.phase_current_scale * _iC
        return iA, iB, iC

    def turn_off(self):
        """ 
            Send the command to the motor to turn it off.
            
            NOTE: Resets motor inner multi-turns angle.
        """
        code = self.protocol["turn_off"]
        self.command = code + 7 * b"\x00"
        self.execute()
        self.check()

    def pause(self):
        """ 
            Send the command to the motor to pause it.
            
            NOTE: Do not resets motor inner multi-turns angle.
        """
        code = self.protocol["stop"]
        self.command = code + 7 * b"\x00"
        self.execute()
        self.check()

    def resume(self):
        """ 
            Send the command to the motor to start it again after a pause.
   
            BUG: Do not recover control mode which it has before pause.
        """
        code = self.protocol["resume"]
        self.command = code + 7 * b"\x00"
        self.execute()
        self.check()

    def __set_current(self, current):
        """
            Send the command to the motor to control its torque current output.
            Get temperature, torque current, speed and encoder position of the
            motor in response.
            
            Parameters
            ----------
            current : `int16_t`
                Range from -2000 to 2000, with the actual torque current range
                from -32 A to 32 A (the bus current and the actual torque of
                the motor vary with deffirent motors).
            
            Returns
            -------
            temperature : `int8_t`
                Unit: `1 C / LSB`.
            current : `int16_t`
                Range from -2048 to 2048, with the actual torque current range
                from -33 A to 33 A.
            speed : `int16_t`
                Unit: `1 (degree / s) / LSB`
            pos : `uint16_t`
                Current position of the motor. Range from 0 to 16383, with the
                actual range from 0 to 360 degrees.
        """
        current = self.clip(current, self.current_min, self.current_max)
        code = self.protocol["control_current"]
        self.command = code + 3 * b"\x00"
        self.command += self.to_bytes(2, current, signed=True)
        self.command += 2 * b"\x00"
        self.execute()
        self.check()

        temperature = self.reply[1] # BUG: inadequate temperature numbers
        current = self.from_bytes(self.reply[2:4], signed=True)
        speed = self.from_bytes(self.reply[4:6], signed=True)
        pos = self.from_bytes(self.reply[6:], signed=False)
        return temperature, current, speed, pos

    def set_current(self, current):
        _current = self.current_scale_set * current
        _state = self.__set_current(_current)
        state = self.__convert_state(_state)
        return state

    def __set_speed(self, speed):
        """
            Send the command to the motor to control its speed.
            Get temperature, torque current, speed and encoder position of the
            motor in response.
            
            Parameters
            ----------
            speed : `int32_t`
                Unit: `0.01 (degree / s) / LSB`.
            
            Returns
            -------
            temperature : `int8_t`
                Unit: `1 C / LSB`.
            current : `int16_t`
                Range from -2048 to 2048, with the actual torque current range
                from -33 A to 33 A.
            speed : `int16_t`
                Unit: `1 (degree / s) / LSB`
            pos : `uint16_t`
                Current position of the motor. Range from 0 to 16383, with the
                actual range from 0 to 360 degrees.
        """
        speed = self.clip(speed, self.int32_min, self.int32_max)
        code = self.protocol["control_speed"]
        self.command = code + 3 * b"\x00"
        self.command += self.to_bytes(4, speed, signed=True)
        self.execute()
        self.check()

        temperature = self.reply[1] # BUG: inadequate temperature numbers
        current = self.from_bytes(self.reply[2:4], signed=True)
        speed = self.from_bytes(self.reply[4:6], signed=True)
        pos = self.from_bytes(self.reply[6:], signed=False)
        return temperature, current, speed, pos

    def set_speed(self, speed):
        _speed = self.speed_scale_set * math.degrees(speed)
        _state = self.__set_speed(_speed)
        state = self.__convert_state(_state)
        return state

    def __set_multi_turns_angle(self, mt_angle, speed_limit=None):
        """
            Send the command to the motor to control its multi-turns angle.
            Get temperature, torque current, speed and encoder position of the
            motor in response.
            
            Parameters
            ----------
            mt_angle : `int32_t`
                Unit: `0.01 degree / LSB`.
                Positive value indicates clockwise angle, negative value
                indicates counter-clockwise angle.
            speed_limit : `uint16_t`, optional
                Unit: `1 (degree / s) / LSB`.
                Limit the maximum speed of the motor rotation.

            Returns
            -------
            temperature : `int8_t`
                Unit: `1 C / LSB`.
            current : `int16_t`
                Range from -2048 to 2048, with the actual torque current range
                from -33 A to 33 A.
            speed : `int16_t`
                Unit: `1 (degree / s) / LSB`
            pos : `uint16_t`
                Current position of the motor. Range from 0 to 16383, with the
                actual range from 0 to 360 degrees.
        """
        mt_angle = self.clip(mt_angle, self.int32_min, self.int32_max)
        
        if speed_limit is not None:
            code = self.protocol["control_position_2"]
            speed_limit = self.clip(speed_limit, self.uint16_min, self.uint16_max) # FIXME: This line is too long (speed -> vel ?)
            speed_limit_bytes = self.to_bytes(2, speed_limit, signed=False)
        else:
            code = self.protocol["control_position_1"]
            speed_limit_bytes = 2 * b"\x00"

        self.command = code + b"\x00"
        self.command += speed_limit_bytes
        self.command += self.to_bytes(4, mt_angle, signed=True)
        self.execute()
        self.check()

        temperature = self.reply[1] # BUG: inadequate temperature numbers
        current = self.from_bytes(self.reply[2:4], signed=True)
        speed = self.from_bytes(self.reply[4:6], signed=True)
        pos = self.from_bytes(self.reply[6:], signed=False)
        return temperature, current, speed, pos

    def set_multi_turns_angle(self, mt_angle, speed_limit=None):
        _mt_angle = self.angle_set * math.degrees(mt_angle)

        if speed_limit is not None:
            _speed_limit = math.degrees(speed_limit)
        else:
            _speed_limit = None

        _state = self.__set_multi_turns_angle(_mt_angle, _speed_limit)
        state = self.__convert_state(_state)
        return state

    def __set_single_circle_angle(self, sc_angle, speed_limit=None, dir=0):
        """
            Send the command to the motor to control its multi-turns angle.
            Get temperature, torque current, speed and encoder position of the
            motor in response.
            
            Parameters
            ----------
            sc_angle : `uint16_t`
                Unit: `0.01 degree / LSB`.
                Single-circle angle of the motor, starting from encoder zero,
                increased by clockwise rotation. Range from 0 to 35999, with
                the actual range from 0 to 359,99 degrees.
            speed_limit : `uint16_t`, optional
                Unit: `1 (degree / s) / LSB`
                Limit the maximum speed of the motor rotation.
            dir : `uint8_t`, optional
                This value sets the direction in which the motor rotates.
                `0x00` for clockwise rotation and `0x01` for counter clockwise.

            Returns
            -------
            temperature : `int8_t`
                Unit: `1 C / LSB`.
            current : `int16_t`
                Range from -2048 to 2048, with the actual torque current range
                from -33 A to 33 A.
            speed : `int16_t`
                Unit: `1 (degree / s) / LSB`
            pos : `uint16_t`
                Current position of the motor. Range from 0 to 16383, with the
                actual range from 0 to 360 degrees.
        """
        sc_angle = self.clip(sc_angle, self.sc_angle_min, self.sc_angle_max)

        if speed_limit is not None:
            code = self.protocol["control_position_4"]    
            speed_limit = self.clip(speed_limit, self.uint16_min, self.uint16_max) # FIXME: This line is too long (speed -> vel ?)
            speed_limit_bytes = self.to_bytes(2, speed_limit, signed=False)
        else:
            code = self.protocol["control_position_3"]
            speed_limit_bytes = 2 * b"\x00"

        self.command = code
        self.command += self.to_bytes(1, dir, signed=False)
        self.command += speed_limit_bytes
        self.command += self.to_bytes(2, sc_angle, signed=False)
        self.command += 2 * b"\x00"
        self.execute()
        self.check()

        temperature = self.reply[1] # BUG: inadequate temperature numbers
        current = self.from_bytes(self.reply[2:4], signed=True)
        speed = self.from_bytes(self.reply[4:6], signed=True)
        pos = self.from_bytes(self.reply[6:], signed=False)
        return temperature, current, speed, pos

    def set_single_circle_angle(self, sc_angle, speed_limit=None, dir=0):
        _sc_angle = self.angle_set * math.degrees(sc_angle)
        
        if speed_limit is not None:
            _speed_limit = math.degrees(speed_limit)
        else:
            _speed_limit = None
        
        _state = self.__set_single_circle_angle(_sc_angle, _speed_limit, dir)
        state = self.__convert_state(_state)
        return state