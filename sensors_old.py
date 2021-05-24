from can import CANDevice
from time import perf_counter


class CANSensors(CANDevice):
    """"""

    def __init__(self, can_bus=None, device_id=0x01, labels=None):
        
        super().__init__(can_bus, device_id)

        if not labels:
            raise Exception("Provide labels list!")
        self.labels = labels

        self.bytes = {}
        self.data_map = {}
        self.counts = {}
        self.offsets = {}
        self.scales = {}
        self.measurements = {}
        self.timestamps = {}
        self.differences = {}
        self.data = {}
        self.overflow_buffer = {}
        self.turns = {}
        self.filters = {}
        self.buffer_size = {}

        # i is a number of the particular sensor l
        for i, l in enumerate(self.labels):
            self.bytes[l] = 0
            self.data_map[l] = [i*2, (i + 1)*2]
            self.counts[l] = [0]
            self.offsets[l] = 0
            self.scales[l] = 1
            self.measurements[l] = 0
            self.timestamps[l] = [0]
            self.differences[l] = 0
            self.data[l] = [0]
            self.overflow_buffer[l] = 0
            self.turns[l] = 0
            self.filters[l] = None
            self.buffer_size[l] = 1

        self.protocol = {
            "init_sensors": b"\x9B",
            "get_sensors": b"\x9C",
            "reset_counters": b"\x05",
        }

        self.raw = {}

        self.init_time = perf_counter()

    def init_arrays(self, l):
        self.counts[l] = self.buffer_size[l]*[0]
        self.timestamps[l] = self.buffer_size[l]*[0]
        self.data[l] = self.buffer_size[l]*[0]

    def set_differences(self, labels):
        for l in labels:
            if self.buffer_size[l] < 2:
                self.buffer_size[l] = 2
                self.init_arrays(l)
            self.differences[l] = 0
            self.init_arrays(l)

    def enable_overflow(self, buffers):
        sensors_to_overflow = buffers.keys()
        for l in sensors_to_overflow:
            if self.buffer_size[l] < 2:
                self.buffer_size[l] = 2
                self.init_arrays(l)
            self.overflow_buffer[l] = buffers[l]

    def request_reply(self, command=None):
        self.command = self.protocol["get_sensors"] + 7*b"\x00"
        self.execute()
        pass

    def counter_overflow(self, counts, prev_counts, buffer, threshold=None):
        if not threshold:
            threshold = buffer / 2

        if prev_counts - counts >= threshold:
            return 1
        elif prev_counts - counts <= -threshold:
            return -1
        else:
            return 0

    def parse_data(self):
        for l in self.labels:
            for i in range(1, self.buffer_size[l]):
                self.timestamps[l][i] = self.timestamps[l][i-1]
                self.counts[l][i] = self.counts[l][i-1]
                self.data[l][i] = self.data[l][i-1]
                # self.differences[sensor][i] = self.differences[sensor][i-1]

            self.bytes[l] = self.reply[self.data_map[l][0]:self.data_map[l][1]]
            self.counts[l][0] = self.from_bytes(self.bytes[l])
            self.timestamps[l][0] = perf_counter() - self.init_time

            if self.overflow_buffer[l]:
                self.turns[l] += self.counter_overflow(
                    self.counts[l][0], self.counts[l][1], self.overflow_buffer[l])
            self.data[l][0] = self.scales[l] * \
                (self.turns[l]*self.overflow_buffer[l] +
                 self.counts[l][0] - self.offsets[l])

            if self.buffer_size[l] > 1:
                dt = self.timestamps[l][0] - self.timestamps[l][1]
                self.differences[l] = (
                    self.data[l][0] - self.data[l][1])/dt

            if self.filters[l]:
                # do the filtering
                pass

            self.measurements[l] = self.data[l][0]

    def reset_counters(self, sensors=None, output=False):
        """Reset counters for specific sensors provided by their labels"""

        if sensors is None:
            sensors_to_reset = self.labels
        else:
            sensors_to_reset = sensors

        print(sensors_to_reset)

        self.request_reply()
        self.parse_data()

        for sensor in sensors_to_reset:
            self.offsets[sensor] = self.counts[sensor][0]
            self.turns[sensor] = 0

        if output:
            print(f'New offsets are setted as:\n{self.offsets}')