self.multi_turns_degrees = self.angle_degrees_scale * self.multi_turns_counts
self.multi_turns_radians = self.angle_radians_scale * self.multi_turns_degrees

self.single_circle_degrees = self.angle_degrees_scale * self.single_circle_counts
self.single_circle_radians = self.angle_radians_scale * self.single_circle_degrees

def multiturn_encoder(self):
    if self.pos_counts_prev - self.pos_counts >= self.turn_threshold:
        self.motor_turns += 1
    elif self.pos_counts_prev - self.pos_counts <= -self.turn_threshold:
        self.motor_turns -= 1
    self.pos_counts_prev = self.pos_counts

    return self.pos_counts + (self.encoder_resolution) * self.motor_turns

counts = self.multiturn_encoder()
self.pos_degrees = self.pos_degrees_scale * counts

self.pos_degrees_scale = 360 / self.encoder_resolution
self.pos_radians_scale = 2 * math.pi / self.encoder_resolution

self.motor_turns = 0
self.encoder_resolution = 2**14
self.turn_threshold = self.encoder_resolution / 2

self.speed_degrees = 0
self.speed_radians = 0
self.speed_degrees_scale = 1 / 10
self.speed_radians_scale = 2 * math.pi / 360


self.angle_degrees_scale = 0.01 # degree / LSB
self.angle_radians_scale = math.pi / 180

self.multi_turns_degrees = 0
self.multi_turns_radians = 0
self.single_circle_degrees = 0
self.single_circle_radians = 0

self.temperature_scale = 1 # Celsius / LSB
self.voltage_scale = 0.1 # v / LSB

self.current = self.voltage_scale * self.current_counts
self.speed = self.speed_scale * self.speed_couts
self.temperature = self.temperature_scale * self.temperature_counts
