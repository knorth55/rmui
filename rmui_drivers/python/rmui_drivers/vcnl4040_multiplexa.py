
class VCNL4040Multiplexa(object):
    def __init__(self, multiplexa, sensors):
        self.multiplexa = multiplexa
        self.sensors = sensors
        self.n_sensor = len(self.sensors)

    def init_sensor(self, channel):
        self.multiplexa.start(channel)
        self.sensors[channel].init_sensor()
        self.multiplexa.stop()

    def init_sensors(self):
        for channel in range(self.n_sensor):
            self.init_sensor(channel)

    def start_sensor(self, channel):
        self.multiplexa.start(channel)
        self.sensors[channel].start_sensor()
        self.multiplexa.stop()

    def start_sensors(self):
        for channel in range(self.n_sensor):
            self.start_sensor(channel)

    def stop_sensor(self, channel):
        self.multiplexa.start(channel)
        self.sensors[channel].stop_sensor()
        self.multiplexa.stop()

    def stop_sensors(self):
        for channel in range(self.n_sensor):
            self.stop_sensor(channel)

    def read_proximity(self, channel):
        self.multiplexa.start(channel)
        prx_d = self.sensors[channel].read_proximity()
        self.multiplexa.stop()
        return prx_d

    def read_proximities(self):
        prx_data = []
        for channel in range(self.n_sensor):
            prx_d = self.read_proximity(channel)
            prx_data.append(prx_d)
        return prx_data

    def get_proximity_msg(self, channel, prx_d):
        msg = self.sensors[channel].get_proximity_msg(prx_d)
        return msg

    def get_proximity_msgs(self, prx_data):
        msgs = []
        for channel, prx_d in enumerate(prx_data):
            msg = self.get_proximity_msg(channel, prx_d)
            msgs.append(msg)
        return msgs
