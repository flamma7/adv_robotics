import serial

class PololuController:
	def __init__(self, port='/dev/ttyACM0', device='0x0c', timeout=1, channels=18):
		self.port = port
		self.device = device
		self.timeout = timeout
		self.con = serial.Serial(port, timeout)
		self.pololuProtocol = chr(0xaa) + chr(0x0c)
		self.channels = channels

	def validChannel(self, channel):
		return channel < self.channels-1 and channel >= 0

	def getPosition(self, channel):
		# https://www.pololu.com/docs/0J40/5.e Get Position section
		if not self.validChannel(channel):
			raise(Exception('Not a valid channel.'))
		cmd = self.pololuProtocol + chr(0x10) + chr(channel)
		self.con.write(bytes(cmd, 'latin-1'))
		low = ord(self.con.read())
		high = ord(self.con.read())
		return (high << 8) + low

        def setTarget(self, channels, values):
                for i in range(channels):
                        if not self.validChannel(channels[i]):
                                raise(Exception('Not a valid channel.'))

                        lowbyte = values[i] & 0xFF
                        highbyte = values[i] >> 8
                        cmd = self.pololuProtocol + chr(0x04) + chr(channels[i]) + chr(lowbyte) + chr(highbyte)
                        self.con.write(bytes(cmd, 'latin-1'))
                return
                                      
        def setMotors(self, msg, drive_channel, turn_channel):
                #HARD CODED CHANNELS RIGHT NOW
                turn_cmd = msg[0] * 20 + 6000
                drive_cmd = msg[1] * 20 + 6000
                cmd_vals = [turn_cmd, drive_cmd]
                cmd_channels = [turn_channel, drive_channel]
                self.setTarget(cmd_channels, cmd_vals)

        def killMotors(self, msg, drive_channel, turn_channel):
                turn_cmd = 6000
                drive_cmd = 6000
                cmd_vals = [turn_cmd, drive_cmd]
                cmd_channels = [turn_channel, drive_channel]
                self.setTarget(cmd_channels, cmd_vals)
                
