import serial

DRIVE_INDEX = 0
YAW_INDEX = 1

class PololuController:
	def __init__(self, drive_channel, turn_channel, ir_channel1, ir_channel2, port='/dev/ttyACM0', device='0x0c', timeout=1, channels=18):
                self.drive_channel = drive_channel
                self.turn_channel = turn_channel
		self.port = port
		self.device = device
		self.timeout = timeout
		self.con = serial.Serial(port, timeout)
		self.pololuProtocol = [chr(0xaa), chr(0x0c)]
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

        def turn_motor(self):
                channel = 1
                value = 6000
                lowbyte = value & 0x7F
                highbyte = value >> 7
                msg = bytearray([0xAA, 0x0C, 0x04, channel, lowbyte, highbyte])
                self.con.write(msg)
                
        def setTarget(self, channel_list, value_list):
                for i in range(len(channel_list)):
                        if not self.validChannel(channel_list[i]):
                                raise(Exception('Not a valid channel.'))

                        lowbyte = value_list[i] & 0x7F
                        highbyte = value_list[i] >> 7
                        cmd_list = self.pololuProtocol + [0x04, channel_list[i], lowbyte, highbyte]
                        cmd = bytearray(cmd_list)
                        self.con.write(cmd)
                                      
        def setMotors(self, cmd_list):
                #HARD CODED CHANNELS RIGHT NOW
                turn_cmd = cmd_list[YAW_INDEX] * 20 + 6000
                drive_cmd = cmd_list[DRIVE_INDEX] * 20 + 6000
                cmd_vals = [turn_cmd, drive_cmd]
                cmd_channels = [self.turn_channel, self.drive_channel]
                self.setTarget(cmd_channels, cmd_vals)

        def killMotors(self, msg, drive_channel, turn_channel):
                turn_cmd = 6000
                drive_cmd = 6000
                cmd_vals = [turn_cmd, drive_cmd]
                cmd_channels = [turn_channel, drive_channel]
                self.setTarget(cmd_channels, cmd_vals)
                
def test():
        drive_channel = 1
        turn_channel = 2
        p = PololuController(drive_channel,turn_channel,2,3)
        p.setTarget([drive_channel, turn_channel], [6000, 6000])
if __name__ == "__main__":
        test()
