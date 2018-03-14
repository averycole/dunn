import time, serial, struct

ENCODER_CALIB = False
ENCODER_PORT = '/dev/ttyUSB0'

# US Digital SEI driver
# read() requests the position of both encoders on the bus and stores
# their raw output.  No calibrated conversion is done here; this is
# required later.  Here we're reading 2 encoders, and the result is a
# comma separated list of the two readings.  The wheel rotation value (whose
# encoder is in "multiple turn mode") is in the form xxxx-yyyy, where xxxx is
# the turn count of the encoder since it was reset, and yyyy is the angular
# position.  Each is a 16-bit value, but the resolution can only be
# trusted to 12.  The steering value (whose encoder is in "single turn mode")
# is a single 16-bit value ("xxxx") with the same 12-bit accuracy.
class UsDigitalSei:
	def __init__(self, port='/dev/ttyUSB0', platform='GD'):
		# Store platform (Git-R-Dunn or trailer)
		self.platform = platform

		# Serial settings
		self.port = port
		self.altport = '/dev/ttyUSB1' # Alternate port
		self.MaxBytes = 10
		self.SleepTime = 0.2
		
		# Encoder bus addresses for vehicle
		self.WheelEncAddr = '0000' # Wheel encoder
		self.SteerEncAddr = '0011' # Steering encoder
		
		# Encoder bus addresses for trailer
		self.EncAddrL = '0001'
		self.EncAddrR = '0010'
		
		# Encoder bus addresses for the MDA trailers.  Use the script utils/sei_assign.py
		# To assign the encoder addresses.
		self.MDAAddrL = '0001'
		self.MDAAddrR = '0010'
		
		# Serial baud rates
		self.BaudRate = 115200
		self.BaudRateDefault = 9600
		
		# Timeout for reading SEI
		self.Timeout = 0.01
		
		# SEI commands (half byte) specified in the encoder protocol
		self.CmdPos = '0001'
		self.CmdStrobe = '0100'
		self.CmdSleep = '0101'
		self.CmdWakeup = '0110'
		
		# Initialize serial connection
		self.ser = serial.Serial()
		self.ser.port = self.port
		self.ser.timeout = self.Timeout
		self.ser.baudrate = self.BaudRateDefault # Set the baud rate to the encoder default, we will change it later
		self.ser.open()
		time.sleep(self.SleepTime)
		
		# Do a test read.  If it doesn't succeed then it's probably because the OS has switched
		# the name of the serial port (happens with the serial->USB converter.)  In this case we
		# just open a connection on a different port.
		try:
			if self.platform == 'GD':
				data = self.sei_read(self.WheelEncAddr)
			elif self.platform == 'TR':
				data = self.sei_read(self.EncAddrL)
			elif self.platform == 'MDA':
				data = self.sei_read(self.MDAAddrL)
		except OSError:
			self.ser.port = self.altport
			self.ser.open()
		
		if ENCODER_CALIB and (self.platform == 'GD'):
			# Change steering power up mode to "single-turn"
			query = self.sei_multibyte(self.SteerEncAddr, (chr(0x0d), '00001000'))
			tmp = self.sei_query(query)
			time.sleep(self.SleepTime)
		
		# Reset all encoders on the bus
		query = self.sei_multibyte('1111', (chr(0x0e),))
		tmp = self.sei_query(query)
		time.sleep(self.SleepTime)

		if ENCODER_CALIB and (self.platform == 'GD'):
			# Set the new steering zero to the current position
			query = self.sei_multibyte(self.SteerEncAddr, (chr(0x0d), chr(0x01)))
			tmp = self.sei_query(query)
			time.sleep(self.SleepTime)

		# Set encoder resolution
		# The encoder can report up to 16 bit resolution, but it is only accurate
		# up to 12.  We set 16 here and will trust the result up to 12.
		query = self.sei_multibyte('1111', (chr(0x0a), chr(0x00), chr(0x00)))
		tmp = self.sei_query(query)
		time.sleep(self.SleepTime)

		# Set baud rate to max (115200)
		# The computer and encoders must agree on the baud rate in order to make any sense
		# of the data.  Here we send the signal to change the encoder resolution and wait for
		# the change to propagate, then change the baud rate of the receiving COM port
		# accordingly.
		query = self.sei_multibyte('1111', (chr(0x0f), chr(0x00)))
		tmp = self.sei_query(query)
		time.sleep(self.SleepTime)
		self.ser.baudrate = self.BaudRate
	
	# Destructor
	def __del__(self):
		self.ser.close()
	
	# Returns a single byte request for the SEI
	# req is a length-8 string representation of the binary sequence
	# of the byte, as specified in the SEI communication protocol.
	def sei_request(self, addr, cmd):
		# Byte representation of the request
		return chr(int(cmd + addr, 2))
	
	# Returns a multibye command for the SEI.  The request byte is always required
	# as the first byte of the query, and extra is a list of additional bytes.
	def sei_multibyte(self, addr, extra):
		rval = self.sei_request(addr, '1111') # Request byte specifying multibyte
		
		# Append extra bytes
		for b in extra:
			rval += b
		
		return rval
	
	# Send a single or multiple-byte query to the SEI and return the response
	def sei_query(self, query):
		# Send command & read the response
		self.ser.write(query)
		resp = self.ser.read(self.MaxBytes)
		return resp
	
	# Take and return a reading from the encoder specified by address "addr".
	def sei_read(self, addr):
		# Initialize
		rval = ''
		
		# Send a position request to the appropriate encoder and wait for response
		resp = self.sei_query(self.sei_request(addr, self.CmdPos))
		
		# Steering encoder is single-turn mode (special case)
		if addr == self.SteerEncAddr:
			if len(resp) == 3:
				rval = str(struct.unpack('H', resp[2] + resp[1])[0])
			else:
				rval = ''
		
		# All other encoders use multi-turn mode
		else:
			# Should receive a 5-byte response
			if len(resp) == 5:
				# Interpret bytes 2-3 as a counter value of encoder turns
				ec = str(struct.unpack('H', resp[2] + resp[1])[0])

				# Interpret the last 2 bytes as a short int between 0 and 65535.
				# This represents the angular position of the encoder.
				# We switch the order because struct apparently assumes little endian data.
				ep = str(struct.unpack('H', resp[4] + resp[3])[0])

				# Return a string containing both the turn count and angular position
				rval = ec + '-' + ep
			else:
				# Invalid response from the encoders
				rval = ''
		
		return rval

	# Read SEI measurement
	def read(self):
		# Read both encoders
		if self.platform == 'GD':
			r1 = self.sei_read(self.WheelEncAddr)
			r2 = self.sei_read(self.SteerEncAddr)
		elif self.platform == 'TR':
			r1 = self.sei_read(self.EncAddrL)
			r2 = self.sei_read(self.EncAddrR)
		elif self.platform == 'MDA':
			r1 = self.sei_read(self.MDAAddrL)
			r2 = self.sei_read(self.MDAAddrR)
		
		# Ensure we have a measurement from each
		if (r1 is not '') and (r2 is not ''):
			rval = r1 + ',' + r2
		else:
			rval = ''

		return rval

	# End UsDigitalSei

