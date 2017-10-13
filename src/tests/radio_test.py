from xbee import XBee
import serial
# Connect to Xbee
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1000)

# Send data (a string)
ser.write(bytes('Hello, World!'))

# Read data
print('MSG: waiting for data')
data = ser.read()
print('MSG: recieved data')
