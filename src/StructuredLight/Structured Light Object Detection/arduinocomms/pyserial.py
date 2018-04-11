# Python to Arduino
import serial
import time

## Open a serial connection with Arduino.
serial_port = input("Please input port number of Arduino: ")
baud_rate = input("Please input serial baudrate: ")
ser = serial.Serial("serial_port", baud_rate, timeout=.1)   # open serial port that Arduino is using
print(ser)                           # print serial config
start_time = time.time()

print("Sending serial data")

while (time.time() - start_time < 5):
	serialmsg = "Hello World"
	ser.write(serialmsg.encode())
	data = ser.readline()
	print(data)
# Reminder to close the connection when finished
if(ser.isOpen()):
	print("Serial connection is still open.")
	# Arduino to Python
	'''
	import serial
	arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=.1)
	while True:
		data = arduino.readline()[:-2] #the last bit gets rid of the new-line chars
		if data:
			print(data)
			'''
