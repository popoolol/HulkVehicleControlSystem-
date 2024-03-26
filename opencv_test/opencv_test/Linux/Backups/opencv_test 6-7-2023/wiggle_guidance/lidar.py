import math
import serial
import time

# Make a connection to the com port.
serialPortName = '/dev/ttyUSB0'#.usbserial-FTCZQ50G'
serialPortBaudRate = 115200
port = serial.Serial(serialPortName, serialPortBaudRate, timeout=0.1)

# Enable serial mode by sending some characters over the serial port.
port.write(('www\r\n').encode())

# Read and ignore any unintended responses
port.readline().decode("ascii")

# Get the product information
port.write(('?\r\n').encode())
productInfo = port.readline().decode("ascii")

#Create array that is used for the loop of values.
elements = 10
arr = []
for x in range(elements):
	arr.append(0)



#Reading the current distance from the LiDAR 
def get_distance():
	# Get distance reading (First return, default filtering)
	port.write(('LD\r\n').encode())
	distanceStr = port.readline().decode("ascii")

        # Convert the distance string response into a number
	distancecm = float(distanceStr) * 100
	return distancecm



#First loop to find object. 
#Scanning until an object is sensed. Robot rotates and LiDAR reads values.
#If there is an abrupt change in the distance, in an empty room, object found. 
#Then move forward.
def scan():
#	print("scanning")
	test = 0
	loop1 = 1
	while loop1 > 0:
		arr[test] = get_distance()
#		print(arr)
		if loop1 >= 2:
			check = arr[test] - arr[test-1]
#			print("Difference is: ", check)
			if check >= 10 or check <= -10:
#				print("OBJECT HERE")
#				print("Move Forward")
				break
		test += 1
		if test == 10:
#			print("NEW SET")
			test = 0
		loop1 += 1	
		time.sleep(0.25)



#Second loop to get 3 feet from object.
#As the robot drives straight forward LiDAR gets distance values. 
#Once the robot is 91.44cm (3 feet) stop moving. 
#The 3 feet can be changed to whatever we decide.
def move_forward():
	loop2 = 0
	while loop2 < 1:
#		print(get_distance())
		if get_distance() <= 91.44:         #3 feet = 91.44cm, change if necassary.
			loop2 += 1
		#	print("3 feet away. STOP!")
		time.sleep(0.05)		

#Initiate scanning by calling this function:
#scan()

#Initiate movement until designated distance (3 feet) by calling this function:
#move_forward()



