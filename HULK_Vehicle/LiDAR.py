import math
import time
import serial


# Make a connection to the com port.
serialPortName = '/dev/tty.usbserial-FTCZQ50G'
serialPortBaudRate = 115200
port = serial.Serial(serialPortName, serialPortBaudRate, timeout=0.1)

# Enable serial mode by sending some characters over the serial port.
port.write(('www\r\n').encode())

# Read and ignore any unintended responses
port.readline().decode("ascii")

# Get the product information
port.write(('?\r\n').encode())
productInfo = port.readline().decode("ascii")
print('Product information: ' + productInfo)

#Create 2d array
rows=10
cols=10
arr=[]

for x in range(rows):
	col_elements=[]
	for y in range(cols):
		col_elements.append(0)
	arr.append(col_elements)
print(arr)


#10x10 test scanning
r=0
c=0
x=0
while x < 100:	
	# Get distance reading (First return, default filtering)
	port.write(('LD\r\n').encode())
	distanceStr = port.readline().decode("ascii")

	# Convert the distance string response into a number
	distancecm = float(distanceStr) * 100
	
	print(distancecm)
	
	arr[c][r] = distancecm	
	#print(c,r)	

	#Recursive array to only keep last 5 values

#ROW AND COLUMNS ARE SWITCHED
	if x>1:
		change = arr[c][r] - arr[c][r-1]
		if r == 0:
			change = arr[c][r] - arr[c-1][9]
		print("First value:", arr[c][r])
		print("Second value:", arr[c][r-1])
		print(change)
		if abs(change) > 2:
			print("OBJECT HERE")
			x += 100

	for i in arr: # Outer loop
                for j in i: # inner loop
                        print(j, end = " ") # print inserted elements.
                print()

	if r == 9:
		c+=1
		r=0
	else:
		r+=1
	# Wait for 50ms before the next reading is taken
	time.sleep(0.5)

	if c==10:
                x+=1
                print("STOP")
	x+=1

check=0

while check < 1:
	# Get distance reading (First return, default filtering)
	port.write(('LD\r\n').encode())
	distanceStr = port.readline().decode("ascii")

        # Convert the distance string response into a number
	distancecm = float(distanceStr) * 100

        # Do what you want with the distance information here
	print(distancecm)

	if distancecm <= 91.44:
		check += 1
		print("3 feet away")

