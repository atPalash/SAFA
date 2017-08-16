

from BrickPi import *   #import BrickPi.py file to use BrickPi operations

BrickPiSetup()  # setup the serial port for sudo su communication
BrickPiSetupSensors()       #Send the properties of sensors to BrickPi
BrickPiUpdateValues()

print ("Input is in ticks! Enter the ticks to move.")
BrickPi.EncoderOffset[PORT_A] = BrickPi.Encoder[PORT_A]
BrickPiUpdateValues()
while True:
	try:
		i = int(raw_input())
		result = BrickPiUpdateValues() 
		if not result :							# if updating values succeeded
			print ("=============")
			print ( "Encoder Value: " + str(((BrickPi.Encoder[PORT_A])) /2))	# print the encoder raw 
			power=[100] # 0 to 255
			deg = [i]
			port=[PORT_A]
			motorRotateDegree(power,deg,port, 0, 0)
			print ( "Encoder Value: " + str(((BrickPi.Encoder[PORT_A])) /2))	# print the encoder raw
			print ("=============")
	except:
		print("Try again.  Enter a number value.")
