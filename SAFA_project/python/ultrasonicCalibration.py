from BrickPi import *   								#import BrickPi.py file to use BrickPi operations
BrickPiSetup()  										# setup the serial port for communication
############################################
# !  Set the sensor type on the line below.  
BrickPi.SensorType[PORT_1] = TYPE_SENSOR_ULTRASONIC_CONT
BrickPi.SensorType[PORT_2] = TYPE_SENSOR_ULTRASONIC_CONT
BrickPiSetupSensors()   								#Send the properties of sensors to BrickPi.  Set up the BrickPi.
# There's often a long wait for setup with the EV3 sensors.  Up to 5 seconds.

while True:
	result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors 
	if not result :
		us1 = BrickPi.Sensor[PORT_1]
		us2 = BrickPi.Sensor[PORT_2]
		print('us1', us1, 'us2', us2)
		
	time.sleep(.01)     # sleep for 10 ms
