import socket  # Import socket module\
from BrickPi import *
import time
from PID import PID


def send_to_client(i=0):
    while i < 5:
        i += 1
    conn.send("complete" + "\r\n")

# Obstruction detection
def obstruction():
    distance = BrickPi.Sensor[sensor1]
    # print("distance", distance)
    return distance
# Direction selection
def decide_left_right():
    degree_obstruction_pos = 0
    degree_obstruction_neg = 0
    turn(-50)
    for j in range(-40, 50, 10):
        print "---angle---", j
        if j == 0:
            continue
        elif j < 0:
            turn(j)
            degree_obstruction_neg = degree_obstruction_neg + obstruction()
            print "--degree obs neg--", degree_obstruction_neg
        else:
            turn(j)
            degree_obstruction_pos = degree_obstruction_pos + obstruction()
            print "--degree obs pos--", degree_obstruction_pos
    turn(0)
    if degree_obstruction_pos >= degree_obstruction_neg:
        print "take right"
        direction = "right"
    else:
        print "take left"
        direction = "left"
    return direction

# Move Forward
def fwd():
    # BrickPi.Sensor[PORT] stores the value obtained from sensor
    BrickPi.MotorSpeed[motor2] = motor_speed
    BrickPi.MotorSpeed[motor3] = motor_speed
    BrickPiUpdateValues()

# Move backward
def back():
    # print "in back"
    BrickPi.MotorSpeed[motor2] = -motor_speed
    BrickPi.MotorSpeed[motor3] = -motor_speed
    BrickPiUpdateValues()

# Stop
def stop():
    BrickPi.MotorSpeed[motor2] = 0
    BrickPi.MotorSpeed[motor3] = 0
    BrickPiUpdateValues()

# turn the vehicle
def turn(angle):
    while True:
        gyro = BrickPi.Sensor[sensor2]
        print "----gyro----", gyro
        if gyro == angle:
            print "done"
            stop()
            time.sleep(1)
            break
        elif gyro > angle:
            BrickPi.MotorSpeed[motor2] = 150
            BrickPi.MotorSpeed[motor3] = -150
            BrickPiUpdateValues()
        else:
            BrickPi.MotorSpeed[motor2] = -150
            BrickPi.MotorSpeed[motor3] = 150
            BrickPiUpdateValues()


soc = socket.socket()  # Create a socket object
host = "localhost"  # Get local machine name
port = 2004  # Reserve a port for your service.
soc.bind((host, port))  # Bind to the port
soc.listen(5)  # Now wait for client connection.

BrickPiSetup()  # setup the serial port for communication
motor_speed = 200
motor2 = PORT_B
motor3 = PORT_C

BrickPi.MotorEnable[motor2] = 1  # Enable the Motor B
BrickPi.MotorEnable[motor3] = 1  # Enable the Motor C

sensor1 = PORT_2
sensor2 = PORT_1
BrickPi.SensorType[sensor2] = TYPE_SENSOR_EV3_GYRO_M0
BrickPi.SensorType[sensor1] = TYPE_SENSOR_ULTRASONIC_CONT

BrickPiSetupSensors()  # Send the properties of sensors to BrickPi
BrickPi.Timeout = 10000  # Set timeout value for the time till which to run the motors after the last command is pressed
BrickPiSetTimeout()  # Set the timeout

step = 0
decision = ""
BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
BrickPi.EncoderOffset[motor2] = BrickPi.Encoder[motor2]
BrickPi.EncoderOffset[motor3] = BrickPi.Encoder[motor3]
BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors

while True:
    conn, addr = soc.accept()  # Establish connection with client.
    print("Got connection from", addr)
    msg = ""
    while True:
        part = conn.recv(1024)
        msg += part
        text = msg.split('\n')
        if len(text) != 1:
            if text[1] == '':
                msg = text[0]
                break
        else:
            if text[0] == '':
                break
    x = len(text) == 1
    y = msg == ''
    if x & y:
        continue
    if msg == "start":
        while True:
            BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
            print "distance_to_obstruction", obstruction()
            if obstruction() == -1:
                continue
            if obstruction() > 20 and step == 0:
                fwd()
            elif obstruction() <= 20 and step == 0:
                stop()
                decision = decide_left_right()
                step = 1
            else:
                if decision == "right":
                    turn(90)
                    break
                else:
                    turn(-90)
                    break
        send_to_client()
    elif msg == "stop":
        stop()
        send_to_client()
    else:
        print("NA")
        send_to_client()
    time.sleep(.01)  # sleep for 10 ms
