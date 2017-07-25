import socket  # Import socket module\


# from BrickPi import *


def send_to_client(who, i=0):
    while i < 5:
        # conn.send("done " + who + "\r\n")
        i += 1
        print i
    conn.send("complete" + "\r\n")


# Move Forward
def fwd():
    print("in forward")
    # BrickPi.MotorSpeed[motor1] = 100
    send_to_client("exiting forward")


# Move backward
def back():
    print("in back")
    # BrickPi.MotorSpeed[motor1] = -100
    send_to_client("exiting back")


# Stop
def stop():
    print("in stop")
    # BrickPi.MotorSpeed[motor1] = 0
    send_to_client("exiting stop")


soc = socket.socket()  # Create a socket object
host = "localhost"  # Get local machine name
port = 2004  # Reserve a port for your service.
soc.bind((host, port))  # Bind to the port
soc.listen(5)  # Now wait for client connection.
conn, addr = soc.accept()  # Establish connection with client.

# BrickPiSetup()  # setup the serial port for communication
# motor1 = PORT_A
# BrickPi.MotorEnable[motor1] = 1  # Enable the Motor A
# BrickPiSetupSensors()  # Send the properties of sensors to BrickPi
# BrickPi.Timeout = 10000  # Set timeout value for the time till which to run the motors after the last command is pressed
# BrickPiSetTimeout()  # Set the timeout

while True:
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
        break
    if msg == "forward":
        print("in forward")
        fwd()
    elif msg == "stop":
        print("stop")
        stop()
    elif msg == "back":
        print("back")
        back()
    else:
        print("NA")
        send_to_client("NA")

        # print(BrickPiUpdateValues())
        # BrickPiUpdateValues()  # Update the motor values
        #
        # time.sleep(.01)  # sleep for 10 ms
