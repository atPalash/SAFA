from BrickPi import *  # import BrickPi.py file to use BrickPi operations
import time
from PID import PID


# Obstruction detection
def obstruction():
    distance = BrickPi.Sensor[sensor1]
    # print("distance", distance)
    return distance


def localise():
    x_workspace_length = 120
    y_workspace_length = 80
    x_0 = coordinates(0)
    y_90 = coordinates(90)
    x_180 = coordinates(180)
    y_neg90 = coordinates(-90)
    x = (x_0 + x_workspace_length - x_180) / 2
    y = (y_90 + y_workspace_length - y_neg90) / 2
    turn(0)
    return [x, y]


def coordinates(angle):
    if angle == 0 or angle == 180:
        turn(angle)
        print "in x"
        x_coordinate_list = []
        for i in range(10):
            x_coordinate_list.append(int(BrickPi.Sensor[sensor1]))
        print x_coordinate_list
        x_coordinate = max(set(x_coordinate_list), key=x_coordinate_list.count)
        print "x-", x_coordinate
        return x_coordinate
    elif angle == 90 or angle == -90:
        turn(angle)
        print "in y"
        y_coordinate_list = []
        for i in range(10):
            y_coordinate_list.append(int(BrickPi.Sensor[sensor1]))
        y_coordinate = max(set(y_coordinate_list), key=y_coordinate_list.count)
        print y_coordinate_list
        print "y-", y_coordinate
        return y_coordinate


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
def fwd(x_coordinate, y_coordinate, block, axis):
    while True:
        if axis == "x":
            print "x_coordinate - block*18", x_coordinate - block*18
            while BrickPi.Sensor[sensor1] > x_coordinate - block*18:
                print "in fwd_x", BrickPi.Sensor[sensor1]
                power_adjustment(0)
                BrickPi.MotorSpeed[motor2] = get_motor_speed2()
                BrickPi.MotorSpeed[motor3] = get_motor_speed3()
                BrickPiUpdateValues()
                print BrickPi.Encoder[motor2], BrickPi.Encoder[motor3]
                print "2,3", get_motor_speed2(), get_motor_speed3()
        elif axis == "y":
            while BrickPi.Sensor[sensor1] > y_coordinate - block*18:
                power_adjustment(0)
                BrickPi.MotorSpeed[motor2] = get_motor_speed2()
                BrickPi.MotorSpeed[motor3] = get_motor_speed3()
                BrickPiUpdateValues()
                print BrickPi.Encoder[motor2], BrickPi.Encoder[motor3]
                print "2,3", get_motor_speed2(), get_motor_speed3()
        break
    stop()
def back():
    # print "in back"
    BrickPi.MotorSpeed[motor2] = -200
    BrickPi.MotorSpeed[motor3] = -200
    BrickPiUpdateValues()


# Stop
def stop():
    BrickPi.MotorSpeed[motor2] = 0
    BrickPi.MotorSpeed[motor3] = 0
    BrickPiUpdateValues()


# Move Right
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


def search_path_right(call_num):
    fwd()
    time.sleep(2)
    stop()
    turn(-90)
    if obstruction() < 20:
        fwd()
        time.sleep(2)
        stop()
        turn(-90)
        fwd()
        time.sleep(call_num * 2)
        stop()
        turn(90)
        return True
    else:
        next_call_num = call_num + 1
        turn(90)
        search_path_right(next_call_num)


def check_cumulitive_speed():
    if get_motor_speed2() + get_motor_speed3() == 400:
        print "---do nothing---"
    else:
        set_motor_speed3(200)
        set_motor_speed2(200)


def power_adjustment(set_point):
    BrickPiUpdateValues()
    gyro = BrickPi.Sensor[sensor2]
    check_cumulitive_speed()
    p = PID(1.0, 0.4, 1.2)
    p.setPoint(set_point)
    print "gyro", str(gyro)
    if -3 <= gyro <= 3:
        print "on route"
    elif 3 < gyro <= 90:
        print "---tilting right take left---"
        speed_correction = int(p.update(gyro))
        print "---speed correction----", speed_correction
        motor_spd3 = int(get_motor_speed3() - abs(speed_correction))
        motor_spd2 = int(get_motor_speed2() + abs(speed_correction))
        set_motor_speed2(motor_spd2)
        set_motor_speed3(motor_spd3)

    elif -90 <= gyro < -3:
        print "---tilting left take right ---"
        speed_correction = int(p.update(gyro))
        print "---speed correction----", speed_correction
        motor_spd3 = int(get_motor_speed3() + abs(speed_correction))
        motor_spd2 = int(get_motor_speed2() - abs(speed_correction))
        set_motor_speed2(motor_spd2)
        set_motor_speed3(motor_spd3)
    else:
        print "on route"


motor_speed3 = 200
motor_speed2 = 200


def get_motor_speed2():
    BrickPiUpdateValues()
    return motor_speed2


def get_motor_speed3():
    BrickPiUpdateValues()
    return motor_speed3


def set_motor_speed2(speed):
    BrickPiUpdateValues()
    global motor_speed2
    if abs(speed) < 240:
        motor_speed2 = speed
    else:
        print "exceeding limit"
    print "motor_speed2", motor_speed2
    BrickPiUpdateValues()


def set_motor_speed3(speed):
    BrickPiUpdateValues()
    global motor_speed3
    if abs(speed) < 240:
        motor_speed3 = speed
    else:
        print "exceeding limit"
    print "motor_speed3", motor_speed3
    BrickPiUpdateValues()


BrickPiSetup()  # setup the serial port for communication

motor2 = PORT_B
motor3 = PORT_C

BrickPi.MotorEnable[motor2] = 1  # Enable the Motor B
BrickPi.MotorEnable[motor3] = 1  # Enable the Motor C

sensor1 = PORT_1
sensor2 = PORT_4
# sensor3 = PORT_3
BrickPi.SensorType[sensor1] = TYPE_SENSOR_ULTRASONIC_CONT
BrickPi.SensorType[sensor2] = TYPE_SENSOR_EV3_GYRO_M0
# BrickPi.SensorType[sensor3] = TYPE_SENSOR_EV3_US_M0
BrickPiSetupSensors()  # Send the properties of sensors to BrickPi
# BrickPi.Timeout = 10000  # Set timeout value for the time till which to run the motors after the last command is pressed
# BrickPiSetTimeout()  # Set the timeout


step = 0
decision = ""
current_position = [0, 0]
# BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
# BrickPi.EncoderOffset[motor2] = BrickPi.Encoder[motor2]
# BrickPi.EncoderOffset[motor3] = BrickPi.Encoder[motor3]
# BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors

while True:
    try:
        BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
        gyro = BrickPi.Sensor[sensor2]
        print str(gyro)
        print "distance_to_obstruction", obstruction()
        if obstruction() == -1:
            continue
        if step == 0:
            current_position = localise()
            print "---current_position---", current_position[0], current_position[1]
            step = 1
        elif obstruction() > 20 and step == 1:
            print "distance_to_obstruction", obstruction()
            print "---current_position---", current_position[0], current_position[1]
            fwd(current_position[0], current_position[1], 2, "x")
        elif obstruction() <= 20 and step == 1:
            stop()
            decision = decide_left_right()
            step = 2
        else:
            if decision == "right":
                turn(90)
                step = 3
            else:
                turn(-90)
                step = 3
        if step == 3:
            time.sleep(5)  # sleep for 10 ms
            step = 1

    except (KeyboardInterrupt, SystemExit):
        break
