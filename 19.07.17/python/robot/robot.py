from BrickPi import *  # import BrickPi.py file to use BrickPi operations
import time
from PID import PID
from basicRobot_planner import PathPlanner


def obstruction(gyr):
    distance = BrickPi.Sensor[sensor1]
    crd = coordinates(gyr)
    if gyr == 0:
        if (crd[0] - distance) >= 5:
            print "--obstruction ahead--"
            grid_loc = distance//20
    return distance


def localise_in_grid(coord):
    x = coord[0]
    y = coord[1]
    x_grid = x // 18
    y_grid = y // 18
    grid_crd = [x_grid, y_grid]
    return grid_crd


def coordinate_finder(ang):
    deg = [ang]
    port = [motor1]
    power = [120]
    motorRotateDegree(power, deg, port, 0, 0)
    coordinate_list = []
    for i in range(10):
        coordinate_list.append(int(BrickPi.Sensor[sensor2]))
    crd = sum(coordinate_list) / len(coordinate_list)
    return crd


def coordinates(gyr):
    deg = [2]
    port = [motor1]
    power = [120]
    motorRotateDegree(power, deg, port, 0, 0)
    angle = [-2, -90, -270]
    x_pos = 0
    x_neg = 0
    y_pos = 0
    y_neg = 0
    if -3 <= gyro <= 10:
        for ang in angle:
            if ang == angle[2]:
                y_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[1]:
                y_pos = coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[0]:
                x_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(0.5)
        x = x_neg
        y = (y_pos + y_neg)/2
        return [x, y]
    elif gyr <= -90:
        for ang in angle:
            if ang == angle[2]:
                y_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[1]:
                x_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[0]:
                x_pos = coordinate_finder(ang)
                stop()
                time.sleep(0.5)
        x = (x_pos + x_neg)/2
        y = y_neg / 2
        return [x, y]
    elif gyr >= 90:
        for ang in angle:
            if ang == angle[2]:
                y_pos = coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[1]:
                x_pos = coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[0]:
                x_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(0.5)
        x = (x_pos + x_neg)/2
        y = y_neg / 2
        return [x, y]
# Move Forward
def fwd(obs_distance, axis):
    while True:
        if axis == "x":
            # print "x_coordinate - block*18", x_coordinate - block*18
            while (obs_distance - BrickPi.Sensor[sensor1]) <= 10:
                # print obs_distance
                # print "in fwd_x", BrickPi.Sensor[sensor1]
                power_adjustment(0)
                BrickPi.MotorSpeed[motor2] = get_motor_speed2()
                BrickPi.MotorSpeed[motor3] = get_motor_speed3()
                BrickPiUpdateValues()
                # print BrickPi.Encoder[motor2], BrickPi.Encoder[motor3]
                # print "2,3", get_motor_speed2(), get_motor_speed3()
        elif axis == "y":
            while (obs_distance - BrickPi.Sensor[sensor1]) <= 10:
                power_adjustment(0)
                BrickPi.MotorSpeed[motor2] = get_motor_speed2()
                BrickPi.MotorSpeed[motor3] = get_motor_speed3()
                BrickPiUpdateValues()
                # print BrickPi.Encoder[motor2], BrickPi.Encoder[motor3]
                # print "2,3", get_motor_speed2(), get_motor_speed3()
        break
    stop()


def fwd_by_block(curr_pos):
    y = curr_pos[1]
    while abs(y - BrickPi.Sensor[sensor2]) >= 20:
        # power_adjustment(0)
        BrickPi.MotorSpeed[motor2] = get_motor_speed2()
        BrickPi.MotorSpeed[motor3] = get_motor_speed3()
        BrickPiUpdateValues()

def turn(angle):
    while True:
        gyro = BrickPi.Sensor[sensor3]
        # print "----gyro----", gyro
        if gyro == angle:
            # print "done"
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


# Stop
def stop():
    BrickPi.MotorSpeed[motor1] = 0
    BrickPi.MotorSpeed[motor2] = 0
    BrickPi.MotorSpeed[motor3] = 0
    BrickPiUpdateValues()


def back(curr_pos):
    while abs(curr_pos[0] - BrickPi.Sensor[sensor2]) < 20:
        BrickPi.MotorSpeed[motor2] = -200
        BrickPi.MotorSpeed[motor3] = -200
        BrickPiUpdateValues()


def check_cumulitive_speed():
    if get_motor_speed2() + get_motor_speed3() != 400:
        set_motor_speed3(200)
        set_motor_speed2(200)


def power_adjustment(set_point):
    BrickPiUpdateValues()
    gyro = BrickPi.Sensor[sensor3]
    check_cumulitive_speed()
    p = PID(1.0, 0.4, 1.2)
    p.setPoint(set_point)
    # print "gyro", str(gyro)
    # if -3 <= gyro <= 3:
    #     print "on route"
    if 3 < gyro <= 90:
        # print "---tilting right take left---"
        speed_correction = int(p.update(gyro))
        # print "---speed correction----", speed_correction
        motor_spd3 = int(get_motor_speed3() - abs(speed_correction))
        motor_spd2 = int(get_motor_speed2() + abs(speed_correction))
        set_motor_speed2(motor_spd2)
        set_motor_speed3(motor_spd3)

    elif -90 <= gyro < -3:
        # print "---tilting left take right ---"
        speed_correction = int(p.update(gyro))
        # print "---speed correction----", speed_correction
        motor_spd3 = int(get_motor_speed3() + abs(speed_correction))
        motor_spd2 = int(get_motor_speed2() - abs(speed_correction))
        set_motor_speed2(motor_spd2)
        set_motor_speed3(motor_spd3)

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
    # print "motor_speed2", motor_speed2
    BrickPiUpdateValues()


def set_motor_speed3(speed):
    BrickPiUpdateValues()
    global motor_speed3
    if abs(speed) < 240:
        motor_speed3 = speed
    else:
        print "exceeding limit"
    # print "motor_speed3", motor_speed3
    BrickPiUpdateValues()


BrickPiSetup()  # setup the serial port for communication

motor1 = PORT_A
motor2 = PORT_B
motor3 = PORT_C

sensor1 = PORT_1
sensor2 = PORT_2
sensor3 = PORT_3

BrickPi.SensorType[sensor1] = TYPE_SENSOR_ULTRASONIC_CONT
BrickPi.SensorType[sensor2] = TYPE_SENSOR_ULTRASONIC_CONT
BrickPi.SensorType[sensor3] = TYPE_SENSOR_EV3_GYRO_M0

BrickPi.MotorEnable[motor1] = 1  # Enable the Motor A
BrickPi.MotorEnable[motor2] = 1  # Enable the Motor B
BrickPi.MotorEnable[motor3] = 1  # Enable the Motor C

BrickPiSetupSensors()  # Send the properties of sensors to BrickPi

step = 0
decision = ""
current_position = [0, 0]
grid_object = PathPlanner()
goal = [0, 0]
grid_object.set_goal(goal)

count = 0
BrickPi.EncoderOffset[motor1] = BrickPi.Encoder[motor1]
while True:
    gyro = BrickPi.Sensor[sensor3]

    count += 1
    print "counter", count
    try:
        BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
        obs = obstruction()
        if obstruction() == -1:
            continue
        if step == 0:
            grid_path = grid_object.optimum_policy()
            # print grid_path
            gyro = BrickPi.Sensor[sensor3]
            print '--initial gyro--', gyro
            current_position = coordinates(gyro)
            print "---current_position---", current_position[0], current_position[1]
            grid_location = localise_in_grid(current_position)
            print grid_location
            movement = grid_path[grid_location[0]][grid_location[1]]
            if 30 >= obs and current_position[0] >= obs or 30 >= obs and current_position[1] >= obs:
                print "--gyro--", gyro
                print "--obstruction ahead--", obs
                if gyro == 0 or -3:
                    print "--in grid change--", grid_location[0] - 2, grid_location[1]
                    if grid_location[0] - 2 >= 0:
                        grid_object.set_grid([grid_location[0] - 2, grid_location[1]])
                        print grid_object.optimum_policy()
                elif gyro == 90:
                    grid_object.set_grid([grid_location[0], grid_location[1] - 2])
                elif gyro == -90:
                    grid_object.set_grid([grid_location[0], grid_location[1] + 2])
                elif gyro == 180:
                    grid_object.set_grid([grid_location[0] + 2, grid_location[1]])
            print "---movement---", movement
            if movement == ' ':
                back(current_position)
            elif movement == 'U':
                # print "---forward called---"
                turn(0)
                fwd(obs, "x")
            elif movement == 'L':
                turn(-90)
                fwd_by_block(current_position)
                stop()
                # fwd(current_position[0], current_position[1], 1, "y", -90)
                # turn(90)


            # elif movement == 'D':
            #     turn(180)
            #     fwd(current_position[0], current_position[1], 1, "x", 180)
            #     turn(-180)
            # elif movement == 'R':
            #     turn(90)
            #     stop()
            #     fwd(current_position[0], current_position[1], 1, "y", 90)
            #     turn(-90)
            #     stop()
            time.sleep(1)
    except (KeyboardInterrupt, SystemExit):
        break


        # coord = coordinates()
        # print coord
        # grid_coord = localise_in_grid(coord)
        # print grid_coord

        # print coordinates(0)
        # stop()
        # print ("Encoder Value: " + str(((BrickPi.Encoder[PORT_A])) / 2))
        # print coordinates(-90)
        # stop()
        # print ("Encoder Value: " + str(((BrickPi.Encoder[PORT_A])) / 2))
        # break


        # movement = str(grid_path[grid_location[0]][grid_location[1]])
        # print movement
        # step = 1
        # elif obstruction() > 20 and step == 1:
        #     print "distance_to_obstruction", obstruction()
        #     print "---current_position---", current_position[0], current_position[1]
        #     fwd(current_position[0], current_position[1], 2, "x")
        # elif obstruction() <= 20 and step == 1:
        #     stop()
        #     decision = decide_left_right()
        #     step = 2
        # else:
        #     if decision == "right":
        #         turn(90)
        #         step = 3
        #     else:
        #         turn(-90)
        #         step = 3
        # if step == 3:
        #     time.sleep(5)  # sleep for 10 ms
        #     step = 1
