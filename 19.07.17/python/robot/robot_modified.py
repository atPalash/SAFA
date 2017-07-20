from BrickPi import *  # import BrickPi.py file to use BrickPi operations
import time
from PID import PID
from basicRobot_planner import PathPlanner


def obstruction(curr_pos, gyr):
    if -3 <= gyr <= 3:
        distance = BrickPi.Sensor[sensor1]
        if (curr_pos[0] - (distance + 10)) <= 5:
            print "--end of workspace--"
        elif (curr_pos[0] - (distance + 10)) > 5:
            print "--obstruction ahead--", distance
            grid_loc = (distance + 20) // 20
            print "--in grid change--", grid_location[0] - grid_loc, grid_location[1]
            if grid_location[0] - grid_loc >= 0 and grid_path[grid_location[0] - grid_loc][grid_location[1]] != "*":
                grid_object.set_grid([grid_location[0] - grid_loc, grid_location[1]])
                print grid_object.optimum_policy()


def localise_in_grid(coord):
    x = coord[0]
    y = coord[1]
    x_grid = x // 20
    y_grid = y // 20
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
    deg = [1]
    port = [motor1]
    power = [120]
    motorRotateDegree(power, deg, port, 0, 0)
    angle = [-1, -90, -270]
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
        x = x_neg + 10
        y = (y_pos + y_neg) / 2
        return [x, y]
    elif gyr <= -90:
        for ang in angle:
            if ang == angle[0]:
                y_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[1]:
                x_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[2]:
                x_pos = coordinate_finder(ang)
                stop()
                time.sleep(0.5)
        x = (x_pos + x_neg) / 2
        y = y_neg + 10
        return [x, y]
    elif gyr >= 90:
        for ang in angle:
            if ang == angle[0]:
                y_pos = coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[1]:
                x_pos = coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[2]:
                x_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(0.5)
        x = (x_pos + x_neg) / 2
        y = y_pos + 10
        return [x, y]


def check_path(path_arr, gyr, grid_loc):
    path = []
    if -3 <= gyr <= 3:
        for i in reversed(range(grid_loc[0] + 1)):
            if path_arr[i][grid_loc[1]] == ' ':
                path.append('X')
            else:
                path.append(path_arr[i][grid_loc[1]])
        obs_index = path.index('X') if 'X' in path else 4
        path = path[:obs_index]
        print "new path is:", path
        fwd(path)


# Move Forward
def fwd(path_to_take):
    for movement in path_to_take:
        if movement == "U":
            turn(0)
            stop()
            initial_obs = BrickPi.Sensor[sensor1]
            while (initial_obs - BrickPi.Sensor[sensor1] <= 15) and BrickPi.Sensor[sensor1] >= 10:
                power_adjustment(0)
                BrickPi.MotorSpeed[motor2] = get_motor_speed2()
                BrickPi.MotorSpeed[motor3] = get_motor_speed3()
                BrickPiUpdateValues()
            stop()
        if movement == "L":
            turn(-90)
            stop()
            initial_obs = BrickPi.Sensor[sensor1]
            print "--initial obs--", initial_obs
            while (initial_obs - BrickPi.Sensor[sensor1] <= 15) and BrickPi.Sensor[sensor1] >= 10:
                print BrickPi.Sensor[sensor1]
                power_adjustment(-90)
                BrickPi.MotorSpeed[motor2] = get_motor_speed2()
                BrickPi.MotorSpeed[motor3] = get_motor_speed3()
                BrickPiUpdateValues()
            print "--distance--", BrickPi.Sensor[sensor1]
            stop()
            turn(0)
            stop()
        if movement == "R":
            turn(90)
            stop()
            initial_obs = BrickPi.Sensor[sensor1]
            while (initial_obs - BrickPi.Sensor[sensor1] <= 15) and BrickPi.Sensor[sensor1] >= 10:
                power_adjustment(90)
                BrickPi.MotorSpeed[motor2] = get_motor_speed2()
                BrickPi.MotorSpeed[motor3] = get_motor_speed3()
                BrickPiUpdateValues()
            stop()
            turn(0)
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
    gyr = BrickPi.Sensor[sensor3]
    check_cumulitive_speed()
    p = PID(1.0, 0.4, 1.2)
    p.setPoint(0)
    if set_point == 0:
        if 3 < gyr:
            speed_correction = int(p.update(gyr))
            motor_spd3 = int(get_motor_speed3() - abs(speed_correction))
            motor_spd2 = int(get_motor_speed2() + abs(speed_correction))
            set_motor_speed2(motor_spd2)
            set_motor_speed3(motor_spd3)
        elif gyr < -3:
            speed_correction = int(p.update(gyr))
            motor_spd3 = int(get_motor_speed3() + abs(speed_correction))
            motor_spd2 = int(get_motor_speed2() - abs(speed_correction))
            set_motor_speed2(motor_spd2)
            set_motor_speed3(motor_spd3)
    elif set_point == -90:
        if -87 < gyr:
            print "-90 power adjustment"
            speed_correction = int(p.update(gyr - (-87)))
            motor_spd3 = int(get_motor_speed3() - abs(speed_correction))
            motor_spd2 = int(get_motor_speed2() + abs(speed_correction))
            set_motor_speed2(motor_spd2)
            set_motor_speed3(motor_spd3)
        elif gyr < -93:
            speed_correction = int(p.update(gyr - (-93)))
            motor_spd3 = int(get_motor_speed3() + abs(speed_correction))
            motor_spd2 = int(get_motor_speed2() - abs(speed_correction))
            set_motor_speed2(motor_spd2)
            set_motor_speed3(motor_spd3)
    elif set_point == 90:
        if 93 < gyr:
            speed_correction = int(p.update(gyr - 93))
            motor_spd3 = int(get_motor_speed3() - abs(speed_correction))
            motor_spd2 = int(get_motor_speed2() + abs(speed_correction))
            set_motor_speed2(motor_spd2)
            set_motor_speed3(motor_spd3)
        elif gyr < 87:
            speed_correction = int(p.update(gyr - 87))
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
    count += 1
    print "counter", count
    grid_path = grid_object.optimum_policy()
    try:
        BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
        # print BrickPi.Sensor[sensor1]
        # fwd(['U', 'U', 'L', 'L'])
        gyro = BrickPi.Sensor[sensor3]
        if step == 0:
            gyro = BrickPi.Sensor[sensor3]
            print '--initial gyro--', gyro
            current_position = coordinates(gyro)
            print "---current_position---", current_position[0], current_position[1]
            grid_location = localise_in_grid(current_position)
            print grid_location
            obstruction(current_position, gyro)
            if grid_path[grid_location[0]][grid_location[1]] == "*":
                print "=====REACHED GOAL====="
            else:
                print "---searching path to goal---"
                obstruction(current_position, gyro)
                grid_path = grid_object.optimum_policy()
                check_path(grid_path, gyro, grid_location)
            time.sleep(1)
    except (KeyboardInterrupt, SystemExit):
        break

        # movement = grid_path[grid_location[0]][grid_location[1]]
        # print "---movement---", movement
        # if movement == ' ':
        #     back(current_position)
        # elif movement == 'U':
        #     # print "---forward called---"
        #     turn(0)
        #     fwd(obs, "x")
        # elif movement == 'L':
        #     turn(-90)
        #     fwd_by_block(current_position)
        #     stop()
        #     # fwd(current_position[0], current_position[1], 1, "y", -90)
        #     # turn(90)


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
