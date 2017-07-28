import os
from BrickPi import *
import time
from PID import PID
from basicRobot_planner import PathPlanner
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

def obstruction(obst_crd):
    grid_object.set_grid(obst_crd)
    print grid_object.optimum_policy()


def localise_in_grid(coord):
    x = coord[0]
    y = coord[1]
    x_grid = x // 20
    y_grid = y // 20
    grid_crd = [x_grid, y_grid]
    print "--grid coord--", grid_crd
    return grid_crd


def coordinate_finder(ang):
    deg = [ang]
    port = [motor1]
    power = [120]
    motorRotateDegree(power, deg, port, 0, 0)
    coordinate_list = []
    for i in range(10):
        coordinate_list.append(int(BrickPi.Sensor[sensor1]))
    print coordinate_list
    crd = sum(coordinate_list) / len(coordinate_list)
    # print crd
    return crd


def coordinates(gyr):
    in_angle = -1
    deg = [in_angle]
    port = [motor1]
    power = [120]
    motorRotateDegree(power, deg, port, 0, 0)
    stop()
    time.sleep(0.5)
    angle = [70, 160]
    x_neg = 0
    y_neg = 0

    if -10 <= gyr <= 10:
        for ang in angle:
            if ang == angle[0]:
                y_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[1]:
                x_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(0.5)
        x = x_neg
        y = y_neg
        deg = [in_angle]
        port = [motor1]
        power = [120]
        motorRotateDegree(power, deg, port, 0, 0)
        stop()
        time.sleep(0.5)
        return [x, y]
    elif -110 <= gyr <= -80:
        for ang in angle:
            if ang == angle[1]:
                y_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[0]:
                x_neg = coordinate_finder(ang)
                stop()
                time.sleep(0.5)
        x = x_neg
        y = y_neg
        deg = [in_angle]
        port = [motor1]
        power = [120]
        motorRotateDegree(power, deg, port, 0, 0)
        stop()
        time.sleep(0.5)
        return [x, y]


def move(st_pt):
    power_adjustment(st_pt)
    BrickPi.MotorSpeed[motor2] = get_motor_speed2()
    BrickPi.MotorSpeed[motor3] = get_motor_speed3()
    BrickPiUpdateValues()


prev_grid = [0, 0]
prev_movement = ''
def set_previous_states(loc, mov):
    global prev_grid
    prev_grid = loc
    global prev_movement
    prev_movement = mov


def fwd(direction, c_pos, g_pos):
    turn(0)
    stop()
    # gyr = BrickPi.Sensor[sensor3]
    curr_pos = c_pos
    grd_pos = g_pos
    if direction == 'U':
        turn(0)
        stop()
        if curr_pos[0] - BrickPi.Sensor[sensor1] <= 10:
            print "--this X-lane is clear--"
            grid_object.reset_grid(grd_pos)
            grid_object.optimum_policy()
            print grid_object.optimum_policy()
            init_obs = BrickPi.Sensor[sensor1]
            while init_obs > 12:
                init_obs = BrickPi.Sensor[sensor1]
                move(0)
                if init_obs < 0 or init_obs > 100:
                    init_obs = 20
                    # continue
                print init_obs
            print "out of while fwd", init_obs
            obs_flag = True
            set_previous_states(grd_pos, direction)
            stop()
            return obs_flag
        elif grd_pos[0] == goal[0] and grd_pos[1] == goal[1]:
            while BrickPi.Sensor[sensor1] >= 10:
                print "--towards goal--"
                move(0)
            obs_flag = False
            set_previous_states(grd_pos, direction)
            stop()
            return obs_flag
        else:
            print "obstruction detected at X"
            init_obs = BrickPi.Sensor[sensor1]
            if init_obs >= 20:
                init_obs = BrickPi.Sensor[sensor1]
                var_init_prevobs = init_obs
                while init_obs > 16:
                    move(0)
                    init_obs = BrickPi.Sensor[sensor1]
                    if init_obs < 0 or init_obs > 100:
                        init_obs = var_init_prevobs
                        # continue
                    var_init_prevobs = init_obs
                    print init_obs
                obs_flag = True
                set_previous_states(grd_pos, direction)
                stop()
                return obs_flag
            else:
                obs_flag = True
                stop()
                return obs_flag
    elif direction == 'D':
        turn(180)
        stop()
        if (80 - curr_pos[0]) - BrickPi.Sensor[sensor1] <= 20:
            init_obs = BrickPi.Sensor[sensor1]
            print "--this X-lane is clear--EOF at", init_obs
            if init_obs >= 20:
                var_init_obs = BrickPi.Sensor[sensor1]
                var_init_prevobs = var_init_obs
                while init_obs - var_init_obs <= 17:
                    move(180)
                    var_init_obs = BrickPi.Sensor[sensor1]
                    if var_init_obs < 0 or var_init_obs > 100:
                        var_init_obs = var_init_prevobs
                        continue
                    var_init_prevobs = var_init_obs
                    print 'var_init_obs', var_init_obs

            obs_flag = True
            set_previous_states(grd_pos, direction)
            stop()
            turn(0)
            # move(0)
            stop()
            return obs_flag
        elif grd_pos[0] == goal[0] and grd_pos[1] == goal[1]:
            while BrickPi.Sensor[sensor1] >= 10:
                print "--towards goal--"
                move(-90)
            set_previous_states(grd_pos, direction)
            stop()
            turn(0)
            stop()
        else:
            init_obs = BrickPi.Sensor[sensor1]
            print 'obstruction detected at X', init_obs
            if init_obs >= 20:
                while init_obs - BrickPi.Sensor[sensor1] <= 17:
                    # print "--lane change--", init_obs - BrickPi.Sensor[sensor1]
                    move(180)
                print 'out of whileBrickPi.Sensor[sensor1]', BrickPi.Sensor[sensor1]
                obs_flag = True
                set_previous_states(grd_pos, direction)
                stop()
                turn(0)
                stop()
                return obs_flag
            else:
                obs_flag = True
                stop()
                return obs_flag

    elif direction == 'L':
        turn(-90)
        stop()
        if curr_pos[1] - BrickPi.Sensor[sensor1] <= 20:
            init_obs = BrickPi.Sensor[sensor1]
            print "--this Y-lane is clear--EOF at", init_obs
            if init_obs >= 20:
                var_init_obs = BrickPi.Sensor[sensor1]
                var_init_prevobs = var_init_obs
                while init_obs - var_init_obs <= 17:
                    move(-90)
                    var_init_obs = BrickPi.Sensor[sensor1]
                    if var_init_obs < 0 or var_init_obs > 100:
                        var_init_obs = var_init_prevobs
                        continue
                    var_init_prevobs = var_init_obs
                    print 'var_init_obs', var_init_obs

            obs_flag = True
            set_previous_states(grd_pos, direction)
            stop()
            turn(0)
            move(0)
            stop()
            return obs_flag
        elif grd_pos[0] == goal[0] and grd_pos[1] == goal[1]:
            while BrickPi.Sensor[sensor1] >= 10:
                print "--towards goal--"
                move(-90)
            set_previous_states(grd_pos, direction)
            stop()
            turn(0)
            stop()
        else:
            init_obs = BrickPi.Sensor[sensor1]
            print 'obstruction detected at Y', init_obs
            if init_obs >= 20:
                while init_obs - BrickPi.Sensor[sensor1] <= 17:
                    # print "--lane change--", init_obs - BrickPi.Sensor[sensor1]
                    move(-90)
                print 'out of whileBrickPi.Sensor[sensor1]', BrickPi.Sensor[sensor1]
                obs_flag = True
                set_previous_states(grd_pos, direction)
                stop()
                turn(0)
                stop()
                return obs_flag
            else:
                obs_flag = True
                stop()
                return obs_flag

    elif direction == 'R':
        turn(90)
        stop()
        if curr_pos[1] - BrickPi.Sensor[sensor1] <= 20:
            init_obs = BrickPi.Sensor[sensor1]
            print "--this Y-lane is clear--EOF at", init_obs
            if init_obs >= 20:
                var_init_obs = BrickPi.Sensor[sensor1]
                var_init_prevobs = var_init_obs
                while init_obs - var_init_obs <= 17:
                    move(-90)
                    var_init_obs = BrickPi.Sensor[sensor1]
                    if var_init_obs < 0 or var_init_obs > 100:
                        var_init_obs = var_init_prevobs
                        continue
                    var_init_prevobs = var_init_obs
                    print 'var_init_obs', var_init_obs

            obs_flag = True
            set_previous_states(grd_pos, direction)
            stop()
            turn(0)
            move(0)
            stop()
            return obs_flag
        elif grd_pos[0] == goal[0] and grd_pos[1] == goal[1]:
            while BrickPi.Sensor[sensor1] >= 10:
                print "--towards goal--"
                move(-90)
            set_previous_states(grd_pos, direction)
            stop()
            turn(0)
            stop()
        else:
            init_obs = BrickPi.Sensor[sensor1]
            print 'obstruction detected at Y', init_obs
            if init_obs >= 20:
                while init_obs - BrickPi.Sensor[sensor1] <= 17:
                    # print "--lane change--", init_obs - BrickPi.Sensor[sensor1]
                    move(-90)
                print 'out of whileBrickPi.Sensor[sensor1]', BrickPi.Sensor[sensor1]
                obs_flag = True
                set_previous_states(grd_pos, direction)
                stop()
                turn(0)
                stop()
                return obs_flag
            else:
                obs_flag = True
                stop()
                return obs_flag


def turn(angle):
    while True:
        gyr = BrickPi.Sensor[sensor2]
        # print "----gyro----", gyr
        if gyr == angle:
            # print "done"
            stop()
            time.sleep(1)
            break
        elif gyr > angle:
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


def back():
    initial_obs = BrickPi.Sensor[sensor1]
    while (BrickPi.Sensor[sensor1] - initial_obs) <= 7:
        BrickPi.MotorSpeed[motor2] = -200
        BrickPi.MotorSpeed[motor3] = -200
        BrickPiUpdateValues()


def check_cumulitive_speed():
    if get_motor_speed2() + get_motor_speed3() != 400:
        set_motor_speed3(200)
        set_motor_speed2(200)


def power_adjustment(set_point):
    BrickPiUpdateValues()
    gyr = BrickPi.Sensor[sensor2]
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
            # print "-90 power adjustment"
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
    BrickPiUpdateValues()


def set_motor_speed3(speed):
    BrickPiUpdateValues()
    global motor_speed3
    if abs(speed) < 240:
        motor_speed3 = speed
    BrickPiUpdateValues()

observer_flag = True
class MyHandler(FileSystemEventHandler):
    def on_modified(self, event):
        if event.src_path.endswith('java_reply.txt'):
            path_javareply = os.path.join(
                fs + "home" + fs + "pi" + fs + "Desktop" + fs + "testingFiles" + fs + "28.07.17" + fs + "java_reply.txt")
            fo = open(path_javareply, "r+")
            msg = fo.read(15)
            print "Read String is : ", msg
            msg_split = msg.split(' ')
            first = msg_split[0].split('-')
            second = msg_split[1].split('-')
            if first[0] == 'start' and second[0] == 'end':
                fo = open(path_javareply, "w")
                fo.write("route taken")
                fo.close()
                x = int(first[1][0])
                y = int(first[1][1])
                global start_coordinates
                start_coordinates = [x, y]
                # print start_coordinates
                x = int(second[1][0])
                y = int(second[1][1])
                global end_coordinates
                end_coordinates = [x, y]
                global observer_flag
                observer_flag = False
                # print end_coordinates

BrickPiSetup()  # setup the serial port for communication

motor1 = PORT_A
motor2 = PORT_B
motor3 = PORT_C

sensor1 = PORT_1
sensor2 = PORT_2

BrickPi.SensorType[sensor1] = TYPE_SENSOR_ULTRASONIC_CONT
BrickPi.SensorType[sensor2] = TYPE_SENSOR_EV3_GYRO_M0

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
start_coordinates = [0, 0]
end_coordinates = [0, 0]
count = 0
obstruction_flag = False
fs = os.sep
BrickPi.EncoderOffset[motor1] = BrickPi.Encoder[motor1]

while True:
    event_handler = MyHandler()
    observer = Observer()
    observer.schedule(event_handler, fs + "home" + fs + "pi" + fs + "Desktop" + fs + "testingFiles" + fs + "28.07.17",
                      recursive=False)
    observer.start()
    while observer_flag:
        time.sleep(1)
        BrickPiUpdateValues()
    observer.stop()
    while True:
        BrickPiUpdateValues()
        gyro = BrickPi.Sensor[sensor2]
        print 'gyro', gyro
        if step == 0:
            print "in step 0"
            goal = start_coordinates
            grid_object.set_goal(goal)
        elif step == 1:
            print "in step 1"
            goal = end_coordinates
            grid_object.set_goal(goal)
        count += 1
        print "counter", count
        grid_path = grid_object.optimum_policy()
        print grid_path
        current_position = coordinates(gyro)
        print 'while-current_position', current_position
        print 'previous movement, coord', prev_movement, prev_grid
        if current_position is None:
            current_position = [-1, -1]
            print "skipping"
            continue
        else:
            if 0 >= current_position[0] or current_position[0] >= 80 or 0 >= current_position[1] or \
                            current_position[1] >= 80:
                print "skipping"
                continue
            else:
                grid_location = localise_in_grid(current_position)
                movement = grid_path[grid_location[0]][grid_location[1]]
                if prev_movement == "U" or prev_movement == "D":
                    if prev_grid[1] != grid_location[1] or prev_grid[0] == grid_location[0]:
                        turn(0)
                        stop()
                        print "skipping"
                        continue
                elif prev_movement == "L":
                    if prev_grid[0] != grid_location[0] or prev_grid[1] - 1 != grid_location[1]:
                        turn(0)
                        print "skipping"
                        continue
                if obstruction_flag:
                    obs_dist = BrickPi.Sensor[sensor1]
                    print 'BrickPi.Sensor[sensor1], movement, gyro', obs_dist, movement, gyro
                    if obs_dist <= 15 and movement == 'L' and -100 <= gyro <= -80:
                        if grid_location[1] - 1 >= 0:
                            obstruction_crd = [grid_location[0], grid_location[1] - 1]
                            print 'obstruction_crd', obstruction_crd
                            obstruction(obstruction_crd)
                        turn(0)
                        stop()
                        obstruction_flag = False
                    elif obs_dist <= 15 and movement == 'U' and -10 <= gyro <= 10:
                        if grid_location[0] - 1 >= 0:
                            obstruction_crd = [grid_location[0] - 1, grid_location[1]]
                            obstruction(obstruction_crd)
                        obstruction_flag = False
                    elif obs_dist <= 10 and movement == 'D' and 170 <= gyro <= 190:
                        if grid_location[0] + 1 <= 3:
                            obstruction_crd = [grid_location[0] + 1, grid_location[1]]
                            obstruction(obstruction_crd)
                        obstruction_flag = False
                    grid_path = grid_object.optimum_policy()
                    movement = grid_path[grid_location[0]][grid_location[1]]
                    print '(prev_grid[0] != grid_location[0]) and (prev_grid[1] != grid_location[1])', prev_grid, grid_location
                    print "Lower movement", movement
                if movement == "*":
                    print "=====REACHED GOAL====="
                    path_pythonReply = os.path.join(
                        fs + "home" + fs + "pi" + fs + "Desktop" + fs + "testingFiles" + fs + "28.07.17" + fs + "python_reply.txt")

                    if step == 0:
                        fp = open(path_pythonReply, "w")
                        fp.write("reached start")
                        fp.close()
                        # time.sleep(10)
                        step = 1
                        BrickPiUpdateValues()
                    elif step == 1:
                        fp = open(path_pythonReply, "w")
                        fp.write("reached end")
                        fp.close()
                        step = 0
                        BrickPiUpdateValues()
                else:
                    print "---searching path to goal, movement---", movement
                    obstruction_flag = fwd(movement, current_position, grid_location)
                time.sleep(1)
