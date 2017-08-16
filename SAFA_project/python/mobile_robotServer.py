from BrickPi import *
from basicRobot_planner import PathPlanner
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import os
import time
import urllib2
import json

BrickPiSetup()  # setup the serial port for communication

# set motor PORT's
motor1 = PORT_A
motor2 = PORT_B
motor3 = PORT_C
BrickPi.MotorEnable[motor1] = 1  # Enable the Motor A
BrickPi.MotorEnable[motor2] = 1  # Enable the Motor B
BrickPi.MotorEnable[motor3] = 1  # Enable the Motor C

# set sensor PORT's
sensor1 = PORT_1
sensor2 = PORT_2
BrickPi.SensorType[sensor1] = TYPE_SENSOR_ULTRASONIC_CONT
BrickPi.SensorType[sensor2] = TYPE_SENSOR_ULTRASONIC_CONT

BrickPiSetupSensors()  # Send the properties of sensors to BrickPi

# reset the motor Encoder values
BrickPi.EncoderOffset[motor1] = BrickPi.Encoder[motor1]
BrickPi.EncoderOffset[motor2] = BrickPi.Encoder[motor2]
BrickPi.EncoderOffset[motor3] = BrickPi.Encoder[motor3]
BrickPiUpdateValues()

ini_coordinates = [0, 0]
start_coordinates = [0, 0]
end_coordinates = [0, 0]
step = 0
decision = ""
current_position = [0, 0]
grid_object = PathPlanner()
count = 0
obstruction_flag = False
fs = os.sep
prev_grid = [0, 0]
prev_movement = ''
gyration_angle = 0
movement_step = 650
rotation_step = 450
observer_flag = True


# update the grid-map with the new obstruction coordinate
def obstruction(obst_crd):
    BrickPiUpdateValues()
    grid_object.set_grid(obst_crd)
    print grid_object.optimum_policy()
    try:
        req = urllib2.Request("http://192.168.1.34:8000/path")
        req.add_header('Content-Type', 'application/json')
        response = urllib2.urlopen(req, json.dumps(grid_object.get_grid()))
    except urllib2.URLError:
        print "path server not found"


# create an array of ten samples with the higher mounted "ultrasonic sensor(sensor1)" value and return the average of
#  the array
def coordinate_finder(ang):
    BrickPiUpdateValues()
    deg = [ang]
    port = [motor1]
    power = [100]
    motorRotateDegree(power, deg, port, 0, 0)
    coordinate_list = []
    for i in range(10):
        coordinate_list.append(int(BrickPi.Sensor[sensor1]))
    print coordinate_list
    crd = sum(coordinate_list) / len(coordinate_list)
    return crd


# return a 2D-array with different combination of observations of X-Y values, as observed by higher mounted
# "ultrasonic sensor( sensor1). the sensor is rotated by a motor in steps of "90" to look at the four sides of the
# enclosure."
def coordinates(gyr):
    BrickPiUpdateValues()
    port = [motor1]
    power = [100]
    angle = [1, 90, 180, -90]
    x_pos = 0
    x_neg = 0
    y_pos = 0
    y_neg = 0

    if gyr == 0:
        for ang in angle:
            print 'angle', ang
            if ang == angle[0]:
                x_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(1)
            elif ang == angle[1]:
                y_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(1)
            elif ang == angle[2]:
                x_pos = coordinate_finder(ang)
                stop()
                time.sleep(1)
            elif ang == angle[3]:
                y_pos = coordinate_finder(ang)
                stop()
                time.sleep(1)
        coordinates_array = [[x_pos, y_pos], [x_pos, y_neg], [x_neg, y_pos], [x_neg, y_neg]]
        deg = [angle[0]]
        motorRotateDegree(power, deg, port, 0, 0)
        stop()
        time.sleep(1)
        return coordinates_array
    elif gyr == -90:
        for ang in angle:
            print 'angle', ang
            if ang == angle[0]:
                y_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[1]:
                x_pos = coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[2]:
                y_pos = coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[3]:
                x_neg = 80 - coordinate_finder(ang)
        coordinates_array = [[x_pos, y_pos], [x_pos, y_neg], [x_neg, y_pos], [x_neg, y_neg]]
        deg = [angle[0]]
        motorRotateDegree(power, deg, port, 0, 0)
        stop()
        time.sleep(0.5)
        return coordinates_array
    elif gyr == 90:
        for ang in angle:
            if ang == angle[0]:
                y_pos = coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[1]:
                x_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[2]:
                y_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[3]:
                x_pos = coordinate_finder(ang)
                stop()
                time.sleep(0.5)
        coordinates_array = [[x_pos, y_pos], [x_pos, y_neg], [x_neg, y_pos], [x_neg, y_neg]]
        deg = [angle[0]]
        motorRotateDegree(power, deg, port, 0, 0)
        stop()
        time.sleep(0.5)
        return coordinates_array
    elif gyr == 180:
        for ang in angle:
            if ang == angle[0]:
                x_pos = coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[1]:
                y_pos = coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[2]:
                x_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(0.5)
            elif ang == angle[3]:
                y_neg = 80 - coordinate_finder(ang)
                stop()
                time.sleep(0.5)
        coordinates_array = [[x_pos, y_pos], [x_pos, y_neg], [x_neg, y_pos], [x_neg, y_neg]]
        deg = [angle[0]]
        motorRotateDegree(power, deg, port, 0, 0)
        stop()
        time.sleep(0.5)
        return coordinates_array


# localise the current position of the vehicle in the 4 X 4 matrix of size - 20 X 20
def localise_in_grid(coord):
    BrickPiUpdateValues()
    x = coord[0]
    y = coord[1]
    x_grid = x // 20
    y_grid = y // 20
    grid_crd = [x_grid, y_grid]
    print "--grid coord--", grid_crd
    return grid_crd


# keeps a record of the previous state (coordinates in the 4 X 4 matrix) and the previous movement of the vehicle
def set_previous_states(loc, mov):
    global prev_grid
    prev_grid = loc
    global prev_movement
    prev_movement = mov


# set the orientation of the vehicle
def set_gyro(angle):
    global gyration_angle
    gyration_angle = angle


# move the vehicle according to the encoder values provided, this function enables the vehicle for both linear
# motion and rotational motion
def move(enc1, end2):
    print "-----------------------------------------------------"
    print((BrickPi.Encoder[motor2]) / 2, (BrickPi.Encoder[motor3]) / 2)
    BrickPi.EncoderOffset[motor2] = BrickPi.Encoder[motor2]
    BrickPi.EncoderOffset[motor3] = BrickPi.Encoder[motor3]
    BrickPiUpdateValues()
    print((BrickPi.Encoder[motor2]) / 2, (BrickPi.Encoder[motor3]) / 2)
    deg = [enc1, end2]
    port = [motor2, motor3]
    power = [150, 150]
    motorRotateDegree(power, deg, port, 0, 0)
    print((BrickPi.Encoder[motor2]) / 2, (BrickPi.Encoder[motor3]) / 2)
    BrickPi.EncoderOffset[motor2] = BrickPi.Encoder[motor2]
    BrickPi.EncoderOffset[motor3] = BrickPi.Encoder[motor3]
    BrickPiUpdateValues()
    print((BrickPi.Encoder[motor2]) / 2, (BrickPi.Encoder[motor3]) / 2)
    print "------------------------------------------------------"


# initiate vehicle motion at the direction given, set grid if no obstruction is present, set obstruction flag in case
#  of obstruction, call move command with appropriate encoder values for achieving the desired motion of the vehicle.
#  the obstruction is detected by the lower "ultrasonic sensor(sensor2)". this sensor is fixed and the vehicle can
# observe obstruction in front of it. It is assumed that the obstruction is not high enough to come is sight of the
# higher ultrasonic sensor (sensor1)
def fwd(direction, c_pos, g_pos):
    curr_pos = c_pos
    grd_pos = g_pos
    if direction == 'U':
        set_gyro(0)
        if (curr_pos[0] - 6) - BrickPi.Sensor[sensor2] <= 10:
            init_obs = BrickPi.Sensor[sensor2]
            print "--this X-lane clear--EOF at", init_obs
            grid_object.reset_grid(grd_pos)
            grid_object.optimum_policy()
            if init_obs >= 15:
                move(movement_step, movement_step)
                obs_flag = True
                set_previous_states(grd_pos, direction)
                stop()
                return obs_flag
            else:
                obs_flag = True
                stop()
                return obs_flag
        else:
            init_obs = BrickPi.Sensor[sensor2]
            print "obstruction detected at X, direction", init_obs, direction
            if init_obs >= 15:
                move(movement_step, movement_step)
                obs_flag = True
                set_previous_states(grd_pos, direction)
                stop()
                return obs_flag
            else:
                obs_flag = True
                stop()
                return obs_flag
    elif direction == 'D':
        move(-2 * rotation_step, 2 * rotation_step)
        stop()
        set_gyro(180)
        if (80 - (curr_pos[0] - 6)) - BrickPi.Sensor[sensor2] <= 10:
            init_obs = BrickPi.Sensor[sensor2]
            print "--this X-lane is clear--EOF at", init_obs
            if init_obs >= 15:
                move(movement_step, movement_step)
                obs_flag = True
                set_previous_states(grd_pos, direction)
                stop()
                return obs_flag
            else:
                obs_flag = True
                stop()
                return obs_flag
        else:
            init_obs = BrickPi.Sensor[sensor2]
            print 'obstruction detected at X', init_obs
            if init_obs >= 15:
                move(movement_step, movement_step)
                obs_flag = True
                set_previous_states(grd_pos, direction)
                stop()
                return obs_flag
            else:
                obs_flag = True
                stop()
                return obs_flag
    elif direction == 'L':
        move(rotation_step, -rotation_step)
        stop()
        set_gyro(-90)
        if curr_pos[1] - BrickPi.Sensor[sensor2] <= 10:
            init_obs = BrickPi.Sensor[sensor2]
            print "--this Y-lane is clear--EOF at", init_obs
            if init_obs >= 15:
                move(movement_step, movement_step)
                obs_flag = True
                set_previous_states(grd_pos, direction)
                stop()
                return obs_flag
            else:
                obs_flag = True
                stop()
                return obs_flag
        else:
            init_obs = BrickPi.Sensor[sensor2]
            print 'obstruction detected at Y', init_obs
            if init_obs >= 15:
                move(movement_step, movement_step)
                obs_flag = True
                set_previous_states(grd_pos, direction)
                stop()
                return obs_flag
            else:
                obs_flag = True
                stop()
                return obs_flag

    elif direction == 'R':
        move(-rotation_step, rotation_step)
        stop()
        set_gyro(90)
        if (80 - curr_pos[1]) - BrickPi.Sensor[sensor2] <= 10:
            init_obs = BrickPi.Sensor[sensor2]
            print "--this Y-lane is clear--EOF at", init_obs
            if init_obs >= 15:
                move(movement_step, movement_step)
                obs_flag = True
                set_previous_states(grd_pos, direction)
                stop()
                return obs_flag
            else:
                obs_flag = True
                stop()
                return obs_flag
        else:
            init_obs = BrickPi.Sensor[sensor2]
            print 'obstruction detected at Y', init_obs
            if init_obs >= 15:
                move(movement_step, movement_step)
                obs_flag = True
                set_previous_states(grd_pos, direction)
                stop()
                return obs_flag
            else:
                obs_flag = True
                stop()
                return obs_flag


# Stop
def stop():
    BrickPi.MotorSpeed[motor1] = 0
    BrickPi.MotorSpeed[motor2] = 0
    BrickPi.MotorSpeed[motor3] = 0
    BrickPiUpdateValues()


# class for handling file read when a change in the file occurs. The class listens for files system change event.
# here the "java_reply.txt" is the file which initiates the process by providing the initial coordinates,
# 1st goal coordinates, 2nd goal coordinate
class MyHandler(FileSystemEventHandler):
    def on_modified(self, event):
        if event.src_path.endswith('java_reply.txt'):
            path_javareply = os.path.join(
                fs + "home" + fs + "pi" + fs + "Desktop" + fs + "testingFiles" + fs + "16.08.17" + fs + "java_reply.txt")
            fo = open(path_javareply, "r+")
            msg = fo.read(20)
            print "Read String is : ", msg
            msg0 = msg.split(",")[0]
            msg_split = msg0.split(' ')
            first = msg_split[0].split('-')
            second = msg_split[1].split('-')
            if first[0] == 'start' and second[0] == 'end':
                global ini_coordinates
                ini_coordinates_x = int(msg.split(",")[1][0])
                ini_coordinates_y = int(msg.split(",")[1][1])
                ini_coordinates = [ini_coordinates_x, ini_coordinates_y]
                fo = open(path_javareply, "w")
                fo.write("route taken")
                fo.close()
                x = int(first[1][0])
                y = int(first[1][1])
                global start_coordinates
                start_coordinates = [x, y]
                x = int(second[1][0])
                y = int(second[1][1])
                global end_coordinates
                end_coordinates = [x, y]
                global observer_flag
                observer_flag = False


while True:
    BrickPiUpdateValues()
    event_handler = MyHandler()
    observer = Observer()
    observer.schedule(event_handler, fs + "home" + fs + "pi" + fs + "Desktop" + fs + "testingFiles" + fs + "16.08.17",
                      recursive=False)  # wait for file change and start the loop
    observer.start()
    while observer_flag:
        BrickPiUpdateValues()
        time.sleep(1)
    observer.stop()
    BrickPiUpdateValues()
    try:
        grid_array = json.loads(
            urllib2.urlopen("http://192.168.1.34:8000/path").read())  # GET request to local PC to get the 4 X 4 matrix
        print "---grid_array--", grid_array
        grid_object.set_initial_grid(grid_array)  # initialise the grid object with the 4 X 4 matrix
    except urllib2.URLError:
        print "path server not found"
    BrickPiUpdateValues()
    while True:
        BrickPiUpdateValues()
        if step == 0:
            print "in step 0"
            goal = start_coordinates  # set goal to 1st START coordinate
            grid_object.set_goal(goal)
        elif step == 1:
            print "in step 1"
            goal = end_coordinates  # set goal to 2nd END coordinate
            grid_object.set_goal(goal)
        elif step == 2:
            print "in step 2-job complete"
            step = 0
            observer_flag = True
            break
        count += 1
        print "counter", count
        grid_path = grid_object.optimum_policy()  # get the updated 4 X 4 matrix with the  directions to reach goal
        print grid_path
        print "---gyration angle---", gyration_angle  # orientation of vehicle
        # receive the 2D array of X-Y coordinates for vehicle position
        current_position_array = coordinates(gyration_angle)
        i = 0
        for i in range(len(current_position_array)):  # go through the 2D array
            current_position = current_position_array[i]
            print 'current_position', current_position
            print 'previous movement, coord', prev_movement, prev_grid
            if current_position is None:  # None value of current is ignored and the loop is continues
                current_position = [-1, -1]
                print "skipping"
                continue
            else:
                # filtering of erroneous value of X-Y coordinates
                if 0 >= current_position[0] or current_position[0] >= 80 or 0 >= current_position[1] or \
                                current_position[1] >= 80:
                    print "skipping"
                    continue
                else:
                    grid_location = localise_in_grid(current_position)  # get the grid coordinates for the vehicle
                    movement = grid_path[grid_location[0]][grid_location[1]]  # get direction of motion
                    # since the vehicle moves in steps the grid location of the vehicle can be accurately predicted.
                    # the prediction is confirmed by the sensor observation. the erroneous values are filtered out.
                    if (prev_grid[0] == 0 and prev_grid[1] == 0) and (
                                    grid_location[0] != ini_coordinates[0] or grid_location[1] != ini_coordinates[1]):
                        print "skipping initial coordinates mismatch"
                        continue
                    elif (count != 1 and prev_movement == "U") and (
                                    prev_grid[1] != grid_location[1] or prev_grid[0] != grid_location[0] + 1):
                        print "skipping U"
                        continue
                    elif (count != 1 and prev_movement == "D") and (
                                    prev_grid[1] != grid_location[1] or prev_grid[0] != grid_location[0] - 1):
                        print "skipping D"
                        continue
                    elif (count != 1 and prev_movement == "L") and (
                                    prev_grid[0] != grid_location[0] or prev_grid[1] != grid_location[1] + 1):
                        print "skipping L"
                        continue
                    elif (count != 1 and prev_movement == "R") and (
                                    prev_grid[0] != grid_location[0] or prev_grid[1] != grid_location[1] - 1):
                        print "skipping R"
                        continue
                    else:
                        print "matched with ", i

                    # each movement of vehicle sets the obstruction flag which next again checked to confirm the
                    # obstruction and set the grid accordingly
                    if obstruction_flag:
                        obs_dist = BrickPi.Sensor[sensor2]
                        print 'obstruction, movement, gyro', obs_dist, movement, gyration_angle
                        if gyration_angle == -90:
                            if obs_dist <= 20:
                                if grid_location[1] - 1 >= 0:
                                    obstruction_crd = [grid_location[0], grid_location[1] - 1]
                                    print 'obstruction_crd', obstruction_crd
                                    if obstruction_crd != start_coordinates and obstruction_crd != end_coordinates:
                                        obstruction(obstruction_crd)
                            move(-rotation_step, rotation_step)
                            print "reseting to 0, L"
                            stop()
                            set_gyro(0)
                            obstruction_flag = False
                        elif gyration_angle == 90:
                            if obs_dist <= 20:
                                if grid_location[1] + 1 <= 3:
                                    obstruction_crd = [grid_location[0], grid_location[1] + 1]
                                    print 'obstruction_crd', obstruction_crd
                                    if obstruction_crd != start_coordinates and obstruction_crd != end_coordinates:
                                        obstruction(obstruction_crd)
                            move(rotation_step, -rotation_step)
                            print "reseting to 0, R"
                            stop()
                            set_gyro(0)
                            obstruction_flag = False
                        elif gyration_angle == 0:
                            if obs_dist <= 20:
                                if grid_location[0] - 1 >= 0:
                                    obstruction_crd = [grid_location[0] - 1, grid_location[1]]
                                    print 'obstruction_crd', obstruction_crd
                                    if obstruction_crd != start_coordinates and obstruction_crd != end_coordinates:
                                        obstruction(obstruction_crd)
                            print "reseting to 0, U"
                            set_gyro(0)
                            obstruction_flag = False
                        elif gyration_angle == 180:
                            if obs_dist <= 20:
                                if grid_location[0] + 1 <= 3:
                                    obstruction_crd = [grid_location[0] + 1, grid_location[1]]
                                    print 'obstruction_crd', obstruction_crd
                                    if obstruction_crd != start_coordinates and obstruction_crd != end_coordinates:
                                        obstruction(obstruction_crd)
                            move(2 * rotation_step, -2 * rotation_step)
                            print "reseting to 0, U"
                            stop()
                            set_gyro(0)
                            obstruction_flag = False
                        grid_path = grid_object.optimum_policy()
                        movement = grid_path[grid_location[0]][grid_location[1]]
                        print '(prev_grid[0] != grid_location[0]) and (prev_grid[1] != grid_location[1])', prev_grid, grid_location
                        print "Lower movement", movement
                    if movement == "*":
                        print "=====REACHED GOAL====="
                        # write the python_reply.txt to inform the initiator agent that the mobile agent has reached
                        # to receive the cargo
                        path_pythonReply = os.path.join(
                            fs + "home" + fs + "pi" + fs + "Desktop" + fs + "testingFiles" + fs + "16.08.17" + fs + "python_reply.txt")
                        if step == 0:
                            fp = open(path_pythonReply, "w")
                            # inform initiator agent the mobile agent has reached the 2nd END point and the 1st step
                            # is complete
                            fp.write("reached start")
                            fp.close()
                            step = 1
                            BrickPiUpdateValues()
                            ini_coordinates = grid_location
                        elif step == 1:
                            fp = open(path_pythonReply, "w")
                            # inform initiator agent the mobile agent has reached the 2nd END point and the job is
                            # complete
                            fp.write("reached end")
                            fp.close()
                            step = 2
                            BrickPiUpdateValues()
                            ini_coordinates = grid_location
                    else:
                        print "---searching path to goal, movement---", movement
                        obstruction_flag = fwd(movement, current_position, grid_location)
                    time.sleep(1)
            break
