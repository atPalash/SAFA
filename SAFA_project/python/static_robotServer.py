from BrickPi import *
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import os


class MyHandler(FileSystemEventHandler):
    def on_modified(self, event):
        if event.src_path.endswith('java_reply.txt'):
            path_javareply = os.path.join(
                fs + "home" + fs + "pi" + fs + "Desktop" + fs + "testingFiles" + fs + "09.08.17" + fs + "java_reply.txt")
            fo = open(path_javareply, "r+")
            msg = fo.read(15)
            print "Read String is : ", msg

            if msg =='reached start':
                fo = open(path_javareply, "w")
                fo.write("loaded")
                fo.close()
                global observer_flag
                observer_flag = False


BrickPiSetup()

BrickPi.MotorEnable[PORT_A] = 1

BrickPiSetupSensors()
fs = os.sep
while True:
    print "in while-static"
    observer_flag = True
    event_handler = MyHandler()
    observer = Observer()
    observer.schedule(event_handler, fs + "home" + fs + "pi" + fs + "Desktop" + fs + "testingFiles" + fs + "09.08.17",
                      recursive=False)
    observer.start()
    while observer_flag:
        time.sleep(1)
        BrickPiUpdateValues()
    observer.stop()
    print "turn motor on"
    for i in range(1000):
        BrickPiUpdateValues()
        BrickPi.MotorSpeed[PORT_A] = 250
    BrickPi.MotorSpeed[PORT_A] = 0
    BrickPiUpdateValues()
    time.sleep(0.01)
