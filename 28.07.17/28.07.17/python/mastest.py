import os
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

observer_flag = True
class MyHandler(FileSystemEventHandler):
    def on_modified(self, event):
        print event.src_path.endswith('python_reply.txt')
        print "Got it!"
        global observer_flag
        observer_flag = False


if __name__ == "__main__":
    fs = os.sep
    event_handler = MyHandler()
    observer = Observer()
    observer.schedule(event_handler, path='C:'+fs+'Users'+fs+'halder'+fs+'Desktop'+fs+'testingFiles'+fs+'28.07.17 - Combined'+fs+'mobile', recursive=False)
    observer.start()
    while observer_flag:
        time.sleep(1)
    observer.stop()
    # try:
    #     while observer_flag:
    #         time.sleep(1)
    # except KeyboardInterrupt:
    #     observer.stop()
    # observer.join()
    print "done"
