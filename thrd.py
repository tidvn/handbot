from threading import Thread
import time
def foo():
    yield time.time(),1

class ThreadWithReturnValue(Thread):
    def rt(self):
        for num in foo():
            return num
    
twrv = ThreadWithReturnValue()
twrv.start()
while True:
    
    print(twrv.rt())
