import threading
import sys
sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/src')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')
sys.path.append('/home/apeters/project/env/src/robot-hat')

from VelocityObserver_Read import real_time_velocity_plotting
from RemoteControl import RemoteControl
from CameraView import PlainCameraView

#Observer Control 

def run_functions():
    # Create two threads for each function
    thread1 = threading.Thread(target=RemoteControl)  # For controlling the car
    thread2 = threading.Thread(target=real_time_velocity_plotting, args=(0.1203, 1.1349))  # For real-time velocity plotting
    thread3 = threading.Thread(target=PlainCameraView)  # For camera feed

    # Start all threads
    thread1.start()
    thread2.start()
    thread3.start()

    # Wait for all threads to complete
    thread1.join()
    thread2.join()
    thread3.join()

if __name__ == "__main__":
    run_functions()
    
    
    