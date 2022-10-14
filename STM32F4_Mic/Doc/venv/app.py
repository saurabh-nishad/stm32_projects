import serial
import time
import matplotlib.pyplot as plt
start = time.time()
x = []
y = []
ser = serial.Serial('COM14', 115200, timeout=0)
time.sleep(2)
fig = plt.figure()
plt.ion()  # turn on interactive mode
plt.ylim(0, 65535)
fig.canvas.draw()
plt.show(block=False)

while True:
    line = ser.readline() # read a byte
    string = line.decode()
    num = int(string)
    if num:
         # convert the byte string to a unicode string
        #num = re.findall(r"[-+]?\d*\.\d+|\d+", string)
        
        end = time.time()
        # y.append(num)
        time_elapsed= end - start
        x.append(time_elapsed)
        plt.plot(x, num)
        plt.pause(0.05) 
        plt.draw()