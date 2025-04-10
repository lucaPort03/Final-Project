import socket
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import threading
import signal 
import sys

UDP_IP = "0.0.0.0" 
UDP_PORT = 55151 #STM sending port 
BUFFER_SIZE = 1472 #Maximum UDP frame size

plt_mbps = []
plt_time = []

#Ctrl C handler for quitting the thread
def signal_handler(sig, frame):
    print("Program Terminated")
    stop_event.set() 
    sock.close()
    plt.close()
    sys.exit(0)
#endSignalHandler

def animate(i):
    curr_idx = []
    x_padding = 0.25
    plt.clf()
    plt.title("STM32 UDP Throughput", pad=10)
    plt.xlabel("Time (s)")
    plt.ylabel("Throughput (Mbps)")
    plt.ylim(0,100)
    plt.yticks(np.arange(0, 101, 10))

    #Dynamic x-axis sclaing 
    if (len(plt_time) > 0):
        if(len(plt_time) > 20):
            xmin = plt_time[-20]-x_padding
            xmax = plt_time[-1]+x_padding
        else:
            xmin = plt_time[0]-x_padding
            xmax = plt_time[-1]+x_padding
        #endif

        plt.xlim(xmin, xmax)
        
        for (i,t) in enumerate(plt_time): 
            if (xmin <= t <= xmax):
                curr_idx.append(i) 
            #endif
        #endfor
        
        visible_mpbs = [plt_mbps[i] for i in curr_idx] 
        avgThroughput = np.mean(visible_mpbs)

        plt.grid(alpha=0.3, linestyle = '--')
        plt.plot(plt_time, plt_mbps, label="Throughput (Mbps)",marker='o', markersize=4, linewidth=2, alpha=0.8)
        plt.axhline(avgThroughput, label="Avg: %.2f Mbps" %(avgThroughput), color='grey', linestyle='dashed', linewidth=1.5, alpha=0.4)
        
        x_max = plt.xlim()[1]  
        x_offset = x_max + 0.05

        plt.legend()
    #endif
#endAnimate

def sampleData(stop_event):
    throughput_Mbps = 0
    total_elapsed = 0.0
    byte_count = 0

    start_time = time.perf_counter() #Ensures precise timescale
    t0 = start_time
   
    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            byte_count += len(data)
            elapsed_time = time.perf_counter() - start_time 
            total_elapsed += elapsed_time
            if (elapsed_time >= 1.0): #second has elapsed
                sampledTime = time.perf_counter() - t0;    
                throughput_Mbps = byte_count*8/(elapsed_time*1_000_000) 
                print("Throughput: %d bytes (%.2f Mbps)" % (byte_count, throughput_Mbps))

                plt_mbps.append(throughput_Mbps)
                plt_time.append(sampledTime)

                byte_count = 0
                throughput_Mbps = 0
                start_time = time.perf_counter()
            #endif
        except (socket.timeout):
            continue 
        #endtry
    #endwhile
#EndSampleData

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print("Listening for UDP packets on port", UDP_PORT)

signal.signal(signal.SIGINT, signal_handler)
ani = animation.FuncAnimation(plt.gcf(), animate, interval = 500)

stop_event = threading.Event()
t1 = threading.Thread(target=sampleData, args=(stop_event,)) 
t1.daemon = True
t1.start()
plt.plot()
plt.show()

stop_event.set()
t1.join() 
sock.close()