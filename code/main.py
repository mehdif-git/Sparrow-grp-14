import time
from MPU9250 import MPU9250

mpu = MPU9250()
p1 = 1013.25
z1 = 0
flt_time = 10000


def start_buzzer():
    pass

def accel(ax,ay,az):
    return sqrt(ax**2 + ay**2 + az**2)

def alt(press):
    return 44330.8*(1-(press/p1)**0.190289)

def alt_cst(press):
    

def dataWrite(ax,ay,az,p,y,r):
    file = open("data.csv", "w")
    t1 = time.ticks_ms()
    dt = 0
    i = 0
    alts = [0,0,0,0,0,0,0,0,0,0]
    
    while dt < flt_time:
        t2 = time.ticks_ms()
        dt = time.ticks_diff(t2,t1)
        line = str(dt) +","+ str(ax)+","+ str(ay)+","+ str(az) +","+ str(p) +","+ str(y)+","+ str(r)
        
        file.write(line)
        print(line)
        i = i+1
        time.sleep_ms(500) 

def landing():
    pass

while True:
    [ax,ay,az,p,y,r] = mpu.getData()
    accel = accel(ax,ay,az)
    if accel > 15:  
            dataWrite(ax,ay,az,p,y,r)

    
file.close()




