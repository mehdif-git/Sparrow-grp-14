from MPU9250 import MPU9250
import time

mpu9250 = MPU9250()
file=open("test.csv","w")
t_end = time.ticks_ms()
t_beg = t_end

for _ in range(50):
    t_end = time.ticks_ms()
    diff = time.ticks_diff(t_end, t_beg)/1000
    ax, ay, az, pitch, roll, yaw = mpu9250.getData()
    print('\r\nRoll = %d , Pitch = %d , Yaw = %d\r\n'%(roll,pitch,yaw))
    print('\r\nAcceleration:  X = %.1f , Y = %.1f , Z = %.1f\r\n'%(ax, ay, az))
    file.write(str(diff) + "," + str(ax) + "," + str(ay) + "," + str(az) + "," + str(pitch) + "," + str(roll) + "," + str(yaw) + "\n")
    time.sleep(0.2)

file.close()