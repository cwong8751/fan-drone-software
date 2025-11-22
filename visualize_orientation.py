import serial
from vpython import box, vector, rate, radians

ser = serial.Serial('COM9', 115200)  
uav = box(length=2, height=0.5, width=1, color=vector(0, 0, 1))

while True:
    line = ser.readline().decode('utf-8').strip()
    roll, pitch, yaw = map(float, line.split(',')) # 3 values separated by commas
    rate(50)
    uav.axis = vector(1, 0, 0)
    uav.up = vector(0, 0, 1)
    uav.rotate(angle=radians(roll), axis=vector(1, 0, 0))
    uav.rotate(angle=radians(pitch), axis=vector(0, 1, 0))
    uav.rotate(angle=radians(yaw), axis=vector(0, 0, 1))

