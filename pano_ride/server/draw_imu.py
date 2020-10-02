import os
import os.path
import shutil
import json
import os
from os import listdir
import matplotlib as mpl
import matplotlib.pyplot as plt

imu_file="/workspace/ws/imu0.csv"
f = open(imu_file, "r")
line = f.readline()
cnt = 1
acc_xs=[]
acc_ys=[]
acc_zs=[]
gyro_xs=[]
gyro_ys=[]
gyro_zs=[]
while line:
    
    if len(gyro_xs)>10050:
        break
    if cnt>1000:
        if cnt%100==0:
            data_vec = line.split(",")
            gyro_xs.append(float(data_vec[1]))
            gyro_ys.append(float(data_vec[2]))
            gyro_zs.append(float(data_vec[3]))
            acc_xs.append(float(data_vec[4]))
            acc_ys.append(float(data_vec[5]))
            acc_zs.append(float(data_vec[6]))
    line = f.readline()
    cnt += 1
    
    
#plt.axis('equal')
plt.plot(acc_xs,"r.")
plt.plot(acc_ys,"g.")
plt.plot(acc_zs,"b.")
plt.savefig('./acc_debug.png')
plt.clf()
plt.plot(gyro_xs,"r.")
plt.plot(gyro_ys,"g.")
plt.plot(gyro_zs,"b.")
plt.savefig('./gyro_debug.png')
plt.clf()
