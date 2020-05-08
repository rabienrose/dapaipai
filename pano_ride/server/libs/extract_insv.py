import os
import os.path
from os import listdir
from shutil import copyfile
from os.path import isfile, join
import math
from array import array
import struct
import json


def read_seg(input_file):
    data=input_file.read(4)
    if len(data)==0:
        return ""
    length = struct.unpack('>I', data)[0]
    if length>0XFFFFFF:
        return ""
    data=input_file.read(4)
    if len(data)==0:
        return ""
    code = struct.unpack('4s', data)[0].decode("utf-8")
    input_file.seek(length-8, 1)
    return True
    
def sub_save_file(input_file, start, end):
    CHUNK_SIZE = 1024*1024
    file_number = 1
    input_file.seek(start, 0)
    chunk = input_file.read(CHUNK_SIZE)
    save_file=open('other_part.bin', "wb")
    while chunk:
        save_file.write(chunk)
        chunk = input_file.read(CHUNK_SIZE)
    save_file.close()

def extract_meta(workspace,file_name, progress, err_msg):
    progress[0]=0
    input_file = open(workspace+"/"+file_name, 'rb')
    input_file.seek(-1,2)
    accs=[]
    gyros=[]
    imu_times=[]
    img_times=[]
    gpss=[]
    gps_times=[]
    gps_speeds=[]
    gps_tracks=[]
    input_file.seek(-1,1)
    file_size=input_file.tell()
    start_pos=-1
    for i in range(1000000):
        data=input_file.read(2)
        tag = struct.unpack('<H', data)[0]
        if tag==0x0e0a:
            input_file.seek(-2,1)
            start_pos=input_file.tell()
            break
        input_file.seek(-3,1)
    if start_pos==-1 or file_size-start_pos>10000:
        err_msg[0]="insv parse error"
        return False
    progress[0]=10
    for i in range(10):
        input_file.seek(-6,1)
        data=input_file.read(2)
        tag = struct.unpack('<H', data)[0]
        data=input_file.read(4)
        length = struct.unpack('<I', data)[0]
#        print(hex(input_file.tell()))
#        print(hex(tag))
        if tag==0x200:
            input_file.seek(-6,1)
            input_file.seek(-length,1)
#            print("sss "+hex(input_file.tell()))
            continue
        if tag==0x300:
            input_file.seek(-6,1)
            input_file.seek(-length,1)
            imu_count=length/56
            print("imu count: "+str(imu_count))
            if float(round(imu_count)) != imu_count:
                print("imu data err!!!!!!!")
                break
            imu_count=int(imu_count)
            for j in range(imu_count):
                data=input_file.read(8)
                timestamp = struct.unpack('<Q', data)[0]
                data=input_file.read(8)
                ax = struct.unpack('<d', data)[0]
                data=input_file.read(8)
                ay = struct.unpack('<d', data)[0]
                data=input_file.read(8)
                az = struct.unpack('<d', data)[0]
                data=input_file.read(8)
                gx = struct.unpack('<d', data)[0]
                data=input_file.read(8)
                gy = struct.unpack('<d', data)[0]
                data=input_file.read(8)
                gz = struct.unpack('<d', data)[0]
                accs.append([ax, ay, az])
                gyros.append([gx, gy, gz])
                imu_times.append(timestamp/1000.0)
            input_file.seek(-length,1)
            continue
        if tag==0x400:
            input_file.seek(-6,1)
            input_file.seek(-length,1)
            img_time_count=length/16
            print("img count: "+str(img_time_count))
            if float(round(img_time_count)) != img_time_count:
                print("img time data err!!!!!!!")
                break
            img_time_count=int(img_time_count)
            for j in range(img_time_count):
                data=input_file.read(8)
                timestamp = struct.unpack('<Q', data)[0]
                data=input_file.read(8)
                img_times.append(timestamp/1000.0)
            input_file.seek(-length,1)
            continue
        if tag==0x700:
            input_file.seek(-6,1)
            input_file.seek(-length,1)
            gps_count=length/53
            print("gps count: "+str(gps_count))
            if float(round(gps_count)) != gps_count:
                print("gps data err!!!!!!!")
                break
            gps_count=int(gps_count)
            for j in range(gps_count):
                data=input_file.read(8)
                timestamp = struct.unpack('<Q', data)[0]
                data=input_file.read(3)
                data=input_file.read(8)
                lat = struct.unpack('<d', data)[0]
                data=input_file.read(1)
                data=input_file.read(8)
                lng = struct.unpack('<d', data)[0]
                data=input_file.read(1)
                data=input_file.read(8)
                speed = struct.unpack('<d', data)[0]
                data=input_file.read(8)
                track = struct.unpack('<d', data)[0]
                data=input_file.read(8)
                alt = struct.unpack('<d', data)[0]
                gpss.append([lat, lng, alt])
                gps_times.append(timestamp)
                gps_speeds.append(speed)
                gps_tracks.append(track)
#            break;
        input_file.seek(-6,1)
        if input_file.tell()-length>0:
            input_file.seek(-length,1)
    if len(gps_times)==0 or len(imu_times)==0 or len(img_times)==0:
        err_msg[0]="insv no gps!!"
        return False
    imu_time_diff=imu_times[len(imu_times)-1] - imu_times[0]
    print("imu time elapse:"+str(imu_time_diff))
    img_time_diff=img_times[len(img_times)-1] - img_times[0]
    print("img time elapse:"+str(img_time_diff))
    gps_time_diff=gps_times[len(gps_times)-1] - gps_times[0]
    print("gps time elapse:"+str(gps_time_diff))
    if abs(gps_time_diff-img_time_diff)>5 or abs(gps_time_diff-imu_time_diff)>5:
        err_msg[0]="insv gps imu img time error"
        return False
    progress[0]=50
    last_gps_time=0
    same_gps_count=0
    start_i=0
    if len(gps_times)>0:
        time_offset = -gps_times[0]+img_times[0]
        for i in range(len(gps_times)):
            if i==0:
                last_gps_time=gps_times[0]
                continue
            if last_gps_time!=gps_times[i] or i==len(gps_times)-1:
                same_gps_count=same_gps_count+1
                step=1.0/same_gps_count
                temp_count=0
                for j in range(start_i, i):
                    gps_times[j]=last_gps_time+temp_count*step+time_offset
#                    print(str(j)+" "+str(gps_times[j]))
                    temp_count=temp_count+1
                same_gps_count=0
                start_i=i
                last_gps_time=gps_times[i]
            else:
                same_gps_count=same_gps_count+1
            
    image_datas=[]
    cur_imu_pos=0
    cur_gps_pos=0
    for i in range(len(img_times)):
        img_time=img_times[i]
        image_data={}
        image_data['time']=img_time
        for j in range(cur_imu_pos, 10000000000):
            if j>=len(imu_times)-1:
                break;
            if imu_times[j]<img_time and imu_times[j+1]>=img_time:
                image_data['acc']=accs[j]
                image_data['gyro']=gyros[j]
                cur_imu_pos=j
                break;
        for j in range(cur_gps_pos, 10000000000):
            if j>=len(gps_times)-1:
                break;
            if gps_times[j]<img_time and gps_times[j+1]>=img_time:
                image_data['gps']=gpss[j]
                image_data['speed']=gps_speeds[j]
                image_data['track']=gps_tracks[j]
                cur_gps_pos=j
                break;
        if not 'acc' in image_data.keys():
            break
#        if not 'gps' in image_data:
#            break
        image_datas.append(image_data)
    progress[0]=100
    out_path = workspace+"/meta.json"
    json_data = json.dumps(image_datas)
    fileObject = open(out_path, 'w')
    fileObject.write(json_data)
    fileObject.close()
    return True

if __name__ == "__main__":
    workspace="/workspace/raw_files/office"
    file_name="chamo.insv"
    progress=[0]
    extract_meta(workspace,file_name,progress)
    
    
    
    
    
    
    
