import os
import os.path
from os import listdir
from shutil import copyfile
from os.path import isfile, join
import math
from array import array
import struct
import json
import shutil

def cal_dist(v1, v2):
    return sqrt((v1[0]-v2[0])*(v1[0]-v2[0])+(v1[0]-v2[0])*(v1[0]-v2[0]))

def inter_val(v1, v2, t1, t2, t):
    return (v2-v1)*(t-t1)/(t2-t1)+v1
    
def get_image_type(img_folder):
    file_list = os.listdir(img_folder)
    for item in file_list:
        if 'chamo' in item:
            return item.split(".")[1]

def filter_imgs(workspace, frame_per_meter, err, progress):
    meta_data=[]
    out_img_count=[0]
    root=workspace
    config_file=root+"/meta.json"
    src_imgs=root+"/imgs"
    out_imgs=root+"/out_imgs"
    out_meta=root+"/frame_info.csv"
    if not os.path.exists(config_file):
        print(config_file)
        print("[filter_imgs] meta.json not exist !!!")
        err[0]="meta.json not exist"
        return False
    if (os.path.isdir(out_imgs)==False):
        os.mkdir(out_imgs)
    else:
        shutil.rmtree(out_imgs)
        os.mkdir(out_imgs)
    dirs = os.listdir( src_imgs )
    if len(dirs)<100:
        print("[filter_imgs] too few imgs !!!")
        err[0]="too few src imgs"
        return False
    f_config=open(config_file, 'r')
    meta_data = json.load(f_config)
    img_meta_list=[]
    img_name_list=[]
    img_type = get_image_type(src_imgs)
    list_files = os.listdir(src_imgs)
    total_imgs=0
    for item in list_files:
        if "chamo" in item:
            total_imgs=total_imgs+1
    for i in range(1,total_imgs+1):
        img_name = "chamo_"+'{:06d}'.format(i)+"."+img_type
        img_name_list.append(img_name)
    
    print("done check file exist")
    mis_align_rate = len(meta_data)/len(img_name_list)-1
    if abs(mis_align_rate)>0.01:
        print(abs(mis_align_rate))
        err[0]="video length and data mismatch!"
        return False
    data_time_end=meta_data[len(meta_data)-1]['time']
    data_time_diff=data_time_end - meta_data[0]['time']
    img_file_time_step=data_time_diff/len(img_name_list)
    first_meta_time=meta_data[0]['time']
    img_file_times=[]
    for i in range(0,len(img_name_list)):
        img_file_times.append(i*img_file_time_step+first_meta_time)
    speed_aligh_to_img_file=[]
    last_data_time_ind=0
    last_speed=meta_data[0]['speed']
    for i in range(len(img_file_times)):
        find_time=False
        for j in range(last_data_time_ind,len(meta_data)-1):
            if img_file_times[i]>meta_data[j]['time'] and img_file_times[i]<=meta_data[j+1]['time']:
                inter_speed = inter_val(meta_data[j]['speed'], meta_data[j+1]['speed'], meta_data[j]['time'], meta_data[j+1]['time'], img_file_times[i])
                speed_aligh_to_img_file.append(inter_speed)
                last_speed=inter_speed
                last_data_time_ind=j
                find_time=True
                break
        if find_time==False:
            speed_aligh_to_img_file.append(last_speed)
            print(str(i)+" : "+str(len(img_file_times)))
    if len(speed_aligh_to_img_file)==total_imgs-1:
        speed_aligh_to_img_file.append(last_speed)
    elif len(speed_aligh_to_img_file)==total_imgs:
        print("ok")
    else:
        print("error")
        err[0]="images file number not equal meta data"
        return False
    
    temp_cul_dist=0
    last_img_time=0
    out_img_count=0
    for i in range(len(img_name_list)):
        if i>=len(meta_data):
            break;
        file_name=img_name_list[i]
        out_file_name=file_name
        if last_img_time==0:
            last_img_time=meta_data[i]['time']
            shutil.copyfile(src_imgs+"/"+file_name, out_imgs+"/"+out_file_name)
            continue
        time_diff=meta_data[i]['time']-last_img_time
        last_img_time=meta_data[i]['time']
        temp_cul_dist=temp_cul_dist+meta_data[i]['speed']*time_diff
        #if meta_data[i]['speed']*time_diff>0.1:
        #print(str(temp_cul_dist)+" : "+file_name)
        if temp_cul_dist>frame_per_meter:
            temp_cul_dist=0
            out_img_count=out_img_count+1
            out_file_name = "chamo_"+'{:06d}'.format(out_img_count)+"."+img_type
            shutil.copyfile(src_imgs+"/"+file_name, out_imgs+"/"+out_file_name)
            gps=meta_data[i]['gps']
            acc=meta_data[i]['acc']
            info=[gps[0],gps[1],gps[2], acc[1]]
            img_meta_list.append(info)
    fileObject = open(out_meta, 'w')
    for item in img_meta_list:
        record_str=str(item[0])+","+str(item[1])+","+str(item[2])+","+str(item[3])+"\n";
        fileObject.write(record_str)
    return True

if __name__ == "__main__":
    err_msg=['']
    filter_imgs("/workspace/raw_files/office", 1, err_msg)
