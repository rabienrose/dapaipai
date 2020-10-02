import requests
import _thread
import os
import os.path
from flask import Flask
from flask import render_template
from flask import request, redirect, url_for
import random
import math
import re
import json
import operator
import oss2
from oss2.headers import OSS_OBJECT_TAGGING
from libs.extract_insv import extract_meta
from libs.filter_imgs import filter_imgs
from libs.chamo_common.util import project
import subprocess
import threading
import pymongo
import time
import shutil
from libs.config import get_oss_mongo
[bucket, myclient]=get_oss_mongo()
mydb = myclient["panomap"]
traj_table = mydb["trajs"]
map_table=mydb["map"]
id_table=mydb["id"]
oss_shangrila="maps/shangrila"
oss_shangrila_insv=oss_shangrila+"/insv"
oss_shangrila_raw=oss_shangrila+"/raw_data"
local_path="gps_tmp"

def set_task_status(oss_file, task, status):
    myquery = { "name": oss_file }
    newvalues = { "$set": { "task": task, "status": status} }
    traj_table.update_one(myquery, newvalues, True)
    return

def read_json_from_oss(oss_addr, pre):
    temp_file=pre+"temp_file.json"
    exist = bucket.object_exists(oss_addr)
    print(oss_addr)
    re_data=[]
    if exist:
        bucket.get_object_to_file(oss_addr, temp_file)
        f = open(temp_file, "r")
        try:
            re_data = json.load(f)
        except TypeError:
            print("Unable to deserialize the object")
        f.close()
    return re_data

err_msg=[]
progress=[0]
frame_per_meter=1
        
def get_task_list(task, status):
    task_list=[]
    for x in traj_table.find({"task":task, "status":status},{"_id":0,"name":1}):
        task_list.append(x["name"])
    return task_list

def get_new_id():
    next_id=0
    for x in id_table.find({}):
        next_id=x["next"]
    id_table.update_one({"next":next_id},{"$set":{"next":next_id+1}}, True)
    return next_id

while True:
    task_list = get_task_list("insv", 2)
    for traj_name in task_list:
        set_task_status(traj_name, "gps", 1)
        oss_mp4_addr=oss_shangrila_raw+"/"+traj_name+"/chamo.MP4"
        img_root=local_path+"/imgs"
        
        if (os.path.isdir(local_path)==False):
            os.makedirs(local_path)
        else:
            shutil.rmtree(local_path)
            os.mkdir(local_path)
        exist = bucket.object_exists(oss_mp4_addr)
        if exist:
            bucket.get_object_to_file(oss_mp4_addr, local_path+"/"+"chamo.MP4")
        else:
            set_task_status(traj_name, "gps", -3)
            continue
        os.makedirs(img_root)
        cmd_str = "ffmpeg -i "+local_path+"/"+"chamo.MP4 -qscale:v 5 "+'-vf "drawbox=x=0:y=0:w=2048:h=214:color=black@1.0:t=max, drawbox=x=0:y=704:w=2048:h=320:color=black@1.0:t=max" '+img_root+"/chamo_%06d.jpg"
        print(cmd_str)
        os.system(cmd_str)
        
        meta_data=read_json_from_oss(oss_shangrila_raw+"/"+traj_name+"/meta.json", "gps_")
        img_name_list=[]
        list_files = os.listdir(img_root)
        total_imgs=0
        for item in list_files:
            if "chamo" in item:
                total_imgs=total_imgs+1
        if total_imgs<100:
            set_task_status(traj_name, "gps", -4)
            print("no imgs")
            continue
        if abs(total_imgs-len(meta_data))>1000:
            set_task_status(traj_name, "gps", -5)
            print("gps miss")
            continue
        for i in range(1,total_imgs+1):
            img_name = "chamo_"+'{:06d}'.format(i)+".jpg"
            img_name_list.append(img_name)
        last_img_time=-1
        last_id=[]
        img_ids=[]
        temp_cul_dist=0
        out_img_count=0
        for i in range(len(img_name_list)):
            if i>=len(meta_data):
                break;
            if last_img_time==-1:
                last_img_time=meta_data[i]['time']
                out_file_name = "000000.jpg"
                local_img=img_root+"/"+img_name_list[i]
                oss_img= oss_shangrila_raw+"/"+traj_name+"/imgs/"+out_file_name
                re = bucket.put_object_from_file(oss_img, local_img)
                if re.status != 200:
                    set_task_status(traj_name, "insv", -9)
                    continue
                img_info={"gps":meta_data[i]['gps']}
                img_info["conn"]=[]
                new_id=out_file_name.split(".")[0]
                full_new_id=traj_name+"-"+new_id
                map_table.update_one({"id":full_new_id}, {"$set":img_info}, True)
                img_ids.append(new_id)
                last_id=full_new_id
                out_img_count=out_img_count+1
            else:
                time_diff=meta_data[i]['time']-last_img_time
                last_img_time=meta_data[i]['time']
                temp_cul_dist=temp_cul_dist+meta_data[i]['speed']*time_diff
                if temp_cul_dist>frame_per_meter:
                    temp_cul_dist=0
                    out_file_name = '{:06d}'.format(out_img_count)+".jpg"
                    local_img=img_root+"/"+img_name_list[i]
                    oss_img= oss_shangrila_raw+"/"+traj_name+"/imgs/"+out_file_name
                    re = bucket.put_object_from_file(oss_img, local_img)
                    if re.status != 200:
                        set_task_status(traj_name, "insv", -8)
                        continue
                    img_info={"gps":meta_data[i]['gps']}
                    new_id=out_file_name.split(".")[0]
                    full_new_id=traj_name+"-"+new_id
#                    start_time = time.time()
                    map_table.update_one({"id":full_new_id}, {"$set":img_info}, True)
#                    print("update_one %s seconds" % (time.time() - start_time))
                    img_ids.append(new_id)
                    last_id=full_new_id
                    out_img_count=out_img_count+1
        traj_table.update_one({"name":traj_name},{"$set":{"imgs":img_ids}})
        set_task_status(traj_name, "gps", 2)
    print("start wait!!!")
    time.sleep(600)
        
        
        
        
