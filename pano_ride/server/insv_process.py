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
oss_shangrila="maps/shangrila"
oss_shangrila_insv=oss_shangrila+"/insv"
oss_shangrila_raw=oss_shangrila+"/raw_data"
local_path="insv_tmp"

def set_task_status(oss_file, task, status):
    myquery = { "name": oss_file }
    newvalues = { "$set": { "task": task, "status": status} }
    traj_table.update_one(myquery, newvalues, True)
    return

def get_task_list(task, status):
    task_list=[]
    for x in traj_table.find({"task":task, "status":status},{"_id":0,"name":1}):
        task_list.append(x["name"])
    return task_list

err_msg=[]
progress=[0]

while True:
    pre_list = get_task_list("pre", 2)
    if len(pre_list)==0:
        for obj in oss2.ObjectIterator(bucket, prefix=oss_shangrila_insv+"/"):
            name_vec=obj.key.split("/")
            if len(name_vec)!=4 or name_vec[3]=="":
                continue
            traj_name=name_vec[-1].split(".")[0]
            pre_list.append(traj_name)
    for traj_name in pre_list:
        if (os.path.isdir(local_path)==False):
            os.makedirs(local_path)
        else:
            shutil.rmtree(local_path)
            os.mkdir(local_path)
        insv_curs = get_task_list("insv", 1)
        if traj_name in insv_curs:
            continue
        print(traj_name)
        set_task_status(traj_name, "insv", 1)
        traj_name_vec = traj_name.split("-")
        date=int(traj_name_vec[0])
        author=traj_name_vec[1]
        myquery = { "name": traj_name }
        newvalues = { "$set": { "author": author, "date": date} }
        traj_table.update_one(myquery, newvalues)
        oss_file_addr=oss_shangrila_insv+"/"+traj_name+".insv"
        exist = bucket.object_exists(oss_file_addr)
        if exist:
            re=bucket.get_object_to_file(oss_file_addr, local_path+"/"+"chamo.insv")
            if re.status != 200:
                set_task_status(traj_name, "insv", -9)
                continue
        else:
            set_task_status(traj_name, "insv", -3)
            continue
        err_msg=[""]
        re = extract_meta(local_path,"chamo.insv", progress, err_msg)
        if re==False:
            set_task_status(traj_name, "insv", err_msg[0])
            continue
        insv_file = local_path+"/"+"chamo.insv"
        mp4_file = local_path+"/"+"chamo.MP4"
        if not os.path.exists(insv_file):
            set_task_status(traj_name, "insv", -2)
            continue
        cmd_str1 = "export LD_LIBRARY_PATH=/workspace/native/lib"
        cmd_str2 = "./native/stitcherSDKDemo -inputs "+insv_file+" -output "+mp4_file+" -stitch_type optflow -hdr_type multiimagehdr_mbb -enable_flowstate"
        print(cmd_str1+" && "+cmd_str2)
        os.system(cmd_str1+" && "+cmd_str2)
        if not os.path.exists(mp4_file):
            set_task_status(traj_name, "insv", -4)
            continue
        oss_mp4_file = oss_shangrila_raw+"/"+traj_name+"/chamo.MP4"
        re = bucket.put_object_from_file(oss_mp4_file, mp4_file)
        if re.status != 200:
            set_task_status(traj_name, "insv", -5)
            continue
        oss_insv_file = oss_shangrila_raw+"/"+traj_name+"/chamo.insv"
        re = bucket.put_object_from_file(oss_insv_file, insv_file)
        if re.status != 200:
            set_task_status(traj_name, "insv", -6)
            continue
        local_image_info=local_path+"/meta.json"
        oss_image_info= oss_shangrila_raw+"/"+traj_name+"/meta.json"
        re = bucket.put_object_from_file(oss_image_info, local_image_info)
        if re.status != 200:
            set_task_status(traj_name, "insv", -7)
            continue
        local_imu_info=local_path+"/imu0.csv"
        oss_imu_info= oss_shangrila_raw+"/"+traj_name+"/imu0.csv"
        re = bucket.put_object_from_file(oss_imu_info, local_imu_info)
        if re.status != 200:
            set_task_status(traj_name, "insv", -8)
            continue
        bucket.delete_object(oss_file_addr)
        set_task_status(traj_name, "insv", 2)
    time.sleep(600)
        
        
        
        
