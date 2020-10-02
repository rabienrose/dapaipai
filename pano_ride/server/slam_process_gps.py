import requests
import _thread
import os
import os.path
import random
import math
import re
import json
import operator
import oss2
from libs.chamo_common.util import project
from libs.chamo_common.util import read_json_from_oss
from libs.chamo_common.util import unproject
from libs.chamo_common.util import get_dist2d
from libs.chamo_common.util import get_sub2d
import nudged
import subprocess
import threading
import pymongo
import time
import shutil
from PIL import Image, ImageDraw, ImageFont
import numpy as np
from libs.config import get_oss_mongo
[bucket, myclient]=get_oss_mongo()
mydb = myclient["panomap"]
traj_table = mydb["trajs"]
map_table=mydb["map"]

raw_img_dir="raw"
out_img_dir="out"
oss_shangrila="maps/shangrila"
oss_shangrila_raw=oss_shangrila+"/raw_data"

def read_pose():
    traj_info_file="frame_trajectory.txt"
    f=open(traj_info_file, "r")
    line = f.readline()
    ids=[]
    poses=[]
    while line:
        line_vec = line.split(" ")
        ids.append(int(line_vec[0]))
        float_list=[]
        for i in range(1,len(line_vec)):
            float_list.append(float(line_vec[i]))
        float_list.append(0)
        float_list.append(0)
        float_list.append(0)
        float_list.append(1)
        pose = np.array(float_list).reshape(4,4)
        poses.append(pose)
        line = f.readline()
    f.close()
    return [poses, ids]

traj_name="20200815-yue-4square_slam_gps"
raw_traj_name="20200815-yue-4square"
traj_info={"task":"gps","status":2,"name":traj_name,"author":"yue","date":20200815,"imgs":[]}
meta_traj = read_json_from_oss(oss_shangrila_raw+"/"+raw_traj_name+"/meta.json", "slam_", bucket)
gps_coors=[]
offset=[]
zone=[]
[pose_list, ids] = read_pose()
slam_coors=[]
slam_coors1=[]
for i in range(len(pose_list)):
    slam_coors.append([pose_list[i][2,3],pose_list[i][0,3]])
    slam_coors1.append([pose_list[i][2,3]*10+10,pose_list[i][0,3]*10+15])
for i in range(len(ids)):
    gps_axis = meta_traj[ids[i]]['gps']
    coordinates = (gps_axis[1], gps_axis[0])
    axis_utm = project(coordinates)
    cur_posi=[axis_utm[2], axis_utm[3]]
    if len(offset)==0:
        zone=[axis_utm[0], axis_utm[1]]
        offset=[axis_utm[2], axis_utm[3]]
    gps_coors.append(get_sub2d(cur_posi, offset))
trans = nudged.estimate(slam_coors, slam_coors1)
print(trans.get_matrix())
r=trans.get_rotation()
s = trans.get_scale()
[tx, ty]=trans.get_translation()
print(r,s,tx,ty)
new_trans = nudged.Transform(s, r, tx, ty)
quit()
trans_slam_coors=[]
for i in range(len(slam_coors)):
    tmp_posi = new_trans.transform(slam_coors[i])
    trans_slam_coors.append(tmp_posi)

posis=[]
scale=5
slam_axis=[pose_list[0][2,3],pose_list[0][0,3]]
cur_dist=0
last_pt=[]
remain_ids=[]
remain_poses=[]
for i in range(len(trans_slam_coors)):
    x=trans_slam_coors[i][0]+offset[0]
    y=trans_slam_coors[i][1]+offset[1]
    latlng2 = unproject(axis_utm[0], axis_utm[1], x, y)
    if len(last_pt)==0:
        remain_ids.append(ids[i])
        remain_poses.append(pose_list[i])
        posis.append(latlng2)
        last_pt=[x,y]
    dist = get_dist2d([x,y],last_pt)
    cur_dist=cur_dist+dist
    if cur_dist>1:
        cur_dist=0
        posis.append(latlng2)
        remain_poses.append(pose_list[i])
        remain_ids.append(ids[i])
    last_pt=[x,y]
print(remain_ids)
for ii in range(0,len(remain_ids)):
#    Twc=remain_poses[ii]
#    x_w=Twc[2,2]
#    y_w=Twc[0,2]
#    theta=math.atan2(y_w,x_w)
#    offset_w=int(theta/(2*3.1415926)*2048)
#    print(offset_w)
#    img_file_name='chamo_{:06d}'.format(remain_ids[ii]+1)+".jpg"
#    img = Image.open(raw_img_dir+"/"+img_file_name, 'r')
#    img_w, img_h = img.size
#    background = Image.new('RGB', (2048, 1024), (255, 255, 255))
#    bg_w, bg_h = background.size
#    offset = (offset_w, 0)
#    background.paste(img, offset)
#    if offset_w-img_w<-offset_w:
#        offset = (offset_w+img_w, 0)
#    else:
#        offset = (offset_w-img_w, 0)
#    background.paste(img, offset)
    img_file_id='{:06d}'.format(ii)
    img_file_name=img_file_id+".jpg"
    img_full_id=traj_name+"-"+img_file_id
#    background.save(out_img_dir+"/"+img_file_name)
    traj_info["imgs"].append(img_file_id)
    img_info={"id":img_full_id,"gps":[posis[ii][1],posis[ii][0]]}
    map_table.update_one({"id":img_full_id},{"$set":img_info},True)
#    local_file=out_img_dir+"/"+img_file_name
#    oss_file= oss_shangrila_raw+"/"+traj_name+"/imgs/"+img_file_name
#    re = bucket.put_object_from_file(oss_file, local_file)
print(traj_name)
traj_table.update_one({"name":traj_name},{"$set":traj_info},True)


