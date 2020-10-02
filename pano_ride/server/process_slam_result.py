import os
import os.path
import random
import math
import re
import json
import operator
import subprocess
import threading
import time
import shutil
from PIL import Image, ImageDraw, ImageFont
from libs.chamo_common.util import get_dist2d
import numpy as np
import sklearn.neighbors as sn

out_img_dir="filter_imgs"
raw_img_dir="imgs"
traj_name="chamo"
def read_pose():
    traj_info_file="frame.txt"
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


if (os.path.isdir(out_img_dir)==False):
    os.mkdir(out_img_dir)
else:
    shutil.rmtree(out_img_dir)
    os.mkdir(out_img_dir)

[pose_list, ids] = read_pose()
scale=1
cur_dist=0
last_pt=[]
remain_ids=[]
remain_poses=[]
remain_posis=[]
for i in range(150,len(pose_list)):
    x=pose_list[i][0,3]*scale
    y=pose_list[i][1,3]*scale
    z=pose_list[i][2,3]*scale
    if len(last_pt)==0:
        remain_ids.append(ids[i])
        remain_poses.append(pose_list[i])
        remain_posis.append([x,y,z])
        last_pt=[x,z]
    dist = get_dist2d([x,z],last_pt)
    cur_dist=cur_dist+dist
    if cur_dist>0.02:
        cur_dist=0
        remain_poses.append(pose_list[i])
        remain_ids.append(ids[i])
        remain_posis.append([x,y,z])
    last_pt=[x,z]
    
    
do_img_proc=True
if do_img_proc:
    for ii in range(0,len(remain_ids)):
        Twc=remain_poses[ii]
        x_w=Twc[2,2]
        y_w=Twc[0,2]
        theta=math.atan2(y_w,x_w)
        offset_w=int(theta/(2*3.1415926)*3840)
        img_file_name='chamo_{:06d}'.format(remain_ids[ii]+1)+".jpg"
        img = Image.open(raw_img_dir+"/"+img_file_name, 'r')
        img_w, img_h = img.size
        background = Image.new('RGB', (3840, 1920), (255, 255, 255))
        bg_w, bg_h = background.size
        offset = (offset_w, 0)
        print("1",offset)
        background.paste(img, offset)
        if offset_w>0:
            offset = (offset_w-img_w, 0)
        else:
            offset = (offset_w+img_w, 0)
        print("2",offset)
        background.paste(img, offset)
        img_file_id='{:06d}'.format(ii)
        img_file_name=img_file_id+".jpg"
        img_full_id=traj_name+"-"+img_file_id
        background.save(out_img_dir+"/"+img_file_name)

conns=[]
vertice=[]
vertice.append({"id":0,"posi":remain_posis[0]})
for i in range(1, len(remain_posis)):
    vertice.append({"id":i,"posi":remain_posis[i]})
    conn={"v1":int(i-1),"v2":int(i)}
    conns.append(conn)
all_pts_np=np.array(remain_posis)
tree = sn.KDTree(all_pts_np, leaf_size=10)
inds,dists = tree.query_radius(all_pts_np, r=0.04, return_distance=True)
for i in range(len(inds)):
    min_dist=-1
    min_ind=-1
    for j in range(1, len(inds[i])):
        if dists[i][j]<min_dist or min_dist==-1:
            if abs(inds[i][j]-i)>50:
                min_dist=dists[i][j]
                min_ind=j
    conn={"v1":int(i),"v2":int(inds[i][min_ind])}
    conns.append(conn)
f_conn=open("meta.json","w")
meta={"conn":conns, "node":vertice}
json.dump(meta, f_conn)
f_conn.close()



