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

[pose_list, ids] = read_pose()
conns=[]
vertice=[]
for i in range(0,len(pose_list)):
    x=pose_list[i][0,3]
    y=pose_list[i][1,3]
    z=pose_list[i][2,3]
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



