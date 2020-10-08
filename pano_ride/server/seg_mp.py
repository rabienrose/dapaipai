import os
import os.path
from os import listdir
from shutil import copyfile
from os.path import isfile, join
import sklearn.neighbors as sn
import numpy as np
import math
from array import array
import struct
import json
import shutil

def list_to_nparray(python_list):
    temp_count=0
    pc = np.zeros((len(python_list),3), dtype=np.float32)
    for posi in python_list:
        pc[temp_count,:]=np.array(posi).reshape(1,3)
        temp_count=temp_count+1
    return pc

if __name__ == "__main__":
    ws_root="new_ws"
    mp_file_addr=ws_root+"/"+"mps.txt"
    traj_file_addr=ws_root+"/"+"frame_trajectory.txt"
    mp_by_frame_out=ws_root+"/"+"mps_seg.json"
    frames_out=ws_root+"/"+"frames.json"
    graph_out=ws_root+"/"+"graph.json"
    f=open(mp_file_addr,"r")
    line = f.readline()
    mps=[]
    while line:
        line_vec = line.split(" ")
        posi=[float(line_vec[0]),float(line_vec[1]),float(line_vec[2])]
        mps.append(posi)
        line = f.readline()
    f.close()
    f=open(traj_file_addr,"r")
    line = f.readline()
    frame_posis=[]
    frame_dir=[]
    frame_ind_table=[]
    while line:
        line_vec = line.split(" ")
        posi=[float(line_vec[4]),float(line_vec[8]),float(line_vec[12])]
        frame_posis.append(posi)
        theta=math.atan2(float(line_vec[3]),float(line_vec[11]))*2*3.1415926
        frame_dir.append(theta)
        frame_ind_table.append(int(line_vec[0]))
        line = f.readline()
    f.close()
    frame_np = list_to_nparray(frame_posis)
    mp_np = list_to_nparray(mps)
    frame_tree = sn.KDTree(frame_np, leaf_size=10)
    [distances, indices] = frame_tree.query(mp_np, k=1)
    mp_by_frame={}
    for i in range(len(indices)):
        tmp_ind=frame_ind_table[indices[i][0]]
        if tmp_ind in mp_by_frame:
            mp_by_frame[tmp_ind].append(mps[i])
        else:
            mp_by_frame[tmp_ind]=[mps[i]]
    f=open(mp_by_frame_out, "w")
    json.dump(mp_by_frame, f)
    f.close()
    frame_list=[]
    for i in range(len(frame_posis)):
        frame_tmp=[frame_ind_table[i],frame_posis[i],frame_dir[i]]
        frame_list.append(frame_tmp)
    f=open(frames_out, "w")
    json.dump(frame_list, f)
    f.close()
    conns=[]
    vertice=[]
    for i in range(len(frame_list)):
        vex={"id":i}
        vertice.append(vex)
        if i>0:
            conn={"v1":i-1, "v2":i}
            conns.append(conn)
    graph={"conn":conns,"node":vertice}
    f=open(graph_out, "w")
    json.dump(graph, f)
    f.close()
