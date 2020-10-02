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
import subprocess
import shutil
import threading
from libs.chamo_common.util import project
import sklearn.neighbors as sn
import pymongo
import numpy as np
import dijkstra
from libs.config import get_oss_mongo
[bucket, myclient]=get_oss_mongo()
mydb = myclient["panomap"]
traj_table = mydb["trajs"]
map_table=mydb["map"]
task_table = mydb["task"]

app = Flask(__name__)

def get_angle(v1,v2):
    v=[v2[0]-v1[0],v2[1]-v1[1]]
    angle = math.atan2(v[1],v[0])*180/3.1415926
    return angle

@app.route('/fetch_imgs_list', methods=['GET', 'POST'])
def fetch_imgs_list():
    lev = request.args.get('lev')
    lon = request.args.get('lon')
    lat = request.args.get('lat')
    img_gpss=[]
    for x in map_table.find({}):
        img_gpss.append([x["gps"][1], x["gps"][0], x["id"], x["conn"]])
    return json.dumps(img_gpss)
    
@app.route('/fetch_traj_list', methods=['GET', 'POST'])
def fetch_traj_list():
    traj_gpss=[]
    query_list=[]
    for x in traj_table.find({"task":"gps","status":2}):
        if len(x["imgs"])>10:
            first_img_id=x["name"]+"-"+x["imgs"][10]
            query_list.append(first_img_id)
    for y in map_table.find({"id":{"$in":query_list}}):
        if y["gps"][1]==0 and y["gps"][0]==0:
            continue
        vec_temp = y["id"].split("-")
        traj_name=vec_temp[0]+"-"+vec_temp[1]+"-"+vec_temp[2]
        traj_gpss.append([y["gps"][1], y["gps"][0], traj_name])
    return json.dumps(traj_gpss)
    
@app.route('/fetch_traj_data', methods=['GET', 'POST'])
def fetch_traj_data():
    traj_id = request.args.get('traj_id')
    traj_info=[]
    for x in traj_table.find({"name":traj_id}):
        img_ids=[]
        for i in range(0,len(x["imgs"]),10):
            img_ids.append(traj_id+"-"+x["imgs"][i])
        for y in map_table.find({"id":{"$in":img_ids}},{"_id":0,"gps":1,"id":1}):
            if y["gps"][1]==0 and y["gps"][0]==0:
                continue
            info=[y["gps"][1], y["gps"][0], y["id"]]
            traj_info.append(info)
    return json.dumps(traj_info)

def get_traj_from_id(id):
    id_vec = id.split("-")
    return [id_vec[0]+"-"+id_vec[1]+"-"+id_vec[2],id_vec[3]]

def takeSecond(elem):
    return elem[1]
    
def find_path(graph, start, end, path=[]):
    path = path + [start]
    if start == end:
        return path
    if not start in graph:
        return None
    for node in graph[start]:
        if node not in path:
            newpath = find_path(graph, node, end, path)
            if newpath: return newpath
    return None

@app.route('/get_route_by_graph', methods=['GET', 'POST'])
def get_route_by_graph():
    start_id = request.args.get('start_id')
    dst_id = request.args.get('dst_id')
    #start_id="20200815-yue-4square_slam-000859"
    #dst_id="20200815-yue-4square_slam-000001"
    [start_traj,start_img]=get_traj_from_id(start_id)
    [dst_traj,dst_img]=get_traj_from_id(dst_id)
    if start_traj!=dst_traj:
        return json.dumps([])
    temp_node_names=[]
    temp_node_gpss=[]
    temp_node_posi=[]
    count_tmp=0
    first_pt=[]
    zone=[]
    start_tmp_id=-1
    dst_tmp_id=-1
    for x in map_table.find({"id":{"$regex":"^"+dst_traj}},{"_id":0,"gps":1,"id":1}):
        temp_node_names.append(x["id"])
        temp_node_gpss.append(x["gps"])
        
        coordinates = (x["gps"][1], x["gps"][0])
        axis_utm = project(coordinates)
        if len(first_pt)==0:
            zone=[axis_utm[0],axis_utm[1]]
            first_pt=[axis_utm[2], axis_utm[3]]
            temp_node_posi.append([0,0])
            continue
        temp_node_posi.append([axis_utm[2]-first_pt[0],axis_utm[3]-first_pt[1]])
        if x["id"]==start_id:
            start_tmp_id=count_tmp
        if x["id"]==dst_id:
            dst_tmp_id=count_tmp
        count_tmp=count_tmp+1
    if start_tmp_id==-1 or dst_tmp_id==-1:
        print("arg erro!!!")
        return json.dumps([])
    all_pts_np = np.array(temp_node_posi)
    tree = sn.KDTree(all_pts_np, leaf_size=10)
    query_list_np=np.array(temp_node_posi)
    inds,dists = tree.query_radius(query_list_np, r=5.0, return_distance=True)
    graph = dijkstra.Graph()
    for i in range(0,len(inds)):
        candis = inds[i].tolist()
        dist_list = dists[i].tolist()
        center_posi=temp_node_posi[i]
        tmp_rank_list=[]
        for j in range(len(candis)):
            if dist_list[j]<0.1:
                continue
            angle_tmp=get_angle(center_posi,temp_node_posi[candis[j]])
            tmp_rank_list.append([candis[j], angle_tmp, dist_list[j]])
        tmp_rank_list.sort(key=takeSecond)
        while True:
            min_angle=-1
            min_id=-1
            for j in range(1, len(tmp_rank_list)):
                d_angle = tmp_rank_list[j][1]-tmp_rank_list[j-1][1]
                if min_angle==-1 or min_angle>d_angle:
                    min_angle=d_angle
                    min_id=j
            if min_angle<30 and min_angle>0:
                if tmp_rank_list[min_id][2] < tmp_rank_list[min_id-1][2]:
                    tmp_rank_list.remove(tmp_rank_list[min_id-1])
                else:
                    tmp_rank_list.remove(tmp_rank_list[min_id])
            else:
                break
        for j in range(0, len(tmp_rank_list)):
            graph.add_edge(i, tmp_rank_list[j][0], tmp_rank_list[j][2])
    path_obj = dijkstra.DijkstraSPF(graph, start_tmp_id)
    path_list = path_obj.get_path(dst_tmp_id)
    path_name_list=[]
    path_name_order={}
    for i in range(len(path_list)):
        full_name=start_traj+"-"+'{:06d}'.format(path_list[i])
        path_name_list.append(full_name)
        path_name_order[full_name]=i
    route_info_order=[[]]*len(path_name_list)
    for y in map_table.find({"id":{"$in":path_name_list}},{"_id":0,"gps":1,"id":1}):
        info=[y["gps"][1], y["gps"][0], y["id"]]
        route_info_order[path_name_order[y["id"]]]=info
    return json.dumps(route_info_order)
        

@app.route('/get_route', methods=['GET', 'POST'])
def get_route():
    start_id = request.args.get('start_id')
    dst_id = request.args.get('dst_id')
    [start_traj,start_img]=get_traj_from_id(start_id)
    [dst_traj,dst_img]=get_traj_from_id(dst_id)
    if start_traj!=dst_traj:
        return json.dumps([])
    s_img_id=int(start_img)
    d_img_id=int(dst_img)
    query_ids=[]
    b_reverse=1
    if s_img_id<d_img_id:
        for i in range(s_img_id, d_img_id+1):
            out_file_name = '{:06d}'.format(i)
            query_ids.append(start_traj+"-"+out_file_name)
    else:
        b_reverse=-1
        for i in range(s_img_id, d_img_id-1, -1):
            out_file_name = '{:06d}'.format(i)
            query_ids.append(start_traj+"-"+out_file_name)
    if len(query_ids)==0:
        return json.dumps([])
    route_info=[]
    for y in map_table.find({"id":{"$in":query_ids}},{"_id":0,"gps":1,"id":1}).sort([("id",b_reverse)]):
        info=[y["gps"][1], y["gps"][0], y["id"]]
        route_info.append(info)
    return json.dumps(route_info)

@app.route('/add_output_task', methods=['GET', 'POST'])
def add_output_task():
    start_id = request.args.get('start_id')
    end_id = request.args.get('end_id')
    name = request.args.get('name')
    [start_traj,start_img]=get_traj_from_id(start_id)
    [dst_traj,dst_img]=get_traj_from_id(end_id)
    if start_traj!=dst_traj:
        return json.dumps([])
    s_img_id=int(start_img)
    d_img_id=int(dst_img)
    print(s_img_id, d_img_id, start_traj)
    task_table.update_one({"name":name},{"$set":{"task":"output","status":0,"param":[s_img_id, d_img_id, start_traj]}}, True)
    return json.dumps([])

if __name__ == '__main__':
    app.config['SECRET_KEY'] = 'xxx'
    app.run('0.0.0.0', port=8000, debug=False)
