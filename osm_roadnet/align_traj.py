from libs.extract_insv import extract_meta
import oss2
import overpass
import pprint
import requests
import os
import os.path
import random
import math
import json
import subprocess
import shutil
import pyproj
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sklearn.neighbors as sn
import pymongo
url="mongodb://root:Lance1809@dds-2ze6c59cc1a69bf41591-pub.mongodb.rds.aliyuncs.com:3717,dds-2ze6c59cc1a69bf42307-pub.mongodb.rds.aliyuncs.com:3717/admin?replicaSet=mgset-32634159"
#url="mongodb://root:Lance1809@dds-2ze6c59cc1a69bf41.mongodb.rds.aliyuncs.com:3717,dds-2ze6c59cc1a69bf42.mongodb.rds.aliyuncs.com:3717/admin?replicaSet=mgset-32634159"
myclient = pymongo.MongoClient(url)
mydb = myclient["panomap"]
node_db_list = mydb["nodes"]
access_key_id = os.getenv('OSS_TEST_ACCESS_KEY_ID', 'LTAI4GJDtEd1QXeUPZrNA4Yc')
access_key_secret = os.getenv('OSS_TEST_ACCESS_KEY_SECRET', 'rxWAZnXNhiZ8nemuvshvKxceYmUCzP')
#endpoint = os.getenv('OSS_TEST_ENDPOINT', 'oss-cn-beijing-internal.aliyuncs.com')
endpoint = os.getenv('OSS_TEST_ENDPOINT', 'oss-cn-beijing.aliyuncs.com')
oss_input="maps/shangrila"
oss_input_mid=oss_input+"/middle-files/"
local_raw_root="/workspace"
bucket_name='ride-v'
bucket = oss2.Bucket(oss2.Auth(access_key_id, access_key_secret), endpoint, bucket_name=bucket_name)
api = overpass.API(timeout=600)
pp = pprint.PrettyPrinter(indent=4)
def get_aera_traj(traj_data):
    min_lon=-1
    max_lon=-1
    min_lat=-1
    max_lat=-1
    for item in traj_data:
        if item[0]<min_lat or min_lat==-1:
            min_lat=item[0]
        if item[0]>max_lat or max_lat==-1:
            max_lat=item[0]
        if item[1]<min_lon or min_lon==-1:
            min_lon=item[1]
        if item[1]>max_lon or max_lon==-1:
            max_lon=item[1]
    re_str = str(min_lat)+","+str(min_lon)+","+str(max_lat)+","+str(max_lon)
    return re_str
    
_projections = {}
def zone(coordinates):
    if 56 <= coordinates[1] < 64 and 3 <= coordinates[0] < 12:
        return 32
    if 72 <= coordinates[1] < 84 and 0 <= coordinates[0] < 42:
        if coordinates[0] < 9:
            return 31
        elif coordinates[0] < 21:
            return 33
        elif coordinates[0] < 33:
            return 35
        return 37
    return int((coordinates[0] + 180) / 6) + 1


def letter(coordinates):
    return 'CDEFGHJKLMNPQRSTUVWXX'[int((coordinates[1] + 80) / 8)]

def project(coordinates):
    z = zone(coordinates)
    l = letter(coordinates)
    if z not in _projections:
        _projections[z] = pyproj.Proj(proj='utm', zone=z, ellps='WGS84')
    x, y = _projections[z](coordinates[0], coordinates[1])
    if y < 0:
        y += 10000000
    return z, l, x, y

def get_dist(v1, v2):
    x = v1[0]-v2[0]
    y = v1[1]-v2[1]
    z = v1[2]-v2[2]
    return math.sqrt(x*x+y*y+z*z)
    
def get_dist_2d(v1, v2):
    x = v1[0]-v2[0]
    y = v1[1]-v2[1]
    return math.sqrt(x*x+y*y)
    
def get_sub(v1, v2):
    x = v1[0]-v2[0]
    y = v1[1]-v2[1]
    z = v1[2]-v2[2]
    return [x, y, z]
    
def get_sub2d(v1, v2):
    x = v1[0]-v2[0]
    y = v1[1]-v2[1]
    return [x, y]

def unproject(z, l, x, y):
    if z not in _projections:
        _projections[z] = pyproj.Proj(proj='utm', zone=z, ellps='WGS84')
    if l < 'N':
        y -= 10000000
    lng, lat = _projections[z](x, y, inverse=True)
    return (lng, lat)
    
def download_video(file_name, bucket, in_oss):
    video_name=file_name.split(".")[0]
    type=file_name.split(".")[1]
    if os.path.isdir(local_raw_root+"/"+video_name):
        if os.path.isfile(local_raw_root+"/"+video_name+"/"+"chamo."+type):
            return True
    else:
        os.mkdir(local_raw_root+"/"+video_name)
    exist = bucket.object_exists(in_oss+file_name)
    if exist:
        bucket.get_object_to_file(in_oss+file_name, local_raw_root+"/"+video_name+"/"+"chamo."+type)
    return exist

def show_maps(traj, name, save,c,type):
    plt.axis('equal')
    xs=[]
    ys=[]
    for item in traj:
        xs.append(item[0])
        ys.append(item[1])
    plt.plot(xs, ys, c=(c[0],c[1],c[2]),linestyle=type)
    if save:
        plt.savefig("./re/"+name+'.png')
        plt.clf()

def inter_seg(pt1, pt2):
    length=get_dist_2d(pt2, pt1)
    if length<=1:
        return []
    step_count=math.floor(length)
    dir=get_sub2d(pt2, pt1)
    unity_dir=[dir[0]/length, dir[1]/length]
    out_pts=[]
    for i in range(1,step_count):
        new_pt_x=pt1[0]+unity_dir[0]*i
        new_pt_y=pt1[1]+unity_dir[1]*i
        out_pts.append([new_pt_x, new_pt_y])
    return out_pts
    
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

#video_name="dongxiaokou2"
#download_video(video_name+".insv", bucket, oss_shangrila_insv)
draw_map=False
if draw_map:
    plt.figure(figsize=(20,20))
f = open("frame_info.json", "r")
traj = json.load(f)
#
#rect_str = get_aera_traj(traj)
#print(rect_str)
##response = api.get('way('+rect_str+')[highway];(._; >;);', responseformat="json")
#response = api.get('way('+rect_str+')["highway"~"primary|secondary|tertiary"];(._; >;);', responseformat="json")
#temp_file="temp_file.json"
#f = open(temp_file, "w")
#json.dump(response, f)
#f.close()
#response = api.get('way(202255194);(._;>;)', responseformat="json")
#pp.pprint(response)

temp_file="temp_file.json"
f = open(temp_file, "r")
re_data = json.load(f)
f.close()

all_nodes={}
all_ways={}
all_way_dir={}
graph={}
for i in range(len(re_data["elements"])):
    item =re_data["elements"][i]
    type = item["type"]
    if type=="node":
        all_nodes[item["id"]]=[item["lat"],item["lon"]]
    if type=="way":
        all_ways[item["id"]]=item["nodes"]
        is_single_way=False
        if "oneway" in item:
            is_single_way=True
            all_way_dir[item["id"]]=True
        for j in range(len(item["nodes"])-1):
            node_id=item["nodes"][j]
            next_node_id=-1
            next_node_id=item["nodes"][j+1]
            if node_id in graph:
                graph[node_id].append(next_node_id)
            else:
                graph[node_id]=[next_node_id]
            if is_single_way==False:
                if next_node_id in graph:
                    graph[next_node_id].append(node_id)
                else:
                    graph[next_node_id]=[node_id]

traj_posis=[]
ori_gps=[]
for i in range(len(traj)):
    print(traj[i])
    posi = project((traj[i][1], traj[i][0]))
    if len(ori_gps)==0:
        ori_gps=[posi[2], posi[3]]
    new_posi = get_sub2d([posi[2], posi[3]], ori_gps)
    traj_posis.append(new_posi)

if draw_map:
    colors = np.random.rand(1,3)
    show_maps(traj_posis, "chamo", False,colors.tolist()[0],"solid")
all_ways_posi=[]
ways_id_list=[]
way_dir=[]
node_id_list=[]
cur_zone=[]
for i in all_ways:
    nodes=all_ways[i]
    way_posis=[]
    ways_id_list.append(i)
    node_ids=[]
    if i in all_way_dir:
        way_dir.append(True)
    else:
        way_dir.append(False)
    for j in range(len(nodes)):
        if nodes[j] in all_nodes:
            node_ids.append(nodes[j])
            item = all_nodes[nodes[j]]
            lat=item[0]
            lon=item[1]
            coordinates=(lon, lat)
            posi = project(coordinates)
            if len(cur_zone)==0:
                cur_zone=[posi[0],posi[1]]
            new_posi = get_sub2d([posi[2],posi[3]], ori_gps)
            way_posis.append(new_posi)
    node_id_list.append(node_ids)
    if draw_map:
        colors = np.random.rand(1,3)
        show_maps(way_posis, "chamo", False,colors.tolist()[0],"dotted")
    all_ways_posi.append(way_posis)
if draw_map:
    show_maps([], "chamo", True,[0,0,0],"dotted")

merge_mp_pts=[]
way_range=[]
for i in range(len(all_ways_posi)):
    for j in range(1,len(all_ways_posi[i])):
        kfs = inter_seg(all_ways_posi[i][j-1], all_ways_posi[i][j])
        merge_mp_pts.append(all_ways_posi[i][j-1])
        way_range.append([i,j,0])
        for k in range(len(kfs)):
            merge_mp_pts.append(kfs[k])
            way_range.append([i,j,k+1])
        merge_mp_pts.append(all_ways_posi[i][j])
        way_range.append([i,j,len(kfs)+1])

all_kfs_np = np.array(merge_mp_pts)
tree = sn.KDTree(all_kfs_np, leaf_size=10)
all_traj_np=np.array(traj_posis)
dist, ind = tree.query(all_traj_np, k=1)
res = ind.reshape(1, len(all_traj_np)).tolist()[0]
#start_way=way_range[res[0]][0]
#start_node=way_range[res[0]][1]
#start_nodeid = node_id_list[start_way][start_node]
#path=find_path(graph, start_nodeid, end_nodeid, path=[])
cur_max_id=0
last_node_id=-1
for i in range(len(res)):
    posi=merge_mp_pts[res[i]]
    offset_posi=[posi[0]+ori_gps[0], posi[1]+ori_gps[1]]
    lnglat = unproject(cur_zone[0], cur_zone[1], offset_posi[0], offset_posi[1])
    new_node_info={}
    new_node_info["id"]=cur_max_id
    new_node_info["posi"]=lnglat
    if last_node_id!=-1:
        new_node_info["nodes"]=[last_node_id]
    node_db_list.insert_one(new_node_info)
    last_node_id=new_node_info["id"]
    cur_max_id=cur_max_id+1
#plt.figure(figsize=(20,20))
#show_maps(temp_posi_list, "chamo", True,[0,0,0],"solid")
