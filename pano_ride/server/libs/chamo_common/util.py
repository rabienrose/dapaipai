import json
from pprint import pprint
import os
import os.path
from os import listdir
from shutil import copyfile
from os.path import isfile, join
import pyproj
import math
import os
import oss2
import pymongo


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

def diff_vec2(v1, v2):
    return math.sqrt((v1[0]-v2[0])*(v1[0]-v2[0])+(v1[1]-v2[1])*(v1[1]-v2[1]))

def project(coordinates):
    z = zone(coordinates)
    l = letter(coordinates)
    if z not in _projections:
        _projections[z] = pyproj.Proj(proj='utm', zone=z, ellps='WGS84')
    x, y = _projections[z](coordinates[0], coordinates[1])
    if y < 0:
        y += 10000000
    return z, l, x, y


def unproject(z, l, x, y):
    if z not in _projections:
        _projections[z] = pyproj.Proj(proj='utm', zone=z, ellps='WGS84')
    if l < 'N':
        y -= 10000000
    lng, lat = _projections[z](x, y, inverse=True)
    return (lng, lat)
        
def get_name_from_gps(gps):
    return "["+str(round(gps[0]*100000)/100000.0)+"]["+str(round(gps[1]*100000)/100000.0)+"]"

def calc_garage_center(garage_root, garage_name):
    garage_addr=garage_root+"/"+garage_name
    posi_file_addr=garage_addr+"/posi.txt"
    gps_list=[]
    for file in os.listdir(garage_addr):
        if '.json' in file:
            with open(garage_addr+"/"+file, 'r') as f:
                data = json.load(f)
                lat=float(data['Entrance_location']['GPS_Latitude'])
                lon=float(data['Entrance_location']['GPS_Longitude'])
                gps_list.append([lat, lon])
    avg_gps=[0,0]
    count=len(gps_list)
    for item in gps_list:
        avg_gps[0]=avg_gps[0]+item[0]/count
        avg_gps[1]=avg_gps[1]+item[1]/count
    if os.path.isfile(posi_file_addr):
        os.remove(posi_file_addr)
    with open(posi_file_addr, 'w') as f:
        f.write(str(avg_gps[0])+","+str(avg_gps[1]));

def get_garage_center(garage_root, garage_name):
    garage_addr=garage_root+"/"+garage_name
    posi_file_addr=garage_addr+"/posi.txt"
    with open(posi_file_addr, 'r') as f:
        gps_str = f.read();
        gps_posi_vec = gps_str.split(',')
        coordinates=(float(gps_posi_vec[1]), float(gps_posi_vec[0]))
        posi = project(coordinates)
        return [posi,coordinates]
        
def transform_pt(angle, offset, pt):
    pt_ori=[pt[0]-offset[0], pt[1]-offset[1]]
    angle=-angle
    pt_rot_ori_x=math.cos(angle)*pt_ori[0]-math.sin(angle)*pt_ori[1]
    pt_rot_ori_y=math.sin(angle)*pt_ori[0]+math.cos(angle)*pt_ori[1]
    return [pt_rot_ori_x, pt_rot_ori_y]

def get_slot_pts(center, heading):
    para_depth  =2.2/2
    vert_depth  =4.8/2
    corner = [[vert_depth,para_depth],[vert_depth,-para_depth],[-vert_depth,-para_depth],[-vert_depth,para_depth]]
    re_corners=[]
    heading=heading/180*3.14151926
    for item in corner:
        temp_x=math.cos(heading)*item[0]-math.sin(heading)*item[1]
        temp_y=math.sin(heading)*item[0]+math.cos(heading)*item[1]
        re_corners.append([temp_x+center[0], temp_y+center[1]])
    return re_corners
    
def get_city_code_name():
    city_data={}
    with open("./data/china_coordinates.csv", 'r') as f:
        content = f.readlines()
        for city_str in content:
            city_str_vec = city_str.split(',')
            if len(city_str_vec)==4:
                id=int(city_str_vec[0])
                if id%100==0 and id%10000!=0 or id==110000 or id==120000 or id==500000 or id==310000:
                    if id!=419000 and id!=429000 and id!=469000 and id!=139000:
                        coordinates=(float(city_str_vec[2]), float(city_str_vec[3]))
                        posi = project(coordinates)
                        item={"posi":posi, "name":city_str_vec[1]}
                        city_data[id]=item
    city_data[-1]={"name":"other"}
    return city_data
    

def get_rank_info(file_name, trim_op=-1):
    re=[]
    f = open(file_name,'r')
    for line in f:
        str_splited = line.split(",")
        re.append([str_splited[0],int(str_splited[1])])
        if trim_op==1:
            if len(re)>=20:
                break;
    if trim_op==2:
        trim_re=[]
        step=math.floor(len(re)/20)
        print(len(re))
        for i in range(0,len(re),step):
            trim_re.append(re[len(re)-i-1])
        re=trim_re
    f.close()
    return re
    
def read_json_from_oss(oss_addr, pre, bucket):
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

def get_dist2d(v1, v2):
    x = v1[0]-v2[0]
    y = v1[1]-v2[1]
    return math.sqrt(x*x+y*y)
    
def get_sub2d(v1, v2):
    x = v1[0]-v2[0]
    y = v1[1]-v2[1]
    return [x,y]
    
def set_task_status(oss_file, task, status, task_table):
    myquery = { "name": oss_file }
    newvalues = { "$set": { "task": task, "status": status} }
    task_table.update_one(myquery, newvalues, True)
    return

def get_task_list(task, status, task_table):
    task_list=[]
    for x in traj_table.find({"task":task, "status":status},{"_id":0,"name":1}):
        task_list.append(x["name"])
    return task_list
