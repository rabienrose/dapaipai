import requests
import _thread
import os
import os.path
from flask import Flask
from flask import render_template
from flask import request, redirect, url_for
from PIL import Image,ImageFont, ImageDraw
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
from libs.chamo_common.util import get_task_list
from libs.chamo_common.util import set_task_status
import subprocess
import threading
import pymongo
import time
import shutil

from libs.config import get_oss_mongo
[bucket, myclient]=get_oss_mongo()
mydb = myclient["panomap"]
task_table = mydb["task"]
map_table=mydb["map"]
oss_shangrila="maps/shangrila"
oss_shangrila_insv=oss_shangrila+"/insv"
oss_shangrila_raw=oss_shangrila+"/raw_data"
local_path="output_tmp"

err_msg=[]
progress=[0]

while True:
    for x in task_table.find({"task":"output", "status":0},{"_id":0,"name":1,"param":1}):
        if (os.path.isdir(local_path)==False):
            os.makedirs(local_path)
        else:
            shutil.rmtree(local_path)
            os.mkdir(local_path)
        img_root= local_path+"/imgs"
        os.mkdir(img_root)
        name=x["name"]
        start_id=x["param"][0]
        end_id=x["param"][1]
        traj_name=x["param"][2]
        print(traj_name, start_id, end_id)
        set_task_status(name, "output", 1, task_table)
        oss_jpg_root=oss_shangrila_raw+"/"+traj_name+"/imgs"
        local_out_mp4=local_path+"/map.mp4"
        local_out_csv=local_path+"/chamo.csv"
        local_out_png=local_path+"/chamo.png"
        tmp_count=0
        traj_step=int((end_id-start_id)/256)
        if traj_step==0:
            traj_step=1
        posi_img_list=[]
        min_x=-1
        min_y=-1
        max_x=-1
        max_y=-1
        for i in range(start_id, end_id+1):
            img_file_id='{:06d}'.format(i)
            img_file_name=img_file_id+".jpg"
            oss_jpg_addr=oss_jpg_root+"/"+img_file_name
            img_file_id='{:06d}'.format(tmp_count)
            img_file_name=img_file_id+".jpg"
#            bucket.get_object_to_file(oss_jpg_addr, img_root+"/"+img_file_name)
            if tmp_count%traj_step==0:
                print(traj_name+"-"+img_file_name)
                for x in map_table.find({"id":traj_name+"-"+img_file_id},{"gps":1,"_id":0}):
                    gps_posi = x["gps"]
                    if gps_posi[0]==0 and gps_posi[1]==0 and gps_posi[2]==0:
                        break
                    coordinates=(float(gps_posi[1]), float(gps_posi[0]))
                    posi_utm = project(coordinates)
                    posi_img_list.append([posi_utm, tmp_count, gps_posi[2]])
                    if min_x==-1 or min_x>posi_utm[2]:
                        min_x=posi_utm[2]
                    if min_y==-1 or min_y>posi_utm[3]:
                        min_y=posi_utm[3]
                    if max_x==-1 or max_x<posi_utm[2]:
                        max_x=posi_utm[2]
                    if max_y==-1 or max_y<posi_utm[3]:
                        max_y=posi_utm[3]
                    break
            tmp_count=tmp_count+1
        img_size=256
        width=max_x-min_x
        height=max_y-min_y
        mid_x=(max_x+min_x)/2
        mid_y=(max_y+min_y)/2
        scale=img_size/height
        x_offset=(128-(mid_x-min_x)*scale)
        y_offset=0
        if width>height:
            scale=img_size/width
            x_offset=0
            y_offset=(128-(mid_y-min_y)*scale)
        f_out_traj = open(local_path+"/chamo.csv","w")
        line_str=""
        pix_xy=[]
        for i in range(len(posi_img_list)):
            px=int((posi_img_list[i][0][2]-min_x)*scale+x_offset)
            py=int((posi_img_list[i][0][3]-min_y)*scale+y_offset)
            pix_xy.append([px,py])
            line_str=line_str+str(posi_img_list[i][1])+","
            line_str=line_str+str(px)+","+str(py)+","
            line_str=line_str+str(posi_img_list[i][2])+"\n"
        f_out_traj.write(line_str)
        f_out_traj.close()
        img = Image.new('RGB', (img_size, img_size), color = (255, 255, 255))
        draw = ImageDraw.Draw(img)
        temp_line_pts=[]
        for i in range(0,len(pix_xy)):
            temp_line_pts.append(pix_xy[i][0])
            temp_line_pts.append(pix_xy[i][1])
        line_pts = tuple(temp_line_pts)
        print(line_pts)
        draw.line(line_pts, fill=(255,0,0))
        img.save(local_path+"/chamo.png")
        cmd_str = "ffmpeg -i "+img_root+"/%06d.jpg -vcodec libx264 -crf 5 -pix_fmt yuv420p "+local_out_mp4
        print(cmd_str)
#        os.system(cmd_str)
#        if not os.path.exists(local_out_mp4):
#            set_task_status(name, "output", -1, task_table)
#            continue
        out_oss=oss_shangrila+"/final"
#        bucket.put_object_from_file(out_oss+"/"+name+"/chamo.mp4", local_out_mp4)
        bucket.put_object_from_file(out_oss+"/"+name+"/chamo.csv", local_out_csv)
        bucket.put_object_from_file(out_oss+"/"+name+"/chamo.png", local_out_png)
        set_task_status(name, "output", 2, task_table)
    break
    time.sleep(600)
        
        
        
        
