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
import subprocess
import shutil
import threading
from libs.chamo_common.util import project


access_key_id = os.getenv('OSS_TEST_ACCESS_KEY_ID', 'LTAI4GJDtEd1QXeUPZrNA4Yc')
access_key_secret = os.getenv('OSS_TEST_ACCESS_KEY_SECRET', 'rxWAZnXNhiZ8nemuvshvKxceYmUCzP')
endpoint = os.getenv('OSS_TEST_ENDPOINT', 'oss-cn-beijing-internal.aliyuncs.com')
#endpoint = os.getenv('OSS_TEST_ENDPOINT', 'oss-cn-beijing.aliyuncs.com')
bucket_name='ride-v'
oss_root="raw_files/"
local_raw_root="raw_files"
oss_map="video_files/"
oss_shangrila="maps/shangrila"
oss_shangrila_insv=oss_shangrila+"/insv/"
oss_shangrila_mid=oss_shangrila+"/middle-files/"

process_task=""
process_file=""
action=""
progress=[0]
err_msg=[""]
bStop=False

task_list=[]

app = Flask(__name__)

def get_sys_stat():
    include_proc=['python3', 'ffmpeg', 'stitcher']
    include_disk=['osxfs', '/dev/vda1']
    output = subprocess.check_output("top -b -n 1", shell=True)
    #os.system("top -b -n 1 > chamo_out1.txt")
    output=output.decode("utf-8")
    out_lines = output.splitlines()
    cpu_proc_stat={}
    mem_proc_stat={}
    for line in out_lines:
        line_vec=line.split()
        line_vec_len=len(line_vec)
        if line_vec_len<4:
            continue
        #print(line_vec)
        for app in include_proc:
            if app in line_vec[line_vec_len-1]:
                cpu_temp=float(line_vec[line_vec_len-4])
                mem_temp=float(line_vec[line_vec_len-3])
                if app in cpu_proc_stat:
                    cpu_proc_stat[app]=cpu_proc_stat[app]+cpu_temp
                    mem_proc_stat[app]=mem_proc_stat[app]+mem_temp
                else:
                    cpu_proc_stat[app]=cpu_temp
                    mem_proc_stat[app]=mem_temp
    mem_total=0
    cpu_total=0
    for app in cpu_proc_stat:
        cpu_total=cpu_total+cpu_proc_stat[app]
    for app in cpu_proc_stat:
        mem_total=mem_total+mem_proc_stat[app]
    output = subprocess.check_output("df -h", shell=True)
    output=output.decode("utf-8")
    out_lines = output.splitlines()
    total_disk=""
    for line in out_lines:
        line_vec=line.split()
        line_vec_len=len(line_vec)
        if line_vec_len<6:
            continue
        for disk in include_disk:
            if disk==line_vec[0]:
                total_disk=line_vec[3]
                break
        
    return [mem_total, cpu_total, total_disk]
                    

def percentage(consumed_bytes, total_bytes):
    if total_bytes:
        progress[0] = int(100 * (float(consumed_bytes) / float(total_bytes)))

def convert_insv(video_name, err_msg):
    insv_file = local_raw_root+"/"+video_name+"/"+"chamo.insv"
    map_file = local_raw_root+"/"+video_name+"/"+"chamo.MP4"
    if not os.path.exists(insv_file):
        err_msg[0]="insv_file does not exist!"
        return False
    if os.path.exists(map_file):
        return True
    cmd_str1 = "export LD_LIBRARY_PATH=/workspace/native/lib"
    cmd_str2 = "./native/stitcherSDKDemo -inputs "+insv_file+" -output "+map_file+" -stitch_type optflow -hdr_type multiimagehdr_mbb -enable_flowstate > chamo_out.txt"
    print(cmd_str1+" && "+cmd_str2)
    os.system(cmd_str1+" && "+cmd_str2)
    return True
    

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
        bucket.get_object_to_file(in_oss+file_name, local_raw_root+"/"+video_name+"/"+"chamo."+type, progress_callback=percentage)
    return exist
    
def del_oss_file(file_addr, bucket):
    exist = bucket.object_exists(file_addr)
    if exist:
        bucket.delete_object(file_addr)
     
def get_new_name(name_root, bucket):
    largest_num=-1
    for obj in oss2.ObjectIterator(bucket, delimiter = '/', prefix = name_root):
        splited_name1=obj.key.split("/")
        if len(splited_name1)>1:
            splited_name2=splited_name1[len(splited_name1)-2].split("_")
            if len(splited_name2)>1:
                last_num=splited_name2[len(splited_name2)-1]
                if last_num.isnumeric():
                    if largest_num<int(last_num):
                        largest_num=int(last_num)
    return name_root+"_"+str(largest_num+1)

def cal_v_diff(v1,v2):
    return math.sqrt((v1[0]-v2[0])*(v1[0]-v2[0])+(v1[1]-v2[1])*(v1[1]-v2[1]))

def get_route_len(local_frame_file):
    f_config=open(local_frame_file, 'r')
    gps_data = json.load(f_config)
    last_pose=[]
    total_dist=0
    for item in gps_data:
        lat=float(item[0])
        lon=float(item[1])
        if lat==0 and lon==0:
            continue
        coordinates=(lon, lat)
        posi = project(coordinates)
        if len(last_pose)==0:
            last_pose=[posi[2], posi[3]]
            continue
        dist = cal_v_diff([posi[2], posi[3]],last_pose)
        last_pose=[posi[2], posi[3]]
        total_dist=total_dist+dist
    return int(total_dist)

def upload_output_oss(video_name, bucket, out_adddr, rename):
    global err_msg
    local_file=local_raw_root+"/"+video_name+"/map.mp4"
    if not os.path.exists(local_file):
        err_msg[0]="find no frame info in upload"
        return False
    local_frame_file=local_raw_root+"/"+video_name+"/frame_info.json"
    if not os.path.exists(local_frame_file):
        err_msg[0]="find no video in upload"
        return False
    route_length = get_route_len(local_frame_file)
    if route_length==0:
        err_msg[0]="extract route length wrong in upload"
        return False
    new_name=out_adddr+video_name
    if rename:
        new_name = get_new_name(out_adddr+video_name, bucket)
    print(new_name)
    oss_object=new_name+"/"
    oss_video_object=oss_object+"video.mp4"
    oss_frame_object=oss_object+"frame_info.json"
    exist = bucket.object_exists(oss_object)
    if not exist:
        bucket.put_object(oss_object, u'')
    headers = dict()
    headers[OSS_OBJECT_TAGGING] = "length="+str(route_length)
    bucket.put_object_from_file(oss_video_object, local_file, progress_callback=percentage, headers=headers)
    bucket.put_object_from_file(oss_frame_object, local_frame_file, progress_callback=percentage)

def process_raw_thread(params):
    global action
    global progress
    global process_task
    global process_file
    global bStop
    if process_task!="":
        return
    video_name=params["video_name"]
    src_oss_folder=params["src_folder"]
    dst_oss_folder=params["dst_folder"]
    clear_ws=params["clear_ws"]
    rename_b=params["rename"]
    process_task=video_name
    bucket = oss2.Bucket(oss2.Auth(access_key_id, access_key_secret), endpoint, bucket_name=bucket_name)
    re=True
    if re:
        action="download_insv"
        print(action)
        re = download_video(video_name+".insv", bucket, src_oss_folder)
    if bStop:
        bStop=False
        process_task=""
        process_file=""
        return
    if re:
        action="extract_insv"
        print(action)
        re = extract_meta(local_raw_root+"/"+video_name,"chamo.insv", progress, err_msg)
    if bStop:
        bStop=False
        process_task=""
        process_file=""
        return
    if re:
        action="convert_insv"
        print(action)
        re = convert_insv(video_name, err_msg)
        loca_mp4_file = local_raw_root+"/"+video_name+"/"+"chamo.MP4"
        oss_mp4_file = dst_oss_folder+video_name+"/"+"chamo.MP4"
        bucket.put_object_from_file(oss_mp4_file, loca_mp4_file)
    if bStop:
        bStop=False
        process_task=""
        process_file=""
        return
    video_root=local_raw_root+"/"+video_name+"/"
    if re:
        action="extract_mp4"
        print(action)
        imgs_addr=video_root+"/imgs"
        if (os.path.isdir(imgs_addr)==False):
            os.mkdir(imgs_addr)
        else:
            shutil.rmtree(imgs_addr)
            os.mkdir(imgs_addr)
        cmd_str = "ffmpeg -i "+ local_raw_root+"/"+video_name+"/"+"chamo.MP4 -qscale:v "+str(params["jpg_quality"])+" "+video_root+"/imgs/chamo_%06d."+params["extract_type"]+" 2> chamo_out.txt"
        print(cmd_str)
        os.system(cmd_str)
    if bStop:
        bStop=False
        process_task=""
        process_file=""
        return
    if re:
        action="filter_imgs"
        print(action)
        re = filter_imgs(video_root, params["frame_per_meter"],err_msg,progress )
        print(err_msg[0])
    if bStop:
        bStop=False
        process_task=""
        process_file=""
        return
    if re:
        action="gen_mp4"
        print(action)
        if os.path.exists(video_root+"map.mp4"):
            os.remove(video_root+"map.mp4")
        resolution_full=str(str(params["out_resolution"]*2))+":"+str(params["out_resolution"])
        cmd_str = "ffmpeg -r 25 -i "+video_root+"out_imgs/chamo_%06d."+params["extract_type"]+' -vf "scale='+resolution_full+',drawbox=x=0:y=0:w=960:h=100:color=black@1.0:t=max, drawbox=x=0:y=330:w=960:h=150:color=black@1.0:t=max"'+" -vcodec libx264 -crf "+str(params["out_quality"])+" -pix_fmt yuv420p "+video_root+"map.mp4 2> chamo_out.txt"
        print(cmd_str)
        os.system(cmd_str)
    if bStop:
        bStop=False
        process_task=""
        process_file=""
        return
    
    if re:
        action="upload_map"
        print(action)
        re = upload_output_oss(video_name, bucket, dst_oss_folder, rename_b)
    print("[process_raw_thread] down all process")
    if clear_ws:
        video_addr= local_raw_root+"/"+video_name
        if (os.path.isdir(video_addr)==True):
            shutil.rmtree(video_addr)
    process_task=""
    process_file=""
    return re

def del_folder(prefix, bucket):
    for obj in oss2.ObjectIterator(bucket, prefix=prefix):
        bucket.delete_object(obj.key)

def read_progress_from_log():
    f = open("chamo_out.txt", "r")
    total_s=0
    all_lines=f.readlines()
    total_lines=len(all_lines)
    if total_lines==0:
        return -1
    if action=="convert_insv":
        splited_str = all_lines[-1].split("=")
        if len(splited_str)==2:
            return int(splited_str[1])/100.0
    else:
        for x in all_lines:
            if "Duration" in x:
                splited = x.split(":")
                h=float(splited[1])
                m=float(splited[2])
                s=float(splited[3].split(",")[0])
                total_s = h*3600+m*60+s
                break;
        last_line_vec=all_lines[total_lines-2].split("=")
        if last_line_vec[0]=="frame":
            for item in last_line_vec:
                if "bitrate" in item:
                    vec_time = item.split(" ")[0].split(":")
                    h=float(vec_time[0])
                    m=float(vec_time[1])
                    s=float(vec_time[2])
                    cur_s = h*3600+m*60+s
                    return cur_s/total_s;
    return -1

@app.route('/process_all', methods=['GET', 'POST'])
def process_all():
    bucket = oss2.Bucket(oss2.Auth(access_key_id, access_key_secret), endpoint, bucket_name=bucket_name)
    for obj in oss2.ObjectIterator(bucket, prefix = oss_shangrila_insv):
        split_info=obj.key.split("/")
        if len(split_info)==4:
            split_str = split_info[-1].split(".")
            if split_str[-1]!="insv":
                continue
            video_name=split_str[0]
            exist1 = bucket.object_exists(oss_shangrila_mid+video_name+"/video.mp4")
            exist2 = bucket.object_exists(oss_shangrila_mid+video_name+"/frame_info.json")
            if exist1 and exist2:
                continue
            task_list.append({"name":video_name})
    timer = threading.Timer(1.0, fix_update)
    timer.start()
    return "ok"

@app.route('/check_process_status', methods=['GET', 'POST'])
def check_process_status():
    out={"action":action}
    rate=0
    if action=="extract_mp4" or action=="gen_mp4" or action=="convert_insv":
        rate=read_progress_from_log()*100
    else:
        rate=progress[0]
    if rate==-1:
        out['progress']="none"
    else:
        out['progress']=('% 6.2f' % rate)+"%"
    [mem, cpu, disk]=get_sys_stat()
    out['cpu']=str(cpu)+"%"
    out['mem']=str(mem)+"%"
    out['disk']=disk
    out['err']=err_msg[0]
    return json.dumps(out)

@app.route('/del_raw', methods=['GET', 'POST'])
def del_raw():
    video_name = request.args.get('name')
    bucket = oss2.Bucket(oss2.Auth(access_key_id, access_key_secret), endpoint, bucket_name=bucket_name)
    del_oss_file(oss_root+ video_name+".insv", bucket)
    del_oss_file(oss_root+ video_name+".MP4", bucket)
    return json.dumps("ok")
    
@app.route('/del_map', methods=['GET', 'POST'])
def del_map():
    map_name = request.args.get('name')
    bucket = oss2.Bucket(oss2.Auth(access_key_id, access_key_secret), endpoint, bucket_name=bucket_name)
    del_folder(oss_map+map_name+"/", bucket)
    return json.dumps("ok")
    
@app.route('/clear_ws', methods=['GET', 'POST'])
def clear_ws():
    video_name = request.args.get('name')
    video_addr= local_raw_root+"/"+video_name
    if (os.path.isdir(video_addr)==True):
        shutil.rmtree(video_addr)
    return json.dumps("ok")

@app.route('/stop_process', methods=['GET', 'POST'])
def stop_process():
    global process_task
    global bStop
    bStop=True
    process_task=""
    return json.dumps("ok")

@app.route('/process_raw', methods=['GET', 'POST'])
def process_raw():
    video_name = request.args.get('name')
    extract_type = request.args.get('extract_type')
    jpg_quality = int(request.args.get('jpg_quality'))
    frame_per_meter = float(request.args.get('frame_per_meter'))
    out_resolution = int(request.args.get('out_resolution'))
    out_quality = int(request.args.get('out_quality'))
    params={
        "video_name":video_name,
        "extract_type":extract_type,
        "jpg_quality":jpg_quality,
        "frame_per_meter":frame_per_meter,
        "out_resolution":out_resolution,
        "out_quality":out_quality,
        "src_folder":oss_root,
        "dst_folder":oss_map,
        "clear_ws":False,
        "rename":True
    }
    _thread.start_new_thread( process_raw_thread, (params,) )
    return json.dumps("ok")

def get_file_size(file_addr):
    return

def takeTime(elem):
    return elem["time"]

@app.route('/show_map_list', methods=['GET', 'POST'])
def show_map_list():
    bucket = oss2.Bucket(oss2.Auth(access_key_id, access_key_secret), endpoint, bucket_name=bucket_name)
    map_info_list=[]
    for obj in oss2.ObjectIterator(bucket, prefix = oss_map):
        split_info=obj.key.split("/")
        if len(split_info)==3:
            if split_info[2]=="video.mp4":
                video_infos={}
                video_infos["name"]=split_info[1]
                video_infos["size"]="mp4("+str(round(obj.size/1024/1024))+"MB)"
                video_infos["time"]=obj.last_modified
                result = bucket.get_object_tagging(obj.key)
                if "length" in result.tag_set.tagging_rule:
                    route_len = result.tag_set.tagging_rule["length"]
                    video_infos["length"]=route_len+"m"
                map_info_list.append(video_infos)
    map_info_list.sort(key=takeTime)
    return json.dumps(map_info_list)

@app.route('/show_auto_proc_status', methods=['GET', 'POST'])
def show_auto_proc_status():
    print(process_file)
    status_str="Waiting List: </br>"
    for item in task_list:
        status_str=status_str+ item["name"]+"</br>"
    status_str=status_str+ "</br>"
    status_str=status_str+ "Processing: "+process_file
    return status_str

@app.route('/show_raw_list', methods=['GET', 'POST'])
def show_raw_list():
    bucket = oss2.Bucket(oss2.Auth(access_key_id, access_key_secret), endpoint, bucket_name=bucket_name)
    video_infos={}
    for obj in oss2.ObjectIterator(bucket, delimiter = '/', prefix = oss_root):
        splited_key = obj.key.split(".")
        if len(splited_key)==2:
            if splited_key[len(splited_key)-1]=="insv" or splited_key[len(splited_key)-1]=="MP4":
                splited_key2 = splited_key[len(splited_key)-2].split("/")
                name = splited_key2[len(splited_key2)-1]
                if name in video_infos:
                    video_infos[name][splited_key[len(splited_key)-1]]=obj.size
                else:
                    video_infos[name]={splited_key[len(splited_key)-1]:obj.size}
                video_infos[name]['time']=obj.last_modified
    video_info_list=[]
    for key in video_infos:
        str_insv=""
        str_MP4=""
        if 'insv' in video_infos[key]:
            str_insv="insv("+str(round(round(video_infos[key]['insv']/1024/1024)/10.0)/100)+"GB)"
        str_process=""
        if "insv" in str_insv:
            str_process="process"
        if key == process_task:
            str_process="processing"
        info_t={"name": key, "insv": str_insv, "process": str_process, 'time':video_infos[key]['time']}
        video_info_list.append(info_t)
        #full_splited = obj.key.split("/")
    video_info_list.sort(key=takeTime)
    return json.dumps(video_info_list)

def fix_update():
    global task_list
    global process_file
    if process_file=="":
        if len(task_list)>0:
            task=task_list.pop()
            process_file=task["name"]
            params={
                "video_name":task["name"],
                "extract_type":"jpg",
                "jpg_quality":15,
                "frame_per_meter":1.0,
                "out_resolution":480,
                "out_quality":15,
                "src_folder":oss_shangrila_insv,
                "dst_folder":oss_shangrila_mid,
                "clear_ws":True,
                "rename":False
            }
            _thread.start_new_thread( process_raw_thread, (params,) )
    if len(task_list)>0:
        timer = threading.Timer(1.0, fix_update)
        timer.start()

if __name__ == '__main__':
    app.config['SECRET_KEY'] = 'xxx'
    if (os.path.isdir(local_raw_root)==False):
        os.mkdir(local_raw_root)
    app.run('0.0.0.0', port=8000, debug=False)
