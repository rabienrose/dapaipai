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
from datetime import datetime
import matplotlib as mpl
import matplotlib.pyplot as plt
import time

from libs.chamo_common.util import project
from libs.config import get_oss_mongo
[bucket, myclient]=get_oss_mongo()
mydb = myclient["panoapp"]
map_table = mydb["map"]
oss_root="pano_maps"
ws_root="init_tmp"

def show_lines(traj, c, name, save):
    ys=[]
    for item in traj:
        ys.append(item)
    plt.plot(ys,'bo', color=c)
    if save:
        plt.savefig(name+'.png')
        plt.clf()

def show_maps(traj, name, save,c):
    plt.axis('equal')
    xs=[]
    ys=[]
    for item in traj:
        xs.append(item[0])
        ys.append(item[1])
    plt.scatter(xs,ys,color=c)
    if save:
        plt.savefig(name+'.png')
        plt.clf()

def get_dist(v1, v2):
    x = v1[0]-v2[0]
    y = v1[1]-v2[1]
    z = v1[2]-v2[2]
    return math.sqrt(x*x+y*y+z*z)

def list_to_nparray(python_list):
    temp_count=0
    pc = np.zeros((len(python_list),3), dtype=np.float32)
    for posi in python_list:
        pc[temp_count,:]=np.array(posi).reshape(1,3)
        temp_count=temp_count+1
    return pc
    
def get_and_download_a_new_task(map_name, insv_list):
    if (os.path.isdir(ws_root)==False):
        os.mkdir(ws_root)
    else:
        shutil.rmtree(ws_root)
        os.mkdir(ws_root)
    for i in range(len(insv_list)):
        insv_name=insv_list[i]
#        oss_file_addr=oss_root+"/"+map_name+"/"+insv_name+".insv"
#        exist = bucket.object_exists(oss_file_addr)
#        if exist:
#            bucket.get_object_to_file(oss_file_addr, ws_root+"/"+insv_name+".insv")
#        else:
#            return False
        oss_file_addr=oss_root+"/"+map_name+"/"+insv_name+".mp4"
        print(oss_file_addr)
        exist = bucket.object_exists(oss_file_addr)
        if exist:
            bucket.get_object_to_file(oss_file_addr, ws_root+"/"+insv_name+".mp4")
        else:
            return False
        str_cmd = "ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=p=0 "+ws_root+"/"+insv_name+".mp4"+" > "+ws_root+"/tmp.txt"
        os.system(str_cmd)
        f=open(ws_root+"/tmp.txt")
        str = f.readline()
        str_vec = str.split(",")
        w=int(str_vec[0])
        h=int(str_vec[1])
        if w/h!=2:
            return False
    return True
        
def extract_meta(insv_list):
    for insv in insv_list:
        cmd_str1 = "export LD_LIBRARY_PATH=/workspace/DataExtractor/InsMetadataSDK/lib"
        cmd_str2 = "./DataExtractor/extractor "+ws_root+"/"+insv+".insv"+" "+ws_root+" "+insv
        print(cmd_str1+" && "+cmd_str2)
        os.system(cmd_str1+" && "+cmd_str2)
        #to-do check output file
    return True
    
def get_trajectory_and_mps(insv_list):
    mps_name="mps.txt"
    frames_name="frames.txt"
    for insv in insv_list:
        img_addr=ws_root+"/"+insv
        cmd_str1 = "mkdir "+img_addr
        cmd_str2 = "ffmpeg -i "+ws_root+"/"+insv+".mp4"+" -qscale:v 3 -vf scale=2048:1024 ./"+img_addr+"/%06d.jpg"
        os.system(cmd_str1+" && "+cmd_str2)
        #to-do check jps count
        out_map_addr=ws_root+"/loc_map.bin"
        if os.path.exists(out_map_addr):
            cmd_str1="./vslam/build/run_image_localization -v ./vslam/orb_vocab/orb_vocab.dbow2 -i "+img_addr+" -c ./vslam/chamo.yaml --no_sleep -p "+out_map_addr+" --mask ./vslam/mask.png --no_sleep --mapping"
        else:
            cmd_str1="./vslam/build/run_image_slam -v ./vslam/orb_vocab/orb_vocab.dbow2 -i "+img_addr+" -c ./vslam/chamo.yaml --no_sleep -p "+out_map_addr+" --mask ./vslam/mask.png"
        os.system(cmd_str1)
        #to-do check output files
    for insv in insv_list:
        out_map_addr=ws_root+"/loc_map.bin"
        if os.path.exists(out_map_addr):
            cmd_str1="./vslam/build/run_image_localization -v ./vslam/orb_vocab/orb_vocab.dbow2 -i "+img_addr+" -c ./vslam/chamo.yaml --no_sleep -p "+out_map_addr+" --mask ./vslam/mask.png --no_sleep"
            os.system(cmd_str1)
            #to-do check output files
        else:
            return False
    mps_addr=ws_root+"/"+mps_name
    frames_addr=ws_root+"/"+frames_name
    if os.path.exists(mps_addr) and os.path.exists(frames_addr):
        out_scene_addr=ws_root+"/output"
        os.mkdir(out_scene_addr)
        shutil.copyfile(frames_addr, out_scene_addr+"/"+frames_name)
        shutil.copyfile(mps_addr, out_scene_addr+"/"+mps_name)
    else:
        return False
    return True

def takeSecond(elem):
    return elem[1]

def check_edge_dup(edges, edge):
    for e in edges:
        if e[0]==edge[0] and e[1]==edge[1] or e[0]==edge[1] and e[1]==edge[0]:
            return True
    return False

def generate_graph(insv_list, simple=False):
    out_scene_addr=ws_root+"/output"
    traj_file_addr=out_scene_addr+"/"+"frames.txt"
    graph_out=out_scene_addr+"/"+"graph.json"
    segment_out=out_scene_addr+"/"+"segments.json"
    frames_out=out_scene_addr+"/"+"frames.json"
    f=open(traj_file_addr,"r")
    line = f.readline()
    frame_posis=[]
    frame_dir=[]
    frame_ind_table=[]
    frame_scales=[]
    insv_name=insv_list[0]
    while line:
        line_vec = line.split(" ")
        posi=[float(line_vec[4]),float(line_vec[8]),float(line_vec[12])]
        frame_posis.append(posi)
        theta=math.atan2(float(line_vec[3]),float(line_vec[11]))*180/3.1415926
        frame_dir.append(theta)
        frame_scales.append(float(line_vec[13]))
        frame_ind_table.append(int(line_vec[0])+1)
        line = f.readline()
    f.close()
    frame_np = list_to_nparray(frame_posis)
    frame_tree = sn.KDTree(frame_np, leaf_size=10)
    frame_status=[1]*len(frame_posis)
    avg_scale=0
    scale_count=0
    for i in range(len(frame_posis)):
        if frame_scales[i]!=-1:
            avg_scale=avg_scale+frame_scales[i]
            scale_count=scale_count+1
    avg_scale=avg_scale/scale_count
    for i in range(len(frame_posis)):
        if frame_status[i]!=0:
            frame_status[i]=2
            np_posi=np.array([frame_posis[i]])
            inds,dists = frame_tree.query_radius(np_posi,r=avg_scale*0.01,return_distance=True)
            for ind in inds[0]:
                if frame_status[ind]==1:
                    frame_status[ind]=0
    remain_frames=[]
    remain_frameids=[]
    for i in range(len(frame_status)):
        if frame_status[i]==2:
            remain_frames.append(frame_posis[i])
            remain_frameids.append(i)
    if simple==True:
        frame_list=[]
        for i in range(len(remain_frames)):
            frame_tmp=[i,remain_frames[i],frame_dir[remain_frameids[i]]]
            frame_list.append(frame_tmp)
        f=open(frames_out, "w")
        json.dump(frame_list, f)
        f.close()
        segments=[]
        seg=[0,0,len(remain_frames)-1]
        segments.append(seg)
        f=open(segment_out, "w")
        json.dump(segments, f)
        f.close()
        
    else:
        all_pts_np = np.array(remain_frames)
        tree = sn.KDTree(all_pts_np, leaf_size=10)
        query_list_np=np.array(remain_frames)
        inds,dists = tree.query_radius(query_list_np, r=avg_scale*0.2, return_distance=True)
        nodes=[]
        edges=[]
        for i in range(0,len(inds)):
            frame_id=remain_frameids[i]
            nodes.append([remain_frames[i], frame_dir[frame_id]])
            candis = inds[i].tolist()
            dist_list = dists[i].tolist()
            for j in range(0, len(candis)):
                if i!=candis[j] and check_edge_dup(edges, [i, candis[j]])==False:
    #                print([i, candis[j], dist_list[j]])
                    edges.append([i, candis[j]])
        graph={"node":nodes, "edge": edges}
        f=open(graph_out, "w")
        json.dump(graph, f)
        f.close()
    new_imgs=ws_root+"/out_img"
    os.mkdir(new_imgs)
    frame_id=0
    for img_id in remain_frameids:
        img_filename=str(1000000+img_id+1)[1:7]+".jpg"
        img_addr=ws_root+"/"+insv_name+"/"+img_filename
        new_img_filename=str(1000000+frame_id)[1:7]+".jpg"
        frame_id = 1+frame_id
        if os.path.exists(img_addr):
            shutil.copyfile(img_addr, new_imgs+"/"+new_img_filename)
        else:
            return False
    return True

def output_2_mp4():
    #cmd_str = "ffmpeg -i init_tmp/out_img/%06d.jpg -vcodec libx265 -crf 28 -preset fast -tag:v hvc1 -pix_fmt yuv420p "+ws_root+"/output/chamo.mp4"
    cmd_str = "ffmpeg -i init_tmp/out_img/%06d.jpg -vcodec libx264 -crf 28 -preset fast -pix_fmt yuv420p "+ws_root+"/output/0.mp4"
    os.system(cmd_str)
    cmd_str = "ffmpeg -i init_tmp/out_img/%06d.jpg -vcodec libx264 -crf 28 -preset fast -pix_fmt yuv420p -vf reverse "+ws_root+"/output/0r.mp4"
    os.system(cmd_str)
    return True

def output_2_mp4():

def add_file_to_oss(local_path, oss_path, file_name, map_name):
    if not os.path.exists(local_path):
        return
    oss_file_addr=oss_path+"/"+file_name
    bucket.put_object_from_file(oss_file_addr, local_path+"/"+file_name)
    re_count=map_table.count_documents({"name":map_name,"insv.name":file_name})
    os.stat(local_path+"/"+file_name)
    package_size=str(os.stat(local_path+"/"+file_name).st_size)
    now = datetime.now()
    time_str = now.strftime("%Y%m%d")
    if re_count==0:
        map_table.update_one({"name":map_name},{"$push":{"insv":{"name":file_name,"size":package_size, "time":time_str}}})
    else:
        map_table.update_one({"name":map_name, "insv.name":file_name},{"$set":{"insv.$.time":time_str, "insv.$.size":package_size}})
    return True
    
def del_file_oss(oss_path, file_name, map_name):
    return True

def upload_mp4_and_meta(map_name):
    oss_file_addr=oss_root+"/"+map_name
    local_file_addr=ws_root+"/output"
    add_file_to_oss(local_file_addr, oss_file_addr, "chamo.mp4", map_name)
    add_file_to_oss(local_file_addr, oss_file_addr, "chamorev.mp4", map_name)
    add_file_to_oss(local_file_addr, oss_file_addr, "graph.json", map_name)
    return True

def post_process(insv_list):
    out_scene_addr=ws_root+"/output"
    mp_file_addr=out_scene_addr+"/"+"mps.txt"
    traj_file_addr=out_scene_addr+"/"+"frames.txt"
    mp_by_frame_out=out_scene_addr+"/"+"mps_seg.json"
    frames_out=out_scene_addr+"/"+"frames.json"
    graph_out=out_scene_addr+"/"+"graph.json"
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
    frame_scales=[]
    while line:
        line_vec = line.split(" ")
        posi=[float(line_vec[4]),float(line_vec[8]),float(line_vec[12])]
        frame_posis.append(posi)
        theta=math.atan2(float(line_vec[3]),float(line_vec[11]))*180/3.1415926
        frame_dir.append(theta)
        frame_scales.append(float(line_vec[13]))
        frame_ind_table.append(int(line_vec[0])+1)
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
    cul_dist=0
    last_posi=None
    vertice.append({"id":0})
    for i in range(len(frame_list)):
        cur_posi=frame_list[i][1]
        if last_posi is None:
            last_posi=cur_posi
        cul_dist= cul_dist+get_dist(last_posi, cur_posi)
        last_posi=cur_posi
        if cul_dist<frame_scales[i]*0.02:
            continue
        else:
            cul_dist=0
        if i>0:
            vex={"id":i}
            vertice.append(vex)
            conn={"v1":len(vertice)-2, "v2":len(vertice)-1}
            conns.append(conn)
    graph={"conn":conns,"node":vertice}
    f=open(graph_out, "w")
    json.dump(graph, f)
    f.close()
    os.mkdir(out_scene_addr+"/imgs")
    for insv in insv_list:
        count=0
        for node in graph["node"]:
            count=count+1
            img_id=frame_list[node["id"]][0]
            img_filename=str(1000000+count)[1:7]+".jpg"
            shutil.copyfile(ws_root+"/"+insv+"/"+img_filename, out_scene_addr+"/imgs/"+img_filename)
    return True

def download_package(map_name):
    if (os.path.isdir(ws_root)==False):
        os.mkdir(ws_root)
    else:
        shutil.rmtree(ws_root)
        os.mkdir(ws_root)
    oss_file_addr=oss_root+"/"+map_name+"/"+map_name+".zip"
    exist = bucket.object_exists(oss_file_addr)
    if exist:
        bucket.get_object_to_file(oss_file_addr, ws_root+"/"+map_name+".zip")
        cmd="unzip -q "+ws_root+"/"+map_name+".zip"+" -d "+ws_root
        os.system(cmd)
    else:
        return False
    return True

def trim_mps():
    out_scene_addr=ws_root+"/output"
    mp_by_frame_out=out_scene_addr+"/"+"mps_seg.json"

def get_sub(v1,v2):
    return [v1[0]-v2[0],v1[1]-v2[1],v1[2]-v2[2]]
    
def get_angle(v1,v2):
    v1_np=np.array(v1)
    v2_np=np.array(v2)
    l_v1_np=np.linalg.norm(v1_np)
    l_v2_np=np.linalg.norm(v2_np)
    if l_v1_np<0.1 or l_v2_np<0.1:
        return 0
    unit_vector_1=v1_np/np.linalg.norm(v1_np)
    unit_vector_2=v2_np/np.linalg.norm(v2_np)
    dot_product=np.dot(unit_vector_1,unit_vector_2)
    angle=np.arccos(dot_product)/3.1415926*360
    if angle>180:
        angle=angle-360
    return angle

def cal_scale_gravity_gps():
    plt.figure(figsize=(30,30))
    out_scene_addr=ws_root+"/output"
    frame_in=out_scene_addr+"/"+"frames.json"
    gps_in=out_scene_addr+"/"+"20200825-yue-yiheyuanB07xiequyuan_GpsData.txt"
    f=open(frame_in)
    frames = json.load(f)
    f.close()
    f=open(gps_in)
    gps_raw=[]
    line=f.readline()
    count=-1
    first_pt=None
    gps_angles=[]
    while line:
        count=count+1
        if count%10!=0:
            line=f.readline()
            continue
        str_splited = line.split(",")
        coordinates=(float(str_splited[2]),float(str_splited[1]))
        posi_utm = project(coordinates)
        posi = [posi_utm[2], posi_utm[3], float(str_splited[3])]
        if first_pt is None:
            first_pt=posi
        posi=[posi[0]-first_pt[0],posi[1]-first_pt[1],posi[2]-first_pt[2]]
        gps_raw.append(posi)
        line=f.readline()
    for i in range(2, len(gps_raw)):
        v1=get_sub(gps_raw[i-1],gps_raw[i-2])
        v2=get_sub(gps_raw[i],gps_raw[i-1])
        angle  = get_angle(v1,v2)
        gps_angles.append(angle)
    frame_posis=[]
    frame_times=[]
    for i in range(0,len(frames),30):
        frame_posis.append([frames[i][1][0],frames[i][1][2],frames[i][1][1]])
    frame_angles=[]
    for i in range(len(frame_posis)):
        v1=get_sub(frame_posis[i-1],frame_posis[i-2])
        v2=get_sub(frame_posis[i],frame_posis[i-1])
        angle=get_angle(v1,v2)
        frame_angles.append(angle)
    #show_lines(frame_angles, [0,1,0], "gps_frame", True)
    #show_lines(gps_angles, [1,0,0], "gps_frame", True)
    show_maps(gps_raw, "gps_path", True,[1,0,0])
    show_maps(frame_posis, "frame_path", True,[0,1,0])

def resample():
    sample_step=0.1
    out_scene_addr=ws_root+"/output"
    frame_in=out_scene_addr+"/"+"frames.json"
    graph_out=out_scene_addr+"/"+"graph.json"
    f=open(frame_in)
    frames = json.load(f)
    f.close()
    f=open(graph_out)
    graphs_old = json.load(f)
    f.close()
    last_pt=None
    remain_frame_id=[]
    for i in range(len(frames)):
        b_exist=False
        for node in graphs_old["node"]:
            if node["id"]==i:
                b_exist=True
                break
        if b_exist==False:
            continue
        if last_pt is None:
            last_pt=frames[i][1]
        tmp_dist = get_dist(last_pt, frames[i][1])
        if tmp_dist<sample_step:
            continue
        remain_frame_id.append(i)
        last_pt=frames[i][1]
    conns=[]
    nodes=[]
    for i in range(len(remain_frame_id)):
        nodes.append({"id":remain_frame_id[i]})
        if i>0:
            conns.append({"v1":i-1,"v2":i})
    graphs={"conn":conns,"node":nodes}
    f=open(graph_out,"w")
    json.dump(graphs, f)
    f.close()

def trim():
    new_imgs=ws_root+"/output/new_imgs"
    os.mkdir(new_imgs)
    out_scene_addr=ws_root+"/output"
    graph_in=out_scene_addr+"/"+"graph.json"
    f=open(graph_in)
    graph = json.load(f)
    f.close()
    frame_in=out_scene_addr+"/"+"frames.json"
    f=open(frame_in)
    frames = json.load(f)
    f.close()
    for v in graph["node"]:
        frame_id = v["id"]
        img_id=frames[frame_id][0]+1
        img_filename=str(1000000+img_id)[1:7]+".jpg"
        img_addr=out_scene_addr+"/imgs/"+img_filename
        if os.path.exists(img_addr):
            shutil.copyfile(img_addr, new_imgs+"/"+img_filename)
        else:
            return False
    shutil.rmtree(out_scene_addr+"/imgs")
    os.rename(new_imgs,out_scene_addr+"/imgs")

def zip_and_upload_result(map_name):
    cmd="cd "+ws_root+" && zip -r -q "+map_name+" output"
    os.system(cmd)
    package_name=map_name+".zip"
    oss_file_addr=oss_root+"/"+map_name+"/"+package_name
    bucket.put_object_from_file(oss_file_addr, ws_root+"/"+package_name)
    re_count=map_table.count_documents({"name":map_name,"insv.name":package_name})
    os.stat(ws_root+"/"+package_name)
    package_size=str(os.stat(ws_root+"/"+package_name).st_size)
    now = datetime.now()
    time_str = now.strftime("%Y%m%d")
    if re_count==0:
        map_table.update_one({"name":map_name},{"$push":{"insv":{"name":package_name,"size":package_size, "time":time_str}}})
    else:
        map_table.update_one({"name":map_name, "insv.name":package_name},{"$set":{"insv.$.time":time_str, "insv.$.size":package_size}})
    return True

def set_map_status(map_name,status):
    myquery = { "name": map_name }
    newvalues = { "$set": { "status": status} }
    map_table.update_one(myquery, newvalues, True)
    return

if __name__ == "__main__":
    while True:
        for x in map_table.find({"status":"waiting_proc"}):
            insv_list=[]
            map_name=x["name"]
            set_map_status(map_name,"processing")
            for filename in x["insv"]:
                if ".mp4" in filename["name"] and filename["name"]!="chamo.mp4":
                    insv_list.append(filename["name"].split(".mp4")[0])
            if len(insv_list)==0:
                set_map_status(map_name,"no_insv_error")
                continue
            set_map_status(map_name,"download_file")
            if not get_and_download_a_new_task(map_name, insv_list):
                set_map_status(map_name,"download_file_error")
                continue
            set_map_status(map_name,"slam")
            if not get_trajectory_and_mps(insv_list):
                set_map_status(map_name,"slam_error")
                continue
            set_map_status(map_name,"generate_graph")
            if not generate_graph(insv_list):
                set_map_status(map_name,"generate_graph_error")
                continue
            set_map_status(map_name,"output_2_mp4")
            if not output_2_mp4():
                set_map_status(map_name,"output_2_mp4_error")
                continue
            set_map_status(map_name,"upload_mp4_and_meta")
            if not upload_mp4_and_meta(map_name):
                set_map_status(map_name,"upload_mp4_and_meta_error")
                continue
            set_map_status(map_name,"done")
        time.sleep(60)
        
    
