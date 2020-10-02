import os
import os.path
import shutil
from libs.extract_insv import extract_meta
import ffmpeg
import json
import os
from os import listdir
img_type="png"
def extract_cam(cam_topic):
    crop_offset=0
    if cam_topic=="cam1":
        crop_offset=1920
    src_img_dir=workspace+"/"+cam_topic
    if (os.path.isdir(src_img_dir)==False):
        os.makedirs(src_img_dir)
    else:
        shutil.rmtree(src_img_dir)
        os.mkdir(src_img_dir)
    (
        ffmpeg
        .input(insv_name_addr)
        .filter("crop", 1920,1920,crop_offset,0)
        .filter("scale", 1024,1024)
        .output(src_img_dir+"/chamo_%06d."+img_type, **{'qscale:v': 5})
        .run()
    )
extract_cam1=True

workspace="/workspace/ws"
meta_file=workspace+"/meta.json"
insv_name="chamo.mp4"
insv_name_addr=workspace+"/"+insv_name
out_bag=workspace+"/chamo.bag"
dst_dir=workspace+"/bag_src"
out_timestamps=dst_dir+"/timestamp.txt"
step=1
err_msg=[]
progress=[0]

extract_meta(workspace,insv_name, progress, err_msg)
extract_cam("cam0")
if extract_cam1:
    extract_cam("cam1")

if (os.path.isdir(dst_dir)==False):
    os.makedirs(dst_dir)
else:
    shutil.rmtree(dst_dir)
    os.mkdir(dst_dir)
with open(meta_file, 'r') as f:
    meta_data = json.load(f)
    ######       imu       ########
    shutil.copyfile(workspace+"/imu0.csv", dst_dir+"/imu0.csv")
    ######        img       ########
    img_file_count=len(listdir(workspace+"/cam0"))
    tar_cam0=dst_dir+"/cam0"
    os.mkdir(tar_cam0)
    tar_cam1=dst_dir+"/cam1"
    os.mkdir(tar_cam1)
    f_time=open(out_timestamps, "w")
    for i in range(1, len(meta_data),step):
        if i>=img_file_count or i+15>=len(meta_data):
            break
        item=meta_data[i+15]
        time_stamp=item["time"]
        pad_name= str(1000000+i)
        src_img_name=workspace+"/cam0/chamo_"+pad_name[1:7]+"."+img_type
        time_stamp_str= str(int(time_stamp*1000000000+150000000000000000))
        f_time.write(time_stamp_str+"\n")
        shutil.copyfile(src_img_name, tar_cam0+"/"+time_stamp_str+"."+img_type)
        if extract_cam1:
            src_img_name=workspace+"/cam1/chamo_"+pad_name[1:7]+"."+img_type
            shutil.copyfile(src_img_name, tar_cam1+"/"+time_stamp_str+"."+img_type)
    f_time.close()
#    print("kalibr-cde/kalibr_bagcreater --folder "+dst_dir+"/. --output-bag "+out_bag)
    #os.system("kalibr-cde/kalibr_bagcreater --folder "+dst_dir+"/. --output-bag "+out_bag)
