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
from libs.chamo_common.util import project
import subprocess
import threading
import pymongo
import time
import shutil
from libs.config import get_oss_mongo
[bucket, myclient]=get_oss_mongo()
mydb = myclient["panomap"]
traj_table = mydb["trajs"]
map_table=mydb["map"]
id_table=mydb["id"]
test_table=mydb["test"]
oss_shangrila="maps/shangrila"
oss_shangrila_insv=oss_shangrila+"/insv"
oss_shangrila_raw=oss_shangrila+"/raw_data"
local_path="gps_tmp"

#modfiy_list=[]
#modfiy_list.append("20200811-yue-003")
#modfiy_list.append("20200811-yue-004")
#modfiy_list.append("20200811-yue-006")
#modfiy_list.append("20200811-yue-007")
#modfiy_list.append("20200811-yue-008")
#modfiy_list.append("20200811-yue-009")
#modfiy_list.append("20200810-yue-shangrila6")
#modfiy_list.append("20200810-yue-shangrila7")
#modfiy_list.append("20200810-yue-shangrila8")
#modfiy_list.append("20200810-yue-shangrila9")
#modfiy_list.append("20200810-yue-shangrila10")
#modfiy_list.append("20200810-yue-shangrila11")

#def del_folder(oss_folder):
#    for obj in oss2.ObjectIterator(bucket, prefix=oss_folder):
#        bucket.delete_object(obj.key)
#for item in modfiy_list:
#    print(item)
#    del_folder("maps/shangrila/raw_data/"+item+"/imgs/")

#for item in modfiy_list:
#    traj_table.update_one({"name":item},{"$set":{"task":"insv","status":2}})

for i in range(100):
    start_time = time.time()
    test_table.update_one({"id":"20200810-yue-shangrila10-000001"},{"$set":{"test":1}},True)
    print("update_one %s seconds" % (time.time() - start_time))
        
        
        
