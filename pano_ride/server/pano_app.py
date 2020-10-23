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
import datetime
from libs.config import get_oss_mongo
[bucket, myclient]=get_oss_mongo()
mydb = myclient["panoapp"]
map_table = mydb["map"]
oss_root="pano_maps"
app = Flask(__name__)
@app.route('/create_map', methods=['GET', 'POST'])
def create_map():
    map_name = request.args.get('name')
    author = request.args.get('author')
    x = datetime.datetime.now()
    time_str=str(x.year)+str(x.month)+str(x.day)
    map_table.update_one({"name":map_name},{"$set":{"time":time_str,"insv":[], "author":author, "status":"prepairing"}},True)
    return json.dumps(["ok"])

@app.route('/add_insv', methods=['GET', 'POST'])
def add_insv():
    map_name = request.args.get('map_name')
    insv = request.args.get('insv')
    insv_size = request.args.get('insv_size')
    x = datetime.datetime.now()
    time_str=str(x.year)+str(x.month)+str(x.day)
    count_tmp = map_table.count_documents({"name":map_name})
    if count_tmp==0:
        return json.dumps(["map not exist"])
    re_count=map_table.find({"name":map_name,"insv.name":insv},{"_id":0,"id":1}).count()
    if re_count==0:
        map_table.update_one({"name":map_name},{"$push":{"insv":{"name":insv,"size":insv_size, "time":time_str}}})
    else:
        map_table.update_one({"name":map_name, "insv.name":insv},{"$set":{"insv.$.time":time_str, "insv.$.size":insv_size}})
    return json.dumps(["ok"])
    
@app.route('/process_map', methods=['GET', 'POST'])
def process_map():
    map_name = request.args.get('map_name')
    map_table.update_one({"name":map_name},{"$set":{"status":"waiting_proc"}})
    return json.dumps(["ok"])
    
@app.route('/trim_map', methods=['GET', 'POST'])
def trim_map():
    map_name = request.args.get('map_name')
    map_table.update_one({"name":map_name},{"$set":{"status":"waiting_trim"}})
    return json.dumps(["ok"])

@app.route('/del_insv', methods=['GET', 'POST'])
def del_insv():
    insv = request.args.get('insv')
    map_name = request.args.get('map_name')
    map_table.update_one({"name":map_name},{"$pull":{"insv":{"name":insv}}})
    oss_addr=oss_root+"/"+map_name+"/"+insv
    print(oss_addr)
    bucket.delete_object(oss_addr)
    return json.dumps(["ok"])
    
@app.route('/list_all_maps', methods=['GET', 'POST'])
def list_maps():
    x =map_table.find({},{"_id":0})
    return json.dumps(list(x))
    
if __name__ == '__main__':
    app.config['SECRET_KEY'] = 'xxx'
    app.run('0.0.0.0', port=8001, debug=False)
