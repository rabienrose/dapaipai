import requests
import os
from flask import Flask
from flask import render_template
from flask import request
import random
import re
import json
import operator

app = Flask(__name__)

@app.route('/try', methods=['GET', 'POST'])
def handle_api_evaluation():
    return 'chamo'

@app.route('/upload', methods=['POST'])
def handle_upload():
    imgs = request.files.getlist('myFile')
    usr_id = request.form["usr_id"]
    print(usr_id)
    usr_root='./static/res/'+usr_id
    if not os.path.exists(usr_root):
        os.makedirs(usr_root)
    scene_id = request.form["scene_id"]
    print(scene_id)
    scene_root=usr_root+'/'+scene_id
    if not os.path.exists(scene_root):
        os.makedirs(scene_root)
    image_addr=scene_root+"/img"
    if not os.path.exists(image_addr):
        os.makedirs(image_addr)
    for img in imgs:
        img.save(image_addr+'/' + img.filename)
    return 'chamo'
    
@app.route('/show_list', methods=['GET', 'POST'])
def get_article_keyword():
    d = './static/res'
    re=[]
    for o in os.listdir(d):
        usr_path = os.path.join(d,o)
        print(o)
        for oo in os.listdir(usr_path):
            item={}
            item['usr_id']=o
            item['scene_id']=oo
            full_scene_add=os.path.join(usr_path,oo)
            model_addr=os.path.join(full_scene_add,"fused.ply")
            if os.path.exists(model_addr)==True:
                item['model']="fused.ply"
            else:
                item['model']=""
            re.append(item)
    return json.dumps(re)

@app.route('/main', methods=['GET'])
def server_article_detail():
    return render_template('index.html')

if __name__ == '__main__':
    app.config['SECRET_KEY'] = 'xxx'
    app.config['UPLOAD_FOLDER']='./raw'
    app.run('0.0.0.0', port=8001)
