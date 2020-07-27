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

def percentage(consumed_bytes, total_bytes):
    if total_bytes:
        print(str(100 * (float(consumed_bytes) / float(total_bytes))))

access_key_id = os.getenv('OSS_TEST_ACCESS_KEY_ID', 'LTAI4GJDtEd1QXeUPZrNA4Yc')
access_key_secret = os.getenv('OSS_TEST_ACCESS_KEY_SECRET', 'rxWAZnXNhiZ8nemuvshvKxceYmUCzP')
endpoint = os.getenv('OSS_TEST_ENDPOINT', 'oss-cn-beijing-internal.aliyuncs.com')
#endpoint = os.getenv('OSS_TEST_ENDPOINT', 'oss-cn-beijing.aliyuncs.com')
bucket_name='ride-v'

src_dir="raw_files"
dst_dir="maps/shangrila/insv"
bucket = oss2.Bucket(oss2.Auth(access_key_id, access_key_secret), endpoint, bucket_name=bucket_name)
for obj in oss2.ObjectIterator(bucket, prefix = src_dir):
    splited_key = obj.key.split("/")
    splited_key1 = splited_key[-1].split(".")
    if splited_key1[-1]=="insv":
        print(obj.key)
        bucket.get_object_to_file(obj.key, splited_key[-1])
        bucket.put_object_from_file(dst_dir+"/"+splited_key[-1], splited_key[-1])
        os.remove(splited_key[-1])
