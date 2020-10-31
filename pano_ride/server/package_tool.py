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
from init_process import trim
from init_process import get_trajectory_and_mps
from init_process import post_process
from init_process import resample
from init_process import extract_meta
from init_process import cal_scale_gravity_gps
from init_process import generate_graph
from init_process import output_2_mp4
ws_root="init_tmp"
if __name__ == "__main__":
    insv_list=["20201019-lin-mofan01"]
    #get_trajectory_and_mps(insv_list)
    generate_graph(insv_list, True)
    output_2_mp4()
    
