import time
import random
from PIL import Image, ImageDraw
out_img="out.bmp"
aspect=16/9.0
height=120
width=round(height*aspect)
print(height*aspect)
img = Image.new('RGB', (width, height), color = (255, 255, 255))
img.save('out.bmp')
f=open('out.bmp','r+b')

def draw_point(f, x,y,r,g,b):
    f.seek(54+y*height*3+x*3)
    f.write(bytearray([r,g,b]))

def gen_new_cluster():
    cluster={}
    x=random.randint(0, width-1)
    y=random.randint(0, height-1)
    r=random.randint(0, 255)
    g=random.randint(0, 255)
    b=random.randint(0, 255)
    cluster["posi"]=[x,y]
    cluster["color"]=[r,g,b]
    cluster["pts"]=[[0,0]]
    return cluster
    
def draw_cluster(cluster):
    return
    
def get_cluster_bounding(cluster)
    min_x=-1
    max_x=-1
    min_y=-1
    max_y=-1
    for p in cluster["pts"]:
        if p[0]<min_x or min_x==-1:
            min_x=p[0]
        if p[0]>max_x or max_x==-1:
            max_x=p[0]
        if p[1]<min_y or min_y==-1:
            min_y=p[1]
        if p[1]>max_y or max_y==-1:
            max_y=p[1]
    min_x=min_x+cluster["posi"][0]
    max_x=max_x+cluster["posi"][0]
    min_y=min_y+cluster["posi"][1]
    max_y=max_y+cluster["posi"][1]
    return [min_x, min_y, max_x, max_y]
    
def get_overlap_cluster(c1, c2)

    
move_dir=[[-1,-1],[-1,0],[-1,1],[0,-1],[0,0],[0,1],[1,-1],[1,0],[1,1]]
def check_possible_move():
    possible_dir=[]
    for item in move_dir:
        if next_x>=0 and next_x<width and next_y>=0 and next_y<height:
            possible_dir.append(item)
clusters=[]
clusters.append(gen_new_cluster())
clusters.append(gen_new_cluster())



while True:
    for i in range(len(clusters)):
        
        dir_id = random.randint(0, len(possible_dir)-1)
        next_x=clusters[i]["posi"][0]+possible_dir[dir_id][0]
        next_y=clusters[i]["posi"][1]+possible_dir[dir_id][1]
        clusters[i]["posi"][0]=next_x
        clusters[i]["posi"][y]=next_y
            
    draw_point(f, x,y,r,g,b)
    f.flush()
    time.sleep(0.1)
    
