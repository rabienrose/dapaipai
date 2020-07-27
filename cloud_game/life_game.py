import time
import random
from PIL import Image,ImageFont, ImageDraw
import time
import asyncio
import danmaku
import _thread
import requests, re
import copy
#
out_img="out.bmp"
gird_size=100
cell_size=5
height=gird_size*cell_size
width=height
screen_h=height
screen_w=width+380
font = ImageFont.truetype("chinese.ttf", 24)
img = Image.new('RGB', (screen_w, screen_h), color = (255, 255, 255))
img.save('out.bmp')
draw = ImageDraw.Draw(img)
f=open('out.bmp','r+b')
gird=[]
move_dir=[[-1,-1],[-1,0],[-1,1],[0,-1],[0,1],[1,-1],[1,0],[1,1]]
gird=[[-1] * gird_size for i in range(gird_size) ]
user_list={}
tar_bytes_buf=bytearray([255]*screen_h*screen_w*3)
save_count=100000
roomid=585480
cookie = "_uuid=FCC99AF5-6929-AB63-0328-E28ADFC55F9307057infoc; buvid3=0DEC0D0A-02A9-47B0-BBCB-EE903A23036F190959infoc; LIVE_BUVID=AUTO7115751187077836; sid=5d1ryd59; CURRENT_FNVAL=16; stardustvideo=1; rpdid=|(k|km|~Ylkk0J'ul~luu)~J~; CURRENT_QUALITY=64; LIVE_ROOM_ADMIN_UP_TIP=1; bp_video_offset_3257122=413359025648654201; bp_t_offset_3257122=413359025648654201; Hm_lvt_8a6e55dbd2870f0f5bc9194cddf32a02=1594727616,1594735626,1595092342,1595092364; GIFT_BLOCK_COOKIE=GIFT_BLOCK_COOKIE; bsource=search_baidu; _dfcaptcha=1259f52722c83e74aaf23ef61728d286; DedeUserID=3257122; DedeUserID__ckMd5=0459704718e252a3; SESSDATA=87c6ffe3%2C1610681708%2C0ed5a*71; bili_jct=2bb07096258d0d7e38d454c7460c8923; PVID=21"
token = re.search(r'bili_jct=(.*?);', cookie).group(1)
new_add_list=[]
danmu_cache=[]
def send_msg(text):
    url = 'https://api.live.bilibili.com/ajax/msg'
    form = {
        'roomid': roomid,
        'visit_id': '',
        'csrf_token': token  # csrf_token就是cookie中的bili_jct字段;且有效期是7天!!!
    }
    headers = {
        'User-Agent': 'Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/55.0.2883.87 Safari/537.36',
        'Cookie': cookie
    }
    html = requests.post(url, data=form)
    print(text)
    url_send = 'https://api.live.bilibili.com/msg/send'
    data = {
        'color': int("ff0000", 16),
        'fontsize': '25',
        'mode': '1',
        'msg': text,
        'rnd': 1595133024,
        'roomid': roomid,
        'csrf_token': token,
        'csrf': token
    }
    try:
        html_send = requests.post(url_send, data=data, headers=headers)
        result = html_send.json()
        if result['code'] == 0 and result['msg'] == '':
            print("succ")
    except:
        print("failed")

def takeSecond(elem):
    return elem[0]

def draw_rank(ind, name, color, score):
    text_str=name+":"+str(score)
    draw.ink = color
    draw.text((width+10, ind*25), text_str, font = font)
    return
    
def clear_rank():
    shape = [(width+10, 0), (width+10+380, screen_h)]
    draw.rectangle(shape, fill ="#ffffff")
    return

def game_thread():
    global danmu_cache
    global new_add_list
    tmp_count=0
    tmp_count1=10
    count=0
#    enter_room()
    while True:
        count=count+1
        start_time = time.time()
        dels=[]
        adds=[]
        for i in range(gird_size):
            for j in range(gird_size):
                [re,id] = check_add_or_del(i, j)
                if re==1:
                    adds.append([i,j, id])
                elif re==-1:
                    dels.append([i,j,id])
        for item in dels:
            user_list[item[2]]["score"]=user_list[item[2]]["score"]-1
            draw_point(item[0],item[1],-1)
        for item in adds:
            user_list[item[2]]["score"]=user_list[item[2]]["score"]+1
            draw_point(item[0],item[1],item[2])
        tmp_count=tmp_count+1
        if tmp_count>=2:
            tmp_count=0
            user_rank=[]
            del_user_list=[]
            for key in user_list:
                item=user_list[key]
                score = item["score"]
                if score>0:
                    user_rank.append([score, item])
                else:
                    del_user_list.append(key)
            for item in del_user_list:
                del user_list[item]
            user_rank.sort(key=takeSecond, reverse=True)
            clear_rank()
            for i in range(len(user_rank)):
                draw_rank(i, user_rank[i][1]["name"], user_rank[i][1]["color_int"], user_rank[i][1]["score"])
            if len(new_add_list)>0:
                tmp_add_list=copy.deepcopy(new_add_list)
                new_add_list=[]
                for item in tmp_add_list:
                    handle_user(item[0], item[1], item[2], item[3])
            #print(user_list)
        if len(danmu_cache)>0:
            if tmp_count1>9:
                tmp_count1=0
                tmp_danmu = danmu_cache.pop()
                send_msg(tmp_danmu)
            tmp_count1=tmp_count1+1
        else:
            tmp_count1=10
        rend_gird()
        #print("--- %s seconds ---" % (time.time() - start_time))
        time.sleep(0.5)

async def printer(q):
    global new_add_list
    while True:
        m = await q.get()
        if m['msg_type'] == 'gift':
            #id=random.randint(0, 999999)
            if m['msg_type'] == 'gift':
                data=m["raw"]
                coin_count = data["total_coin"]
                send_name=data["uname"]
                if len(send_name)>5:
                    send_name=send_name[0:6]
                uid=data["uid"]
                giftType=data["giftType"]
                gift_name=data["giftName"]
                if len(gift_name)>5:
                    gift_name=gift_name[0:5]
                if giftType!= 5:
                    new_add_list.append([uid, send_name, coin_count, gift_name])
        elif m['msg_type'] == 'danmaku':
            uname = m['name']
            content = m['content']
            uid = m['id']
#            if uid!=3257122:
#                send_msg(send_text)
            

def handle_user(user_id, user_name, count, gift_name):
    global danmu_cache
    user_count=len(user_list)
    if user_count>20:
        return
    user_info={}
    if user_id in user_list:
        user_info = user_list[user_id]
    else:
        r=random.randint(0, 255)
        g=random.randint(0, 255)
        b=random.randint(0, 255)
        user_info["name"]=user_name
        user_info["id"]=user_id
        user_info["color_int"]= r + g * 256 + b * 256 * 256
        user_info["color_str"]='#%02x%02x%02x'%(r, g, b)
        user_info["score"]=0
        user_list[user_id]=user_info
    count_cell=0
    empty_cell_list=[]
    for i in range(len(gird)):
        for j in range(len(gird[i])):
            if gird[i][j]==-1:
                empty_cell_list.append([i,j])
    count_cell=len(empty_cell_list)
    new_count=count
    occup_count=gird_size*gird_size - count_cell
    if occup_count+new_count<200:
        new_count=1000
    if count_cell<new_count:
        new_count=count_cell
    random.shuffle(empty_cell_list)
    add_new_cell(user_info, empty_cell_list[0:new_count])
    send_text=user_name+"使用"+gift_name+"召唤"+str(new_count)+"小弟"
    danmu_cache.append(send_text)
    return new_count

def add_new_cell(user_info, posis):
    count=0
    for i in range(len(posis)):
        x=posis[i][0]
        y=posis[i][1]
        if gird[x][y]==-1:
            draw_point(x,y,user_info["id"])
            user_info["score"]=user_info["score"]+1

async def main(url):
    q = asyncio.Queue()
    dmc = danmaku.DanmakuClient(url, q)
    asyncio.create_task(printer(q))
    await dmc.start()

def init_game():
    print("init game")
    user_list.clear()
    gird=[[0] * gird_size for i in range(gird_size) ]
    rend_gird()

def draw_point(x,y,b_set):
    gird[x][y]=b_set
    cell_shape = [(x*cell_size, y*cell_size), (x*cell_size+cell_size, y*cell_size+cell_size)]
    if b_set!=-1:
        color=user_list[b_set]["color_str"]
        draw.rectangle(cell_shape, fill =color)
    else:
        draw.rectangle(cell_shape, fill ="#ffffff")

def gen_new_init(count):
    for i in range(count):
        x=random.randint(0, gird_size-1)
        y=random.randint(0, gird_size-1)
        draw_point(x,y,1)

def rend_gird():
#    global save_count
#    img.save("out_"+str(save_count)+".png")
#    save_count=save_count+1
    f.seek(54)
    img_bytes=img.tobytes()
    for i in range(screen_h):
        tar_bytes_buf[(screen_h-i-1)*screen_w*3:(screen_h-i)*screen_w*3]=img_bytes[i*screen_w*3:(i+1)*screen_w*3]
    f.write(tar_bytes_buf)

def check_add_or_del(i, j):
    re=0 #1: add   0:do nothing  -1:del
    id_list=[]
    for item in move_dir:
        x=i+item[0]
        if x<0 or x>=gird_size:
            continue
        y=j+item[1]
        if y<0 or y>=gird_size:
            continue
        if gird[x][y]!=-1:
            id_list.append(gird[x][y])
    
    id=gird[i][j]
    if gird[i][j]!=-1:
        if len(id_list)<=1 or len(id_list)>3:
            re=-1
    else:
        if len(id_list)==3:
            re=1
            if id_list[0]==id_list[1] and id_list[1]==id_list[2]:
                id=id_list[0]
            elif id_list[0]==id_list[1] and id_list[1]!=id_list[2]:
                id=id_list[0]
            elif id_list[1]==id_list[2] and id_list[0]!=id_list[1]:
                id=id_list[1]
            elif id_list[2]==id_list[0] and id_list[0]!=id_list[1]:
                id=id_list[0]
            else:
                x=random.randint(0, 2)
                id=id_list[x]
    return [re,id]

init_game()
#game_thread()
_thread.start_new_thread( game_thread,() )
asyncio.run(main("https://live.bilibili.com/"+str(roomid)))

        
