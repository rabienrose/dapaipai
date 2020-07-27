# !/user/bin/env python
# -*- coding:utf-8 -*- 
# time: 2018/2/2--21:19
__author__ = 'Henry'

'''
爬取B站直播弹幕并发送跟随弹幕
'''

import requests, re
import time
import random


def main():
    print('*' * 30 + '欢迎来到B站直播弹幕小助手' + '*' * 30)
    cookie = "_uuid=FCC99AF5-6929-AB63-0328-E28ADFC55F9307057infoc; buvid3=0DEC0D0A-02A9-47B0-BBCB-EE903A23036F190959infoc; LIVE_BUVID=AUTO7115751187077836; sid=5d1ryd59; CURRENT_FNVAL=16; stardustvideo=1; rpdid=|(k|km|~Ylkk0J'ul~luu)~J~; CURRENT_QUALITY=64; LIVE_ROOM_ADMIN_UP_TIP=1; bp_video_offset_3257122=413359025648654201; bp_t_offset_3257122=413359025648654201; Hm_lvt_8a6e55dbd2870f0f5bc9194cddf32a02=1594727616,1594735626,1595092342,1595092364; GIFT_BLOCK_COOKIE=GIFT_BLOCK_COOKIE; bsource=search_baidu; _dfcaptcha=1259f52722c83e74aaf23ef61728d286; DedeUserID=3257122; DedeUserID__ckMd5=0459704718e252a3; SESSDATA=87c6ffe3%2C1610681708%2C0ed5a*71; bili_jct=2bb07096258d0d7e38d454c7460c8923; PVID=21"
    token = re.search(r'bili_jct=(.*?);', cookie).group(1)
    print(token)
    roomid=585480
    while True:
        # 爬取:
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

#        # 跟随发送:
        url_send = 'https://api.live.bilibili.com/msg/send'
        data = {
            'color': '16777215',
            'fontsize': '25',
            'mode': '1',
            'msg': 'asdfasdfasasd',
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
        
        time.sleep(1)
        
        try:
            html_send = requests.post(url_send, data=data, headers=headers)
            result = html_send.json()
            if result['code'] == 0 and result['msg'] == '':
                print("succ")
        except:
            print("failed")
        break


if __name__ == '__main__':
    main()
