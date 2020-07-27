import time
import random
from PIL import Image,ImageFont, ImageDraw
import time
import asyncio
import danmaku
import _thread


async def printer(q):
    while True:
        m = await q.get()
        print(m)
        if m['msg_type'] == 'gift':
            coion_count = m["total_coin"]
            send_name=m["uname"]
            uid=m["uid"]
            giftType=m["giftType"]

async def main(url):
    q = asyncio.Queue()
    dmc = danmaku.DanmakuClient(url, q)
    asyncio.create_task(printer(q))
    await dmc.start()

asyncio.run(main("https://live.bilibili.com/585480"))

        

