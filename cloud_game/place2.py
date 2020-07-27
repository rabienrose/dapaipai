import subprocess as sp
import time
from PIL import Image
from shutil import copyfile
code='?streamname=live_3257122_5767728&key=97656f68620cc093b890d5b8007c74f8"'
url='"rtmp://live-push.bilivideo.com/live-bvc/'+code
cmd_out = ['ffmpeg','-re','-i','"a.mp4"','-vcodec','copy','-f','flv', url]
#cmd_out = ['ffmpeg','-r','1','-f','image2pipe','-vcodec','png','-i','pipe:0','-vcodec','libx264','-pix_fmt','yuv420p','tmp.mp4']
#cmd_out = ['ffmpeg','-re','-f','image2pipe','-vcodec','png','-i','pipe:0','-vcodec','libx264','-f','flv', url]
#cmd_out=['ffmpeg','-f','image2','-loop','1','-i','./pic/1.jpg','-f','flv', url]

#pipe = sp.Popen(cmd_out, stdin=sp.PIPE)
imgs=["2.jpg","3.jpg","4.jpg"]
while True:
    for img in imgs:
        copyfile("./pic/"+img, "./pic/1.jpg")
        time.sleep(1)

#pipe.stdin.close()
#pipe.wait()
