import time
import random
from PIL import Image,ImageFont, ImageDraw
import time

out_img="out.bmp"
gird_size=100
cell_size=5
font = ImageFont.truetype("OpenSans-Bold.ttf", 24)

height=gird_size*cell_size
width=height+300
img = Image.new('RGB', (width, height), color = (255, 255, 255))
img.save('out.bmp')
draw = ImageDraw.Draw(img)

f=open('out.bmp','r+b')
start_time=time.time()
draw.ink = 255 + 0 * 256 + 0 * 256 * 256
draw.text((50,50), "1sdfasdf", font = font)
shape = [(100, 200), (110, 220)]
draw.rectangle(shape, fill ="#436733")
f.seek(54)
img_bytes=img.tobytes()
f.write(img_bytes)
print("--- %s seconds ---" % (time.time() - start_time))
f.close()




        
