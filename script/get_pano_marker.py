from PIL import Image, ImageDraw
import random
from random import randrange
import math

def draw_point(draw, x, y, r, c):
    leftUpPoint = (x-r, y-r)
    rightDownPoint = (x+r, y+r)
    twoPointList = [leftUpPoint, rightDownPoint]
    draw.ellipse(twoPointList, fill=c)

img = Image.new('RGBA', (3600, 1800), color = 'white')
draw = ImageDraw.Draw(img)
cirle_color=(0,125,125,255)
white=(255,255,255,255)
for vert_pix in range(900-200,900+201,200):
    for hori_pix in range(150,3600,300):
        draw_point(draw, hori_pix, vert_pix, 30, cirle_color)
        draw_point(draw, hori_pix, vert_pix, 25, white)
for vert_pix in (900-400, 900+400):
    for hori_pix in range(200,3600,400):
        draw_point(draw, hori_pix, vert_pix, 30, cirle_color)
        draw_point(draw, hori_pix, vert_pix, 25, white)
for vert_pix in (900-600, 900+600):
    for hori_pix in range(300,3600,600):
        draw_point(draw, hori_pix, vert_pix, 30, cirle_color)
        draw_point(draw, hori_pix, vert_pix, 25, white)
for vert_pix in (900-800, 900+800):
    for hori_pix in range(900,3600,1800):
        draw_point(draw, hori_pix, vert_pix, 30, cirle_color)
        draw_point(draw, hori_pix, vert_pix, 25, white)

img.save('./me.png')

