ffmpeg -re -r 10 -f image2 -loop 1 -i out.bmp -vcodec libx264 -f flv "rtmp://live-push.bilivideo.com/live-bvc/?streamname=live_3257122_5767728&key=97656f68620cc093b890d5b8007c74f8"

#ffmpeg -re -i video.mp4 -vcodec copy -f flv "rtmp://live-push.bilivideo.com/live-bvc/?streamname=live_3257122_5767728&key=97656f68620cc093b890d5b8007c74f8"
