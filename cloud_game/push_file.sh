
#ssh root@139.105.230.163 'docker run -d --rm --name ffmpeg --mount type=bind,source=/root/danmu_place,dst=/workspace -w /workspace insta_sdk /bin/bash start_ffmpeg.sh'
#ssh root@139.105.230.163 'docker run -d --rm --name game --mount type=bind,source=/root/danmu_place,dst=/workspace -w /workspace insta_sdk /bin/bash start_game.sh'
#scp chinese.ttf root@39.105.230.163:/root/danmu_place
scp life_game.py root@39.105.230.163:/root/danmu_place
scp start_game.sh root@39.105.230.163:/root/danmu_place
scp start_ffmpeg.sh root@39.105.230.163:/root/danmu_place
scp danmaku/bilibili.py root@39.105.230.163:/root/danmu_place/danmaku

#scp place.py root@39.105.230.163:/root/danmu_place
