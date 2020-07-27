scp -r static root@39.105.230.163:~/pano_map
scp -r libs root@39.105.230.163:~/pano_map
scp *.py root@39.105.230.163:~/pano_map
scp start.sh root@39.105.230.163:~/pano_map
scp -r native root@39.105.230.163:~/pano_map

#ssh root@39.105.230.163 'docker kill $(docker ps -q)'
#ssh root@39.105.230.163 'docker run -d --rm --mount type=bind,source=/root/pano_map,dst=/workspace -w /workspace -p 8000:8000 insta_sdk /bin/bash start.sh'

#export LD_LIBRARY_PATH=/workspace/native/lib
#gcc native/example/main.cc -o native/stitcherSDKDemo -I/workspace/native/include -L/workspace/native/lib -lMediaSDK -lstdc++
#./native/stitcherSDKDemo -inputs yongdingmen.insv.insv -output yongdingmen.MP4 -stitch_type optflow -hdr_type multiimagehdr_mbb -enable_flowstate

