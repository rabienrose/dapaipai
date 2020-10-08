scp -r static root@39.105.230.163:~/pano_map
scp -r libs root@39.105.230.163:~/pano_map
scp *.py root@39.105.230.163:~/pano_map
scp start.sh root@39.105.230.163:~/pano_map
scp -r native root@39.105.230.163:~/pano_map

scp insv_process.py root@39.105.230.163:~/pano_map

#ssh root@39.105.230.163 'docker kill $(docker ps -q)'
#ssh root@39.105.230.163 'docker run -d --rm --name insv --mount type=bind,source=/root/pano_map,dst=/workspace -w /workspace -p 8000:8000 insta_sdk /bin/bash start.sh'

#export LD_LIBRARY_PATH=/workspace/DataExtractor/InsMetadataSDK/lib
#export LD_LIBRARY_PATH=/workspace/native/lib
#gcc native/example/main.cc -o native/stitcherSDKDemo -I/workspace/native/include -L/workspace/native/lib -lMediaSDK -lstdc++
gcc DataExtractor/InsMetadataSDK/example/main.cc -o /workspace/DataExtractor/extractor -I/workspace/DataExtractor/InsMetadataSDK/include -L/workspace/DataExtractor/InsMetadataSDK/lib -lInsMetaDataSDK -lstdc++ -lpthread

#./native/stitcherSDKDemo -inputs yongdingmen.insv.insv -output yongdingmen.MP4 -stitch_type optflow -hdr_type multiimagehdr_mbb -enable_flowstate

#ffmpeg -i ws/kunyuhe.mp4 -qscale:v 15 -filter:v "crop=1504:1504:0:0,scale=512:512" ./ws/i/chamo_%06d.jpg
#kalibr_bagcreater --folder /media/psf/Home/Documents/dapaipai/pano_ride/server/bag_src --output-bag cali.bag

kalibr-cde/kalibr_bagextractor --image-topics /cam0/image_raw /cam1/image_raw --imu-topics /imu0 --bag dataset-calib-cam7_512_16.bag
kalibr-cde/kalibr_calibrate_cameras --target checkerboard_7x6_50x50cm.yaml --bag cali.bag --models pinhole-equi --topics /cam0/image_raw

kalibr_calibrate_cameras --target /media/psf/Home/Documents/dapaipai/pano_ride/server/april_6x6_50x50cm.yaml --bag cali.bag --models pinhole-equi --topics /cam0/image_raw

2
1499.910, 1516.101, 1508.135
0.000,    0.000,    0.000
1494.510, 4554.735, 1552.938
1.497,    -0.256,   179.284
6080, 3040, 2323
2
1481.2, 1518.81, 1516.2
-0.178493, 0.473543, -179.656
1481.33, 4555.99, 1514.28
0.693851, 0.558506, 0.783005
6080, 3040, 3105
