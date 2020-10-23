scp *.py root@39.105.230.163:~/pano_map/
scp -r libs root@39.105.230.163:~/pano_map/
scp -r static/js root@39.105.230.163:~/pano_map/static/
scp -r static/html root@39.105.230.163:~/pano_map/static/
scp output_process.py root@39.105.230.163:~/pano_map/
scp db_tool.py root@39.105.230.163:~/pano_map/
scp gps_process.py root@39.105.230.163:~/pano_map/

docker run -it --rm --name pano_app --mount type=bind,source=/root/pano_map,dst=/workspace -w /workspace -p 8001:8001 slam_insta python3 pano_app.py

/Users/ziliwang/Library/Android/sdk/platform-tools/adb push xp_HiddenCam_edwardE28Debug.apk /system/app/xp_hiddenCam-E28/xp_hiddenCam-E28.apk
/Users/ziliwang/Library/Android/sdk/platform-tools/adb push libndk_image.so /system/app/xp_hiddenCam-E28/lib/arm/libndk_image.so
