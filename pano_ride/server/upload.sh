#scp -r static root@39.105.230.163:~/pano_map
#scp -r libs root@39.105.230.163:~/pano_map
#scp *.py root@39.105.230.163:~/pano_map
scp start.sh root@39.105.230.163:~/pano_map

#ssh root@106.15.191.244 'docker kill $(docker ps -q)'
#ssh root@106.15.191.244 'docker run -d --rm --mount type=bind,source=/root/web_map,dst=/workspace -w /workspace -p 8000:8000 c_py_ros_cv /bin/bash start.sh'
