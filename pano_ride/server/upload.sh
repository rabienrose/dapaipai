scp -r static root@39.105.230.163:~/pano_map
scp -r libs root@39.105.230.163:~/pano_map
scp *.py root@39.105.230.163:~/pano_map
scp start.sh root@39.105.230.163:~/pano_map

#ssh root@39.105.230.163 'docker kill $(docker ps -q)'
#ssh root@39.105.230.163 'docker run -d --rm --mount type=bind,source=/root/pano_map,dst=/workspace -w /workspace -p 8000:8000 c_py_ros_cv:v1_g2o /bin/bash start.sh'
