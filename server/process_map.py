import os
import os.path
server_id='47.106.243.203'
stream = os.popen('sshpass -p La_009296 ssh -o StrictHostKeyChecking=no root@'+server_id+' "docker ps"')
output = stream.read()
server_status="IDLE"

if "chamo1" in output:
    server_status="BUSY"
else:
    server_status="IDLE"

if server_status=="IDLE":
    d = '/home/chamo/Documents/data/web_server_try'
    for o in os.listdir(d):
        usr_path = os.path.join(d,o)
        print(o)
        for oo in os.listdir(usr_path):
            full_scene_add=os.path.join(usr_path,oo)
            model_addr=os.path.join(full_scene_add,"fused.ply")
            if os.path.exists(model_addr)==False:
                os.system('sshpass -p La_009296 ssh -o StrictHostKeyChecking=no root@'+server_id+' "rm -rf /root/data/*"')
                os.system('sshpass -p La_009296 scp -r -o StrictHostKeyChecking=no '+full_scene_add+'/* root@'+server_id+':/root/data')
                os.system('sshpass -p La_009296 ssh -o StrictHostKeyChecking=no root@'+server_id+' "docker run --rm --gpus all --mount src=/root/data,target=/root/data,type=bind chamo1 /bin/bash /root/start.sh"')
                #os.system('sshpass -p La_009296 scp -o StrictHostKeyChecking=no root@'+server_id+':/root/data/dense/0/fused.ply '+full_scene_add)
            
            
        
        
        
