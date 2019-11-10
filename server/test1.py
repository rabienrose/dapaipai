import os
os.system('sshpass -p La_009296 scp -o StrictHostKeyChecking=no ./test.py root@39.108.229.67:/root')
os.system('sshpass -p La_009296 ssh -o StrictHostKeyChecking=no root@39.108.229.67 "ls"')
