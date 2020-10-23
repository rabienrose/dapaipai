#mkdir init_tmp/20201019-lin-zhongshu01
#ffmpeg -i init_tmp/20201019-lin-zhongshu01.mp4 -qscale:v 3 ./init_tmp/20201019-lin-zhongshu01/%06d.jpg
./vslam/build1/run_image_slam -v ./vslam/orb_vocab/orb_vocab.dbow2 -i Yishigrapebot/imgs -c ./vslam/chamo.yaml --frame-skip 1 --no_sleep -p Yishigrapebot/loc_map.bin --mask ./vslam/mask.png
#./build/run_image_localization -v orb_vocab/orb_vocab.dbow2 -i VID_20201003_170826_00_011 -c chamo.yaml -p loc_map1.bin --no_sleep --mapping
#./build/run_image_localization -v orb_vocab/orb_vocab.dbow2 -i 20200825-yue-yiheyuanB07xiequyuan -c chamo.yaml -p loc_map.bin --no_sleep --auto-term
