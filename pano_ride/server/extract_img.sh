#mkdir init_tmp/20201025-yue-yishigrape02
#ffmpeg -i init_tmp/20201025-yue-yishigrape02.mp4 -qscale:v 3 ./init_tmp/20201025-yue-yishigrape02/%06d.jpg
./vslam/build1/run_image_slam -v ./vslam/orb_vocab/orb_vocab.dbow2 -i init_tmp/20201025-yue-yishigrape02 -c ./vslam/chamo.yaml --frame-skip 1 --no_sleep -p init_tmp/loc_map_mo.bin --auto-term --mask ./vslam/mask.png
#./build/run_image_localization -v orb_vocab/orb_vocab.dbow2 -i VID_20201003_170826_00_011 -c chamo.yaml -p loc_map1.bin --no_sleep --mapping
#./vslam/build1/run_image_localization -v ./vslam/orb_vocab/orb_vocab.dbow2 -i init_tmp/20201020-liz_lin-Postpost1 -c ./vslam/chamo.yaml -p ./init_tmp/loc_map.bin --no_sleep --auto-term
