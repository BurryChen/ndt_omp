#!/bin/bash

# 1.arg
echo "path_arg : $1"

# 2.SLAM, write odom file

for seq in 00 01 02 03 04 05 06 07 08 09 10
do

logfile=${1}/data/${seq}.log
echo $seq $logfile

#touch $logfile
/home/whu/slam_ws/devel/lib/ndt_omp/odom_kitti 04 >$logfile

done 

# 3.eva
cd '/home/whu/data/loam_KITTI/devkit_old/cpp' 
./evaluate_odometry ${1}
cd ~

#evo_traj kitti '/home/whu/data/ndt_odom_KITTI/KITTI_odom_ndt_s2s_step/errors/KITTI_04_scan_error.txt'     
#--plot_mode=xz --save_plot '/home/whu/data/ndt_odom_KITTI/KITTI_odom_ndt_s2s_step/errors/KITTI_04_scan_error.pdf' 

#evo_traj kitti '/home/whu/data/ndt_odom_KITTI/KITTI_odom_ndt_s2s_step/data/KITTI_04_odom.txt'      --plot_mode=xz  
#--ref='/home/whu/data/loam_KITTI/gt/04.txt'  --save_plot '/home/whu/data/ndt_odom_KITTI/KITTI_odom_ndt_s2s_step/data/KITTI_04_odom.pdf'

# 4.error png
python '/home/whu/slam_ws/src/hdl_graph_slam/scripts/error_odom_png.py' ${1}

