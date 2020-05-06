# ndt_omp
This package provides an OpenMP-boosted Normal Distributions Transform (and GICP) algorithm derived from pcl. The NDT algorithm is modified to be SSE-friendly and multi-threaded. It can run up to 10 times faster than its original version in pcl.

### Benchmark (on Core i7-6700K)
```
$ roscd ndt_omp/data
$ rosrun ndt_omp align 251370668.pcd 251371071.pcd
--- pcl::NDT ---
single : 282.222[msec]
10times: 2921.92[msec]
fitness: 0.213937

--- pclomp::NDT (KDTREE, 1 threads) ---
single : 207.697[msec]
10times: 2059.19[msec]
fitness: 0.213937

--- pclomp::NDT (DIRECT7, 1 threads) ---
single : 139.433[msec]
10times: 1356.79[msec]
fitness: 0.214205

--- pclomp::NDT (DIRECT1, 1 threads) ---
single : 34.6418[msec]
10times: 317.03[msec]
fitness: 0.208511

--- pclomp::NDT (KDTREE, 8 threads) ---
single : 54.9903[msec]
10times: 500.51[msec]
fitness: 0.213937

--- pclomp::NDT (DIRECT7, 8 threads) ---
single : 63.1442[msec]
10times: 343.336[msec]
fitness: 0.214205

--- pclomp::NDT (DIRECT1, 8 threads) ---
single : 17.2353[msec]
10times: 100.025[msec]
fitness: 0.208511
```

Several methods for neighbor voxel search are implemented. If you select pclomp::KDTREE, results will be completely same as the original pcl::NDT. We recommend to use pclomp::DIRECT7 which is faster and stable. If you need extremely fast registration, choose pclomp::DIRECT1, but it might be a bit unstable.

<img src="data/screenshot.png" height="400pix" /><br>
Red: target, Green: source, Blue: aligned


### 2018.12.27
python debug: 
python -m pdb '/home/whu/slam_ws/src/hdl_graph_slam/scripts/error_odom_png.py' '/home/whu/data/ndt_odom_KITTI/KITTI_odom_ndt_s2s_XX'

### 2020.5.6
'/home/whu/slam_ws/devel/lib/ndt_omp/align' '/home/whu/slam_ws/src/ndt_omp/data/251370668.pcd' '/home/whu/slam_ws/src/ndt_omp/data/251371071.pcd' 
--- pcl::NDT ---
single : 412.674[msec]
10times: 4034.14[msec]
fitness: 0.213937

--- pclomp::NDT (KDTREE, 1 threads) ---
single : 355.71[msec]
10times: 3497.31[msec]
fitness: 0.213892

--- pclomp::NDT (DIRECT7, 1 threads) ---
single : 164.942[msec]
10times: 1643.96[msec]
fitness: 0.21415

--- pclomp::NDT (DIRECT1, 1 threads) ---
single : 42.0168[msec]
10times: 375.549[msec]
fitness: 0.208487

--- pclomp::NDT (KDTREE, 8 threads) ---
single : 84.2158[msec]
10times: 793.837[msec]
fitness: 0.213892

--- pclomp::NDT (DIRECT7, 8 threads) ---
single : 42.9538[msec]
10times: 390.322[msec]
fitness: 0.21415

--- pclomp::NDT (DIRECT1, 8 threads) ---
single : 14.465[msec]
10times: 112.421[msec]
fitness: 0.208487
