#运行A-LOAM
##1 Prerequisites
###1.1 Ceres Solver
[Follow Installation Ceres Solver](https://www.jianshu.com/p/7b1f94d7b5a6)
###1.2 PCL
[Follow Installation PCL](https://blog.csdn.net/mush_room/article/details/78339578)
##2 Build A-LOAM
Clone the repository and catkin_make:

    cd ~/catkin_ws/src
    git clone https://github.com/WeiChunyu-star/A-LOAM
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
##3 Velodyne VLP-16 Example
Download [NSH indoor outdoor](链接：https://pan.baidu.com/s/1Rp6HEj_ck0XmZkuXmulk3w) to YOUR_DATASET_FOLDER. Number is 879q

	roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
	rosbag play YOUR_DATASET_FOLDER/nsh_indoor_outdoor.bag
#运行LOAM
[Like this](https://www.cnblogs.com/chenbokai/p/7299069.html)
