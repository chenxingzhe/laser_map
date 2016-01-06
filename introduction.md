#代码说明文档
##一.节点
###1. talker节点  
####对应文件:  
**src/talker.cpp  
src/lsd.cpp  
src/lsd_lines.cpp  
src/lsd_opencv.cpp  
src/lsd_wrap.cpp**  
####节点说明：
* 图像二值化：先设定一个亮度区间将亮度低的点过滤掉。又因为激光线的宽度比较固定，大约1到10像素之间，利用这一点将比较大的亮块过滤掉，可以减少阳光的影响。该过程在talker.cpp的myRange函数中。
* 直线检测：对上一步二值化的图像进行直线检测，使用lsd_lines检测算法。
* 目标点定位：因为激光线是有宽度的所以一条激光线可以检测上下两条直线，所以目标点是竖直线与两条直线交点的中点。
* 目标点投影：知道图像坐标后，利用相机内参和与相机跟激光平面的外参算出激光点的世界坐标。
* 世界坐标转化为激光数据：因为激光数据是极坐标信息，所以将直角坐标系下的坐标转化为极坐标。
* 激光数据发布：利用ros publisher发布信息。
###2. calib节点
####对应文件:
**src/calib.cpp** 
####节点说明：
* 因为所计算出的激光数据与里程计数据存在一个类似线性关系，所以可以将机器人平行面向一堵墙，让它向着墙走一米，里程计并且每0.1米记录里程计与激光信息，可以利用差值计算出激光数据与里程计相差的一个比例系数，统一尺度。

##二.ros节点
###gmapping
用来构建地图，参数存储在launch/test.launch。需要激光与里程计数据。
###amcl
自定位节点，参数在launch/map.launch。需要激光与里程计数据。
###map_server
地图读取存储节点。  
当gmapping构建完地图后，使用
`rosrun map_server map_saver` 存储地图。  
读取地图在launch/map.launch文件中。
###move_base
* local planner参数：`base_local_planner_params.yaml`
* costmap参数：`costmap_common_params.yaml,local_costmap_params.yaml,global_costmap_params.yaml`
* 节点使用在launch/map.launch文件中
##三.整体使用
###1.安装
[下载代码](https://github.com/chenxingzhe/laser_map "下载代码")

```
cd ~/laser_map
catkin_make
```
###2.使用
cd ~/laser_map  
里程计开启：bash odom.sh  
机器人控制：bash robot.sh  
gmapping:bash test.sh  
navigation:bash map.sh(这个文件里有调整曝光的命令)


