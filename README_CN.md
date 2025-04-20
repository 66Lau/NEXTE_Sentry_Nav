# The navigation system for omnidirectional wheel robot in real world

[![README in English](https://img.shields.io/badge/English-d9d9d9)](./README.md)
[![简体中文版自述文件](https://img.shields.io/badge/简体中文-d9d9d9)](./README_CN.md)


## Introduction
本仓库整合各开源算法在轮式机器人平台实现2-D的定位与导航

本文的代码地址：https://github.com/66Lau/NEXTE_Sentry_Nav

感谢[rain](https://github.com/rain11ki?tab=following)在该框架下延伸的3维地图导航实现:https://github.com/rain11ki/NEXTE_Sentry_Nav3D 

3-D 导航与探索实现: https://github.com/66Lau/NEXTE_Sentry_Nav_3D

如果需要仿真环境：https://github.com/66Lau/sentry_sim

环境：
- ros-noetic 
- ubuntu 20.04  
- Lidar: Mid360

本仓库对各模块有不同修改和适配，建议直接clone本仓库，以下为本仓库的开发思路和流程

<div align="center"><img src="doc/sentry_navigation.png" width=90% /></div>

## Hardware info
- [MID360 offical web官网](https://www.livoxtech.com/cn/mid-360)
- [Quick-start-doc|MID360快速开始手册](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/Livox_Mid-360_Quick_Start_Guide_multi.pdf)
- [user-manual|MID360用户手册](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/20230727/Livox_Mid-360_User_Manual_CHS.pdf)
- [Livox_sdk2源地址](https://github.com/Livox-SDK/Livox-SDK2)
- [livox_ros_driver2源地址](https://github.com/Livox-SDK/livox_ros_driver2)

- [livox ros driver2安装博客](https://blog.csdn.net/qq_29912325/article/details/130269367?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169734904416800182711632%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169734904416800182711632&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-130269367-null-null.142^v96^pc_search_result_base9&utm_term=livox_sdk2&spm=1018.2226.3001.4187)

- [虚拟机和mid360桥接博客](https://blog.csdn.net/sinat_39110395/article/details/123545816?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169735401816800227447255%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169735401816800227447255&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-123545816-null-null.142^v96^pc_search_result_base9&utm_term=%E8%99%9A%E6%8B%9F%E6%9C%BA%E8%BF%9E%E6%8E%A5%E9%9B%B7%E8%BE%BE&spm=1018.2226.3001.4187)




## Livox和Fast-Lio配置流程
1. 安装[Livox_sdk2](https://github.com/Livox-SDK/Livox-SDK2),readme有写相关过程，注意：要更改主机ip为192.168.1.50[ubuntu修改方法](https://blog.csdn.net/sinat_39110395/article/details/123545816?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169735401816800227447255%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169735401816800227447255&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-123545816-null-null.142^v96^pc_search_result_base9&utm_term=%E8%99%9A%E6%8B%9F%E6%9C%BA%E8%BF%9E%E6%8E%A5%E9%9B%B7%E8%BE%BE&spm=1018.2226.3001.4187)，本人雷达ip为192.168.1.180
2. 安装[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2), readme有写相关过程， 注意运行前要注意更改config里面的主机ip和雷达IP
3. 配置fast-lio  

参考：  
[FAST_LIO原地址](https://github.com/hku-mars/FAST_LIO)  
[FAST-LIO配置中文博客](https://blog.csdn.net/qq_42108414/article/details/131530293?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169737102216800185825796%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169737102216800185825796&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-131530293-null-null.142^v96^pc_search_result_base9&utm_term=fast%20lio%E9%85%8D%E7%BD%AE&spm=1018.2226.3001.4187)  
[关于在ROS1下用MID360配置FAST-LIO2备忘](https://blog.csdn.net/qq_52784762/article/details/132736322?ops_request_misc=&request_id=&biz_id=102&utm_term=fast%20lio%E9%85%8D%E7%BD%AE&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-132736322.142^v96^pc_search_result_base9&spm=1018.2226.3001.4187)  

```bash
sudo apt install libeigen3-dev
sudo apt install libpcl-dev
```

```bash
# ros2需要安装
sudo apt install ros-humble-pcl-ros
```

``` bash
# 编译fast-lio
#这里进入的是你自己的工作空间的src，注意替换$A_ROS_DIR$
cd ~/$A_ROS_DIR$/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd ../..
catkin_make
source devel/setup.bash
# 注意，如果使用的是mid360，即使用的是livox_ros_driver2而非1的话，
# 需要前往fast-lio的CmakeLists文件修改find_package里的livox_ros_driver为livox_ros_driver2，同时package.xml里面的也一样，对应的src中的cpp文件也需要修改
```

``` bash
# 安装sophus
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build
cd build
cmake ../ -DUSE_BASIC_LOGGING=ON
make
sudo make install
```
上述步骤可能会报错,[解决方案](https://blog.csdn.net/DerrickRose25/article/details/130173310?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169737303816800215088736%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=169737303816800215088736&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-8-130173310-null-null.142^v96^pc_search_result_base9&utm_term=lvalue%20required%20as%20left%20operand%20of%20assignment%20%20%20%20unit_complex_.real%28%29%20%3D%201.%3B&spm=1018.2226.3001.4187)
``` bash
/home/lau/Sophus/sophus/so2.cpp:32:26: error: lvalue required as left operand of assignment
   unit_complex_.real() = 1.;
                          ^~
/home/lau/Sophus/sophus/so2.cpp:33:26: error: lvalue required as left operand of assignment
```
打开其位置so2.cpp:32:26改为
``` bash
SO2::SO2()
{
  unit_complex_.real(1.);
  unit_complex_.imag(0.);
}
```
sophus安装成功后再重新编译fast-lio

最后运行
``` bash
# 进入livox_ros_driver2所在的工作空间
cd ~/$A_ROS_DIR$
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
#再开一个终端，进入fast_lio所在的工作空间
cd ~/$A_ROS_DIR$
source devel/setup.bash
roslaunch fast_lio mapping_mid360.launch

```
<div align="center"><img src="doc/fast_lio_1.png" width=90% /></div>
<div align="center"><img src="doc/fast_lio_2.png" width=90% /></div>
<div align="center"> Fast_Lio建图效果，自动存于PCD目录</div>


## 导航流程
入门简介：
- [ROS入门(九)——机器人自动导航](https://blog.csdn.net/Netceor/article/details/118997851?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169779395316800215096913%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169779395316800215096913&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-118997851-null-null.142^v96^pc_search_result_base9&utm_term=ros%E5%AF%BC%E8%88%AA%E6%B5%81%E7%A8%8B&spm=1018.2226.3001.4187)

- [带你理清：ROS机器人导航功能实现、解析、以及参数说明](https://blog.csdn.net/qq_42406643/article/details/118754093)

---
### 1.Relocalization
完成建图后，在导航中，对于机器人的定位操作，有如下两种方式：
- 方式一：假设初始位置始终不变，使用fast_lio的估计里程计进行机器人定位。劣势：1.每次运行需要初始位置一致，否则引入初始偏移误差；2.里程计会产生累计误差。
- 方式二：使用地图进行重定位，通过当前雷达的点云和已构建的地图匹配来找到机器人位姿。重定位方式一般会要求手动设置初始位置估计，也可结合机器人其他传感器进行粗略的初始位置估计。  

为了避免累计误差，采用方式二进行定位：[FAST_LIO_LOCALIZATION](https://github.com/davidakhihiero/FAST_LIO_LOCALIZATION-ROS-NOETIC)

```bash
# 所需包，适配python3.8
sudo apt install ros-$ROS_DISTRO-ros-numpy
pip install numpy==1.21
pip install open3d
```
 
原仓库[FAST_LIO_LOCALIZATION](https://github.com/davidakhihiero/FAST_LIO_LOCALIZATION-ROS-NOETIC)使用的是python2，本仓库替换为python3。  


为了适配本仓库，对[FAST_LIO_LOCALIZATION](https://github.com/davidakhihiero/FAST_LIO_LOCALIZATION-ROS-NOETIC)的修改内容: 
- `global_localization.py`  
  - `#!/usr/bin/python3` 解释器修改为python3
  - `*import _thread*`, python3中使用_thread
  - 在open3d的最新版本, `o3d.registration`应被替换为 `o3d.pipelines.registration`
  - `FOV = 6.28` in 222 line 应该改成实际雷达的扫描范围. 对于MID360范围为360度, 即 2*pi (rad)
  - FOV_FAR = 30, 更改为实际雷达的最远距离

- `localization_MID360.launch`
  - `fastlio_mapping`, 使用fast_lio2中的mid360的launch文件启动
  - `args="$(arg map) 5 _frame_id:=map cloud_pcd:=map" />` in line 28, 匹配tf树
  - `<arg name="map" default="/home/rm/ws_sentry/src/FAST_LIO/PCD/scans.pcd" />`, 更换为先验地图保存路径

Usage:
```bash 
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio_localization localization_MID360.launch 
# 发布初始位姿(或者使用rviz手动发布)
rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0

```

---

### 2. 地图转换（PCD to 二维栅格地图）
<div align="center"><img src="doc/sentry_navigation.png" width=90% /></div>
<div align="center">红色点云为从PCD文件加载的先验三维点云地图，黑白地图为稠密二维栅格地图，绿色点云为当前lidar实时点云</div>
<br>


地图转换：因为move_base是基于2d的栅格地图进行路径规划，而fast_lio默认的输出地图是三维点云的PCD文件，需转换为2d的栅格地图，有以下几种方式：
  1. 用fast_lio构建好PCD地图后，将PCD地图转换为栅格地图  
    方式一：使用[pcd_package](https://github.com/Hinson-A/pcd2pgm_package)开源功能包，参考[离线将PCD地图转换为pgm栅格地图](https://blog.csdn.net/Draonly/article/details/124537069?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165207936116781435426048%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=165207936116781435426048&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-6-124537069-null-null.142%5Ev9%5Econtrol,157%5Ev4%5Econtrol&utm_term=pcd%E5%9C%B0%E5%9B%BE%E8%BD%AC%E6%8D%A2%E4%B8%BA%E6%A0%85%E6%A0%BC%E5%9C%B0%E5%9B%BE&spm=1018.2226.3001.4187)  
    方式二：使用`octomap_server`功能包,离线将pcd转换成栅格地图，参考[octomap_server使用－－生成二维占据栅格地图和三维概率地图](https://blog.csdn.net/sru_alo/article/details/85083030?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169804282616800213031883%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169804282616800213031883&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-85083030-null-null.142^v96^pc_search_result_base9&utm_term=%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E7%94%9F%E6%88%90%E6%A0%85%E6%A0%BC%E5%9C%B0%E5%9B%BE&spm=1018.2226.3001.4187)  
  2. 在fast_lio构建三维点云地图的同时，也实时构建2d的栅格地图

本文的代码仓库里两种方式都有，常用的是第二种
#### 配置：
```bash
sudo apt install ros-noetic-map-server
# 打开一个终端.(ctrl+alt+T)输入下面指令安装octomap.
sudo apt-get install ros-noetic-octomap-ros #安装octomap
sudo apt-get install ros-noetic-octomap-msgs
sudo apt-get install ros-noetic-octomap-server
 
# 安装octomap 在 rviz 中的插件
sudo apt-get install ros-noetic-octomap-rviz-plugins
# install move_base
sudo apt-get install ros-noetic-move-base

#如果使用方式一，还需将pcd2pgm拉到工作空间的src目录下编译
#本文代码仓库已经包含了该仓库，再sentry_tools/pcd2pgm，如果直接使用本文代码仓库，则不需要再拉
git clone https://github.com/Hinson-A/pcd2pgm_package.git
```
#### 方式一实现
```bash
# pcd2pgm offline
# 修改 pcd2pgm中的run.launch文件，修改输入的pcd文件路径等
roslaunch pcd2pgm run.launch
```
#### 方式二实现
使用`octomap_server`功能包中的`octomap_server_node`节点, 实时读取三维点云, 并生成栅格地图

我们在 `FAST_LIO` 功能包中添加了 `Pointcloud2Map.launch`, 建立3维点云地图的同时也构建2d栅格地图

我们在`sentry_build.launch`集成了`SLAM`, `实时构建栅格地图`，运行此启动文件后，系统便会开始自动同步构建栅格地图.


如果你对你构建的三维点云地图和二维栅格地图满意，并希望保存下来：
  1. 三维点云地图pcd文件会在`sentry_build.launch`运行结束后自动保存到fast_lio/PCD文件夹下
  2. 如果你希望保存二位栅格地图，请运行以下命令：


```bash
# save the pgm map file
rosrun map_server map_saver map:=/<Map Topic> -f PATH_TO_YOUR_FILE/mymap
#eg，举例:
rosrun map_server map_saver map:=/projected_map -f /home/rm/ws_sentry/src/FAST_LIO/PCD/scans
```
---

### 3. Pointcloud2 to Lasercan
<div align="center"><img src="doc/sentry_navigation.png" width=90% /></div>
<div align="center">绿点表示的是实时的雷达2维点云，用于实时避障</div>
<br>
在move_base导航框架下，构建局部代价地图，需要输入当前的laserscan的实时二位点云。 FAST_LIO的输出形式为3d点云 `/pointclouds2`. 然而`move_base`的点云输入是 `/Laserscan`. 因此需将`/pointclouds2` 转换成 `/Laserscan`. 


基于[pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan.git)。  

有关这个包的博客 `pointcloud_to_laserscan` : [pointcloud_to_laserscan_blog](https://blog.csdn.net/qq_43176116/article/details/86095482?ops_request_misc=&request_id=&biz_id=102&utm_term=pointcloud2%20to%20laserscan&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-2-86095482.142^v96^pc_search_result_base2&spm=1018.2226.3001.4187)

启动文件为`PointsCloud2toLaserscan.launch`

---
### 4. 坐标系映射

input：`body` frame，即机器人在三维点云坐标系下的位姿  
output: `body_2d` frame，即机器人在二维栅格地图坐标系下的位姿

由于`fast_lio_localization`输出的 `body` frame是当前机器人在三维点云坐标系下的位姿，而`move_base`需要的`map` frame是二维栅格地图坐标系下的坐标`body_2d`，因此需要进行坐标系转换。

此处需要根据你的雷达安装方式进行相应的转换。如：你的雷达是正向安装，或者正向稍微倾斜一点安装，即mid360的底部始终指向地面。这种情况下，对于坐标系的位置，只需要把body frame中的(x,y,z)取(x,y,0)赋给body_2d即可。同时，对于坐标系的姿态，把body frame 中的四元数(x,y,z,w)取(0,0,z,w)赋给body_2d即可。

这样最后的效果就是，3d中的机器人坐标系映射到二维`body_2d`，位置信息z始终为0，而位置信息x，y始终跟随`body` frame同步。同时姿态信息只有yaw轴会跟着3d中的机器人同步。这样坐标系`body_2d`的xy轴就可以始终贴合地图，便于路径规划。

如果你的机器人将mid360反装，即底部指向天空，你需要修改tf的转换

此处的代码请见: `\sentry_nav\src\Trans_TF_2d.cpp`  
修改tf发布和订阅教程: [tf/Tutorials](http://wiki.ros.org/tf/Tutorials)


---
### 5. MOVE_BASE 避障和路径规划

此仓库采用开源的成熟框架move_base进行路径规划，避障。

#### `input`（至少需要这些信息）: 
- `/Laserscans`, 机器人坐标系的2维点云数据，
- `/tf`, tf中需要至少包含以下坐标系
  - `/map`, 二维栅格地图坐标系, 在我的源码中名字为`map`
  - `/odom`, 机器人里程计坐标系, 在我的源码中名字为`camera_init`
  - `/base_link`, 机器人坐标系, 在我的源码中名字为`body_2d`
  - PS: 如何看自己的坐标系是否正确呢？终端输入rqt，打开插件中的tf tree就能看到
- `/map`,  这个/map不是坐标系信息，而是使用map_server发布的栅格地图信息
  - 在我的代码中，调用map_server写在fast_lio_localization包中的sentry_localize.launch 文件
- `move_base_simple/goal`， 机器人希望到达的位置，可以使用rviz的红色箭头发布
- `/odom`, 里程计信息
  - 在我的代码中, 在运行sentry_localize.launch文件时，会自动运行fast_lio发布里程计信息

#### `output`:(这个比较多，发布的东西很全，挑几个重点关注的)
- 发布`/cmd_vel`话题，控制机器人运动
  - 如果你是第一次使用ros，这个信息需要重点关注，因为输出的就是xyz方向的线速度和绕xyz轴旋转的角速度，有了这个消息就可以用控制机器人导航了，检查cmd_vel时，建议使用rqt中的波形图，检查是否正确
- 发布 global path 和local_path
- 发布 cost map, 即代价地图
<div align="center"><img src="doc/sentry_navigation_1.gif" width=60% /></div>
<div align="center">实机测试</div>
<br>

#### 具体实现
其他信息不再过多阐述，建议参考[move_base官方wiki](http://wiki.ros.org/move_base) |  [dwa_local_planner官方wiki](http://wiki.ros.org/dwa_local_planner) | [autolabor的ros教程(导航实现04_路径规划)](http://www.autolabor.com.cn/book/ROSTutorials/di-7-zhang-ji-qi-ren-dao-822a28-fang-771f29/72-dao-hang-shi-xian/724-dao-hang-shi-xian-04-lu-jing-gui-hua.html)
本文有关move_base的相关参数设置和代码请见 `Sentry_Nav`功能包

  1. build the map  构建地图

     - `roslaunch roslaunch livox_ros_driver2 msg_MID360.launch`
     - `roslaunch fast_lio_localization sentry_build_map.launch`
     - 运行 `rosrun map_server map_saver map:=/projected_map -f /home/rm/ws_sentry/src/sentry_slam/FAST_LIO/PCD/scans`, 来保存栅格地图，三维点云的PCD会在运行结束后自动保存到相同路径
  2. navigation 导航
     - 检查2d地图, 尤其 `scans.yaml`, 确认 `origin`[x,y,yaw]值不包含 nan，或修改为0.   
     - `roslaunch roslaunch livox_ros_driver2 msg_MID360.launch`
     - `roslaunch fast_lio_localization sentry_localize.launch`
     - 发布估计初始位置 `rviz` 或者 `rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0`
     - `roslaunch sentry_nav sentry_movebase.launch `
     - 发布目标点 `rviz` 

     使用`rqt`来检查cmd_vel，在ros中，红轴代表x轴，绿色的是y轴，蓝色的是z轴。当角速度大于0时，表示机器人应“逆时针旋转”，当角速度小于0时，表示机器人应“顺时针旋转”。

### Adjusting param
- 为何机器人的速度无法达到上限
  - [Reference1](https://answers.ros.org/question/12066/move_basebase_local_planner-maximum-velocity/)
  - [Referebce2](https://answers.ros.org/question/297226/velocity-doesnt-increase-when-using-move-base-navigation/)
  - [Reference3](https://answers.ros.org/question/267293/navigation-cant-reach-max-speed/)

## Serial and Decision
导航层与底层控制层使用串口通讯


### 1. Serial
导航开始后，系统会生成路径和cmd_vel的话题用于控制机器人移动。

本文使用的是虚拟串口发送给下位机相关的数据。
[ros串口通讯](https://blog.csdn.net/qq_43525704/article/details/103363414?ops_request_misc=&request_id=&biz_id=102&utm_term=ros%20chuan%20kou&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-0-103363414.nonecase&spm=1018.2226.3001.4187)

设置串口全县
```bash
sudo usermod -aG dialout $USER
# USRE is your username
# eg:
sudo usermod -aG dialout lau
```

#### 1. 订阅 `cmd_vel`

#### 2. 调用serial.write发送数据

#### 3. 具体实现在 /sentry_comm/sentry_serial/src/serial_send.cpp

```bash 
rosrun sentry_serial sentry_send <serial port path>

#eg:

rosrun sentry_serial sentry_send /dev/ttyACM0
#默认串口为 /dev/ttyACM0

```

## 运行命令
导航系统运行的最终的命令为:

  1. 构建地图
```bash
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio_localization sentry_build_map.launch
rosrun map_server map_saver map:=/projected_map -f /home/rm/ws_sentry/src/sentry_slam/FAST_LIO/PCD/scans
```



  2. 导航(重定位)
```bash
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio_localization sentry_localize.launch
# 用rviz发布初始位姿或者 `rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0`
roslaunch sentry_nav sentry_movebase.launch
# 用rviz发布目标点
roslaunch sentry_serial sentry_serial.launch
```

  3.导航(里程计定位)
```bash
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio_localization sentry_localize_odom.launch
# 用rviz发布初始位姿或者 `rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0`
roslaunch sentry_nav sentry_movebase.launch
# 用rviz发布目标点
roslaunch sentry_serial sentry_serial.launch
```

如果直接运行发生报错，注意报错信息等，大多是路径问题
<div align="center"><img src="doc/rosgraph-11-10.png" width=100% /></div>
<div align="center">ROS Node Graph</div>
<br>

<div align="center"><img src="doc/topic-11-10-up.png" width=100% /></div>

<div align="center"><img src="doc/topic-11-10-down.png" width=100% /></div>
<div align="center">TOPIC</div>
<br>

<div align="center"><img src="doc/tftree_nav.png" width=90% /></div>
<div align="center">TF_tree</div>
<br>
这里对于tf树做一点说明

 - map->camera_init：tf转换由重定位icp发布，如果直接使用里程计定位，则是由static_transform_publisher静态发布。
 - cmera_init->robot_foot_init：由static_transform_publisher静态发布，这样作用是mid360到机器人足端的转换，robot_foot_init意味着机器人足端的初始位姿
 - camera_init->body：由fast_lio的里程计信息发布，就是雷达的初始位姿，到雷达当前位姿的关系
 - body->body_foot：主要作用是将雷达的位姿转换到机器人足端，也是静态发布
 - map->body_2d：由Trans_body_2d.cpp发布，主要作用是将body投影到2dmap上

## 后续优化或修改
  以上内容作为搭建机器人导航的初始入门。得益于ROS不同功能包之间的良好的解耦，后续可以针对上面slam部分，避障部分，路径规划部分独立修改并优化，后续的优化或修改，可以参考以下内容：

### 更换局部规划器为dwa，同时使cmd_vel输出全向移动机器人的y方向速度而不是使用默认的yaw

### 使用ema滤波算法平滑move_base的输出/cmd_vel.
由于move_base的输出/cmd_vel有速度的突变，且控制频率不高的条件下，会导致机器人运动卡顿，为平滑机器人的运动，我们使用`velocity_smoother_ema`包(基于ema算法)对于输出的速度进行平滑处理，此处也可以使用`yocs_velocity_smoother`.
```bash
git clone https://github.com/seifEddy/velocity_smoother_ema.git

```
velocity_smoother_ema的启动已经添加至sentry_movebase.launch file，并且现在串口订阅的是滤波后的速度即`/smooth_cmd_cel`
<div align="center"><img src="doc/Filter-11-10.png" width=100% /></div>
<div align="center">EMA FILTER</div>
<br>


## F&Q
### 1. 如何确保栅格地图和三维点云地图处于完全重合的状态

采用时候fast_lio构建三维点云地图的同时，将点云数据用octomap压至二维地图,同时构建的地图可以确保relocalize在三维点云中的机器人位姿可以完全映射到二位栅格地图中使用

  2023-12-21更新：

  由于机械结构设计的改变，机器人的雷达安装位置已经改为倒置，修改内容较多在分支[invert_lidar](https://github.com/66Lau/NEXTE_Sentry_Nav/tree/invert_lidar)。




