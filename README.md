# SLAM AND NAVIGATION IN RM2024
## Introduction
本文原本只是自己在拿到mid360后，开始进行开发过程的一些问题和学习的记录。毕竟实物和仿真还是有很多不同，且由于碰到的问题也比较多，READEME也越来越详细，所以就干脆整合起来，做成了一篇使用mid360的搭建入门的导航系统全流程分享。里面用到的都是主流的开源的框架（部分文件做了修改和mid360适配），fast_lio, move_base等等，或许能帮助到第一次开发机器人实物导航的朋友。

本文的代码地址：https://github.com/66Lau/NEXTE_Sentry_Nav

环境：
- ros-noetic 
- ubuntu 20.04  

你可以跟着下文步骤，逐一对clone开源仓库，再进行修改配置，但是建议直接直接克隆本仓库至你的工作空间的src下（因为做了很多修改，如果再clone我引用的原来的仓库，可能有些地方我没记在README里面导致你运行失败）然后再根据本文的顺序逐一进行配置和尝试

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
更建议参考源地址的READEME
```
sudo apt install libeigen3-dev
```
``` bash
sudo apt install libpcl-dev
```
```bash
# ros2需要安装
sudo apt install ros-humble-pcl-ros
```

``` bash
# 编译fast-lio
cd src
git clone https://github.com/zlwang7/S-FAST_LIO.git --recursive
cd ..
catkin_make
# 注意，如果使用的是mid360，即使用的是livox_ros_driver2而非1的话，
# 需要前往fast-lio的CmakeLists文件修改find_package里的livox_ros_driver为livox_ros_driver2，同时package.xml里面的也一样
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
``` bash
# 注意：laserMapping.cpp和laserMapping_re.cpp里面include的livox_ros_driver改为livox_ros_driver_v2
```
最后运行
``` bash
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
#再开一个终端
source devel/setup.bash
roslaunch fast_lio mapping_mid360.launch

```
<div align="center"><img src="doc/fast_lio_1.png" width=90% /></div>
<div align="center"><img src="doc/fast_lio_2.png" width=90% /></div>
<div align="center"> 以上就是使用fast_lio建好的图的效果，最后会自动存在PCD文件夹中</div>


## 导航流程
一些基础的入门介绍：
- [ROS入门(九)——机器人自动导航](https://blog.csdn.net/Netceor/article/details/118997851?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169779395316800215096913%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169779395316800215096913&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-118997851-null-null.142^v96^pc_search_result_base9&utm_term=ros%E5%AF%BC%E8%88%AA%E6%B5%81%E7%A8%8B&spm=1018.2226.3001.4187)

- [带你理清：ROS机器人导航功能实现、解析、以及参数说明](https://blog.csdn.net/qq_42406643/article/details/118754093)

---
### 1.Relocalization
上面的建图完成后，如果我们希望下次机器人导航的时候，能找到机器人自己的当前位置，那么就需要对机器人进行定位操作。也有如下几种方式：
- 方式一：直接在初始位置使用里程计定位，也就是说，如果能确保机器人每次的上电和程序运行的初始位置始终一至，那么就可以使用fast_lio的里程计进行机器人定位。但这样的劣势很明显：1.机器人需要确保自己每次的初始位置一致，否则就会有误差；2.机器人在移动时，里程计会产生累计误差，无法消除。
- 方式二：使用地图进行重定位，简单来说，就是通过当前雷达的点云和已构建的地图进行匹配来找到机器人当前的位姿。一般二维地图采用amcl进行重定位，三维点云采用icp进行重定位，当然，也有一些新的论文会提出新的更好的重定位定位方法，这些优化的算法暂不讨论。这种重定位方式一般会要求人给一个大概的初始位置，又或者可以依靠机器人的其他传感器给出一个粗略的初始位置估计。  

考虑到避免累计误差，我们采用方式二进行定位。我们选用了一个开源的重定位代码(实际采用的也是icp，使用python写的，考虑运行速度的话可以自行使用c++实现或者使用其他icp重定位代码)：[FAST_LIO_LOCALIZATION](https://github.com/davidakhihiero/FAST_LIO_LOCALIZATION-ROS-NOETIC)

```bash
# 所需包
sudo apt install ros-$ROS_DISTRO-ros-numpy
pip install numpy==1.21
pip install open3d
```
目前这套适用python3.8，如果你有多个python环境，自行修改哈。

在配置时，我们发现了一些问题，主要还是版本导致。  
一个是原仓库使用的是python2，python2使用和安装外部库已经不太方便了，所以换成了python3。  
另一个是FAST_LIO_LOCALIZATION里面也包含了FAST_LIO，但是这里面的FAST_LIO还是比较老的版本，建议还是从最新的FAST_LIO仓库里拉最新的(因为做了mid360的适配)，然后记得在cmakelist里修改生成的可执行文件的名称，否则会和上一步的fast_lio的mapping重复报错。  
所以想使用的话建议还是直接使用我们的`sentry_slam/FAST_LIO_LOCALIZATION`和`sentry_slam/FAST_LIO`这两个修改好的包。


同时记录一下我对[FAST_LIO_LOCALIZATION](https://github.com/davidakhihiero/FAST_LIO_LOCALIZATION-ROS-NOETIC)所做的修改，包括以下文件: 
- `global_localization.py`  
  - `#!/usr/bin/python3` 此处我们修改解释器为python3
  - `*import _thread*`, python3中使用thread会报错，已经改名为_thread
  - 在open3d的最新版本, `o3d.registration`应被替换为 `o3d.pipelines.registration`
  - `FOV = 6.28` in 222 line 应该改成你使用的雷达的扫描范围. The scale of MID360 is 360, so 2*pi (rad)
  - FOV_FAR = 30, switch to you lidar max distance

- `localization_MID360.launch`
  - 我们修改了 `fastlio_mapping` 可执行文件的所属包，我们直接使用fast_lio2中的mid360的launch文件启动
  - 使用 `args="$(arg map) 5 _frame_id:=map cloud_pcd:=map" />` in line 28，而不是`/map`,即和你自己的tf树一致
  - modified to `<arg name="map" default="/home/rm/ws_sentry/src/FAST_LIO/PCD/scans.pcd" />`, that used the PCD file in FAST_LIO pkg, If you have your own PCD file, you can change it to your own PCD file path.

Usage:
```bash 
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio_localization localization_MID360.launch 
# 发布初始位姿(也可以用rviz，第一次尝试的时候更建议使用rviz)
rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0
# also you could publish your initial point use RVIZ
# 这里的原点是你建图时候的起点。

```

---

### 2. 地图转换（PCD to 二维栅格地图）
<div align="center"><img src="doc/sentry_navigation.png" width=90% /></div>
<div align="center">这里面红色的点云就是PCD文件显示的三维点云地图，白色的就是熟知的二维栅格地图</div>
<br>


地图转换主要是因为move_base是基于2d的栅格地图进行路径规划，而fast_lio默认的输出地图是三维点云的PCD文件，我们需要用一些方法获取2d的栅格地图，有以下几种方式：
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
# modify run.launch file in pcd2pgm such as the pcd file pasth etc.
# 修改 pcd2pgm中的run.launch文件，修改输入的pcd文件路径等
roslaunch pcd2pgm run.launch
```
#### 方式二实现
使用`octomap_server`功能包中的`octomap_server_node`节点, 实时读取三维点云, 并生成栅格地图.

我们在 `FAST_LIO` 功能包中添加了 `Pointcloud2Map.launch`, which will update the 2D mapping at same time, if you publish the PointCloud2 from FAST_LIO. 

然后我们综合了 `SLAM`, `relocalization`, `实时构建栅格地图`三个功能, in only one launch file ==> `sentry_build.launch`.

运行此功能包后，系统便会开始自动同步构建栅格地图.

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
<div align="center">这里面栅格地图上的黑色像素块（障碍物或者地图边界）上面有一层绿色的小方格，那个就是实时的二维点云</div>
<br>
move_base框架下，我们构建局部代价地图时，需要输入当前的laserscan的实时二位点云

The output format of 3d point clouds of FAST_LIO is `/pointclouds2`. However, the input format of `move_base` is `/Laserscan`. Therefore, it is necessary to transfrom the `/pointclouds2` to `/Laserscan`.

我使用的包是[pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan.git).  
The package we are using is [pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan.git).

有关这个包的博客 `pointcloud_to_laserscan` :   
blog about `pointcloud_to_laserscan` : [pointcloud_to_laserscan_blog](https://blog.csdn.net/qq_43176116/article/details/86095482?ops_request_misc=&request_id=&biz_id=102&utm_term=pointcloud2%20to%20laserscan&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-2-86095482.142^v96^pc_search_result_base2&spm=1018.2226.3001.4187)

启动文件为`PointsCloud2toLaserscan.launch`
The launch file is `PointsCloud2toLaserscan.launch`

---
### 4. 坐标系映射

input：`body` frame，即机器人在三维点云坐标系下的位姿  
output: `body_2d` frame，即机器人在二维栅格地图坐标系下的位姿

由于`fast_lio_localization`输出的 `body` frame是当前机器人在三维点云坐标系下的位姿，而`move_base`需要的`map` frame是二维栅格地图坐标系下的坐标`body_2d`，因此需要进行坐标系转换。

此处需要根据你的雷达安装方式进行相应的转换。如：你的雷达是正向安装，或者正向稍微倾斜一点安装，即mid360的底部始终指向地面。这种情况下，对于坐标系的位置，只需要把body frame中的(x,y,z)取(x,y,0)赋给body_2d即可。同时，对于坐标系的姿态，把body frame 中的四元数(x,y,z,w)取(0,0,z,w)赋给body_2d即可。

这样最后的效果就是，3d中的机器人坐标系映射到二维`body_2d`，位置信息z始终为0，而位置信息x，y始终跟随`body` frame同步。同时姿态信息只有yaw轴会跟着3d中的机器人同步。这样坐标系`body_2d`的xy轴就可以始终贴合地图，便于路径规划。

如果你的机器人将mid360反装，即底部指向天空，你需要修改tf的转换

此处的代码请见: `\sentry_nav\src\Trans_TF_2d.cpp`  
如何修改tf发布和订阅请见: [tf/Tutorials](http://wiki.ros.org/tf/Tutorials)(ps: 没有找见很好的中文材料，且英文教程和源码比较官方详细)


---
### 5. MOVE_BASE 避障和路径规划

我们主要采用开源的成熟框架move_base进行路径规划，避障。  
主要是简单好用，资料丰富，对于小白来说比较合适。  
自然也会有一些不足之处，暂时只是用move_base进行2d的避障和路径规划。如果需要上坡，或者z轴方向上有移动，需要参考更加复杂的路径规划算法，后续找到了鲁棒的方案再更新（2023-10-28）。

#### `input`（至少需要这些信息）: 
- `/Laserscans`, 即机器人坐标系的2维点云数据，
  - 如何用mid360获取，见上面第三点, 如果你需要避障，则必须提供此信息或者`/PointCloud`
- `/tf`, 你的tf中需要至少包含以下坐标系
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
     - 如果你认为当前构建的栅格地图还可以,运行 `rosrun map_server map_saver map:=/projected_map -f /home/rm/ws_sentry/src/sentry_slam/FAST_LIO/PCD/scans`, 来保存栅格地图，注意，三维点云的PCD是运行结束后自动保存到在launch file中指定的路径下的
  2. navigation 导航
     - check the 2d map in PCD dir, especially the `scans.yaml`, make sure the `origin`[x,y,yaw] can not be nan.   
     检查在fast_lio/PCD下中保存的2d地图`scans.yaml`,确保其中参数`origin`[x,y,yaw]不能是nan，如果yaw是nan的话，将其设置为0.
     - `roslaunch roslaunch livox_ros_driver2 msg_MID360.launch`
     - `roslaunch fast_lio_localization sentry_localize.launch`
     - publish the initial pose by using `rviz` or `rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0`
     - `roslaunch sentry_nav sentry_movebase.launch `
     - 发布目标点 `rviz` | publish the goal point through `rviz`

     - using `rqt` to check the cmd_vel, in ros, the red axis delegate the x axis, the green one is the y axis, the blue one is the z axis. Besides, when the `angular velocity` bigger than `0`, it means that the robot should `rotate anticlockwise`, and when the angular velocity smaller than 0, it means that the robot should rotate clockwise.
     使用`rqt`来检查cmd_vel，在ros中，红轴代表x轴，绿色的是y轴，蓝色的是z轴。当角速度大于0时，表示机器人应“逆时针旋转”，当角速度小于0时，表示机器人应“顺时针旋转”。


## Serial and Decision
即机器人控制的决策层和通讯，这里采用的是串口


### 1. Serial
当你的导航部分完成后，系统理论会针对你给的目标点生成路径和cmd_vel的话题，至此，就可以控制机器人移动。

本文使用的是虚拟串口发送给下位机相关的数据，通讯协议是由我们自己定义的，仅供参考。
[Blog about Serial in ros](https://blog.csdn.net/qq_43525704/article/details/103363414?ops_request_misc=&request_id=&biz_id=102&utm_term=ros%20chuan%20kou&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-0-103363414.nonecase&spm=1018.2226.3001.4187)

Setting serial port permissions
```bash
sudo usermod -aG dialout $USER
# USRE is your username
# eg:
sudo usermod -aG dialout lau
```

#### 1. suscribe the `cmd_vel`

#### 2. use serial.write to send data

#### 3. For detail please see /sentry_comm/sentry_serial/src/serial_send.cpp

```bash 
rosrun sentry_serial sentry_send <serial port path>

#eg:

rosrun sentry_serial sentry_send /dev/ttyACM0
#the default seriial port path is /dev/ttyACM0, if you do not offer the param

```
### 2. Decision
决策层就是上位机通过机器人的设计需求和使用需求，决定机器人应该做什么，以及如何做。本导航系统的决策层的主要任务就是，控制机器人在某时刻，某地点，某事件中的目标位置选取。

正因为决策层是由机器人的使用需求决定的，而不同人的使用需求又大相径庭，此处只根据本项目的使用需求进行设计，即RoboMaster赛事的需求，仅供参考，若想使用至其他场景，请自行修改。

搭建实物场地中，所以决策层TBD。

## 运行命令
本导航系统运行的最终的命令为:

  1. build the map  构建地图
```bash
roslaunch roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio_localization sentry_build_map.launch
rosrun map_server map_saver map:=/projected_map -f /home/rm/ws_sentry/src/sentry_slam/FAST_LIO/PCD/scans
```



  2. navigation 导航
```bash
roslaunch roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio_localization sentry_localize.launch
# 用rviz发布初始位姿或者 `rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0`
roslaunch sentry_nav sentry_movebase.launch
# 用rviz发布目标点
rosrun sentry_serial sentry_send /dev/ttyACM0
```
由于文件中不可避免的会出现一些绝对路径的信息，还有诸如dev/ttyACM0这样应取决于你的硬件设备的相关文件，所以直接运行大概率会出问题，一般出问题后仔细查看报错，修改相关文件即可（这里因人而异，本章中无法做到非常详尽的指导，有过ros和c++开发经验应该很快能自己解决）

## 后续优化或修改
  上面的内容可以作为导航系统的雏形，或者说是初学者的快速入门。得益于ROS不同功能包之间的良好的解耦，后续可以针对上面slam部分，避障部分，路径规划部分独立修改并优化，后续的优化或修改，可以参考以下内容：

### 2023-10-28 更换局部规划器为dwa，同时使cmd_vel输出全向移动机器人的y方向速度而不是使用默认的yaw

### 2023-11-02 smooth the /cmd_vel, the output of the move_base, using ema formula.
```bash
git clone https://github.com/seifEddy/velocity_smoother_ema.git

```
the launch file of velocity_smoother_ema has been added to sentry_movebase.launch file
### 2023-11-02 make a little bit change to sentrial_serial.
- using rosparam for imparting the parameters
- now suscribe to the `/smooth_cmd_cel` topic
- add a launch file

## F&Q
1. 如何确保栅格地图和三维点云地图处于完全重合的状态
    - 采用时候fast_lio构建三维点云地图的同时，将点云数据用octomap压至二维地图,同时构建的地图可以确保relocalize在三维点云中的机器人位姿可以完全映射到二位栅格地图中使用

## TODO


