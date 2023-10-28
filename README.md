# SLAM AND NAVIGATION IN 2024

## Hardware info
- [MID360 offical web](https://www.livoxtech.com/cn/mid-360)
- [Quick-start-doc](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/Livox_Mid-360_Quick_Start_Guide_multi.pdf)
- [user-manual](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/20230727/Livox_Mid-360_User_Manual_CHS.pdf)
- [Livox_sdk2](https://github.com/Livox-SDK/Livox-SDK2)
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)

- [livox ros driver2安装](https://blog.csdn.net/qq_29912325/article/details/130269367?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169734904416800182711632%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169734904416800182711632&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-130269367-null-null.142^v96^pc_search_result_base9&utm_term=livox_sdk2&spm=1018.2226.3001.4187)

- [虚拟机和mid360桥接](https://blog.csdn.net/sinat_39110395/article/details/123545816?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169735401816800227447255%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169735401816800227447255&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-123545816-null-null.142^v96^pc_search_result_base9&utm_term=%E8%99%9A%E6%8B%9F%E6%9C%BA%E8%BF%9E%E6%8E%A5%E9%9B%B7%E8%BE%BE&spm=1018.2226.3001.4187)
- [关于在ROS1下用MID360配置FAST-LIO2备忘](https://blog.csdn.net/qq_52784762/article/details/132736322?ops_request_misc=&request_id=&biz_id=102&utm_term=fast%20lio%E9%85%8D%E7%BD%AE&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-132736322.142^v96^pc_search_result_base9&spm=1018.2226.3001.4187)
- [FAST-LIO配置](https://blog.csdn.net/qq_42108414/article/details/131530293?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169737102216800185825796%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169737102216800185825796&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-131530293-null-null.142^v96^pc_search_result_base9&utm_term=fast%20lio%E9%85%8D%E7%BD%AE&spm=1018.2226.3001.4187)


## Livox和Fast-Lio配置流程
1. 安装[Livox_sdk2](https://github.com/Livox-SDK/Livox-SDK2),readme有写相关过程，注意：要更改主机ip为192.168.1.50[ubuntu修改方法](https://blog.csdn.net/sinat_39110395/article/details/123545816?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169735401816800227447255%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169735401816800227447255&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-123545816-null-null.142^v96^pc_search_result_base9&utm_term=%E8%99%9A%E6%8B%9F%E6%9C%BA%E8%BF%9E%E6%8E%A5%E9%9B%B7%E8%BE%BE&spm=1018.2226.3001.4187)，本人雷达ip为192.168.1.180
2. 安装[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2),注意运行前要注意更改config里面的主机ip和雷达IP
3. 配置fast-lio
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
# 注意sfast版本里没有mid360launch，但是原版有，修改的时候记得把文件中的fast_lio改为sfast_lio
```


## 导航流程
[ROS入门(九)——机器人自动导航](https://blog.csdn.net/Netceor/article/details/118997851?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169779395316800215096913%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169779395316800215096913&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-118997851-null-null.142^v96^pc_search_result_base9&utm_term=ros%E5%AF%BC%E8%88%AA%E6%B5%81%E7%A8%8B&spm=1018.2226.3001.4187)

[带你理清：ROS机器人导航功能实现、解析、以及参数说明](https://blog.csdn.net/qq_42406643/article/details/118754093)

---
### 1.Relocalization
[重定位fast-lio-localization](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)


我们修改了开源的重定位项目 [FAST_LIO_LOCALIZATION](https://github.com/davidakhihiero/FAST_LIO_LOCALIZATION-ROS-NOETIC), 包括以下文件: 
- `global_localization.py`  
  - `#!/usr/bin/python3` 此处我们修改解释器为python3
  - `*import _thread*`, python3中使用thread会报错，已经改名为_thread
  - 在open3d的最新版本, `o3d.registration`应被替换为 `o3d.pipelines.registration`
  - `FOV = 6.28` in 222 line 应该改成你使用的雷达的扫描范围. The scale of MID360 is 360, so 2*pi (rad)
  - FOV_FAR = 30, switch to you lidar max distance

- `localization_MID360.launch`
  - 我们修改了 `fastlio_mapping` 可执行文件的所属包，我们直接使用fast_lio2中的mid360的launch文件启动
  - 使用 `args="$(arg map) 5 _frame_id:=map cloud_pcd:=map" />` in line 28，而不是/map
  - modified to `<arg name="map" default="/home/rm/ws_sentry/src/FAST_LIO/PCD/scans.pcd" />`, that used the PCD file in FAST_LIO pkg, If you have your own PCD file, you can change it to your own PCD file path.

Usage:
```bash 
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio_localization localization_MID360.launch 
rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0
# also you could publish your initial point use RVIZ
# the origin point of the PCD map is the location where you launch the fast_lio mapping
# Therefore, You better launch the relocalization in the location where you start the mapping. In this way, you can just publish 0 0 0 0 0 0 to estimate the initail pose
```

---

### 2. 地图转换（PCD to 二位栅格地图）
地图转换主要是因为move_base是基于2d的栅格地图进行路径规划，而fast_lio默认的输出地图是三维点云的PCD文件，我们需要用一些方法获取2d的栅格地图，有以下几种方式：
  1. 用fast_lio构建好PCD地图后，将PCD地图转换为栅格地图  
    方式一：使用[pcd_package](https://github.com/Hinson-A/pcd2pgm_package)开源功能包，参考[离线将PCD地图转换为pgm栅格地图](https://blog.csdn.net/Draonly/article/details/124537069?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165207936116781435426048%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=165207936116781435426048&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-6-124537069-null-null.142%5Ev9%5Econtrol,157%5Ev4%5Econtrol&utm_term=pcd%E5%9C%B0%E5%9B%BE%E8%BD%AC%E6%8D%A2%E4%B8%BA%E6%A0%85%E6%A0%BC%E5%9C%B0%E5%9B%BE&spm=1018.2226.3001.4187)  
    方式二：使用`octomap_server`功能包,离线将pcd转换成栅格地图，参考[octomap_server使用－－生成二维占据栅格地图和三维概率地图](https://blog.csdn.net/sru_alo/article/details/85083030?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169804282616800213031883%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169804282616800213031883&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-85083030-null-null.142^v96^pc_search_result_base9&utm_term=%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E7%94%9F%E6%88%90%E6%A0%85%E6%A0%BC%E5%9C%B0%E5%9B%BE&spm=1018.2226.3001.4187)  
  2. 在fast_lio构建三维点云地图的同时，也实时构建2d的栅格地图



[实时显示octomap](https://blog.csdn.net/lovely_yoshino/article/details/105275396?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169804282616800213031883%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169804282616800213031883&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-105275396-null-null.142^v96^pc_search_result_base9&utm_term=%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E7%94%9F%E6%88%90%E6%A0%85%E6%A0%BC%E5%9C%B0%E5%9B%BE&spm=1018.2226.3001.4187)  

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
move_base框架下，我们构建局部代价地图时，需要输入当前的laserscan的实时二位点云

The output format of 3d point clouds of FAST_LIO is `/pointclouds2`. However, the input format of `move_base` is `/Laserscan`. Therefore, it is necessary to transfrom the `/pointclouds2` to `/Laserscan`.

The package we are using is [pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan.git).

blog about `pointcloud_to_laserscan` : [pointcloud_to_laserscan_blog](https://blog.csdn.net/qq_43176116/article/details/86095482?ops_request_misc=&request_id=&biz_id=102&utm_term=pointcloud2%20to%20laserscan&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-2-86095482.142^v96^pc_search_result_base2&spm=1018.2226.3001.4187)

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

#### 具体实现
其他信息不再过多阐述，建议参考[move_base官方wiki](http://wiki.ros.org/move_base) |  [dwa_local_planner官方wiki](http://wiki.ros.org/dwa_local_planner) | [autolabor的ros教程(导航实现04_路径规划)](http://www.autolabor.com.cn/book/ROSTutorials/di-7-zhang-ji-qi-ren-dao-822a28-fang-771f29/72-dao-hang-shi-xian/724-dao-hang-shi-xian-04-lu-jing-gui-hua.html)
本文有关move_base的相关参数设置和代码请见 `Sentry_Nav`功能包

  1. build the map  构建地图

     - `roslaunch roslaunch livox_ros_driver2 msg_MID360.launch`
     - `roslaunch fast_lio_localization sentry_build_map.launch`
     - 如果你认为当前构建的栅格地图还可以,运行 `rosrun map_server map_saver map:=/projected_map -f /home/rm/ws_sentry/src/sentry_slam/FAST_LIO/PCD/scans`, 来保存栅格地图，注意，三维点云的PCD是运行结束后自动保存到在launch file中指定的路径下的
  2. navigation 导航
     - check the 2d map in PCD dir, especially the `scans.yaml`, make sure the `origin`[x,y,yaw] can not be nan. | 检查在fast_lio/PCD下中保存的2d地图`scans.yaml`,确保其中参数`origin`[x,y,yaw]不能是nan，如果yaw是nan的话，将其设置为0.
     - `roslaunch roslaunch livox_ros_driver2 msg_MID360.launch`
     - `roslaunch fast_lio_localization sentry_localize.launch`
     - publish the initial pose by using `rviz` or `rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0`
     - `roslaunch sentry_nav sentry_movebase.launch `
     - publish the goal point through `rviz`

     - using `rqt` to check the cmd_vel, in ros, the read axis delegate the x axis, the green one is the y axis, the blue one is the z axis. Besides, when the `angular velocity` bigger than `0`, it means that the robot should `rotate anticlockwise`, and when the angular velocity smaller than 0, it means that the robot should rotate clockwise.


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

## 后续优化或修改
  上面的内容可以作为导航系统的雏形，或者说是初学者的快速入门。得益于ROS不同功能包之间的良好的解耦，后续可以针对上面slam部分，避障部分，路径规划部分独立修改并优化，后续的优化或修改，可以参考以下内容：

## F&Q
1. 如何确保栅格地图和三维点云地图处于完全重合的状态
    - 采用时候fast_lio构建三维点云地图的同时，将点云数据用octomap压至二维地图,同时构建的地图可以确保relocalize在三维点云中的机器人位姿可以完全映射到二位栅格地图中使用

## TODO
- 更换局部规划器为dwa，同时使cmd_vel输出全向移动机器人的y方向速度而不是使用默认的yaw-2023-10-28


