# SLAM AND NAVIGATION IN 2023

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

### 1.Relocalization
[重定位fast-lio-localization](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)


We modified the open-source project [FAST_LIO_LOCALIZATION](https://github.com/davidakhihiero/FAST_LIO_LOCALIZATION-ROS-NOETIC), including: 
- `global_localization.py`  
  - `#!/usr/bin/python3` in 1 line, switch from python2 to python3
  - `*import _thread*`, the thread are renamed as _thread in python3
  - In the newest version of Open3D, `o3d.registration`should be replaced by  `o3d.pipelines.registration`
  - `FOV = 6.28` in 222 line should be changed to your lidar Fov scale. The scale of MID360 is 360, so 2*pi (rad)
  - FOV_FAR = 30, switch to you lidar max distance

- `localization_MID360.launch`
  - It calls the `fastlio_mapping` from the newest fast_lio2 pkg which fit with the MID360
  - modified to `args="$(arg map) 5 _frame_id:=map cloud_pcd:=map" />` in line 28
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
