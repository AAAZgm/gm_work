
## Installation Steps (安装步骤)

unzip xxx.zip

cd xxx

sudo python3 setup.py install


sudo pip3 install pyserial
sudo pip3 install smbus2
记得加rviz2imu插件
sudo apt install ros-humble-rviz-imu-plugin
/imu/data
✅ 滤波后的 IMU 数据（带姿态估计）
/imu/data_raw
✅ 原始 IMU 数据
/imu/mag
✅ 三轴磁力计数据，算出朝向
/euler
✅ 欧拉角
/baro
✅ 气压计数据，算高度