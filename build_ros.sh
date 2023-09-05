echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM3
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
# 修改线程数
# make -j4
make -j16
