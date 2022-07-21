echo "Building ROS nodes"

cd Examples/ORB_SLAM3ROS
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
#export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/kobosp/SLMAC/ORB3_Vd1/Examples