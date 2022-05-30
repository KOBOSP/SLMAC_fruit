#echo "Configuring and building Thirdparty/DBoW2 ..."
#cd ./../ORB3Thirdparty/DBoW2
#mkdir build
#cd build
#cmake .. -DCMAKE_BUILD_TYPE=Release
#make -j
#cd ../../../ORB3_Vd1
#
#
#
#echo "Configuring and building Thirdparty/g2o ..."
#cd ./../ORB3Thirdparty/g2o
#mkdir build
#cd build
#cmake .. -DCMAKE_BUILD_TYPE=Release
#make -j
#cd ../../../ORB3_Vd1
#
#
#
#echo "Configuring and building Thirdparty/Sophus ..."
#cd ./../ORB3Thirdparty/Sophus
#mkdir build
#cd build
#cmake .. -DCMAKE_BUILD_TYPE=Release
#make -j
#cd ../../../ORB3_Vd1
#
#
#echo "Uncompress vocabulary ..."
#cd ./../ORB3Vocabulary
#tar -xf ORBvoc.txt.tar.gz
#cd ../ORB3_Vd1


echo "Configuring and building ORB_SLAM3 ..."
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j7

