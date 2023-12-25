echo "Configuring and building Thirdparty/BDIS ..."

cd Thirdparty/OF_dis_bayesian
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j1

echo "Configuring and building Thirdparty/DBoW2 ..."

cd ../../DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j1

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j1

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j1
