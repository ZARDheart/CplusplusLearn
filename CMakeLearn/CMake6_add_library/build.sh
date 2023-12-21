echo "-- PROJECT is building.-------------------------------------------- "

mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=~/Desktop/SLAM/LearnCMake/CMake6/install ..
make
make install
cd ..

echo "-- PROJECT is already builded.------------------------------------- "
