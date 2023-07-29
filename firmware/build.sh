cd ../3rdparty/ConfiguringIP/version
chmod +x build.sh
./build.sh
cd ../firmware/
mkdir build
cd build
cmake ..
make -j4

cd ../../../../version
chmod +x build.sh
./build.sh
cd ../firmware/
mkdir build
cd build
cp ../../3rdparty/ConfiguringIP/firmware/build/configuring_ip ./
cp ../configuring_ip_info.cfg ./
cp ../../3rdparty/linux-opencv-4.4.0/lib/* ./
cmake ..
make -j4