cd 3rdparty/ConfiguringIP
chmod +x build.sh
./build.sh

cd ../../
cp ./3rdparty/ConfiguringIP/enumerate/build/libenumerate.so ./example/
cp ./3rdparty/ConfiguringIP/enumerate/enumerate.h ./example/

rm -r Release
mkdir Release
cd Release
mkdir GUI
cp ../3rdparty/ConfiguringIP/sdk/build/libconfiguring_network.so ./GUI/
cp ../3rdparty/ConfiguringIP/configuring_ip/build/configuring_ip ./GUI/
cp ../3rdparty/ConfiguringIP/configuring_network_gui/build/bin/configuring_ip_gui ./GUI/

cd ../version
chmod +x ./build.sh
./build.sh

cd ../SDK
rm -r build
mkdir build
cd build 
cmake ..
make -j4
cp liblaser_3d_cam_sdk.so ../../example

cd ../../example
cp ../firmware/configuring_ip_info.cfg ./
rm -r build
mkdir build
cd build 
cmake ..
make -j4

cd ../../cmd
rm -r build
mkdir build
cd build 
cmake ..
make -j4

cd ../../gui
rm -r build
mkdir build
cd build 
cmake ..
make -j4

cd ../../Release
cp ../firmware/configuring_ip_info.cfg ./GUI/
cp ../SDK/build/liblaser_3d_cam_sdk.so ./GUI/
cp ../gui/build/bin/* -r ./GUI/
cp ../cmd/build/lumos_cmd ./GUI/
mkdir example
cp ../example/* ./example/

mkdir lib
cp ../example/laser_3d_cam.h ./lib/
cp ../SDK/build/liblaser_3d_cam_sdk.so ./lib/
cp ../example/libenumerate.so ./lib/
cp ../example/enumerate.h ./lib/
cp ../example/camera_status.h ./lib/
cp ../firmware/configuring_ip_info.cfg ./lib/