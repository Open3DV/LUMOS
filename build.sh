cd 3rdparty/ConfiguringIP
chmod +x build.sh
./build.sh

cd ../../
cp ./3rdparty/ConfiguringIP/enumerate/build/libenumerate.so ./example/
cp ./3rdparty/ConfiguringIP/enumerate/enumerate.h ./example/
cp ./3rdparty/ConfiguringIP/enumerate/build/libenumerate.so ./lumos_cpp_example/
cp ./3rdparty/ConfiguringIP/enumerate/enumerate.h ./lumos_cpp_example/

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

cd ../../lumos_cpp
rm -r build
mkdir build
cd build
cmake ..
make -j4
cp ./liblumos_sdk.so ../../lumos_cpp_example
cp ../lcamera.h ../../lumos_cpp_example

cd ../../lumos_cpp_example
cp ../firmware/configuring_ip_info.cfg ./
rm -r build
mkdir build
cd build
cmake ..
make -j4

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
mkdir cpp_example
cp ../lumos_cpp_example/* ./cpp_example/

mkdir lib
mkdir lib/C
cp ../example/laser_3d_cam.h ./lib/C
cp ../SDK/build/liblaser_3d_cam_sdk.so ./lib/C
cp ../example/libenumerate.so ./lib/C
cp ../example/enumerate.h ./lib/C
cp ../example/camera_status.h ./lib/C
cp ../firmware/configuring_ip_info.cfg ./lib/C

mkdir lib/CPP
cp ../lumos_cpp/lcamera.h ./lib/CPP
cp ../lumos_cpp/build/liblumos_sdk.so ./lib/CPP
cp ../example/libenumerate.so ./lib/CPP
cp ../example/enumerate.h ./lib/CPP
cp ../example/camera_status.h ./lib/CPP
cp ../firmware/configuring_ip_info.cfg ./lib/CPP
