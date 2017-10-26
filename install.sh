mkdir .build
cd .build
cmake ..
make
cd ..

sudo cp .build/main /usr/bin/off2pcd
sudo cp .build/main /usr/bin/X11/off2pcd
