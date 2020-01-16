sudo rm -rdf ~/catkin_ws/src/galapagos_lite/
cp -r ../packages/galapagos_lite ~/catkin_ws/src/galapagos_lite/
cd ~/catkin_ws/src/galapagos_lite/
sudo chmod +x .
sudo chmod +x launch/*.launch
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd ~
source ~/.bashrc
