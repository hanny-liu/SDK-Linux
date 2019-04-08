#/bin/sh

cp ../../demo/build/TestIndem ./
export LD_LIBRARY_PATH=./
source /opt/ros/kinetic/setup.bash
./TestIndem
