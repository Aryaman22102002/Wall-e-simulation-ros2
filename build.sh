#!/bin/bash
echo "Enter your package name" 
read PkgName 
cd ../../
colcon build
. install/setup.bash
cd src/$PkgName
