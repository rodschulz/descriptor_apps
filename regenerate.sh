#!/bin/bash

type=$1
folder=./build/

# Remove old build folder
if [ -d $folder ]; then
	echo "Removing build folder"
	rm -rf $folder
fi

# Create build folder
echo "Generating new build folder"
mkdir $folder

# Generate with cmake
cd $folder
if [ "$type" == "-r" ] ; then
	echo "Generating project for release"
	cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Release ../src/
else
	echo "Generating project for debug"
	cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug ../src/
fi
cd ..

# Create folders if needed
if [ ! -d "./input" ]; then
	mkdir ./input/
fi
if [ ! -d "./output" ]; then
	mkdir ./output/
fi

# Make 
./make.sh
