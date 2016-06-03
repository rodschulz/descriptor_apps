#!/bin/bash

type=$1
folder=./build/
aux=./cleaning_aux_folder/

# Go to the script's location
cd "$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Create auxiliary folder
mkdir ./cleaning_aux_folder/

# Save the project files for later
cp ${folder}.project ${aux}
cp ${folder}.cproject ${aux}
cp -r ${folder}.settings ${aux}

# Remove old build folder
if [ -d $folder ]; then
	echo "Removing build folder"
	rm -rf $folder
fi

# Create again the build folder
mkdir $folder

# Copy project files back
cp ${aux}.project ${folder}
cp ${aux}.cproject ${folder}
cp -r ${aux}.settings ${folder}
rm -rf $aux

# Configure with cmake
cd $folder
if [ "$type" == "-r" ] ; then
	echo "Configuring project for RELEASE"
	cmake -DCMAKE_BUILD_TYPE=Release ../src/
else
	echo "Configuring project for DEBUG"
	cmake -DCMAKE_BUILD_TYPE=Debug ../src/
fi
cd ..
