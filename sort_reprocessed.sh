#!/bin/bash

folder=./output/
set1=./set1/
set2=./set2/

cd $folder

rm -rf $set1
mkdir -p $set1
rm -rf $set2
mkdir -p $set2

mv ./cordless_drill* $set1
mv ./thin_beer* $set1
mv ./wood_cube* $set1

mv ./hammer* $set2
mv ./cinder_block* $set2
