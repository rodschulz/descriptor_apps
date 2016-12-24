#!/bin/bash

folder=./output/
set1=./beer_drill/
set2=./hammer/
set3=./cube_block/



cd $folder

rm -rf $set1
mkdir -p $set1

rm -rf $set2
mkdir -p $set2

rm -rf $set3
mkdir -p $set3



mv ./cordless_drill* $set1
mv ./thin_beer* $set1

mv ./hammer*.yaml $set2

mv ./wood_cube* $set3
mv ./cinder_block* $set3
