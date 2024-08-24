#!/bin/bash
export PB3D_GIT_PATH='/home/lloydf/Arduino/PB3D/PB3D-constants'
export PB3D_LIB_PATH='/home/lloydf/Arduino/libraries/'
export PB3D_CONST_PATH='/home/lloydf/Arduino/libraries/PB3D-constants'

mkdir $PB3D_CONST_PATH
cp -r $PB3D_GIT_PATH $PB3D_LIB_PATH
echo "Copied PB3D-constants to library folder."