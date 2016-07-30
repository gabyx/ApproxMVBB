#!/bin/bash

set -e # exit on errors

cd $ROOT_PATH

# Install eigen3 =======================================================
hg clone https://bitbucket.org/eigen/eigen/ ${ROOT_PATH}/eigen3
cd ${ROOT_PATH}/eigen3 && hg update default
mkdir ${ROOT_PATH}/eigen3Build
cd ${ROOT_PATH}/eigen3Build
cmake ../eigen3 -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX
sudo make VERBOSE=1 install

# Install meta =========================================================
git clone https://github.com/ericniebler/meta.git ${ROOT_PATH}/meta
sudo cp -r ${ROOT_PATH}/meta/include/* $INSTALL_PREFIX/include/
#ls -a /usr/local/include/meta

# Install pugixml  =====================================================
git clone https://github.com/zeux/pugixml.git ${ROOT_PATH}/pugixml
perl -pi -e 's/\/\/\s*#define\s*PUGIXML_HAS_LONG_LONG/#define PUGIXML_HAS_LONG_LONG/g' ${ROOT_PATH}/pugixml/src/pugiconfig.hpp 
mkdir ${ROOT_PATH}/pugixmlBuild
cd ${ROOT_PATH}/pugixmlBuild
cmake ../pugixml -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX
sudo make VERBOSE=1 install
