#!/bin/bash

# By default, this script tries to install in /opt/local/or-tools.
# However, it can be installed pretty much anywhere (using the first command
# line argument as install prefix).
# Just be sure it can be found with CMake ('or-tools' is a path suffix hint).

PACKAGE=or-tools_Ubuntu-14.04-64bit_v5.1.4045
FOLDER=or-tools_Ubuntu-14.04-64bit_v5.1.4047    # NOTE: The content doesn't fit
URL=https://github.com/google/or-tools/releases/download/v5.1/${PACKAGE}.tar.gz 

PREFIX=${1:-/opt/local/or-tools}

echo Installing in $PREFIX ...

STARTING_DIR=$PWD

mkdir -p ${PREFIX}/include
mkdir -p ${PREFIX}/lib

cd /tmp
wget $URL
tar -xzvf ${PACKAGE}.tar.gz
install ${FOLDER}/lib/* $PREFIX/lib/
for d in `find ${FOLDER}/include/ -type d`
do
    subd=`basename $d`
    install -d $PREFIX/include/$subd
    install -m 644 $d/* $PREFIX/include/$subd/
done
rm ${PACKAGE}.tar.gz
rm -r ${FOLDER}

cd $STARTING_DIR

echo Done.
