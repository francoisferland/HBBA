#!/usr/bin/env bash

# This scripts downloads and generates a dpkg from a distribution of Google
# or-tools in the current folder (it assumes that a 'dpkg' folder exists and
# contains the package config.

PACKAGE=or-tools_Ubuntu-14.04-64bit_v5.1.4045
FOLDER=or-tools_Ubuntu-14.04-64bit_v5.1.4047    # NOTE: The content doesn't fit
URL=https://github.com/google/or-tools/releases/download/v5.1/${PACKAGE}.tar.gz 

if [ ! -f $PACKAGE.tar.gz ]; then
    wget $URL
fi
if [ ! -f $FOLDER ]; then
    tar -xzvf $PACKAGE.tar.gz
fi

mkdir -p dpkg/opt/or-tools/
cp -r $FOLDER/include dpkg/opt/or-tools/
cp -r $FOLDER/lib dpkg/opt/or-tools/
rm dpkg/opt/or-tools/lib/*.jar

find dpkg/opt -type d -exec chmod 755 {} +
find dpkg/opt -name *.so -exec chmod 755 {} +
find dpkg/opt -name *.h -exec chmod 644 {} +

dpkg-deb --build dpkg or-tools_5.1.4047-1.deb
