#!/usr/bin/env bash

# This scripts downloads and generates a dpkg from a distribution of Google
# or-tools in the current folder (it assumes that a 'dpkg' folder exists and
# contains the package config.

PACKAGE=or-tools_Ubuntu-14.04-64bit_v5.1.4045
FOLDER=or-tools_Ubuntu-14.04-64bit_v5.1.4047    # NOTE: The content doesn't fit
URL=https://github.com/google/or-tools/releases/download/v5.1/${PACKAGE}.tar.gz 

wget $URL
tar -xzvf $PACKAGE.tar.gz
mkdir -p dpkg/opt/or-tools/
cp -r $FOLDER/include dpkg/opt/or-tools/
cp -r $FOLDER/lib dpkg/opt/or-tools/
rm dpkg/opt/or-tools/lib/*.jar

dpkg-deb --build dpkg or-tools_5.1.4047-1.deb
