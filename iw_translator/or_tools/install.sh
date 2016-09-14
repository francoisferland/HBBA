#!/bin/sh

GENPATH=$2
INCPATH=$1/include/or-tools
LIBPATH=$1/lib/or-tools

mkdir -p $LIBPATH
cp $PWD/lib/*.so $LIBPATH

mkdir -p $INCPATH/constraint_solver
cp $PWD/src/constraint_solver/*.h $INCPATH/constraint_solver
cp $PWD/src/gen/constraint_solver/*.h $INCPATH/constraint_solver 
mkdir -p $INCPATH/base
cp $PWD/src/base/*.h $INCPATH/base
mkdir -p $INCPATH/util
cp $PWD/src/util/*.h $INCPATH/util

# Third-party:
cp -r $PWD/dependencies/install/lib/*     $LIBPATH/
cp -r $PWD/dependencies/install/include/* $INCPATH/

