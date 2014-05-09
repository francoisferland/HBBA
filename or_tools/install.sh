#!/bin/sh

mkdir -p $1/lib/
cp $PWD/lib/*.so $1/lib/

mkdir -p $1/include/or-tools/constraint_solver
cp $PWD/src/constraint_solver/*.h $1/include/or-tools/constraint_solver
mkdir -p $1/include/or-tools/base
cp $PWD/src/base/*.h $1/include/or-tools/base
mkdir -p $1/include/or-tools/util
cp $PWD/src/util/*.h $1/include/or-tools/util

