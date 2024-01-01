#!/bin/bash

SCRIPT=$(readlink -f $0)
echo $SCRIPT
SCRIPTPATH=`dirname $SCRIPT`
echo #SCRIPTPATH
cd $SCRIPTPATH/..

find . -iname "*.h" -o -iname "*.hpp" -o -iname "*.cpp" | \
    grep -Ev "^(./comms/mavlink/v1.0/|./.git|./boards/varmint/lib/)" | \
xargs clang-format -i
