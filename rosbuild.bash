#!/bin/bash



# Check if all the installation prerequisites are met.
bash ./isvalid.bash
valid=$?
echo valid is $valid

# fail if installation prerequisites are not met.
if [ "$valid" != "0" ]
then
    echo "abort: prerequisite missing"
    exit 1
fi


GZPLUGINDIR=`pwd`/plugins7
gzversion=7
export GZPLUGINDIR
export gzversion

catkin build  crcl_rosmsgs
catkin build gz_custom_messages
catkin build


