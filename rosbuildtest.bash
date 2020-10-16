#!/bin/bash


GZPLUGINDIR=`pwd`/plugins7
gzversion=7
export GZPLUGINDIR
export gzversion

catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=Release

catkin build   gotraj

catkin build    gzrcs
read  -n 1 -p "Wait gzrcs" mainmenuinput

catkin build   ikfast_fanuc_plugin
read  -n 1 -p "Wait ikfast_fanuc_plugin" mainmenuinput

catkin build   crcllib
read  -n 1 -p "Wait crcllib" mainmenuinput

catkin build     aprs_headers
read  -n 1 -p "Wait aprs_headers" mainmenuinput

catkin build     crcl_rosmsgs
read  -n 1 -p "Wait crcl_rosmsgs" mainmenuinput

catkin build    gz_custom_messages
read  -n 1 -p "Wait gz_custom_messages" mainmenuinput


catkin build   gotraj
read  -n 1 -p "Waitgotraj" mainmenuinput



catkin build    gokin_plugin
read  -n 1 -p "Wait gokin_plugin" mainmenuinput


catkin build    gzmodelplugin
read  -n 1 -p "Wait gzmodelplugin" mainmenuinput


catkin build   gzparallelgripperplugin
read  -n 1 -p "Wait gzparallelgripperplugin" mainmenuinput







catkin build    gzjointcmdplugin
read  -n 1 -p "Wait gzjointcmdplugin" mainmenuinput





