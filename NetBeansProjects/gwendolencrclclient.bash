#!/bin/bash

#    ^^ this has to be bash, not /bin/sh, for arrays to work
# run dos2unix ./runmultiterm.bash
root=`pwd`
export root

apppath=$root/GwendolynCrclClient/dist
export apppath
 
cmd=( gnome-terminal )

cmd+=( --tab  --working-directory="$q" -e 'bash -c "printf \"\e]2;Gwendolen Planner Crcl Client\a\";cd $apppath; java -jar GwendolynCrclClient.jar --loopback;exec bash"')

"${cmd[@]}"

