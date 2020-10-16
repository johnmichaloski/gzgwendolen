#!/bin/bash
p=`pwd`
osver=`lsb_release -sr`
echo $osver


function version_gt() 
{ 
 test "$(printf '%s\n' "$@" | sort -V | head -n 1)" != "$1"; 
}

function package_exists() {
    return dpkg -l "$1" &> /dev/null
}



# fail if no ros kinetic
if ! test -d /opt/ros/kinetic
then
    echo "error: ROS kinetic must be installed for this build to work."
    echo "Remedy: sudo apt-get install ros-kinetic-desktop-full"
    echo "It is not recommeded to install two versions of ROS on the same machine - e.g., kinetic and lunar"
    exit 1
fi

if ! dpkg -l python-catkin-tools 
then
   echo "Installing python-catkin-tools for catkin build - YOU NEED THESE"
   sudo /usr/bin/apt-get install --assume-yes python-catkin-tools
fi

first_version=`protoc --version | awk '{print $2}' `
second_version=2.7
if version_gt $first_version $second_version
then
	echo "PROBLEM: Gazebo ONLY wants protobuf version 2.6"
	echo "you have version protobuf version $first_version"
	exit 1
fi

# Check gazebo existence and version number 
gz=`which gazebo` 
echo gazebo $gz
if  test -f $gz
then
	echo "Now we will check correct gazebo executable version"
	v=`gazebo -v`
	num=`echo $v | sed "s/^.*version \([0-9.]*\).*/\1/"`
	major=${num:0:1}
	minor=${num:1:1}
echo "gazebo version $major . $minor"
	if [[ $major != "9"  &&  $osver = "18.04" ]]
	then
	    echo "error: Gazebo 9 is preferred installation for 18.04 build to work."
read confirm
            exit 1

	fi
	if [[ $major != "7"  &&  $osver = "18.04" ]]
	then
	    echo "error: Gazebo 7 is preferred installation for 16.04 build to work."
	    echo "BECAUSE, ROS gazebo tools are built for gazebo7"
	    echo "Lose/lose"
            exit 1
	fi
else
	echo "Gazebo needs to be installed for rosgazebo support- continue (Y/N)?"
	read confirm
	declare -u  confirm
	if [ "$confirm" = "Y" ]
	then	
		if [ $osver = "18.04" ]; then
		# gazebo9
		echo "Installing Gazebo 9"
		sudo /usr/bin/apt-get install --assume-yes gazebo9
		# Install gazebo development
		sudo /usr/bin/apt-get install --assume-yes libgazebo9-dev
		# Install ros-gazebo tools
                sudo apt install libignition-msgs-dev
		fi

		if [ $osver = "16.04" ]; then
		echo "Installing Gazebo 7"
		# gazebo7
		sudo /usr/bin/apt-get install --assume-yes gazebo7
		# Install gazebo development
		sudo /usr/bin/apt-get install --assume-yes libgazebo7-dev
		# Install ros-gazebo tools
                sudo /usr/bin/apt-get install --assume-yes ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
                sudo apt install libignition-msgs-dev
		fi
	fi

fi





# Test for eigen3
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' libeigen3-dev|grep "install ok installed")
if [ "" == "$PKG_OK" ]; then
  echo "error: eigen3 must be installed for this build to work."
  echo "Installing libeigen3-dev"
  sudo apt-get --force-yes --yes install libeigen3-dev
fi

# Test for code synthesis
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' xsdcxx|grep "install ok installed")
if [ "" == "$PKG_OK" ]; then
  echo "warning: codesynthesis must be installed for XML development to work."
  echo "Installing xsdcxx"
  sudo apt-get --force-yes --yes install xsdcxx
fi

# Test for libxerces  
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' libxerces-c-dev|grep "install ok installed")
if [ "" == "$PKG_OK" ]; then
  echo "error: libxerces-c3.1 must be installed for this build to work."
  echo "Installing libxerces-c3.1"
  sudo apt-get --force-yes --yes install libxerces-c-dev
fi

# Test for boost if not installed install
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' libboost-all-dev|grep "install ok installed")
if [ "" == "$PKG_OK" ]; then
  echo "error: boost must be installed for this build to work."
  echo "Installing libboost-all-dev."
  sudo apt-get --force-yes --yes install libboost-all-dev
fi

PKG_OK=$(dpkg-query -W --showformat='${Status}\n' assimp-utils|grep "install ok installed")
if [ "" == "$PKG_OK" ]; then
  echo "error: assimp must be installed for this build to work."
  echo "Installing assimp-utils."
  sudo apt-get --force-yes --yes install assimp-utils
fi

PKG_OK=$(dpkg-query -W --showformat='${Status}\n' yad|grep "install ok installed")
if [ "" == "$PKG_OK" ]; then
  echo "No yad. Installing yad."
  sudo apt-get --force-yes --yes install yad
fi

# maybe install
#sudo apt-get install libreadline6 libreadline6-dev

PKG_OK=$(dpkg-query -W --showformat='${Status}\n' libreadline6-dev|grep "install ok installed")
if [ "" == "$PKG_OK" ]; then
  echo "No readline. Installing readline version 6."
  sudo apt-get install libreadline6-dev  
fi

# Copy gzrcs headers to appropriate include subdirectory
mkdir -p $p/include/aprs_headers/
cp -r $p/src/aprs_headers/include/aprs_headers $p/include


exit 0


