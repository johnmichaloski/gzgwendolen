#/home/michalos/src/gzaprsros-xenial/src/gzrosrcs/gzrosrcs.pro
QT += core
QT -= gui

TARGET = gzrosrcs
CONFIG += console
CONFIG -= app_bundle
TEMPLATE = app
message("Compiling gzrosrcs")


# You have to add gzversion to Projects "Build Environment"
# for this to work in qmake
gzversion=9
gzversion=$$(gzversion)
exists( /usr/local/include/gazebo-11) {
     message("Gazebo 11")
     gzversion=11
}
exists( /usr/local/include/gazebo-9) {
     message("Gazebo 9")
     gzversion=9
}
exists( /usr/local/include/gazebo-7) {
     message("Gazebo 7")
     gzversion=7
}

# Bug...
gzversion=7
message(GZ Version  $$gzversion)

# Version handling
versionpath="$$_PRO_FILE_PWD_/version.bash"
dummy=$$system($$versionpath )

versionnum=$$cat(version)
major=$$member(versionnum,0)
minor=$$member(versionnum,1)
build=$$member(versionnum,2)
message(major number $$major)
message(minor number $$minor)
message(build number $$build)

#CONFIG +=  c++11 gokin
CONFIG +=  c++11
release: DESTDIR = release
debug:   DESTDIR = debug

OBJECTS_DIR = $$DESTDIR/.obj
MOC_DIR = $$DESTDIR/.moc
RCC_DIR = $$DESTDIR/.qrc
#UI_DIR = $$DESTDIR/.ui

DEFINES+=QT_NO_VERSION_TAGGING
DEFINES+=CNC
DEFINES+=GZCONTACT
DEFINES+=GRIPPER
DEFINES+=GAZEBO
DEFINES+=DEBUG
DEFINES+=KDL_CONVERSION
# These are ROS defines
DEFINES+=ROS
DEFINES+=CRCL_ROS_TOPIC
#DEFINES+=CRCL_DLL

# Supposedly what version ...
DEFINES+=MAJOR=$$major
DEFINES+=MINOR=$$minor
DEFINES+=BUILD=$$build

QMAKE_CXXFLAGS +=-std=c++11
QMAKE_CXXFLAGS +=-Wno-unused-variable
QMAKE_CXXFLAGS +=-Wno-sign-compare
QMAKE_CXXFLAGS +=-Wno-unused-parameter
QMAKE_CXXFLAGS +=-Wno-reorder
QMAKE_CXXFLAGS +=-Wno-format-extra-args
QMAKE_CXXFLAGS +=-Wno-unused-local-typedefs
QMAKE_CXXFLAGS +=-Wno-ignored-qualifiers
QMAKE_CXXFLAGS +=-Wno-deprecated-declarations
QMAKE_CXXFLAGS +=-Wno-unused-function
QMAKE_CXXFLAGS +=-Wno-unused-but-set-variable
QMAKE_CXXFLAGS +=-Wno-missing-field-initializers
QMAKE_CXXFLAGS +=-Wno-unused-parameter


#############################################################
# Gazebo
#############################################################
#LIBS += -L/usr/lib/x86_64-linux-gnu
LIBS += -lgazebo -lgazebo_client
LIBS += -lgazebo_common -lgazebo_transport -lgazebo_msgs
LIBS += -lprotobuf



QMAKE_CXXFLAGS +=-std=c++11
# Local includes
message(Current folder $$PWD)
#INCLUDEPATH += $$PWD/../../include

INCLUDEPATH += "../../install/crcl_rosmsgs/include"
INCLUDEPATH += "../crcl/crcllib/include"
INCLUDEPATH += "../aprs_headers/include"
INCLUDEPATH += "../gotraj/include"
INCLUDEPATH += "../gz_custom_messages/include/gz_custom_messages"


INCLUDEPATH += $$PWD/include
INCLUDEPATH += $$PWD/src

# Ros installation headers
INCLUDEPATH += $$PWD/../../install/include

# Eigen - header only
INCLUDEPATH += "/usr/include/eigen3"
# Gnu readline
INCLUDEPATH += "/usr/local/include"


#ROS
INCLUDEPATH += "/opt/ros/kinetic/include"

# Be careful the order of the libraries is important
# Boost - much could be and is replaced by C11 std
INCLUDEPATH += $$PWD/../../include/dll/include

LIBS += -lboost_system
LIBS += -lboost_chrono
LIBS += -lboost_thread
LIBS += -lboost_filesystem
LIBS += -lboost_date_time


LIBS += -lassimp


# Local ROS libs
LIBS += -L$$PWD/../../lib
LIBS +=  -lgotraj
LIBS +=  -ldl
LIBS += -lreadline
LIBS += -L$$PWD/../../install/lib

# Installed Ros libs
LIBS += -L/opt/ros/kinetic/lib
LIBS += -lroscpp -lcpp_common -lroslib -lrosconsole -lrostime -lrospack
LIBS += -lurdf -lurdfdom_sensor -lurdfdom_model_state -lurdfdom_model -lurdfdom_world
LIBS +=  -ltf -ltf2 -lclass_loader
LIBS +=   -lxmlrpcpp -lroscpp_serialization
LIBS +=  -lrosconsole_bridge  -lrosconsole_log4cxx  -lrosconsole_backend_interface
LIBS += -lactionlib  -ltf2_ros

# Not used.
gokin {
DEFINES+=GOKIN

# gomotion - installed from github into usr/local
INCLUDEPATH += "/usr/local/include"
# To use static lib best to use full path
LIBS += $$(HOME)/src/gomotion/lib/libgokin.a
LIBS += $$(HOME)/src/gomotion/lib/libgo.a
#LIBS += -lgokin -lgo
# OR
#LIBS += -lgofanuckin
}

contains(gzversion, 11){
message("Compiling for gazebo 9")
INCLUDEPATH += "/usr/local/include/ign-math/include"
INCLUDEPATH += "/usr/local/include/gazebo-11/gazebo/msgs"
INCLUDEPATH += "/usr/local/include/gazebo-11/gazebo/transport"
INCLUDEPATH += "/usr/include/sdformat-6.2"
INCLUDEPATH += "/usr/local/include/gazebo-11"
}


contains(gzversion, 9){
message("Compiling for gazebo 9")
INCLUDEPATH += "/usr/include/ignition/math4"
INCLUDEPATH += "/usr/include/ignition/msgs1"
INCLUDEPATH += "/usr/include/ignition/transport4"
INCLUDEPATH += "/usr/include/sdformat-6.2"
INCLUDEPATH += "/usr/include/gazebo-9"
}

INCLUDEPATH += "/usr/include/assimp"

contains(gzversion, 7){
message("Compiling for gazebo 7")
INCLUDEPATH += "/usr/include/ignition/math2"
INCLUDEPATH += "/usr/include/sdformat-4.4"
INCLUDEPATH += "/usr/include/gazebo-7"
INCLUDEPATH += "/usr/local/include/ignition"
LIBS += -lgazebo_math
LIBS+= "/usr/lib/x86_64-linux-gnu/libignition-math2.so"
}
SOURCES += \
    src/Controller.cpp \
    src/CrclApi.cpp \
    src/Globals.cpp \
    src/RobotControlException.cpp \
    src/nist_robotsnc.cpp \
    src/RCSInterpreter.cpp \
    src/Shape.cpp \
    src/Demo.cpp \
    src/gazebo.cpp \
    src/commandlineinterface.cpp \
    src/cros.cpp \
    src/kinematicring.cpp \
    src/gripper.cpp \
    src/assimp.cpp \
    ../gz_custom_messages/include/gz_custom_messages/GripCommand.pb.cc \
    ../gz_custom_messages/include/gz_custom_messages/JointsComm.pb.cc

HEADERS += \
    include/gzrosrcs/Controller.h \
    include/gzrosrcs/CrclApi.h \
    include/gzrosrcs/Demo.h \
    include/gzrosrcs/Globals.h \
    include/gzrosrcs/Gripper.h \
    include/gzrosrcs/RobotControlException.h \
    include/gzrosrcs/RCSInterpreter.h \
    include/gzrosrcs/Shape.h \
    include/gzrosrcs/gazebo.h \
    include/gzrosrcs/cros.h \
    include/gzrosrcs/kinematicring.h \
     include/gzrosrcs/commandlineinterface.h \
    include/gzrosrcs/Communication.h \
    include/gzrosrcs/Controller.h \
    include/gzrosrcs/CrclApi.h \
    include/gzrosrcs/cros.h \
    include/gzrosrcs/Demo.h \
    include/gzrosrcs/gazebo.h \
    include/gzrosrcs/Globals.h \
    include/gzrosrcs/Gripper.h \
    include/gzrosrcs/kinematicring.h \
    include/gzrosrcs/nist_robotsnc.h \
    include/gzrosrcs/RCSInterpreter.h \
    include/gzrosrcs/RobotControlException.h \
    include/gzrosrcs/Shape.h \
    include/gzrosrcs/assimp.h \
    ../aprs_headers/include/aprs_headers/Timing.h \
    ../aprs_headers/include/aprs_headers/seriallinkrobot.h \
    ../aprs_headers/include/aprs_headers/RCSTimer.h \
    ../aprs_headers/include/aprs_headers/RCSThreadTemplate.h \
    ../aprs_headers/include/aprs_headers/RCSPriorityQueue.h \
    ../aprs_headers/include/aprs_headers/RCSMsgQueueThread.h \
    ../aprs_headers/include/aprs_headers/RCSMsgQueue.h \
    ../aprs_headers/include/aprs_headers/IRcs.h \
    ../aprs_headers/include/aprs_headers/IKinematic.h \
    ../aprs_headers/include/aprs_headers/hexdump.h \
    ../aprs_headers/include/aprs_headers/File.h \
    ../aprs_headers/include/aprs_headers/env.h \
    ../aprs_headers/include/aprs_headers/Debug.h \
    ../aprs_headers/include/aprs_headers/Core.h \
    ../aprs_headers/include/aprs_headers/Conversions.h \
    ../aprs_headers/include/aprs_headers/Config.h \
    ../aprs_headers/include/aprs_headers/Path.h\
    ../aprs_headers/include/aprs_headers/Testing.h

DISTFILES += \
    Notes.txt \
    Todo \
    SampleCommands \
    Readme.txt \
    Tests.txt

# Below goes to the build directory
#config_features.path     = "$$OUT_PWD/$$DESTDIR/config"

config_features.path     = ../../install/lib/$$TARGET/config
config_features.files     =    $$PWD/config/Config.ini \
   $$PWD/config/MotomanSia20d.urdf\
   $$PWD/config/FanucLRMate200iD.urdf\
   $$PWD/config/lrmate200id.urdf\
   $$PWD/config/motoman_sia20d.ini\
   $$PWD/config/fanuc-lrmate-200id.ini\
   $$PWD/config/kintest.txt

message("gzrosrcs install $$config_features.path")
message("gzrosrcs instal files $$config_features.files")
INSTALLS  += config_features


test_features.path     = ../../install/lib/$$TARGET/tests
test_features.files     =  $$PWD/tests/FanucLRMate200iD_kintest.txt
INSTALLS  += test_features

buildtest_features.path     = "$$OUT_PWD/$$DESTDIR/tests"
buildtest_features.files     =   $$PWD/tests/FanucLRMate200iD_kintest.txt
INSTALLS  += buildtest_features

# Hard to make distinction between general build and command line qmake build
build_features.path     = "$$OUT_PWD/$$DESTDIR/config"
build_features.files     =    $$PWD/config/Config.ini \
   $$PWD/config/MotomanSia20d.urdf\
   $$PWD/config/FanucLRMate200iD.urdf\
   $$PWD/config/lrmate200id.urdf\
   $$PWD/config/motoman_sia20d.ini\
   $$PWD/config/fanuc-lrmate-200id.ini\
   $$PWD/config/kintest.txt

INSTALLS  += build_features

target.path = ../../install/lib/$$TARGET/
INSTALLS += target
