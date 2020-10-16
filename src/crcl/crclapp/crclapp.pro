
QT += core
QT -= gui

gzversion=7

TARGET = crclapp
CONFIG += console
CONFIG -= app_bundle
TEMPLATE = app
message("Compiling crclapp application")


CONFIG +=  c++11
release: DESTDIR = release
debug:   DESTDIR = debug

OBJECTS_DIR = $$DESTDIR/.obj
MOC_DIR = $$DESTDIR/.moc
RCC_DIR = $$DESTDIR/.qrc
#UI_DIR = $$DESTDIR/.ui

DEFINES+=QT_NO_VERSION_TAGGING
DEFINES+=DEBUG
DEFINES+=GAZEBO



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
QMAKE_CXXFLAGS +=-Wno-write-strings
QMAKE_CXXFLAGS +=-Wno-missing-field-initializers
QMAKE_CXXFLAGS +=-std=c++11

QMAKE_LFLAGS += -g

#INCLUDEPATH += "../../../include"
INCLUDEPATH += "../crcl_rosmsgs/include"
#INCLUDEPATH += "../../../install/include"
INCLUDEPATH += "../../aprs_headers/include"
INCLUDEPATH += "../crcl_rosmsgs/include"

INCLUDEPATH += "./include/crclapp/CrclXsd"
INCLUDEPATH += "./include"
INCLUDEPATH += "./src"
INCLUDEPATH += "./src/CRCL"


#ROS
INCLUDEPATH += "/opt/ros/kinetic/include"

# Local  libs
LIBS += -L../../../lib
#LIBS +=  -lgotraj


# Boost - many could be replace by C11 std
LIBS += -L/usr/lib/x86_64-linux-gnu
LIBS += -lboost_system
LIBS += -lboost_chrono
LIBS += -lboost_thread
LIBS += -lboost_filesystem
LIBS += -lboost_date_time
LIBS += -lboost_regex
LIBS += -lboost_log_setup
LIBS += -lboost_log
LIBS += -lboost_locale

# GNU readline
LIBS += -lreadline

# Ros libs
LIBS += -L/opt/ros/kinetic/lib -lroscpp -lcpp_common -lroslib -lrosconsole -lrostime -lrospack
LIBS += -lurdf -lurdfdom_sensor -lurdfdom_model_state -lurdfdom_model -lurdfdom_world
LIBS +=  -ltf -ltf2 -lclass_loader
LIBS +=   -lxmlrpcpp -lroscpp_serialization
LIBS +=  -lrosconsole_bridge  -lrosconsole_log4cxx  -lrosconsole_backend_interface
LIBS += -lactionlib -lpthread -ltf2_ros -ltf_conversions

# Code Synthesis
# xerces code synthesis dependency - no XML parsing
# Static lib
#LIBS += "/usr/lib/x86_64-linux-gnu/libxerces-c.a"

# xerces code synthesis dependency
#LIBS += "/usr/lib/x86_64-linux-gnu/libxerces-c.a"
LIBS +=  -lxerces-c


SOURCES += \
    src/Crcl.cpp \
    src/NistCrcl.cpp \
    src/CRCL/CRCLCommandInstance.cxx \
    src/CRCL/CRCLCommands.cxx \
    src/CRCL/CRCLProgramInstance.cxx \
    src/CRCL/CRCLStatus.cxx \
    src/CRCL/DataPrimitives.cxx \
    src/Crcl2Rcs.cpp \
    src/CrclServer.cpp \
    src/CommandLineInterface.cpp \
    src/Globals.cpp \
    src/CrclApi.cpp \
    src/main.cpp \
    src/CrclRos.cpp \
    src/CrclWm.cpp \
    src/Demo.cpp \
    src/Shape.cpp \
    src/CrclPublisherInterface.cpp \
    src/CrclSubscriberInterface.cpp \
    src/MTCSOCKET/client.cpp \
    src/MTCSOCKET/socketserver.cpp

HEADERS += \
    include/crclapp/Crcl.h \
    include/crclapp/CrclXsd/CRCLCommandInstance.hxx \
    include/crclapp/CrclXsd/CRCLCommands.hxx \
    include/crclapp/CrclXsd/CRCLProgramInstance.hxx \
    include/crclapp/CrclXsd/CRCLStatus.hxx \
    include/crclapp/CrclXsd/DataPrimitives.hxx \
     include/crclapp/NistCrcl.h \
    include/crclapp/Crcl2Rcs.h \
    include/crclapp/CrclServer.h \
    include/crclapp/CommandLineInterface.h \
    include/crclapp/CrclApi.h \
    include/crclapp/Globals.h \
    include/crclapp/CrclRos.h \
    include/crclapp/NistCrcl.h \
    include/crclapp/CrclWm.h \
    include/crclapp/Demo.h \
    include/crclapp/Shape.h \
    include/crclapp/CrclSubscriberInterface.h \
    include/crclapp/CrclPublisherInterface.h \
    include/crclapp/MTCSOCKET/client.hpp \
    include/crclapp/MTCSOCKET/internal.hpp \
    include/crclapp/MTCSOCKET/socketserver.h

DISTFILES += \
    include/nistcrcl/CrclXsd/CRCLCommandInstance.xsd \
    include/nistcrcl/CrclXsd/CRCLCommands.xsd \
    include/nistcrcl/CrclXsd/CRCLProgramInstance.xsd \
    include/nistcrcl/CrclXsd/CRCLStatus.xsd \
    include/nistcrcl/CrclXsd/DataPrimitives.xsd \
    include/nistcrcl/CrclXsd/CreateTree.bash \
    include/nistcrcl/NIST/RCSTimer.txt \
    Notes \
    notes

# Hard to make distinction between general build and command line qmake build
build_features.path     = "../../../install/lib/crclapp/config"
#build_features.path     = "$$OUT_PWD/$$DESTDIR/config"
build_features.files     =    $$PWD/config/Config.ini \
   $$PWD/config/MotomanSia20d.urdf\
   $$PWD/config/FanucLRMate200iD.urdf\
   $$PWD/config/lrmate200id.urdf\
   $$PWD/config/motoman_sia20d.ini\
   $$PWD/config/fanuc-lrmate-200id.ini

INSTALLS  += build_features

# THis moves not copies executable from build to install
target.path = ../../../install/lib/crclapp
INSTALLS += target


