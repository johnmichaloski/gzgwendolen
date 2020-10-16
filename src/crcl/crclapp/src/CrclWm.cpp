
#include <string>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>


#include "aprs_headers/Debug.h"
#include "aprs_headers/Config.h"
#include "aprs_headers/env.h"
#include "aprs_headers/RCSThreadTemplate.h"
#include "aprs_headers/Conversions.h"

#include "crclapp/CrclWm.h"
#include "crclapp/Globals.h"
#include "crclapp/CrclRos.h"
#include "crclapp/CommandLineInterface.h"
#include "crclapp/NistCrcl.h"
#include "crclapp/Crcl.h"
#include "crclapp/CrclServer.h"

using namespace RCS;
rcs_robot_type rcs_robot;
rcs_world_type rcs_world;
namespace WorldModel
{
WorldModel::CInstances instances;
std::mutex shapemutex;

}
std::shared_ptr<CCrclSession> crclCommServer;
boost::shared_ptr<crcl::crclServer> pCrclServer;
std::shared_ptr<CCrclApi> wm_init()
{

    return  std::shared_ptr<CCrclApi>(new CCrclApi());

}
std::shared_ptr<CCrclApi>  crclApi=wm_init();

double rcs_status::currentRobotJointSpeed()
{
    return crclApi->rates.CurrentTransSpeed();
    //       ::crcl_rosmsgs::CrclMaxProfileMsg profile= crclApi->getSpeeds();
    // fixme: possibly only commanded vel not actual vel
    //       return profile.maxvel;
}
double rcs_status::currentGripperJointSpeed()
{
    return crclApi->rates.CurrentTransSpeed();

}
double rcs_status::currentLinearSpeed()
{
    return crclApi->rates.CurrentTransSpeed();
}
double rcs_status::currentAngularSpeed()
{
    return crclApi->rates.CurrentRotSpeed();
}

////////////////////////////////////////////////////////////////////////////////

void rcs_world_type::configure(std::string robot,std::string inifile)
{
    // Is this really world model or robot dependent?
    // offsets, part lists?
    // REally demo specific...
    Nist::Config wmconfig;
    if(!wmconfig.loadFile(inifile))
        throw "rcs_world_type ini file  " + inifile + "did not open";
    part_list = wmconfig.getTokens<std::string>( robot + ".parts", ",");

    std::vector<double> offset;
    offset = wmconfig.getTokens<double>(robot + ".offset.vesselslot", ",");
    slotoffset["vessel"]=RCS::Convert<std::vector<double>, tf::Pose> (offset);

}

////////////////////////////////////////////////////////////////////////////////

void rcs_robot_type::configure(std::string robot, std::string inifile)
{
    _prefix=robot;

    try {
        Nist::Config robotconfig;
        if(!robotconfig.loadFile(inifile))
            throw "rcs_robot_type ini file  " + inifile + "did not open";

        // CRCL configuration
        std::string crclIp=robotconfig.getSymbolValue<std::string>(robot + ".crcl.Ip", "127.0.0.1");
        long crclPort=robotconfig.getSymbolValue<double>(robot + ".crcl.Port", "64444");


        // ROBOT configuration
        std::string robotName = robotconfig.getSymbolValue<std::string>(robot + ".robot.longname", "ERROR");
        std::string robotTiplink = robotconfig.getSymbolValue<std::string>(robot + ".nc.kinsolver.tiplink", "ERROR");
        std::string robotBaselink = robotconfig.getSymbolValue<std::string>(robot + ".nc.kinsolver.baselink", "ERROR");
        std::string robotUrdfFile = robotconfig.getSymbolValue<std::string>(robot + ".nc.kinsolver.urdffile", "ERROR");

        // Read the robot urdf file
        std::string robot_urdf;
        Globals.readFile(getexefolder() + robotUrdfFile,robot_urdf);
        pCrclServer=boost::shared_ptr<crcl::crclServer>( new crcl::crclServer(
                                                             crclIp,
                                                             crclPort,
                                                             0.1,
                                                             robot_urdf,
                                                             robotBaselink,
                                                             robotTiplink ));

        // Parse and record macro command sequences - e.g., homing, setup
        std::vector<std::string> macronames = robotconfig.getTokens<std::string>(robot + ".macros", ",");
        for (size_t j = 0; j < macronames.size(); j++)
        {
            // Assign robot some preliminary setup commands - FIXME: make names macros?
            std::string setup_cmds = robotconfig.getSymbolValue<std::string>(robot + "." + macronames[j], "");
            std::vector<std::string>  commands = Globals.trimmedTokenize(setup_cmds,",");
            namedCommand[macronames[j]] = commands;
        }
        // Parse and record named joint moves - e.g., home safe
        std::vector<std::string> jointmovenames = robotconfig.getTokens<std::string>(robot + ".joints.movenames", ",");
        for (size_t j = 0; j < jointmovenames.size(); j++) {
            std::vector<double> ds = robotconfig.getTokens<double>(robot + "." + jointmovenames[j], ",");
            std::transform(jointmovenames[j].begin(), jointmovenames[j].end(), jointmovenames[j].begin(), ::tolower);
            namedJointMove[jointmovenames[j]] = ds;
        }

        // Parse and record named pose moves - e.g., gears
        std::vector<std::string> posemovenames = robotconfig.getTokens<std::string>(robot + ".pose.movenames", ",");
        for (size_t j = 0; j < posemovenames.size(); j++) {
            std::vector<double> ds = robotconfig.getTokens<double>(robot + "." + posemovenames[j], ",");
            std::transform(posemovenames[j].begin(), posemovenames[j].end(), posemovenames[j].begin(), ::tolower);
            rcs_robot.namedPoseMove[posemovenames[j]] = RCS::ConvertDblVectorTf(ds);
        }
        std::vector<double> dbase = robotconfig.getTokens<double>(robot + ".nc.xform.base", ",");
        std::vector<double> dbend = robotconfig.getTokens<double>(robot + ".nc.xform.qbend",",");

        // Translate 4 doubles into quaternion
        if(dbase.size() < 6  || dbend.size() < 4)
        {
            throw std::runtime_error(std::string( "dbase or dbend missing values"));

        }
        basePose=RCS::Convert<std::vector<double>, tf::Pose> (dbase);
        QBend = tf::Quaternion(dbend[0], dbend[1], dbend[2], dbend[3]);

        Retract =  RCS::Convert<std::vector<double>, tf::Pose>(
                    robotconfig.getTokens<double>(robot + ".nc.xform.retract", ",")
                    );
        RetractInv=Retract.inverse();

        // Part offsets
        std::vector<double> offset;
        offset = robotconfig.getTokens<double>(robot + ".offset.gripper.smallgear", ",");
        gripperoffset["sku_part_small_gear"]=RCS::Convert<std::vector<double>, tf::Pose> (offset);
        offset = robotconfig.getTokens<double>(robot + ".offset.gripper.mediumgear", ",");
        gripperoffset["sku_part_medium_gear"]=RCS::Convert<std::vector<double>, tf::Pose> (offset);
        offset = robotconfig.getTokens<double>(robot + ".offset.gripper.largegear", ",");
        gripperoffset["sku_part_large_gear"]=RCS::Convert<std::vector<double>, tf::Pose> (offset);
    }
    catch(...)
    {
        std::cout << "Fatal error in rcs_robot_type::configure \n";
    }

}

bool rcs_robot_type::isBusy()
{
    if(_status.get() == NULL)
        return true;
    return  _status->crclcommandstatus!=Crcl::CRCL_Done;
}

std::string rcs_robot_type::robotPrefix()
{
    return _prefix;
}

size_t rcs_robot_type::numJoints()
{
    size_t n = jointNames.size();
    return n;
}

std::vector<unsigned long> rcs_robot_type::allJointNumbers()
{
    std::vector<unsigned long> jointnum(numJoints());
    std::iota(jointnum.begin(), jointnum.end(), 0); // adjusted already to 0..n-1
    return jointnum;
}

bool rcs_robot_type::parseURDF(std::string xml_string, std::string base_link, std::string tip_link)
{
    urdf::Model robot_model;
    if(xml_string.empty())
    {
        return false;
    }

    robot_model.initString(xml_string);

    robotName=robot_model.getName();

    // These vectdors are cleared in case parseURDF is called twice...
    linkNames.clear();
    jointNames.clear();
    axis.clear();
    xyzorigin.clear();
    rpyorigin.clear();
    jointHasLimits.clear();
    jointMin.clear();
    jointMax.clear();
    jointEffort.clear();
    jointVelmax.clear();

    urdf::LinkConstSharedPtr link =robot_model.getLink(tip_link);
    while (link->name != base_link) { // && joint_names.size() <= num_joints_) {
        linkNames.push_back(link->name);
        urdf::JointSharedPtr joint   = link->parent_joint;
        if (joint) {
            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {

                jointNames.push_back(joint->name);
                axis.push_back(Convert<urdf::Vector3, tf::Vector3>(joint->axis));
                xyzorigin.push_back(Convert<urdf::Vector3, tf::Vector3>(joint->parent_to_joint_origin_transform.position));
                double roll, pitch, yaw;
                joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
                rpyorigin.push_back(tf::Vector3(roll, pitch, yaw));

                float lower, upper, maxvel = 0.0, maxeffort = 0.0;
                int hasLimits;
                if (joint->type != urdf::Joint::CONTINUOUS) {
                    maxvel = joint->limits->velocity;
                    maxeffort = joint->limits->effort;
                    if (joint->safety) {
                        lower = joint->safety->soft_lower_limit;
                        upper = joint->safety->soft_upper_limit;
                    } else {
                        lower = joint->limits->lower;
                        upper = joint->limits->upper;
                    }
                    hasLimits = 1;
                } else {
                    lower = -M_PI;
                    upper = M_PI;
                    hasLimits = 0;
                }
                if (hasLimits) {
                    jointHasLimits.push_back(true);
                    jointMin.push_back(lower);
                    jointMax.push_back(upper);
                } else {
                    jointHasLimits.push_back(false);
                    jointMin.push_back(-M_PI);
                    jointMax.push_back(M_PI);
                }
                jointEffort.push_back(maxeffort);
                jointVelmax.push_back(maxvel);
            }
        } else {
            ROS_WARN_NAMED("nc", "no joint corresponding to %s", link->name.c_str());
        }
        link = link->getParent();
    }

    std::reverse(linkNames.begin(), linkNames.end());
    std::reverse(jointNames.begin(), jointNames.end());
    std::reverse(jointMin.begin(), jointMin.end());
    std::reverse(jointMax.begin(), jointMax.end());
    std::reverse(jointHasLimits.begin(), jointHasLimits.end());
    std::reverse(axis.begin(), axis.end());
    std::reverse(xyzorigin.begin(), xyzorigin.end());
    std::reverse(rpyorigin.begin(), rpyorigin.end());
    std::reverse(jointEffort.begin(), jointEffort.end());
    std::reverse(jointVelmax.begin(), jointVelmax.end());

    return true;
}
