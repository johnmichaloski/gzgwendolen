/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */

#include "gzrosrcs/kinematicring.h"
#include <aprs_headers/Conversions.h>  // for tf::Identity()

namespace RCS
{



// ----------------------------------------------------
// KinematicRing

////////////////////////////////////////////////////////////////////////////////
KinematicRing::KinematicRing() :
    _prerobotxform(1, tf::Identity()),
    _postrobotxform(2, tf::Identity()),
    _gripperPose(_postrobotxform[0]),
    _basePose(_prerobotxform[0]),
    _toolPose(_postrobotxform[1])
{
    _postrobotxform.clear();
    _postrobotxform.resize(2, tf::Identity());
}

////////////////////////////////////////////////////////////////////////////////
void KinematicRing::setGripperOffset(tf::Pose offset)
{
    assert(_postrobotxform.size()>0);
    _gripperPose = offset;
}


////////////////////////////////////////////////////////////////////////////////
void KinematicRing::setToolOffset(tf::Pose offset) {
    assert(_postrobotxform.size()>1);
    _toolPose = offset;
}

////////////////////////////////////////////////////////////////////////////////
void KinematicRing::setBaseOffset(tf::Pose offset) {
    assert(_prerobotxform.size()>0);
    _basePose = offset;
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose KinematicRing::worldCoord(tf::Pose robotpose) {
#if 0
    // Fixme: this should be variable depending
    // on current kinematic ring
    for (size_t i = 0; i < prerobotxform.size(); i++)
        robotpose = prerobotxform[i] * robotpose;
    //for (size_t i = 0; i < postrobotxform.size(); i++)
    //    robotpose = robotpose * postrobotxform[i];
    robotpose = robotpose * _gripperPose;
    return robotpose;
#else
    return basePose() * robotpose * gripperPose();
#endif
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose KinematicRing::robotAndGripperCoord(tf::Pose worldpose)
{
    worldpose = basePose().inverse() *  worldpose ;

    return worldpose;
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose KinematicRing::robotRemoveGripperCoord(tf::Pose pose)
{
    pose =  pose * gripperPose().inverse() ;

    return pose;
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose KinematicRing::robotAddGripperCoord(tf::Pose pose)
{
    pose =  pose * gripperPose() ;

    return pose;
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose KinematicRing::robotOnlyCoord(tf::Pose worldpose) {
#if 0
    // Fixme: this should be variable depending
    // on current kinematic ring
    //for(size_t i=0; i< prerobotxform.size(); i++)
    for (size_t i = prerobotxform.size(); i-- > 0;)
        worldpose = prerobotxform[i].inverse() * worldpose;
    //for(size_t i=0; i< postrobotxform.size(); i++)
    for (size_t i = postrobotxform.size(); i-- > 0; )
        worldpose =  worldpose * postrobotxform[i].inverse();
#else
    worldpose = basePose().inverse() *  worldpose * gripperPose().inverse();
#endif

    // return basePose().inverse() *  worldpose * gripperPose().inverse();
    return worldpose;
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose KinematicRing::addBaseTransform(tf::Pose pose) {
    return _basePose * pose;
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose  KinematicRing::gripperPose() {
    return _gripperPose;
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose  KinematicRing::basePose() {
    return _basePose;
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose  KinematicRing::toolPose() {
    return _toolPose;
}
}

// This is here as the kinematic DLL often need urdfdom to parse the URD file
#ifndef CRCL_DLL
#include <urdf/model.h>
#include <aprs_headers/Conversions.h>

using namespace urdf;
using namespace RCS;

// URDF Derived knowledge

static std::vector<std::string> jointNames;
static std::string xmlString;
static std::string baseLink;
static std::string tipLink;
static std::vector<std::string> linkNames;
static std::vector< double> jointvalues;
static std::vector< double> jointMin;
static std::vector< double> jointMax;
static std::vector< bool> jointHasLimits;
static std::vector< double> jointEffort;
static std::vector< double> jointVelmax;
static std::vector<tf::Vector3> axis;
static std::vector<tf::Vector3> xyzorigin;
static std::vector<tf::Vector3> rpyorigin;
static std::string  robotName;
bool parseURDF(std::string xml_string, std::string base_link, std::string tip_link)
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

#endif
