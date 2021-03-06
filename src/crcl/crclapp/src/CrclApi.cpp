/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */


#include "crclapp/CrclApi.h"
#include "crclapp/Globals.h"
#include "crclapp/Crcl2Rcs.h"

#include "aprs_headers/Conversions.h"
using namespace RCS;

// Simplistic Testing code
int CCrclApi::_crclcommandnum = 1;
IRate CCrclApi::rates;

//#define DIRECT_ROS_MSG

////////////////////////////////////////////////////////////////////////////////
void CCrclApi::setDwell(double d)
{
    _mydwell=d;
}
////////////////////////////////////////////////////////////////////////////////
void CCrclApi::slow()
{
    rates.CurrentTransSpeed() = rates.CurrentTransSpeed()/2.;
    rates.MaxJointVelocity() = rates.MaxJointVelocity() / 2.0;
}
////////////////////////////////////////////////////////////////////////////////
void CCrclApi::medium()
{
    setVelocity(1.0);
}
////////////////////////////////////////////////////////////////////////////////
void CCrclApi::fast()
{
    rates.CurrentTransSpeed()= rates.CurrentTransSpeed()*2.;
    rates.MaxJointVelocity() = rates.MaxJointVelocity() * 2.0;
}

////////////////////////////////////////////////////////////////////////////////
void CCrclApi::setGraspingDwell(double d)
{
    _mygraspdwell=d;
}
////////////////////////////////////////////////////////////////////////////////
void CCrclApi::setVelocity(double speed)
{
    rates.CurrentTransSpeed()=speed;
    rates.MaxJointVelocity() = speed;

}

////////////////////////////////////////////////////////////////////////////////
::crcl_rosmsgs::CrclMaxProfileMsg CCrclApi::getSpeeds()
{
    ::crcl_rosmsgs::CrclMaxProfileMsg profile;
    profile.maxvel=rates.CurrentTransSpeed();
    profile.maxacc=10.*rates.CurrentTransSpeed();
    profile.maxjerk= 100.*rates.CurrentTransSpeed();
    return profile;
}


////////////////////////////////////////////////////////////////////////////////
void CCrclApi::setGripper(double ee)
{
#ifdef DIRECT_ROS_MSG
    // FIXME: set gripper to 0..1
    crcl_rosmsgs::CrclCommandMsg cmd;
    CCanonCmd::setRosMsgTimestamp(cmd.header);
    cmd.crclcommand = CanonCmdType::CANON_SET_GRIPPER;
    cmd.crclcommandnum = _crclcommandnum++;
    cmd.eepercent = ee;
    CCrcl2RosMsg::crclcmdsq->addMsgQueue(cmd);
#else
    RCS::CCanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_SET_GRIPPER;
    cmd.crclcommandnum = _crclcommandnum++;
    cmd.eepercent = ee;
    RCS::cmds.addMsgQueue(cmd);
#endif
    cmd.eepercent = 1.0 - ee;
}

////////////////////////////////////////////////////////////////////////////////
void CCrclApi::setContactGripper(double ee)
{
    std::cerr << "CCrclApi::setContactGripper not implemented \n" ;
    return;

    crcl_rosmsgs::CrclCommandMsg cmd;
    CCanonCmd::setRosMsgTimestamp(cmd.header);
    cmd.crclcommand = CanonCmdType::CANON_CONTACT_GRIPPER;
    cmd.crclcommandnum = _crclcommandnum++;
    cmd.eepercent = ee;
    CCrcl2RosMsg::crclcmdsq->addMsgQueue(cmd);

}

////////////////////////////////////////////////////////////////////////////////
void CCrclApi::setVelGripper(double vel, double fmax)
{
#ifdef DIRECT_ROS_MSG
    crcl_rosmsgs::CrclCommandMsg cmd;
    CCanonCmd::setRosMsgTimestamp(cmd.header);
    cmd.crclcommand = CanonCmdType::CANON_SET_EE_PARAMETERS;
    cmd.crclcommandnum = _crclcommandnum++;
    cmd.parameter_names = { "action", "vel", "fmax"};
    cmd.parameter_values = { "Vel/Fmax", Globals.strConvert(vel), Globals.strConvert(fmax)};;
    CCrcl2RosMsg::crclcmdsq->addMsgQueue(cmd);
#else
    RCS::CCanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_SET_EE_PARAMETERS;
    cmd.crclcommandnum = _crclcommandnum++;
    cmd.parameter_names = { "action", "vel", "fmax"};
    cmd.parameter_values = { "Vel/Fmax", Globals.strConvert(vel), Globals.strConvert(fmax)};;

    RCS::cmds.addMsgQueue(cmd);
#endif
}

////////////////////////////////////////////////////////////////////////////////
void CCrclApi::closeGripper()
{
     setGripper(0.0);
}
////////////////////////////////////////////////////////////////////////////////
void CCrclApi::smartCloseGripper()
{
     setContactGripper(0.0);
}

////////////////////////////////////////////////////////////////////////////////
void CCrclApi::openGripper()
{
   setGripper(1.0);
}

////////////////////////////////////////////////////////////////////////////////
void CCrclApi::doDwell(double dwelltime) {
#ifdef DIRECT_ROS_MSG
    crcl_rosmsgs::CrclCommandMsg cmd;
    CCanonCmd::setRosMsgTimestamp(cmd.header);
    cmd.crclcommand = CanonCmdType::CANON_DWELL;
    cmd.crclcommandnum = _crclcommandnum++;
    cmd.dwell_seconds = dwelltime;
    cmd.eepercent=-1.0; // keep as is
    CCrcl2RosMsg::crclcmdsq->addMsgQueue(cmd);
#else
    RCS::CCanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_DWELL;
    cmd.crclcommandnum = _crclcommandnum++;
    cmd.dwell_seconds = dwelltime;
    cmd.eepercent=-1.0; // keep as is
    RCS::cmds.addMsgQueue(cmd);
#endif
}

////////////////////////////////////////////////////////////////////////////////
void CCrclApi::moveTo(tf::Pose pose)
{
#ifdef DIRECT_ROS_MSG
    crcl_rosmsgs::CrclCommandMsg cmd;
    CCanonCmd::setRosMsgTimestamp(cmd.header);
    cmd.crclcommandnum = _crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_TO;
    cmd.profile.push_back(getSpeeds());
    cmd.finalpose = Convert<tf::Pose, geometry_msgs::Pose> (pose);
    cmd.eepercent=-1.0; // keep as is
    CCrcl2RosMsg::crclcmdsq->addMsgQueue(cmd);
#else
    RCS::CCanonCmd cmd;
    cmd.crclcommandnum = _crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_TO;
    cmd.profile.push_back(getSpeeds());
    cmd.finalpose = Convert<tf::Pose, geometry_msgs::Pose> (pose);
    cmd.eepercent=-1.0; // keep as is
    RCS::cmds.addMsgQueue(cmd);
#endif
}


////////////////////////////////////////////////////////////////////////////////
void CCrclApi::moveJoints(std::vector<long unsigned int> jointnum,
                         std::vector<double> positions,
                         double vel)
{
//#ifdef DIRECT_ROS_MSG
    crcl_rosmsgs::CrclCommandMsg cmd;
    CCanonCmd::setRosMsgTimestamp(cmd.header);
    cmd.crclcommandnum = _crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_JOINT;
    cmd.joints = zeroJointState(jointnum.size());
    cmd.joints.position = positions;
    cmd.jointnum=jointnum;
    cmd.joints.name=rcs_robot.jointNames;
    cmd.eepercent=-1.0; // keep as is
    cmd.bCoordinated = true;

    // THe lower level robot control really needs
    // the velocity profile set.
    ::crcl_rosmsgs::CrclMaxProfileMsg profile;
    vel = rates.MaxJointVelocity();
    profile.maxvel=vel;
    profile.maxacc=vel*10.;
    profile.maxjerk=vel*100.;
    cmd.profile.push_back(profile)  ;
    CCrcl2RosMsg::crclcmdsq->addMsgQueue(cmd);
//#else
#if 0
    RCS::CCanonCmd cc;
    cc.crclcommandnum = _crclcommandnum++;
    cc.crclcommand = CanonCmdType::CANON_MOVE_JOINT;
    cc.joints = zeroJointState(jointnum.size());
    cc.joints.position = positions;
    cc.jointnum=jointnum;
    cc.joints.name=rcs_robot.jointNames;
    cc.eepercent=-1.0; // keep as is
    cc.bCoordinated = true;

    // higher bogusian
    vel = rates.MaxJointVelocity();
    cc.Rates().CurrentTransSpeed()=vel;

    RCS::cmds.addMsgQueue(cmd);
#endif
}

////////////////////////////////////////////////////////////////////////////////
void  CCrclApi::pick(tf::Pose pose )
{

    tf::Vector3 offset = pose.getOrigin();

    openGripper(); // make sure griper is open

    // Approach object
    moveTo(rcs_robot.Retract * tf::Pose(rcs_robot.QBend, offset));
    doDwell(_mydwell);
    moveTo(tf::Pose(rcs_robot.QBend, offset));
    doDwell(_mydwell);
    closeGripper();
    doDwell(_mygraspdwell);
    moveTo(rcs_robot.Retract * tf::Pose(rcs_robot.QBend, offset));
}

////////////////////////////////////////////////////////////////////////////////
void CCrclApi::place(tf::Pose pose)
{

    tf::Vector3 offset = pose.getOrigin();

    // Retract
    moveTo(rcs_robot.Retract * tf::Pose(rcs_robot.QBend, offset));
    doDwell(_mydwell);

    // Place the grasped object
    moveTo(tf::Pose(rcs_robot.QBend, offset));
    doDwell(_mydwell);

    // open gripper and wait
    openGripper();
    doDwell(_mygraspdwell);

    // Retract from placed object
    moveTo(rcs_robot.Retract * tf::Pose(rcs_robot.QBend, offset));

}

////////////////////////////////////////////////////////////////////////////////
//int CCrclApi::pickup(std::string objname)
//{

//    if(grasp(objname) < 0)
//        return -1;

//    return retract();
//}
////////////////////////////////////////////////////////////////////////////////
//int CCrclApi::retract(std::string objname)
//{
//    ShapeModel::CShape * shape = ShapeModel::instances.findInstance(objname);
//    if(shape==NULL)
//    {
//        shape = ShapeModel::instances.findSubstrInstance(objname);
//        if(shape==NULL)
//            return -1;
//    }

//    tf::Pose pose = _nc->basePose().inverse() * shape->_location;
//    tf::Vector3 offset = pose.getOrigin();

//    // Retract
//    moveTo(_nc->Retract() * tf::Pose(_nc->QBend(), offset));
//    return 0;
//}

int CCrclApi::retract(double amt)
{
    sensor_msgs::JointState curjoints, nextjoints;
    curjoints.position = _status.r_curjnts.position;
    tf::Pose r_curpose=_status.r_curpose;
    // FIXME: ??
    //_nc->robotKinematics()->FK(curjoints.position, r_curpose);
#if 0
    //FIXME!!!!!!!!
    // instead just move up in z direction
    // This joint motion code was here as ikfast unreliable in straight line up
    r_curpose.getOrigin().setZ(r_curpose.getOrigin().z()+ amt); // move up

    _nc->robotKinematics()->IK(r_curpose,
                           Subset(curjoints.position, _nc->robotKinematics()->NumJoints()),
                           nextjoints.position);
    move_joints(_nc->robotKinematics()->AllJointNumbers(), nextjoints.position);
#endif

    // Retract
    moveTo(rcs_robot.Retract * r_curpose);

    return 0;

}
//////////////////////////////////////////////////////////////////////////////////
//int CCrclApi::approach(std::string objname)
//{
//    ShapeModel::CShape * shape = ShapeModel::instances.findInstance(objname);
//    if(shape==NULL)
//    {
//        shape = ShapeModel::instances.findSubstrInstance(objname);
//        if(shape==NULL)
//            return -1;
//    }


//    tf::Pose pose = _nc->basePose().inverse() * shape->_location;
//    tf::Vector3 offset = pose.getOrigin();
//    this->openGripper();

//    // Approach
//    moveTo(_nc->Retract() * tf::Pose(_nc->QBend(), offset));
//    return 0;
//}

//////////////////////////////////////////////////////////////////////////////////
//int CCrclApi::grasp(std::string objname)
//{

//    ShapeModel::CShape * shape = ShapeModel::instances.findInstance(objname);
//    if(shape==NULL)
//    {
//         return -1;
//    }

//    tf::Pose pose = _nc->basePose().inverse() * shape->_location;
//    tf::Vector3 offset = pose.getOrigin();

//    tf::Pose gripperoffset  = _nc->gripperoffset()[shape->_type];
//    double graspforce = _nc->graspforce()[shape->_type];

//    // Grasp
//    moveTo(tf::Pose(_nc->QBend(), offset) * gripperoffset);
//    doDwell(_mydwell);
//    closeGripper();
//    doDwell(_mygraspdwell);
//    return 0;

//}
//////////////////////////////////////////////////////////////////////////////////
//int CCrclApi::moveTo(std::string objname) {
//    ShapeModel::CShape * shape = ShapeModel::instances.findInstance(objname);
//    if(shape==NULL)
//    {
//        shape = ShapeModel::instances.findSubstrInstance(objname);
//        if(shape==NULL)
//            return -1;
//    }

//    tf::Pose pose = _nc->basePose().inverse() * shape->_location;
//    tf::Vector3 offset = pose.getOrigin();

//    std::cout << "Moveto gear type is " << shape->_type << "\n";

//    tf::Pose gripperoffset  = _nc->gripperoffset()[shape->_type];
//    std::cout << "Moveto gripperoffset     " << dumpPoseSimple(gripperoffset) << "\n";

//    tf::Pose endPose =  tf::Pose(_nc->QBend(), offset) ;
//    std::cout << "Moveto endpose           " << dumpPoseSimple(endPose) << "\n";

//    endPose=endPose* gripperoffset;
//    std::cout << "Moveto endpose & Offset  " << dumpPoseSimple(endPose) << "\n";

//    moveTo(endPose);
//    return 0;
//}
