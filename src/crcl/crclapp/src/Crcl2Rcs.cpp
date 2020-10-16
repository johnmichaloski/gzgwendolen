
/*
 * DISCLAIMER:
 * This software was produced by the National Institute of Standards
 * and Technology (NIST), an agency of the U.S. government, and by statute is
 * not subject to copyright in the United States.  Recipients of this software
 * assume all responsibility associated with its operation, modification,
 * maintenance, and subsequent redistribution.
 *
 * See NIST Administration Manual 4.09.07 b and Appendix I.
 */

#include <urdf/model.h>
#include <CRCLStatus.hxx>
#include "crclapp/Crcl2Rcs.h"
#include <aprs_headers/IRcs.h>
#include <aprs_headers/Debug.h>
#include <crclapp/NistCrcl.h>

using namespace urdf;
using namespace RCS;
namespace RCS {
    CMessageQueue<RCS::CCanonCmd> cmds;
    CCanonWorldModel wm; // for motion control planning wm
}
boost::mutex CCrcl2RosMsg::cncmutex;

size_t num_joints;
size_t num_links;
RCS::CrclMessageQueue *CCrcl2RosMsg::crclcmdsq;



////////////////////////////////////////////////////////////////////////////////
CCrcl2RosMsg::CCrcl2RosMsg(std::string xml_string, std::string base_link, std::string tip_link) :
    RCS::Thread(.01, "CCrcl2RosMsg")
{
    this->xmlString=xml_string;
    this->baseLink=base_link;
    this->tipLink=tip_link;
    this->crclcmdsq=NULL;
}


////////////////////////////////////////////////////////////////////////////////
void CCrcl2RosMsg::statusUpdate(const crcl_rosmsgs::CrclStatusMsg::ConstPtr& statusmsg)
{
    std::lock_guard<std::mutex> guard(_crclmutex);
     try {
        rcs_robot._status=statusmsg;
        crclinterface->crclsettings.StatusID() = (unsigned long long) statusmsg->crclcommandnum;
        static char * value[] =
        {
            "CRCL_Done",
            "CRCL_Error",
            "CRCL_Working",
            "CRCL_Ready"
        };
        //        crclinterface->crclwm.CommandStatus() = ::CommandStateEnumType(CanonStatusType::value(statusmsg->crclcommandstatus));
        if(statusmsg->crclcommandstatus < 4)
            crclinterface->crclsettings.Update( ::CommandStateEnumType( value[statusmsg->crclcommandstatus]));

        // FIXME: this has to be in the agreed upon CRCL units
        tf::Pose pose = Convert< geometry_msgs::Pose, tf::Pose>(statusmsg->statuspose);

        // default tf length units are Meters - convert to crcl
        pose.setOrigin(pose.getOrigin() *  1.0/crclinterface->crclsettings._lengthConversion);
        crclinterface->crclsettings.Update(pose );

        sensor_msgs::JointState js = statusmsg->statusjoints;

        for(size_t i=0; i< js.position.size(); i++)
            js.position[i]=js.position[i]* 1.0/crclinterface->crclsettings._angleConversion;

        crclinterface->crclsettings.Update((sensor_msgs::JointState&) js);
        crclinterface->crclsettings.Gripper().Position() = statusmsg->eepercent;

        for(size_t i=0; i< WorldModel::instances.size(); i++)
        {
            tf::Pose pose = WorldModel::instances[i].centroid();
            crclinterface->crclsettings.Update(WorldModel::instances[i].name(), pose);
            crclinterface->crclsettings.Update(WorldModel::instances[i]);
        }
    }
    catch(...)
    {
        ROS_DEBUG("crclwm command status failed");
    }
}

////////////////////////////////////////////////////////////////////////////////
void CCrcl2RosMsg::setup()
{

    // Controller instantiatio of shared objects - NOT dependent on ROS
    crclinterface = boost::shared_ptr<Crcl::CrclSubscriberDelegateInterface>(
            new Crcl::CrclSubscriberDelegateInterface());
    //crclinterface->SetAngleUnits("DEGREE");

    // Couple code attempts at reading from robot_description parameter - see above
    crclinterface->crclsettings.jointnames.clear();

    rcs_robot.parseURDF(xmlString, baseLink,  tipLink);

    crclinterface->crclsettings.jointnames=rcs_robot.jointNames;
}

////////////////////////////////////////////////////////////////////////////////
int CCrcl2RosMsg::action()
{
    try {

        int n=0;
        /////////////////////////////////////////////////////////////////////////////////////////////
        // See if new CRCL commanded motion - if so, interpret as RCS command in session
        if(CCrclSession::InMessages().sizeMsgQueue() > n)
        {
            CrclMessage msg = CCrclSession::InMessages().popFrontMsgQueue();
            std::string crclcmd = boost::get<0>(msg);

            if(Globals.DEBUG_Crcl_Command)
            {
                std::cout << crclcmd;
            }


            Crcl::CrclReturn ret = crclinterface->DelegateCRCLCmd(crclcmd);

            if (ret == Crcl::CANON_STATUSREPLY )
            {
                Crcl::CrclStatus crclstatus;
                {
                    // can't have reporting and robot controller so make copy
                    std::lock_guard<std::mutex> guard(_crclmutex);
                    crclstatus = crclinterface->crclsettings;
                }
                std::string sStatus = Crcl::CrclPublisherCmdInterface().GetStatusReply(&crclstatus);

                if(!sStatus.empty())
                {
                    // no mutex as there is only one session running handling all client CRCL messages
                    CCrclSession *_pSession;
                    _pSession = boost::get<1>(msg);
                    _pSession->SyncWrite(sStatus);
                    if(Globals.DEBUG_Crcl_Status)
                    {
                        std::cout << sStatus;
                    }
                }

                if(crcl::crclServer::bDebugCrclStatusMsg)
                {
                    ROS_DEBUG("===========================================================\n"
                              "Status:\n%s",
                              sStatus.c_str());
                    ROS_DEBUG("===========================================================");
                }

            }
            if(crcl::crclServer::bFlywheel)
            {
                n=CCrclSession::InMessages().sizeMsgQueue();
            }
        }
        if (Globals.bAutoCrclStatus)
        {
            Crcl::CrclStatus crclstatus;
            {
                // get copy of current crcl status settings
                std::lock_guard<std::mutex> guard(_crclmutex);
                crclstatus = crclinterface->crclsettings;
            }
            std::string sStatus = Crcl::CrclPublisherCmdInterface().GetStatusReply(&crclstatus);

            if(!sStatus.empty())
            {
                // no mutex as there is only one session running handling all client CRCL messages
                CCrclSession *_pSession=crclCommServer.get();
                _pSession->SyncWrite(sStatus);
                if(Globals.DEBUG_Crcl_Status)
                {
                    std::cout << sStatus;
                }
            }
        }

        /////////////////////////////////////////////////////////////////////////////////////////////
        // interpret translated CRCL command. Commands in canonical form: standard units (mm, radians)
        // Note many CRCL commands are NOT translated into corresponding canoncial ROS commands
        // FIXME: this is an extra step that is probably unncessary
        n = 0; // flywheel implementation - whether to process all messages at once or per cycle.
        for (;RCS::cmds.sizeMsgQueue() > n;)
        {
            RCS::CCanonCmd cc = RCS::cmds.popFrontMsgQueue();
            crcl_rosmsgs::CrclCommandMsg rosmsg;

            rosmsg.crclcommand = CanonCmdType::CANON_NOOP; //nocommand;
            //rosmsg.crclcommandnum = cc.CommandID();
            rosmsg.crclcommandnum = (long unsigned int) cc.CrclCommandID();
            CCanonCmd::setRosMsgTimestamp(rosmsg.header);
            // ROS equivalent
            //rosmsg.header.stamp=ros::Time::now();

            if(cc.crclcommand == CanonCmdType::CANON_INIT_CANON)
            {
                rosmsg.crclcommand=CanonCmdType::CANON_INIT_CANON; // initCanon;
            }
            else if(cc.crclcommand == CanonCmdType::CANON_END_CANON)
            {
                rosmsg.crclcommand=CanonCmdType::CANON_END_CANON; // endCanon;
            }
            else if (cc.crclcommand == CanonCmdType::CANON_MOVE_JOINT)
            {
                rosmsg.crclcommand = CanonCmdType::CANON_MOVE_JOINT; // actuatejoints;
                rosmsg.jointnum=cc.jointnum;
                rosmsg.joints = cc.joints; // this passes pos, vel, accel/force/torque
                rosmsg.bCoordinated = cc.bCoordinated;
            }
            else if (cc.crclcommand == CanonCmdType::CANON_MOVE_TO)
            {
                rosmsg.crclcommand = CanonCmdType::CANON_MOVE_TO; //moveto;
                rosmsg.finalpose.position.x = cc.finalpose.position.x;
                rosmsg.finalpose.position.y = cc.finalpose.position.y;
                rosmsg.finalpose.position.z = cc.finalpose.position.z;
                rosmsg.finalpose.orientation.x = cc.finalpose.orientation.x;
                rosmsg.finalpose.orientation.y = cc.finalpose.orientation.y;
                rosmsg.finalpose.orientation.z = cc.finalpose.orientation.z;
                rosmsg.finalpose.orientation.w = cc.finalpose.orientation.w;

                // Fixme: rates are ignored by RCS crcl interpreter
                if(cc.Rates().CurrentTransSpeed() >0.0)
                {
                    // clear rate vector
                    rosmsg.profile.clear();

                    // save current translation rate definition in move
                    // ignores specified acceleration for now
                    ::crcl_rosmsgs::CrclMaxProfileMsg trans_profile;
                    trans_profile.maxvel=cc.Rates().CurrentTransSpeed();
                    trans_profile.maxacc=10.*cc.Rates().CurrentTransSpeed();
                    trans_profile.maxjerk= 100.*cc.Rates().CurrentTransSpeed();
                    rosmsg.profile.push_back(trans_profile);

                    // save current rotational rate definition in move
                    // ignores specified acceleration for now
                    ::crcl_rosmsgs::CrclMaxProfileMsg rot_profile;
                    rot_profile.maxvel=cc.Rates().CurrentRotSpeed();
                    rot_profile.maxacc=10.*cc.Rates().CurrentRotSpeed();
                    rot_profile.maxjerk= 100.*cc.Rates().CurrentRotSpeed();
                    rosmsg.profile.push_back(rot_profile);
                }
            }
            else if (cc.crclcommand == CanonCmdType::CANON_STOP_MOTION)
            {
                // Fixme: there are other parameters specifying stop
                if(crcl::crclServer::bCrclStopIgnore)
                {
                    rosmsg.crclcommand = CanonCmdType::CANON_STOP_MOTION;
                    //cc.stoptype; // fixme: add stoptype to crcl messages
                }            // publish ros message if found corresponding crcl command
                if (rosmsg.crclcommand != CanonCmdType::CANON_NOOP) {
                    ROS_DEBUG("ROS command: [%d] ", rosmsg.crclcommand);
                    crclcmdsq->addMsgQueue(rosmsg);
                }
             }
            else if (cc.crclcommand == CanonCmdType::CANON_MOVE_THRU)
            {
                rosmsg.crclcommand = CanonCmdType::CANON_MOVE_THRU;
                rosmsg.finalpose = cc.finalpose;

                // now save waypoints
                rosmsg.waypoints=cc.waypoints;
            }
            else if (cc.crclcommand == CanonCmdType::CANON_DWELL)
            {
                rosmsg.crclcommand = CanonCmdType::CANON_DWELL;
                rosmsg.dwell_seconds = cc.dwell_seconds;
            }
            else if (cc.crclcommand == CanonCmdType::CANON_SET_GRIPPER)
            {
                rosmsg.crclcommand = CanonCmdType::CANON_SET_GRIPPER;
                rosmsg.eepercent = cc.eepercent;
            }
            else if (cc.crclcommand == CanonCmdType::CANON_PAVEL_GRIPPER)
            {
                rosmsg.crclcommand = CanonCmdType::CANON_PAVEL_GRIPPER;
                rosmsg.eepercent = cc.eepercent;
            }
            else if (cc.crclcommand == CanonCmdType::CANON_SET_EE_PARAMETERS)
            {
                rosmsg.crclcommand = CanonCmdType::CANON_SET_EE_PARAMETERS;
                rosmsg.parameter_names = cc.parameterNames;
                rosmsg.parameter_values = cc.parameterValues;
            }
            // added oct 30 2018 to see if "processing all messages important"
            // actually no flywheel but processAllCrclMessages
            else if(!crcl::crclServer::bProcessAllCrclMessages)
            {
                rosmsg.crclcommand=CanonCmdType::CANON_NOOP;
            }

            // publish ros message if found corresponding crcl command
            if (rosmsg.crclcommand != CanonCmdType::CANON_NOOP) {
                sendCmdRosMessage(rosmsg);
            }


            if(crcl::crclServer::bFlywheel)
            {
                n=RCS::cmds.sizeMsgQueue();
            }

        }
    }
    catch (std::exception & e)
    {
        std::cerr << "Exception in  CController::Action() thread: " << e.what() << "\n";
    }
    catch (...)
    {
        std::cerr << "Exception in CController::Action() thread\n";
    }
    return 1;
}
void CCrcl2RosMsg::sendCmdRosMessage(crcl_rosmsgs::CrclCommandMsg &rosmsg)
{
    // publish ros message if found corresponding crcl command
    if (crcl::crclServer::bDebugCrclCommandMsg) {
        ROS_DEBUG("ROS command: %s", CCanonCmd().Set(rosmsg).toString().c_str());
    }

    if (rosmsg.crclcommand != CanonCmdType::CANON_NOOP) {
        ROS_DEBUG("Send ROS command: [%d] ", rosmsg.crclcommand);
        crclcmdsq->addMsgQueue(rosmsg);
    }
}
