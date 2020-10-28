#ifndef CRCLWM_H
#define CRCLWM_H

#include <map>
#include <vector>

#include "aprs_headers/RCSThreadTemplate.h"
#include "aprs_headers/RCSMsgQueueThread.h"
#include "aprs_headers/IRcs.h"

#include "crcl_rosmsgs/CrclCommandMsg.h"
#include "crcl_rosmsgs/CrclStatusMsg.h"
#include "crclapp/Shape.h"

class CCrclApi;
class CCrcl2RosMsg;
class CCrclSession;

namespace crcl{
  class crclServer;
}
namespace RCS
{

/**
 * @brief cmds is the queue for CRCL XML commands that have been translated
 * into ROS equivalent custom command message.
 */
extern  CMessageQueue<RCS::CCanonCmd> cmds;

/**
 * @brief wm contains the ROS representation of the curfrent robot world
 * model.
 */
extern  CCanonWorldModel wm;  // motion related parameters max vel,acc
};
/**
 * @brief crclApi reference to local class instance to generate ROS commands
 * using CRCL command custom messages.
 */
extern  std::shared_ptr<CCrclApi>  crclApi;
extern std::shared_ptr<CCrclSession> crclCommServer;



extern boost::shared_ptr<crcl::crclServer> pCrclServer;

struct rcs_state
{
    static const int NORMAL = 0;
    static const int EXITING = 1;
    static const int PAUSED = 2;
    static const int ERROR = 3;
    static const int ONCE = 4;
    static const int NOOP = 5;
    static const int AUTO = 6;
    static const int REVERSE=7;
    static const int REPEAT=7;
    static const int STEP=8;
};


struct rcs_robot_type
{

    /**
     * @brief robotPrefix
     * @return
     */
    std::string robotPrefix();

    /**
     * @brief numJoints robot's number of joints (assume serial)
     * @return number of joints
     */
    size_t numJoints();

    /**
     * @brief isBusy read the current status to determine if
     * the robot is still busy executing a command
     * @return true if busy
     */
    bool isBusy();

    /**
     * @brief allJointNumbers for CRCL generate vector of numbers
     * for all the joints
     * @return vector of integers, 0..n where n is the number of joints
     * in the robot
     */
    std::vector<unsigned long> allJointNumbers();
    /**
     * @brief configure read ini config file for parameters
     * @param robot prefix name of robot for ini file
     * @param inifile full path of inifile
     */
    void configure(std::string robot,std::string inifile);
    /**
     * @brief ParseURDF accepts an xml urdf string and parses out the joint information.
     * Really just for joint names since CRCL uses numbers.
     * @param xml_string urdf xml
     * @param base_link  base link of the robot
     * @param tip_link tip link of concern to the robot
     * @return true if successful
     */
    bool parseURDF(std::string xmlString, std::string baseLink, std::string tipLink);

    // URDF Derived knowledge
    std::vector<std::string> jointNames;
    std::vector<std::string> linkNames;
    std::vector< double> jointvalues;
    std::vector< double> jointMin;
    std::vector< double> jointMax;
    std::vector< bool> jointHasLimits;
    std::vector< double> jointEffort;
    std::vector< double> jointVelmax;
    std::vector<tf::Vector3> axis;
    std::vector<tf::Vector3> xyzorigin;
    std::vector<tf::Vector3> rpyorigin;
    std::string  robotName;

    // Saved named robot goal states
    std::map<std::string, std::vector<std::string>> namedCommand;
    std::map<std::string, std::vector<double>> namedJointMove;
    std::map<std::string, tf::Pose> namedPoseMove;

    // latest update
    crcl_rosmsgs::CrclStatusMsg::ConstPtr _status;

    // Canned poses
    tf::Pose Retract;         /**< pose offset for retract */
    tf::Pose RetractInv; /**< inverse pose offset for retract */
    tf::Quaternion QBend; /**< rotation to achieve pose rotation for grasping */
    tf::Pose currentPose;
    tf::Pose basePose;

    std::map<std::string, tf::Pose> gripperoffset;       /// gripper offset for each part

    std::string _prefix;  /**< robot name prefix */
    unsigned int crclcommandstatus;
    std::string s_crclcommandstatus;
    unsigned int crclcommandid;

};

struct rcs_world_type
{
    std::map<std::string, tf::Pose> _gearStartingPoses;
    std::map<std::string, tf::Pose>::iterator gearit;;
    std::vector<std::string> part_list; /// robot parts from set of all parts
    std::map<std::string, tf::Pose> slotoffset;          /// gripper offset for container slot
    void configure(std::string robot,std::string inifile);

};

extern rcs_robot_type rcs_robot;
extern rcs_world_type rcs_world;
namespace WorldModel
{
extern CInstances instances;
}

/**
 * @brief instances list of all instances in world.
 */
#endif // CRCLWM_H
