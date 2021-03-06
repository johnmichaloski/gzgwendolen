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

#ifndef GAZEBO_H
#define GAZEBO_H

#include <string>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif


#if GAZEBO_MAJOR_VERSION >= 8
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
typedef ignition::math::Pose3d Pose;
typedef ignition::math::Vector3d Vector3d;
typedef ignition::math::Quaterniond Quaterniond;
#else
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Quaternion.hh>
typedef gazebo::math::Pose Pose;
typedef gazebo::math::Vector3 Vector3d;
typedef gazebo::math::Quaternion Quaterniond;
#endif

#include "aprs_headers/Config.h"
#include <sensor_msgs/JointState.h>
#include <unordered_set>

// THese must be on include path
#include "GripCommand.pb.h"
#include "JointsComm.pb.h"


typedef const boost::shared_ptr<message::GripCommand const> ConstGripperCmdPtr;
typedef const boost::shared_ptr<message::JointsComm const> ConstRobotCmdPtr;


/**
 * @brief The CGazebo class waits for a gazebo server to be running - i.e., gzserver program.
 * Loads gazebos argc,argv parameters.
 * Creates a node for gazebo protobuf communication used by any gazebo publisher/subscriber.
 */
class CGazebo
{
public:
    /**
     * @brief CGazebo constructor.
     */
    CGazebo();

    /**
     * @brief init waits for a gazebo server to be running - i.e., gzserver program.
     * Loads gazebos argc,argv parameters.
     * Creates a node for gazebo protobuf communication used by any gazebo publisher/subscriber.
     * @param robotname name of robot.
     * @param inifile file path of ini file for all gazebo configuration.
     * @return empty string if no error.
     */
    std::string  init(std::string robotname,
                      std::string inifile);

    /**
     * @brief isInited
     * @return true if gazebo sucessfully inited.
     */
    bool isInited() { return _bInited; }

    /**
     * @brief stop make sure to shut everything down/disconnected in gazebo.
     */
    void stop();

    /**
     * @brief _node used by to establish gazebo communication by all gazebo clients.
     */
    static gazebo::transport::NodePtr _node;
    static std::string _inifile;
    static std::string _robotname;


private:
    static  bool _bInited; /**< inited once to establsh communcation */
    gazebo::transport::SubscriberPtr _sub; /**< subscriber to model info topic */
};
extern CGazebo gz;

/**
 * @brief The CGzModelReader class establishes communication subscriber to read all gz models
 * updates. Update only caputres name and centroid pose.
 */
class CGzModelReader
{
public:
    CGzModelReader();

    /**
     * @brief configure read ini file for parameterization. Don't need robot name.
     * @return empty string success, error message otherwise.
     * Will throw and easy to catch if debugging.
     */
    std::string init();

    /**
     * @brief start subscribes to topic /gazebo/default/ariac/model. Fixme: make configurable
     * topic name.
     */
    void start();
    /**
     * @brief stop cleanly detaches gazebo update of model callback.
     */
    void stop();

    /**
     * @brief reset the gear model poses to their original positions
     */
    void reset();
    /**
     * @brief onUpdate save model instance name and pose. Convert from
     * ignition::math to tf::Pose.
     * Hard coded call to demo  ShapeModel::instances.storeInstance(_msg->name(), tfpose);
     * @param msg model communication message from gazebo.
     */
    void onUpdate(ConstModelPtr & msg);
    void onModelUpdate(ConstModelPtr &_msg);

    static std::map<std::string, long> gzModelName2Id;
    static std::map<std::string, long> gzLinkName2Id;

private:
    gazebo::transport::SubscriberPtr _modelsub; /**< subscriber to topic /gazebo/default/ariac/model */
    gazebo::transport::SubscriberPtr _sub; /**< subscriber to topic /gazebo/default/ariac/model */
    std::mutex _mymutex;
    std::string _modeltopicname;
    gazebo::transport::PublisherPtr modelPub; // = this->node->Advertise<msgs::Model>("~/model/modify");
#if GAZEBO_MAJOR_VERSION >= 8
    std::map<std::string, ignition::math::Pose3d> _gearStartingPoses;
    std::map<std::string, ignition::math::Pose3d>::iterator gearit;
    static std::map<std::string, ignition::math::Vector3d> gzModelBoundingBox;

#else
    std::map<std::string, ::gazebo::msgs::Pose> _gearStartingPoses;
    std::map<std::string, ::gazebo::msgs::Pose>::iterator gearit;;
#endif

};



/**
 * @brief The CGzRobotHandler class two threads are operating. One thread is sending the latest
 * robot joints to a gazebo subscriber. Another thread is updating the joint values with
 * the latest values.
 */
class CGzRobotHandler
{
public:

    /*!
     * \brief CGzRobotHandler constructor .
     */
    CGzRobotHandler() {
        _readCycles=0;
        _writeCycles=0;
    }

    /**
     * @brief Init - initializes connection to gazebo robot
     * @param robotName is the string containing robot name
     */
    virtual std::string init(std::string robotName
                             //std::string gzRobotJointPrefix,
                             //std::string gzRobotCmdTopicName,
                             //std::string gzRobotStatusTopicName
                             );

    /**
     * @brief Start start updating joints
     * @param robotjoingnames names of robot joints that will be updated
     * @param fingernames names of gripper joints that will be updated. Empty if none.
     */
    virtual void start(std::vector<std::string> robotjoingnames,
                       std::vector<std::string> fingernames);

    /**
     * @brief Stop stop advertising gazebo topic
     */
    virtual void stop();

    /**
     * \brief UpdateRobot writes the robot joint_state topic values to listener.
     * \param time time of update
     * \param joint a sensor_msgs::JointState describing robot joints
     * \return boolean if write occurred as expected.
     */
    bool updateRobot(double time, sensor_msgs::JointState joint);

    /**
     * \brief UpdateGripper writes the robot joint_state topic values to listener.
     * \param time time of gripper update
     * \param joint sensor_msgs::JointState describing gripper joints
     * \return boolean if write occurred as expected.
     */
    bool updateGripper(double time, sensor_msgs::JointState joint);

    /**
    * @brief OnStatusUpdate receive robot  status in separate thread
    * @param _msg gazebo message callback describing robot status
    */
    void OnStatusUpdate(ConstRobotCmdPtr &_msg);

    sensor_msgs::JointState robotStatus()
    {
        return _robotStatusJoints; /// robot joints latest values - includes gripper?
    }

    /**
     * @brief readCycles return status feedback cycles
     * @return positive count
     */
    size_t readCycles() { return _readCycles; }
    /**
     * @brief writeCycles returns number of cycle written to robot
     * @return positive  number of cycles
     */
    size_t writeCycles() { return _writeCycles; }
    ////////////////////////////////
private:
    /// \brief number of callback message status reads
    size_t _readCycles;
    size_t _writeCycles;
    /// \brief custom gazebe communication topic name to write robot joints command
    std::string _mGzRobotCmdTopicName;

    /// \brief custom gazebe communication topic name to read robot joints status
    std::string _mGzRobotStatusTopicName;

    /// \brief  gazebo joint prefix for gripper joints
    std::string _mGzRobotJointPrefix;

    gazebo::transport::PublisherPtr _robotCmdPub; ///  custom joints communication  gazebo publisher
    gazebo::transport::SubscriberPtr _robotStatusSub; ///  custom joints communication  gazebo publisher

    bool _bRunning; /// flag indicating whether publish thread is running
    sensor_msgs::JointState _robotCmdJoints; /// robot joints commanded latest values
    sensor_msgs::JointState _robotStatusJoints; /// robot joints latest values - includes gripper?
    sensor_msgs::JointState _gripperJoints;  /// gripper fingers to send vel/fmax
    std::mutex _mymutex;
};


/**
 * @brief The CGzParallelGripper class two threads are operating. One thread is sending the latest
 * gripper command to a gazebo pluging subscriber. Another thread is updating the gripper status  with
 * the latest gripper status.
 */
class CGzParallelGripper
{
public:

    /*!
     * \brief CGzParallelGripper constructor .
     */
    CGzParallelGripper() {
        _readCycles=0;
    }

    /**
     * @brief Init - initializes connection to gazebo robot
     * @param robotName name of robot for gazebo section
     */
    virtual std::string init(std::string robotName);

    /**
     * @brief Start start updating joints
     */
    virtual void start();

    /**
     * @brief Stop stop advertising gazebo topic
     */
    virtual void stop();


    /**
     * \brief UpdateGripper writes the robot joint_state topic values to listener.
     * \param time time of gripper update
     * \param a sensor_msgs::JointState describing gripper joints
     * \return boolean if write occurred as expected.
     */
    bool updateGripper(double time, sensor_msgs::JointState joint);
    bool updateGripper(double time, double eepercent);


    /**
     * @brief OnStatusUpdate receive gripper command status in separate thread
     * @param _msg gzebo messager from plugin describing current status
     */
    void OnStatusUpdate(ConstGripperCmdPtr &_msg);


    /**
     * @brief isGrasping return parameter from reading gripper status
     * @return 1 grasping(closed), 0 not grasping (open)
     */
    int isGrasping();
    /**
     * @brief readCycles return status feedback cycles
     * @return count
     */
    size_t readCycles() { return _readCycles; }
    ////////////////////////////////
private:
    /// \brief number of callback message status reads
    size_t _readCycles;
    ////////////////////////////////
    std::string _mGzGripperCmdTopicName;   /// gazebo topic name to write gripper information
    std::string _mGzGripperStatusTopicName;   /// gazebo topic name to write gripper information

    gazebo::transport::PublisherPtr _gripperpub; /// gripper cmd publisher  to pavel gazebo plugin
    gazebo::transport::SubscriberPtr _grippersub; /// gripper cmd subscriber  to pavel gazebo plugin
    bool _bRunning; /// flag indicating whether publish thread is running
    std::mutex _mymutex;
    double _mEE;
    int _nGripperState;
};


#endif // GAZEBO_H
