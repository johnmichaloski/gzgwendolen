
#ifndef _CRCL2RCS_H
#define _CRCL2RCS_H

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

// C++ includes
#include <list>

// boost includes
#include <boost/shared_ptr.hpp>
#include "boost/iostreams/stream.hpp"
#include "boost/iostreams/device/null.hpp"

// eigen math library includes
#ifdef EIGEN
#include "Eigen/Core"
#include "Eigen/Geometry"
#endif

#include "aprs_headers/RCSThreadTemplate.h"
#include "aprs_headers/RCSMsgQueueThread.h"
#include "aprs_headers/IRcs.h"

#include "crcl_rosmsgs/CrclCommandMsg.h"
#include "crcl_rosmsgs/CrclStatusMsg.h"


#include "crclapp/CrclServer.h"
#include "crclapp/CrclSubscriberInterface.h"
#include "crclapp/CrclPublisherInterface.h"
#include "crclapp/Crcl.h"
#include "crclapp/CrclWm.h"

/**
 * @brief The CCrcl2RosMsg class  handles the command/status interface to ROS.
 * For commands, CCrcl2RosMsg publishes an  CrclCommandMsg
 */
class CCrcl2RosMsg
#ifdef QTHREAD

        : public CMessageQueue  // CMessageQueueThread
#else
        : public RCS::Thread
#endif
{

public:
#ifndef QTHREAD
    CMessageQueue msgq;
#endif

    /**
     * @brief CCrcl2RosMsg handles translation from crcl representation into ros/tf representation
     * @param xml_string - urdf xml string to parse for joint name information
     * @param base_link -  base link of robot for joint name parsing
     * @param tip_link -  tip link of the robot - not including end effector
     */
    CCrcl2RosMsg(std::string xmlString, std::string baseLink, std::string tipLink);


    /**
    * @brief Cyclic loop for the controller. Reads Crcl input mexsage queue, interprets into canon cmds if any, reads canon
    * cmds queue, interprets into robot command messages.
    * @return  1 sucess,  0 problem
    */
    virtual int action();

    /**
     * @brief Initialization routine for the controller..
     */
    virtual void setup();

    /**
     * @brief sendCmdRosMessage handles communication of crcl command message
     * (just queuing).
     * @param rosmsg
     */
    virtual void sendCmdRosMessage(crcl_rosmsgs::CrclCommandMsg &rosmsg);

    /**
     * @brief StatusUpdate accepts rcs/ros status message and translates into CRCL status.
     * @param statusmsg ros message of status
     */
    void statusUpdate(const crcl_rosmsgs::CrclStatusMsg::ConstPtr& statusmsg);

    /**
     * @brief setCmdQueue set CRCL command queue
     * @param crclcmdsq
     */
    void setCmdQueue(RCS::CrclMessageQueue *crclcmdsq)
    {
        // should all referencces to crclcmdsq be mutexed?
        std::lock_guard<std::mutex> guard(CCrcl2RosMsg::_crclmutex);
        this->crclcmdsq=crclcmdsq;
    }




    ////////////////////////////////////////////
    static std::mutex cncmutex; /**< mutex for thread safe access to RobotProgram commands  */

    static std::mutex _crclmutex;


    boost::shared_ptr<Crcl::CrclSubscriberDelegateInterface> crclinterface;
    std::string xmlString;
    std::string baseLink;
    std::string tipLink;

    static long last_cmdnum;


    // interface to sending command message to RCS via DLL
    static RCS::CrclMessageQueue *crclcmdsq;

};
#endif
