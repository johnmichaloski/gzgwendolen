

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */
//#pragma message "Compiling " __FILE__ 
#include <memory>
#include <mutex>

#include "crclapp/Demo.h"
#include "crclapp/Globals.h"
#include "crclapp/Shape.h"
#include "crclapp/CrclApi.h"
#include "crclapp/CrclWm.h"
#include "crclapp/Crcl2Rcs.h"
#include "aprs_headers/Config.h"

using namespace RCS;
using namespace WorldModel;

// polled wait on function with timeout

#include <chrono>
#include <functional>
typedef std::function<bool ()> TPollFcn;

/**
 * @brief The timer struct
 *  while( gripper.state() != 0 && t.seconds_elapsed() > 1 )
        { ::sleep(50) ; }

    if(gripper.state() == 0)
        return error;
 */

//http://www.cplusplus.com/forum/beginner/91449/
struct timer
{
    typedef std::chrono::steady_clock clock ;
    typedef std::chrono::seconds seconds ;

    timer(TPollFcn fcn) : savefcn(fcn)
    {
    }

    int poll(int equality)
    {
        while( savefcn() != equality && seconds_elapsed() > 1 )
        {
            ::sleep(50) ;
        }

        if(savefcn() == equality)
            return 0;

        return 1;

    }

    void reset()
    {
        start = clock::now() ;
    }

    unsigned long long seconds_elapsed() const
    { return std::chrono::duration_cast<seconds>( clock::now() - start ).count() ; }

    private: clock::time_point start = clock::now() ;
    TPollFcn savefcn;
};



// Log once

int CGearDemo::init(std::string inifile, std::string robotName)
{
    Nist::Config robotconfig;
    if(!robotconfig.loadFile(inifile))
        throw "CGearDemo ini file  " + inifile + "did not open";

    WorldModel::CShapes::initDefinitions();

    Globals.bGripperSpeed=robotconfig.getSymbolValue<int>("demo.gripper_speed","0");
    Globals.bClosestFree=robotconfig.getSymbolValue<int>("demo.closest_free","0");
    Globals.bClosestOpenSlot=robotconfig.getSymbolValue<int>("demo.closet_open_slot","0");
    double dDwellTime = robotconfig.getSymbolValue<double>("demo." + robotName + ".dwell.time", "1000.0");
    double dGraspingDwellTime = robotconfig.getSymbolValue<double>("demo." + robotName + ".dwell.grasping", "1000.0");
    if(crclApi.get() == NULL)
    {
        std::cout << "Demo assigning parameter to NULL crclApi\n";
        return -1;
    }
    crclApi->setDwell(dDwellTime);
    crclApi->setGraspingDwell(dGraspingDwellTime);
    crclApi->medium();
    return 0;
}

void CGearDemo::start()
{
}

void CGearDemo::stop()
{

}

////////////////////////////////////////////////////////////////////////////////
CGearDemo::CGearDemo(std::shared_ptr<CCrclApi>  crclApi) : r(crclApi)
{
    WorldModel::CShapes::initDefinitions();
}

////////////////////////////////////////////////////////////////////////////////
int CGearDemo::issueRobotCommands(int & state)
{

    // Finish queuing commands before handling them....
    //std::unique_lock<std::mutex> lock(cncmutex);

    // Must declare all variables beforehand
    RCS::CCanonCmd cmd;
    tf::Pose pickpose;
    std::string gearname;
    tf::Pose affpose;
    tf::Pose gripperoffset;
    tf::Quaternion bend;
    tf::Vector3 offset;
    tf::Pose slotpose;
    tf::Pose slotoffset;
    tf::Pose placepose;

    if( WorldModel::instances.size()==0)
    {
        std::cerr << "Error: No gear instances can be read from Gazebo model- restart Gazebo!\n";
        return -1;
    }

    switch(state)
    {
    case 0:
    {
        // Find a free gear
        if ((_gear = WorldModel::instances.findFreeGear(rcs_world.part_list, rcs_robot.currentPose)) == NULL)
        {
            ROS_FATAL_ONCE ( "Error: No Free Gear in tray to move");
            return -1;
        }
        return state++;
    }
    case 1:
    {

        gearname = _gear->_name;
        // Ok we will move this gear - mark as no longer free standing
        _gear->_attributes["state"] = "stored";

        affpose =  _gear->_location ;
        affpose = rcs_robot.basePose.inverse() * affpose ;

        // The object gripper offset is where on the object it is to be gripped
        // e.g., offset.gripper->largegear = 0.0,0.0, -0.030, 0.0, 0.0.,0.0
        gripperoffset = rcs_robot.gripperoffset[_gear->_type];
       // gripperoffset = tf::Pose(tf::QIdentity(),tf::Vector3(0.0,0.0, - _instance->_height * .8));

        bend=rcs_robot.QBend;

        // The gripperoffset is the robot gripper offset back to the 0T6 equivalent
        pickpose =  tf::Pose(bend, affpose.getOrigin()) * gripperoffset ;

        offset = pickpose.getOrigin();

        // Retract
        r->moveTo(rcs_robot.Retract * tf::Pose(bend, offset));
        r->doDwell(r->_mydwell);
        return state++;
    }
    case 2:
    {
        r->moveTo(tf::Pose(bend, offset) );
        r->doDwell(r->_mydwell);
        return state++;
    }
    case 3:
    {
        r->closeGripper();
        r->doDwell(r->_mygraspdwell);
        return state++;
    }
    case 4:
    {
        r->moveTo(rcs_robot.Retract * tf::Pose(bend, offset));
        r->doDwell(r->_mydwell);
        return state++;
    }

    case 5:
    {
        // Find a kit slot and then its offset from the centroid of the kit
        // Find a gear slot in a kit
        WorldModel::CShape * kit=NULL;
        WorldModel::CShape * slot=NULL;
        if(!WorldModel::instances.findFreeGearKitSlot(_gear,
                                                      slotpose,
                                                      rcs_world.part_list))
        {
            static int bThrottle=1;
            if(bThrottle>0)
                std::cout << "Error: No Free Kit Slot\n";
            bThrottle--;
            return -1;
        }

        slotpose = rcs_robot.basePose.inverse() * slotpose;
        slotoffset = rcs_world.slotoffset["vessel"];
        placepose = tf::Pose(bend, slotpose.getOrigin())* slotoffset; // fixme: what if gear rotated
        offset = placepose.getOrigin();

        // Approach
        r->moveTo(rcs_robot.Retract* tf::Pose(bend, offset));
        r->doDwell(r->_mydwell);
        return state++;
    }
    case 6:
    {
        // Place the grasped object
        r->moveTo(tf::Pose(bend, offset));
        r->doDwell(r->_mydwell);
        return state++;
    }
    case 7:
    {
        // open gripper and wait
        r->openGripper();
        r->doDwell(r->_mygraspdwell);
        return state++;
    }
    case 8:
    {
        // Retract from placed object
        //r->Retract(0.04);
        r->moveTo(rcs_robot.Retract * tf::Pose(bend, offset));
        r->doDwell(r->_mydwell);
        return state++;
    }
    }
    state=0;
    return state;

}

CShape * CGearDemo::findFreeGear(WorldModel::CInstances &now_instances, std::string geartype)
{
    for(size_t i=0; i< now_instances.size(); i++)
    {
        CShape & instance(now_instances[i]);

        // search if one of my robot kits/vessel, if not continue
        if(std::find(rcs_world.part_list.begin(), rcs_world.part_list.end(), instance._name)==rcs_world.part_list.end())
            continue;

        if(!instance.isVessel())
        {
            continue;
        }

        // kit - now do slots
        CShape * slotz = now_instances.getDefinition(instance.type());

        if(slotz==NULL)
            continue;

        for(size_t j=0; j < slotz->_contains.size(); j++)
        {
            CShape & slot(slotz->_contains[j]);
            std::map<std::string, std::string> &p =  instance._properties[slot.name()];
            if(p["type"]==geartype && p["state"]!="open")
            {
                // look up gear by name
              return now_instances.findInstance(p["state"]);
            }
        }

    }
    return nullptr;
}
bool CGearDemo::findOpenKittingGearSlot(WorldModel::CInstances &now_instances,
                                        CShape & kit,
                                        std::map<std::string, std::string> &slotprop)
{
    for(size_t i=0; i< now_instances.size(); i++)
    {
        CShape & instance(now_instances[i]);

        // search if one of my robot kits/vessel, if not continue
        if(std::find(rcs_world.part_list.begin(), rcs_world.part_list.end(), instance._name)==rcs_world.part_list.end())
            continue;

        if(!instance.isKit())
        {
            continue;
        }

        // kit - now do slots
        CShape * slotz = now_instances.getDefinition(instance.type());

        if(slotz==NULL)
            continue;

        // at this point instance is a kitting tray
        for(size_t j=0; j < slotz->_contains.size(); j++)
        {
            CShape & slot(slotz->_contains[j]);
            slotprop =  instance._properties[slot.name()];
            if(slotprop["state"]=="open")
            {
               kit=instances[i];
               return true;
            }
        }

    }
    return false;
}
////////////////////////////////////////////////////////////////////////////////
int CGearDemo::issueCommands(int & state)
{

    // Finish queuing commands before handling them....
    //std::unique_lock<std::mutex> lock(cncmutex);

    // Must declare all variables beforehand
    RCS::CCanonCmd cmd;
    tf::Pose pickpose;
    std::string gearname;
    tf::Pose affpose;
    tf::Pose gripperoffset;
    tf::Quaternion bend;
    tf::Vector3 offset;
    tf::Pose slotpose;
    tf::Pose slotoffset;
    tf::Pose placepose;
    if( WorldModel::instances.size()==0)
    {
        std::cerr << "Error: No gear instances can be read from gazebo_ro_api topic - restart Gazebo!\n";
        return -1;
    }


    WorldModel::CInstances now_instances;
    {
        std::unique_lock<std::mutex> lock(WorldModel::shapemutex);
        now_instances=instances;
    }

    switch(state)
    {
    case 0:
    {
        // Step: Find an open kitting slot and a free gear

        // Search kits for empty slot. Identify type of gear
        // find corredponding gear size in supply gtray.
         ;
        if(!findOpenKittingGearSlot(now_instances, _kit, _openslotprop))
        {
            ROS_FATAL_ONCE ( "Error: No open slots in any kits fopenslotpropound!");
            return -1;
        }

         std::string geartype = _openslotprop["type"];
        // this gear is in this slot should be checked to exist every iteration
        // of state table
        if ((_gear = findFreeGear(now_instances , geartype)) == nullptr)
        {
            ROS_FATAL_ONCE ( "Error: No matching free gear in supply vessel to move");
            return -1;
        }
        if(Globals.bDebug)
            std::cout << "Free slot kit="<< _kit.name() << " slot="<< _openslotprop["name"]<<
                        " Gear=" << _gear->name()  << " at" << RCS::dumpPoseSimple(_gear->centroid()) << "\n";
        return state++;
    }
    case 1:
    {

        // step: approach gear

        gearname = _gear->_name;

        if(Globals.bDebug)
            std::cout <<  "World Gear Coord=" << RCS::dumpPoseSimple(_gear->centroid()) << "\n";


        affpose =  _gear->_location ;
        affpose = rcs_robot.basePose.inverse() * affpose ;
        if(Globals.bDebug)
        {
            std::cout <<  "Robot base    Coord=" << RCS::dumpPoseSimple(rcs_robot.basePose) << "\n";
            std::cout <<  "Robot baseinv Coord=" << RCS::dumpPoseSimple(rcs_robot.basePose.inverse()) << "\n";
            std::cout <<  "Robot Gear Coord   =" << RCS::dumpPoseSimple(affpose) << "\n";
        }

        // The object gripper offset is where on the object it is to be gripped
        // e.g., offset.gripper->largegear = 0.0,0.0, -0.030, 0.0, 0.0.,0.0
        gripperoffset = rcs_robot.gripperoffset[_gear->_type];

        bend=rcs_robot.QBend;

        // The gripperoffset is the robot gripper offset back to the 0T6 equivalent
        pickpose =  tf::Pose(bend, affpose.getOrigin()) * gripperoffset ;

        offset = pickpose.getOrigin();
        if(Globals.bDebug)
            std::cout <<  "Robot Retract Coord=" << RCS::dumpPoseSimple(pickpose) << "\n";

        // Retract
        r->moveTo(rcs_robot.Retract * tf::Pose(bend, offset));
        r->doDwell(r->_mydwell);
        return state++;
    }
    case 2:
    {
        // move to grasping posiiton of gear
        r->moveTo(tf::Pose(bend, offset) );
        r->doDwell(r->_mydwell);
        return state++;
    }
    case 3:
    {
        // step: grasp gear
        r->closeGripper();
        r->doDwell(r->_mygraspdwell);
        return state++;
    }
    case 4:
    {
        // step: retract robot after grasping gear
        r->moveTo(rcs_robot.Retract * tf::Pose(bend, offset));
        r->doDwell(r->_mydwell);
        return state++;
    }

    case 5:
    {
        // step: move to approach kitting open slot
        // Use existing found open kit slot
        // Offset from the centroid of the kit and reorientation
        // should be part of inferences
        //now_instances.findSlot(this->_kit, this->_openslot->name());

        // This xyz already has been reoriented by tray rotation.
        CShape * openslot=instances.findSlot(&_kit, _openslotprop["name"]);

        // compute reorient based on kit rotation
        tf::Pose slotloc = openslot->_location; // offset of locatino in tray
#if 1
        tf::Matrix3x3 m(_kit._location.getRotation());
        tf::Vector3 vec_slot = _kit._location.getOrigin() +  (m*slotloc.getOrigin());
        slotpose=tf::Pose(tf::QIdentity(), vec_slot);
 #endif
        // adjust model z from world coordinate based to robot coordinate frame
        slotpose = rcs_robot.basePose.inverse() * slotpose;

        // up in z onloy for now - hard coded
        slotoffset = rcs_world.slotoffset["vessel"];
        placepose = tf::Pose(bend, slotpose.getOrigin())* slotoffset; // fixme: what if gear rotated

        offset = placepose.getOrigin(); // xyz position

        // Approach
        r->moveTo(rcs_robot.Retract* tf::Pose(bend, offset));
        r->doDwell(r->_mydwell);
        return state++;
    }
    case 6:
    {
        // Place the grasped object
        r->moveTo(tf::Pose(bend, offset));
        r->doDwell(r->_mydwell);
        return state++;
    }
    case 7:
    {
        // open gripper and wait
        r->openGripper();
        r->doDwell(r->_mygraspdwell);
        return state++;
    }
    case 8:
    {
        // Retract from placed object
        //r->Retract(0.04);
        r->moveTo(rcs_robot.Retract * tf::Pose(bend, offset));
        r->doDwell(r->_mydwell);
        return state++;
    }
    }
    state=0;
    return state;

}
int CGearDemo::isDone(int & state)
{
    return state >=9;
}
int CGearDemo::isWorking( )
{

    // FIXME: read status, for done.
    //rcs_robot.crclcommandid
    //rcs_robot.s_crclcommandstatus == "CRCL_Done"
    std::lock_guard<std::mutex> guard(CCrcl2RosMsg::_crclmutex);

    unsigned int lastCmdId=  CCrcl2RosMsg::last_cmdnum;
    bool bFlag = ((rcs_robot.crclcommandid==lastCmdId )
            && boost::iequals(rcs_robot.s_crclcommandstatus , "CRCL_Done"));
    return bFlag;
}








