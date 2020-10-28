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


#ifndef DEMO_H
#define DEMO_H


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

#include "aprs_headers/Core.h"
#include "aprs_headers/IRcs.h"

#include "crclapp/Shape.h"
#include "crclapp/CrclApi.h"


// Gear Demo 
struct CGearDemo
{
    CGearDemo(std::shared_ptr<CCrclApi>  crclApi);
    int init(std::string inifile, std::string robotName);
    int issueRobotCommands(int & state);
    int issueCommands(int & state);
    int isDone(int & state );
    int isWorking( );
    void start();
    void stop();

    WorldModel::CShape * findFreeGear(WorldModel::CInstances &now_instances, std::string geartype);

    bool findOpenKittingGearSlot(WorldModel::CInstances &now_instances,
                                 WorldModel::CShape & kit,
                                 std::map<std::string, std::string> &slotprop);

protected:
    std::shared_ptr<CCrclApi>  r;
    WorldModel::CShape * _gear;
    WorldModel::CShape  _kit;
    std::map<std::string, std::string> _openslotprop;
    WorldModel::CShapes _shapes;
    std::string _path;
    tf::Pose _baseoffset;
 };

#endif
