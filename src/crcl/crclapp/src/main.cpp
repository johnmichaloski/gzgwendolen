#include <string>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>

#include <readline/readline.h>

#include "aprs_headers/Debug.h"
#include "aprs_headers/Config.h"
#include "aprs_headers/env.h"
#include "aprs_headers/RCSThreadTemplate.h"

#include "crclapp/Globals.h"
#include "crclapp/CrclRos.h"
#include "crclapp/CommandLineInterface.h"
#include "crclapp/NistCrcl.h"
#include "crclapp/CrclWm.h"
#include "crclapp/Demo.h"

#ifndef MAJOR
#define MAJOR  1
#define MINOR 0
#define BUILD 1
#endif

// Globals
std::shared_ptr<CGearDemo> geardemo;
std::shared_ptr<CRosCrclClient> rosCrclClient;


static void cleanup()
{
    // This order shuts things down.
    // Stop demo (i.e. testing) related activies
    if(geardemo.get() != nullptr)
        geardemo->stop();

    // Stopping application
    Globals.bRunning=false;

    // subscribers must be shutdown...
    if(rosCrclClient.get() != nullptr)
        rosCrclClient->stop();

    // Shutdown ROS
    Ros.close();

    // This shuts down the crcl socket (port 64444) server
    if(pCrclServer.get() != nullptr)
        pCrclServer->stop();

    // ^C pressed - stop all threads or will hang
    RCS::Thread::stopAll(); // includes thread for Controller, robotstatus


//    // ^C pressed - stop all threads or will hang
//    RCS::Thread::stopAll(); // includes thread for Controller, robotstatus

}


int main(int argc, char** argv)
{
    std::string robot;
    Nist::Config robotconfig;
    Globals.bDebug=true;

    try
    {
        Globals.catchControlC();  // this must be here or ^C seems to be ignored
        std::string config_file="Config.ini";
        char c;
        while ((c = getopt (argc, argv, "f:r:")) != -1)
            switch (c)
            {
            case 'c':
                config_file = optarg;
                break;
            case 'r':
                robot=optarg;
                break;
            }


        std::string inifile =  getexefolder() + "config/"+ config_file;
        if(!robotconfig.loadFile(inifile))
            throw "ini file  " + inifile + "did not open";

        if(!robotconfig.isSection(robot))
        {
            throw "Configuration file does not have robot '"+robot+ "' defined";

        }

        Globals.appProperties["appPath"] = getexepath();
        Globals.appProperties["appName"] = getexepath().substr(getexepath().find_last_of('/') + 1);
        Globals.appProperties["PackageSrcPath"] = getexefolder();
        Globals.appProperties["version"] = std::string("")+std::to_string(MAJOR) +":"+std::to_string(MINOR) +":"+std::to_string(BUILD) ;

        // ROS configuration
        Globals.sRosMasterUrl = robotconfig.getSymbolValue<std::string>("system.RosMasterUrl","http://localhost:11311");
        //Globals.sRosPackageName = robotconfig.getSymbolValue<std::string>("system.RosPackageName","crclapp");
        Globals.sRosPackageName ="crclapp";
        if(robot.empty())
        {
            throw std::string("Empty -r \"robot\"  command line argument\n");
        }
        Globals.sRosPackageName=robot+Globals.sRosPackageName;
        Globals.bAutoCrclStatus=1;

        // Robot configuration
        rcs_robot.configure(robot, inifile);

        // WM configuration
        rcs_world.configure(robot, inifile);

        // Demo setup
        geardemo=std::shared_ptr<CGearDemo>(new CGearDemo(crclApi));
        geardemo->start();
        geardemo->init(inifile,robot);

        rosCrclClient=  std::shared_ptr<CRosCrclClient>(new CRosCrclClient(pCrclServer));

        // Setup command line interface
        RCS::CComandLineInterface cli;
        cli.start();

        // Setup up ROS
        Ros.init();

        pCrclServer->setCmdQueue(&(rosCrclClient->crclcmds));
        pCrclServer->start();

        // Initializes ROS communication to topic using robot prefix...
        rosCrclClient->setup(robot);
        rosCrclClient->start();

        // initialize the demostate state table
        int demostate=0;

        do {

            if(geardemo->isDone(demostate))
                demostate=0;

            // This will hang on auto as it now changes the state back to noop after every empty fetch.
            // Keep last state in cli?
            int clistate= cli.inputState();
            if(clistate==rcs_state::EXITING)
            {
                CGlobals::bRunning=false;
                break;
            }
            if(clistate==rcs_state::PAUSED)
                CGlobals::bPaused=true;

            if(clistate==rcs_state::NORMAL)
                CGlobals::bPaused=false;

            if(clistate==rcs_state::ONCE)
            {
                CGlobals::bPaused=false;
                Globals.bCannedDemo()=true;
            }
            if(clistate==rcs_state::AUTO)
            {
                CGlobals::bPaused=false;
                Globals.bCannedDemo()=true;
            }
            if(clistate==rcs_state::REPEAT)
            {
                CGlobals::bPaused=false;
                Globals.bCannedDemo()=true;
                Globals.bRepeatCannedDemo=true;
            }


            // If canned demo AND finished last commands
            if(!CGlobals::bPaused && Globals.bCannedDemo() && ! rcs_robot.isBusy())
            {
                if(geardemo->issueRobotCommands(demostate)<0)
                {
                    if(Globals.bRepeatCannedDemo)
                    {
                        // FIXME: add again
                        //rosCrclClient->reset();
                        ::sleep(2.0);
                    }
                    else
                        Globals.bCannedDemo()=false;
                }
            }

            if(clistate==rcs_state::ONCE)
            {
                CGlobals::bPaused=true;
            }

            Globals.sleep(100);


        } while (Globals.ok());

        std::cerr << "Cntrl C pressed  or CLI quit\n" << std::flush;

        cleanup();

    }
    catch (std::string & e)
    {
        std::cerr << Globals.strFormat("%s\nAt: %s\nReason: %s\n", "Abnormal exception end to crclapp ",  Globals.getTimeStamp().c_str(), e.c_str() );
        cleanup();
    }
    catch (std::exception & e)
    {
        std::cerr << Globals.strFormat("%s\nAt: %s\nReason: %s\n", "Abnormal exception end to crclapp ", Globals.getTimeStamp().c_str(), e.what());
        cleanup();
    }
    catch (...)
    {
        std::cerr << "Abnormal exception end to  crclapp at " <<  Globals.getTimeStamp().c_str();
        cleanup();
    }
    std::cout << "crclapp: Stopped " << Globals.getTimeStamp().c_str() << "\n" ;
}
