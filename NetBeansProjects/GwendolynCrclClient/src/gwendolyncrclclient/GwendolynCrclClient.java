/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package gwendolyncrclclient;
// Pose math

import java.util.logging.Level;
import java.util.logging.Logger;
import rcs.posemath.PmCartesian;
import rcs.posemath.PmQuaternion;

/**
 * GwendolynCrclClient is the main class.
 * @author michalos
 */
public class GwendolynCrclClient {

    public static CRCLClient crcl;

    /**
     * main method. sets up the glbal flags and '
     * if a "fake" crcl then calls all the fake 
     * setups. Once setup runs through the
     * kitting demo until done.
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        try {
            // TODO code application logic here
            Globals.bDebug=false;  // no in depth diagnostics
            Globals.bLoopback = true;
            CShapes.initDefinitions();  // define gears trays object properties
            // Seems to work
            //System.out.print(KittingDemo.dumpDefinitions());
            rcs_robot.hardcode();  // define robot
            rcs_world.hardcode();

            crcl = new CRCLClient();

            KittingDemo kittingdemo = new KittingDemo(crcl);
            // Seems to work
            if (Globals.bDebug) {
                System.out.print(KittingDemo.dumpInstances());
            }
            
            KittingDemo.fakeFirstOrderLogic();
            // Seems to work
            if (Globals.bDebug) {
                System.out.print(KittingDemo.dumpInferences());
            }

            // read CRCL status, report
            ///statuskitting();
            // test of hard coded fanuc kitting publisher/client
            // fanuckitting();
            int demostate = 0;
            int bozo;

            do {
                if (kittingdemo.isDone(demostate)) {
                    demostate = 0;
                }
                bozo = kittingdemo.issueRobotCommands(demostate);
                demostate = bozo;
                Thread.sleep(50);
            } while (demostate >= 0);
        } catch (Exception ex) {
            Logger.getLogger(GwendolynCrclClient.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

}
